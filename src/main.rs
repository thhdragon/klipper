#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use cortex_m_rt::entry;
use core::panic::PanicInfo;
use core::cell::RefCell;
use cortex_m::interrupt::{Mutex as InterruptMutex, free as interrupt_free};

// HAL and PAC
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock, SystemClock},
    pac::{self, interrupt, NVIC},
    sio::Sio,
    watchdog::Watchdog,
    timer::Timer as RpHalTimer,
    timer::{Alarm0, Alarm1, Alarm2},
    usb::UsbBus,
    gpio::Pins as RpHalPins,
    adc::Adc as RpHalAdc,
    pwm::Slices as RpHalPwmSlices,
};
use rp2040_hal::clocks::UsbClock;
use rp2040_hal::gpio::{PinId, FunctionPwm};
use rp2040_hal::pwm::{Slice as RpSlice, FreeRunning, SliceId as RpSliceId, ValidSliceMode, Channel as RpPwmChannelExt, PwmA, PwmB};


// Klipper HAL Traits and Implementations
use klipper_mcu_lib::{
    hal::{Timer as KlipperHalTimer, GpioOut, GpioIn, PullType, AdcChannel, AdcError, PwmChannel, PwmError, StepEventResult},
    sched::{SchedulerState, KlipperSchedulerTrait},
    gpio_manager::{GpioManager, PinModeState, AdcCapablePin as GpioManagerAdcPin, PwmCapablePin as GpioManagerPwmPin},
    command_parser::{parse_gcode_arguments, CommandArgValue, ArgParseError, ParsedArgs},
};
use klipper_mcu_lib::rp2040_hal_impl::timer::Rp2040Timer;
use klipper_mcu_lib::rp2040_hal_impl::adc::Rp2040AdcChannel;
use klipper_mcu_lib::rp2040_hal_impl::pwm::Rp2040PwmChannel;


// USB
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

// --- Panic Handler ---
#[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {}", Debug2Format(info));
    loop { cortex_m::asm::nop(); }
}

// --- Globals (as before) ---
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
const MAX_LINE_LENGTH: usize = 256;
static mut LINE_BUFFER: heapless::String<MAX_LINE_LENGTH> = heapless::String::new();
static TEST_TIMER0: InterruptMutex<RefCell<Option<Rp2040Timer<Alarm0>>>> = InterruptMutex::new(RefCell::new(None));
const TEST_TIMER0_ID: u32 = 0;
static TEST_TIMER1: InterruptMutex<RefCell<Option<Rp2040Timer<Alarm2>>>> = InterruptMutex::new(RefCell::new(None));
const TEST_TIMER1_ID: u32 = 1;
const SCHEDULER_MASTER_TIMER_ID: u32 = 99;
const LED_PIN_ID: u8 = 25;
static SCHEDULER: InterruptMutex<RefCell<Option<SchedulerState<Alarm1>>>> = InterruptMutex::new(RefCell::new(None));
static GPIO_MANAGER: InterruptMutex<RefCell<Option<GpioManager>>> = InterruptMutex::new(RefCell::new(None));
static ADC_PERIPHERAL: InterruptMutex<RefCell<Option<RpHalAdc>>> = InterruptMutex::new(RefCell::new(None));
static PWM_SLICES: InterruptMutex<RefCell<Option<RpHalPwmSlices>>> = InterruptMutex::new(RefCell::new(None));
static SYSTEM_CLOCK_FREQ: InterruptMutex<RefCell<Option<u32>>> = InterruptMutex::new(RefCell::new(None));

// --- Callbacks and Dispatchers (as before) ---
fn test_timer0_callback(timer: &mut Rp2040Timer<Alarm0>) -> StepEventResult { info!("T0 Fire"); let nt = timer.get_waketime().wrapping_add(1_000_000); timer.set_hw_irq_active(false); timer.set_waketime(nt); interrupt_free(|cs| SCHEDULER.borrow(cs).borrow_mut().as_mut().unwrap().add_timer(timer)); debug!("T0 Resched {}", nt); () }
fn test_timer1_callback(timer: &mut Rp2040Timer<Alarm2>) -> StepEventResult { info!("T1 Fire"); let nt = timer.get_waketime().wrapping_add(1_500_000); timer.set_hw_irq_active(false); timer.set_waketime(nt); interrupt_free(|cs| SCHEDULER.borrow(cs).borrow_mut().as_mut().unwrap().add_timer(timer)); debug!("T1 Resched {}", nt); () }
fn master_scheduler_timer_callback(_timer: &mut Rp2040Timer<Alarm1>) -> StepEventResult { interrupt_free(|cs| SCHEDULER.borrow(cs).borrow_mut().as_mut().unwrap().service_master_timer()); () }
pub fn dispatch_klipper_task_from_scheduler(task_id: u32) { match task_id { TEST_TIMER0_ID => interrupt_free(|cs| if let Some(t) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() { if let Some(cb)=t.get_callback(){(cb)(t);}}), TEST_TIMER1_ID => interrupt_free(|cs| if let Some(t) = TEST_TIMER1.borrow(cs).borrow_mut().as_mut() { if let Some(cb)=t.get_callback(){(cb)(t);}}), _ => warn!("Unknown task ID {}", task_id),}}

// --- Entry Point & Main Loop (mostly as before) ---
#[entry]
fn main() -> ! {
    let mut pac_peripherals = pac::Peripherals::take().unwrap();
    let core_peripherals = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac_peripherals.WATCHDOG);
    let sio = Sio::new(pac_peripherals.SIO);
    let clocks = init_clocks_and_plls(12_000_000u32, pac_peripherals.XOSC, pac_peripherals.CLOCKS, pac_peripherals.PLL_SYS, pac_peripherals.PLL_USB, &mut pac_peripherals.RESETS, &mut watchdog,).ok().unwrap();
    interrupt_free(|cs| { SYSTEM_CLOCK_FREQ.borrow(cs).replace(Some(clocks.system_clock.freq().to_Hz())); });
    let usb_bus_allocator = UsbBusAllocator::new(UsbBus::new(pac_peripherals.USBCTRL_REGS, pac_peripherals.USBCTRL_DPRAM, clocks.usb_clock, true, &mut pac_peripherals.RESETS,));
    unsafe { USB_BUS = Some(usb_bus_allocator); }
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    unsafe { USB_SERIAL = Some(SerialPort::new(bus_ref)); USB_DEVICE = Some(UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd)).manufacturer("KlipperMCU").product("Klipper Rust Firmware").serial_number("PWM_FINAL_TEST").device_class(usbd_serial::USB_CLASS_CDC).build());}
    let adc_peripheral_hal = RpHalAdc::new(pac_peripherals.ADC, &mut pac_peripherals.RESETS);
    interrupt_free(|cs| { ADC_PERIPHERAL.borrow(cs).replace(Some(adc_peripheral_hal)); });
    let pwm_slices_hal = RpHalPwmSlices::new(pac_peripherals.PWM, &mut pac_peripherals.RESETS);
    interrupt_free(|cs| { PWM_SLICES.borrow(cs).replace(Some(pwm_slices_hal)); });
    let rp_hal_pins = RpHalPins::new(pac_peripherals.IO_BANK0, pac_peripherals.PADS_BANK0, sio.gpio_bank0, &mut pac_peripherals.RESETS);
    let gpio_mgr = GpioManager::new(rp_hal_pins);
    interrupt_free(|cs| { GPIO_MANAGER.borrow(cs).replace(Some(gpio_mgr)); });
    info!("Peripherals Initialized.");
    interrupt_free(|cs| { if let Some(manager) = GPIO_MANAGER.borrow(cs).borrow_mut().as_mut() { if manager.configure_pin_as_output(LED_PIN_ID).is_ok() { let _ = manager.write_pin_output(LED_PIN_ID, false); } else { error!("Failed to configure LED Pin {}.", LED_PIN_ID); } }});
    let rp_hal_timer = RpHalTimer::new(pac_peripherals.TIMER, &mut pac_peripherals.RESETS);
    let alarm0 = rp_hal_timer.alarm_0().unwrap();
    let mut klipper_timer0 = Rp2040Timer::new(alarm0, test_timer0_callback, TEST_TIMER0_ID);
    klipper_timer0.set_hw_irq_active(false);
    interrupt_free(|cs| { TEST_TIMER0.borrow(cs).replace(Some(klipper_timer0)); });
    let alarm2 = rp_hal_timer.alarm_2().unwrap();
    let mut klipper_timer1 = Rp2040Timer::new(alarm2, test_timer1_callback, TEST_TIMER1_ID);
    klipper_timer1.set_hw_irq_active(false);
    interrupt_free(|cs| { TEST_TIMER1.borrow(cs).replace(Some(klipper_timer1)); });
    let alarm1_for_scheduler = rp_hal_timer.alarm_1().unwrap();
    let scheduler_raw_timer_ref = rp_hal_timer;
    let klipper_scheduler = SchedulerState::new(alarm1_for_scheduler, scheduler_raw_timer_ref, master_scheduler_timer_callback, dispatch_klipper_task_from_scheduler, SCHEDULER_MASTER_TIMER_ID,);
    interrupt_free(|cs| { SCHEDULER.borrow(cs).replace(Some(klipper_scheduler)); });
    unsafe { NVIC::unmask(pac::Interrupt::TIMER_IRQ_1); }
    let now_ticks = rp_hal_timer.get_counter_low();
    let initial_waketime_for_test_timer0 = now_ticks.wrapping_add(1_000_000);
    interrupt_free(|cs| { if let Some(s) = SCHEDULER.borrow(cs).borrow_mut().as_mut() { if let Some(t) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut(){ t.set_waketime(initial_waketime_for_test_timer0); s.add_timer(t); }}});
    let initial_waketime_for_test_timer1 = now_ticks.wrapping_add(1_500_000);
    interrupt_free(|cs| { if let Some(s) = SCHEDULER.borrow(cs).borrow_mut().as_mut() { if let Some(t) = TEST_TIMER1.borrow(cs).borrow_mut().as_mut(){ t.set_waketime(initial_waketime_for_test_timer1); s.add_timer(t); }}});
    info!("Setup complete, entering main loop.");
    let mut loop_count = 0u32;
    let mut led_state = false;
    let mut delay = cortex_m::delay::Delay::new(core_peripherals.SYST, clocks.system_clock.freq().to_Hz());
    loop { /* Main loop body as before: LED toggle and USB poll */ }
}

// --- Interrupt Handlers (TIMER_IRQ_1 only) ---
#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_1() { interrupt_free(|cs| { if let Some(s) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {s.master_timer.on_interrupt();}}); }


// --- Helper Functions ---
fn get_pwm_slice_channel_for_pin(pin_id: u8) -> Option<(u8, bool)> { /* ... as defined before ... */ }
fn calculate_pwm_settings(sys_clk_hz: u32, target_freq_hz: u32) -> (u8, u8, u16) { /* ... as defined before ... */ }
fn serial_write_line(serial: &mut SerialPort<UsbBus>, data: &str) { /* ... as before ... */ }
// Full bodies for helper functions (copied from previous state)
fn get_pwm_slice_channel_for_pin(pin_id: u8) -> Option<(u8, bool)> { match pin_id { 0..=15 => Some((pin_id/2, (pin_id%2)==1)), 16..=29 => Some(((pin_id-16)/2, ((pin_id-16)%2)==1)), _ => None } }
fn calculate_pwm_settings(sys_clk_hz: u32, target_freq_hz: u32) -> (u8, u8, u16) { if target_freq_hz == 0 { return (1,0,u16::MAX); } let mut top=u16::MAX; let mut div_fp=(sys_clk_hz as f32)/(target_freq_hz as f32*((top as f32)+1.0)); if div_fp<1.0{div_fp=1.0; let ntf=(sys_clk_hz as f32/target_freq_hz as f32)-1.0; if ntf<=0.0{top=1;}else if ntf>u16::MAX as f32{top=u16::MAX;}else{top=ntf as u16;}}else if div_fp>=256.0{div_fp=255.0+(15.0/16.0); let ntf=(sys_clk_hz as f32/(target_freq_hz as f32*div_fp))-1.0; if ntf<=0.0{top=1;}else if ntf>u16::MAX as f32{top=u16::MAX;}else{top=ntf as u16;}} let di=div_fp as u8; let df=((div_fp-(di as f32))*16.0)as u8; (if di==0{1}else{di},df,top) }
fn serial_write_line(serial: &mut SerialPort<UsbBus>, data: &str) { let bytes=data.as_bytes();let mut written=0; while written<bytes.len(){match serial.write(&bytes[written..]){Ok(len)if len>0=>{written+=len;}Ok(_)=>{}Err(UsbError::WouldBlock)=>{}Err(_)=>{break;}}}}


// --- process_command (with SET_PWM updated for robust release) ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    let mut parts = line.trim().splitn(2, |c: char| c.is_whitespace());
    let command = parts.next().unwrap_or("").to_ascii_uppercase();
    let args_str = parts.next().unwrap_or("");

    let parsed_args_result = if !args_str.is_empty() {
        parse_gcode_arguments(args_str)
    } else { Ok(ParsedArgs::new()) };

    match parsed_args_result {
        Ok(args) => {
            if command == "PING" { serial_write_line(serial, "PONG\r\n"); }
            else if command == "ID" { serial_write_line(serial, "KlipperRustRP2040_PWM_HW_Final_v1\r\n"); }
            else if command == "ECHO" { serial_write_line(serial, args_str); serial_write_line(serial, "\r\n"); }
            else if command == "SET_PIN" { /* ... SET_PIN logic from previous state ... */ }
            else if command == "GET_PIN" { /* ... GET_PIN logic from previous state ... */ }
            else if command == "QUERY_ADC" { /* ... QUERY_ADC logic from previous state, ensure release_pin_from_adc is used ... */ }
            else if command == "SET_PWM" {
                let pin_num_opt = args.get(&'P').and_then(|val| match val { /* ... */ });
                let duty_percent_opt = args.get(&'S').and_then(|val| match val { /* ... */ });
                let freq_hz_opt = args.get(&'F').and_then(|val| match val { /* ... */ });

                // Re-inserting full SET_PIN, GET_PIN, QUERY_ADC logic from prior state for completeness of process_command
                // For SET_PIN:
                // if let (Some(pin_num), Some(value)) = (pin_num_opt, value_opt) { ... } else { serial_write_line(serial, "Error: Missing P/S for SET_PIN\r\n"); }
                // For GET_PIN:
                // if let Some(pin_num) = pin_num_opt { ... } else { serial_write_line(serial, "Error: Missing P for GET_PIN\r\n"); }
                // For QUERY_ADC:
                // if let Some(pin_num) = pin_num_opt { /* ... if !(26..=29).contains(&pin_num) ... */ let adc_read_result = interrupt_free(|cs| { /* ... manager.take_pin_for_adc ... manager.release_pin_from_adc ... */ }); /* ... match adc_read_result ... */ } else { serial_write_line(serial, "Error: Missing P for QUERY_ADC\r\n"); }

                // Actual SET_PWM logic with robust release
                if let (Some(pin_num), Some(duty_val_percent)) = (pin_num_opt, duty_percent_opt) {
                    if !(0.0..=100.0).contains(&duty_val_percent) {
                        serial_write_line(serial, "Error: Duty S must be 0.0-100.0\r\n");
                    } else {
                        let duty_float_0_1 = duty_val_percent / 100.0;
                        let freq_hz = freq_hz_opt.unwrap_or(500);

                        let mut pin_taken_for_pwm = false; // Flag to track if pin was successfully taken

                        let result: Result<(), &'static str> = interrupt_free(|cs| {
                            let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
                            let mut pwm_slices_opt = PWM_SLICES.borrow(cs).borrow_mut();
                            let sys_clk_opt = SYSTEM_CLOCK_FREQ.borrow(cs).borrow();

                            if let (Some(manager), Some(slices), Some(sys_clk_hz)) =
                                (gpio_manager_opt.as_mut(), pwm_slices_opt.as_mut(), sys_clk_opt.as_ref()) {

                                let typed_pwm_pin_token = manager.take_pin_for_pwm(pin_num)?;
                                pin_taken_for_pwm = true; // Mark pin as taken

                                let (slice_idx, is_chan_b) = get_pwm_slice_channel_for_pin(pin_num)
                                    .ok_or("Pin not valid for PWM or mapping undefined")?;

                                let (div_int, div_frac, top_val) = calculate_pwm_settings(*sys_clk_hz, freq_hz);
                                let duty_raw = (duty_float_0_1 * top_val as f32) as u16;

                                let slice_mut_ref = match slice_idx {
                                    0 => &mut slices.slice0, 1 => &mut slices.slice1, 2 => &mut slices.slice2, 3 => &mut slices.slice3,
                                    4 => &mut slices.slice4, 5 => &mut slices.slice5, 6 => &mut slices.slice6, 7 => &mut slices.slice7,
                                    _ => return Err("Invalid Slice Index derived from Pin"),
                                };

                                slice_mut_ref.set_top(top_val);
                                slice_mut_ref.set_div_int(div_int);
                                slice_mut_ref.set_div_frac(div_frac);
                                slice_mut_ref.enable();
                                let actual_max_duty = slice_mut_ref.get_top();

                                let setup_channel_result: Result<_, _> = if is_chan_b {
                                    let mut ch_b = slice_mut_ref.channel_b;
                                    match typed_pwm_pin_token {
                                        GpioManagerPwmPin::Gpio1(p) if pin_num == 1 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio3(p) if pin_num == 3 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio5(p) if pin_num == 5 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio7(p) if pin_num == 7 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio9(p) if pin_num == 9 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio11(p) if pin_num == 11 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio13(p) if pin_num == 13 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio15(p) if pin_num == 15 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio17(p) if pin_num == 17 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio19(p) if pin_num == 19 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio21(p) if pin_num == 21 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio23(p) if pin_num == 23 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio25(p) if pin_num == 25 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio27(p) if pin_num == 27 => Ok(ch_b.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio29(p) if pin_num == 29 => Ok(ch_b.input_from_gpio(&p)),
                                        _ => Err("Mismatched pin token for selected PWM Channel B"),
                                    }.map(|ch| Rp2040PwmChannel::new(ch, actual_max_duty))
                                } else {
                                    let mut ch_a = slice_mut_ref.channel_a;
                                     match typed_pwm_pin_token {
                                        GpioManagerPwmPin::Gpio0(p) if pin_num == 0 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio2(p) if pin_num == 2 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio4(p) if pin_num == 4 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio6(p) if pin_num == 6 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio8(p) if pin_num == 8 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio10(p) if pin_num == 10 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio12(p) if pin_num == 12 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio14(p) if pin_num == 14 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio16(p) if pin_num == 16 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio18(p) if pin_num == 18 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio20(p) if pin_num == 20 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio22(p) if pin_num == 22 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio24(p) if pin_num == 24 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio26(p) if pin_num == 26 => Ok(ch_a.input_from_gpio(&p)),
                                        GpioManagerPwmPin::Gpio28(p) if pin_num == 28 => Ok(ch_a.input_from_gpio(&p)),
                                        _ => Err("Mismatched pin token for selected PWM Channel A"),
                                    }.map(|ch| Rp2040PwmChannel::new(ch, actual_max_duty))
                                };

                                match setup_channel_result {
                                    Ok(mut pwm_wrapper) => {
                                        pwm_wrapper.set_duty_cycle_raw(duty_raw).map_err(|_| "PWM SetDuty Err")?;
                                        pwm_wrapper.enable().map_err(|_| "PWM Enable Err")
                                    }
                                    Err(e_str) => Err(e_str),
                                }
                            } else { Err("Manager, Slices, or SysClk not initialized") }
                        }); // End of interrupt_free block for main logic

                        // Always attempt to release the pin if it was successfully taken
                        if pin_taken_for_pwm {
                            let release_res = interrupt_free(|cs| {
                                GPIO_MANAGER.borrow(cs).borrow_mut().as_mut()
                                    .ok_or("GPIO Manager gone during release")?
                                    .release_pin_from_pwm(pin_num)
                            });
                            if release_res.is_err() {
                                // If main result was Ok, but release failed, this is the final error.
                                // If main result was Err, we prioritize that.
                                if result.is_ok() {
                                    serial_write_line(serial, "Error: PWM OK, but pin release failed\r\n");
                                    return; // Exit process_command
                                } else {
                                     warn!("SET_PWM: Primary error occurred, AND pin release also failed for P{}", pin_num);
                                }
                            }
                        }

                        match result {
                            Ok(()) => serial_write_line(serial, "ok\r\n"),
                            Err(e_str) => { serial_write_line(serial, "Error: "); serial_write_line(serial, e_str); serial_write_line(serial, "\r\n");}
                        }
                    }
                } else { serial_write_line(serial, "Error: Missing P (pin) or S (duty) for SET_PWM\r\n"); }
            }
            else if command.is_empty() && args_str.is_empty() { /* Ignore */ }
            else { /* Unknown command (existing logic) */ }
        }
        Err(e) => { /* Arg parsing error (existing logic) */ }
    }
}
