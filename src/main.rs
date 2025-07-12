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
    stepper::{Stepper, StepperDirection},
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

// --- Globals ---
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
static STEPPER_X: InterruptMutex<RefCell<Option<Stepper>>> = InterruptMutex::new(RefCell::new(None));
const STEPPER_X_ID: u32 = 2;
const STEPPER_X_SCHEDULER_TASK_ID: u32 = 2;
const STEPPER_X_STEP_PIN: u8 = 16;
const STEPPER_X_DIR_PIN: u8 = 17;
const LED_PIN_ID: u8 = 25;
static SCHEDULER: InterruptMutex<RefCell<Option<SchedulerState<Alarm1>>>> = InterruptMutex::new(RefCell::new(None));
static GPIO_MANAGER: InterruptMutex<RefCell<Option<GpioManager>>> = InterruptMutex::new(RefCell::new(None));
static ADC_PERIPHERAL: InterruptMutex<RefCell<Option<RpHalAdc>>> = InterruptMutex::new(RefCell::new(None));
static PWM_SLICES: InterruptMutex<RefCell<Option<RpHalPwmSlices>>> = InterruptMutex::new(RefCell::new(None));
static SYSTEM_CLOCK_FREQ: InterruptMutex<RefCell<Option<u32>>> = InterruptMutex::new(RefCell::new(None));

// --- Callbacks and Dispatchers ---
// ... (All callback and dispatcher functions as before) ...
fn test_timer0_callback(timer: &mut Rp2040Timer<Alarm0>) -> StepEventResult { info!("T0 Fire"); let nt = timer.get_waketime().wrapping_add(1_000_000); timer.set_hw_irq_active(false); timer.set_waketime(nt); interrupt_free(|cs| SCHEDULER.borrow(cs).borrow_mut().as_mut().unwrap().add_timer(timer)); debug!("T0 Resched {}", nt); () }
fn test_timer1_callback(timer: &mut Rp2040Timer<Alarm2>) -> StepEventResult { info!("T1 Fire"); let nt = timer.get_waketime().wrapping_add(1_500_000); timer.set_hw_irq_active(false); timer.set_waketime(nt); interrupt_free(|cs| SCHEDULER.borrow(cs).borrow_mut().as_mut().unwrap().add_timer(timer)); debug!("T1 Resched {}", nt); () }
fn master_scheduler_timer_callback(_timer: &mut Rp2040Timer<Alarm1>) -> StepEventResult { interrupt_free(|cs| SCHEDULER.borrow(cs).borrow_mut().as_mut().unwrap().service_master_timer()); () }
pub fn dispatch_klipper_task_from_scheduler(task_id: u32) {
    match task_id {
        TEST_TIMER0_ID => { interrupt_free(|cs| if let Some(t) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() { if let Some(cb)=t.get_callback(){(cb)(t);}})},
        TEST_TIMER1_ID => { interrupt_free(|cs| if let Some(t) = TEST_TIMER1.borrow(cs).borrow_mut().as_mut() { if let Some(cb)=t.get_callback(){(cb)(t);}})},
        STEPPER_X_SCHEDULER_TASK_ID => {
            interrupt_free(|cs| {
                let mut scheduler_opt = SCHEDULER.borrow(cs).borrow_mut();
                let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
                let mut stepper_x_opt = STEPPER_X.borrow(cs).borrow_mut();
                if let (Some(ref mut scheduler), Some(ref mut gpio_manager), Some(ref mut stepper_x)) =
                    (scheduler_opt.as_mut(), gpio_manager_opt.as_mut(), stepper_x_opt.as_mut()) {
                    stepper_x.step_event_callback(scheduler, gpio_manager);
                } else { warn!("dispatch: Failed to get resources for STEPPER_X_SCHEDULER_TASK_ID ({})", task_id); }
            });
        }
        _ => warn!("Scheduler: Dispatch for unknown task ID {}", task_id),
    }
}

// --- Entry Point ---
#[entry]
fn main() -> ! { /* ... peripheral setup, globals init, timer/scheduler/stepper setup as before ... */
    let mut pac_peripherals = pac::Peripherals::take().unwrap();
    let core_peripherals = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac_peripherals.WATCHDOG);
    let sio = Sio::new(pac_peripherals.SIO);
    let clocks = init_clocks_and_plls(12_000_000u32, pac_peripherals.XOSC, pac_peripherals.CLOCKS, pac_peripherals.PLL_SYS, pac_peripherals.PLL_USB, &mut pac_peripherals.RESETS, &mut watchdog,).ok().unwrap();
    interrupt_free(|cs| { SYSTEM_CLOCK_FREQ.borrow(cs).replace(Some(clocks.system_clock.freq().to_Hz())); });
    let usb_bus_allocator = UsbBusAllocator::new(UsbBus::new(pac_peripherals.USBCTRL_REGS, pac_peripherals.USBCTRL_DPRAM, clocks.usb_clock, true, &mut pac_peripherals.RESETS,));
    unsafe { USB_BUS = Some(usb_bus_allocator); }
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    unsafe { USB_SERIAL = Some(SerialPort::new(bus_ref)); USB_DEVICE = Some(UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd)).manufacturer("KlipperMCU").product("Klipper Rust Firmware").serial_number("STEPPER_TEST").device_class(usbd_serial::USB_CLASS_CDC).build());}
    let adc_peripheral_hal = RpHalAdc::new(pac_peripherals.ADC, &mut pac_peripherals.RESETS);
    interrupt_free(|cs| { ADC_PERIPHERAL.borrow(cs).replace(Some(adc_peripheral_hal)); });
    let pwm_slices_hal = RpHalPwmSlices::new(pac_peripherals.PWM, &mut pac_peripherals.RESETS);
    interrupt_free(|cs| { PWM_SLICES.borrow(cs).replace(Some(pwm_slices_hal)); });
    let rp_hal_pins = RpHalPins::new(pac_peripherals.IO_BANK0, pac_peripherals.PADS_BANK0, sio.gpio_bank0, &mut pac_peripherals.RESETS);
    let gpio_mgr = GpioManager::new(rp_hal_pins);
    interrupt_free(|cs| { GPIO_MANAGER.borrow(cs).replace(Some(gpio_mgr)); });
    info!("Peripherals Initialized.");
    interrupt_free(|cs| { if let Some(manager) = GPIO_MANAGER.borrow(cs).borrow_mut().as_mut() { if manager.configure_pin_as_output(LED_PIN_ID).is_ok() { let _ = manager.write_pin_output(LED_PIN_ID, false); } else { error!("Failed to configure LED Pin {}.", LED_PIN_ID); } }});
    interrupt_free(|cs| { if let Some(ref mut manager) = GPIO_MANAGER.borrow(cs).borrow_mut().as_mut() { match Stepper::new(STEPPER_X_ID, STEPPER_X_SCHEDULER_TASK_ID, STEPPER_X_STEP_PIN, STEPPER_X_DIR_PIN, manager,) { Ok(stepper_instance) => { STEPPER_X.borrow(cs).replace(Some(stepper_instance)); info!("STEPPER_X (ID {}) initialized.", STEPPER_X_ID); } Err(e) => { error!("Failed to initialize STEPPER_X: {}", e); } } } else { error!("GpioManager not available for STEPPER_X initialization."); }});
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
    loop { /* Main loop body as before */ }
}

// --- Interrupt Handlers ---
#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_1() { interrupt_free(|cs| { if let Some(s) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {s.master_timer.on_interrupt();}}); }

// --- Helper Functions ---
fn get_pwm_slice_channel_for_pin(pin_id: u8) -> Option<(u8, bool)> { /* ... */ }
fn calculate_pwm_settings(sys_clk_hz: u32, target_freq_hz: u32) -> (u8, u8, u16) { /* ... */ }
fn serial_write_line(serial: &mut SerialPort<UsbBus>, data: &str) { /* ... */ }
// Full bodies for helper functions (copied from previous state)
fn get_pwm_slice_channel_for_pin(pin_id: u8) -> Option<(u8, bool)> { match pin_id { 0..=15 => Some((pin_id/2, (pin_id%2)==1)), 16..=29 => Some(((pin_id-16)/2, ((pin_id-16)%2)==1)), _ => None } }
fn calculate_pwm_settings(sys_clk_hz: u32, target_freq_hz: u32) -> (u8, u8, u16) { if target_freq_hz == 0 { return (1,0,u16::MAX); } let mut top=u16::MAX; let mut div_fp=(sys_clk_hz as f32)/(target_freq_hz as f32*((top as f32)+1.0)); if div_fp<1.0{div_fp=1.0; let ntf=(sys_clk_hz as f32/target_freq_hz as f32)-1.0; if ntf<=0.0{top=1;}else if ntf>u16::MAX as f32{top=u16::MAX;}else{top=ntf as u16;}}else if div_fp>=256.0{div_fp=255.0+(15.0/16.0); let ntf=(sys_clk_hz as f32/(target_freq_hz as f32*div_fp))-1.0; if ntf<=0.0{top=1;}else if ntf>u16::MAX as f32{top=u16::MAX;}else{top=ntf as u16;}} let di=div_fp as u8; let df=((div_fp-(di as f32))*16.0)as u8; (if di==0{1}else{di},df,top) }
fn serial_write_line(serial: &mut SerialPort<UsbBus>, data: &str) { let bytes=data.as_bytes();let mut written=0; while written<bytes.len(){match serial.write(&bytes[written..]){Ok(len)if len>0=>{written+=len;}Ok(_)=>{}Err(UsbError::WouldBlock)=>{}Err(_)=>{break;}}}}

// --- process_command (with DEBUG_STEPPER command) ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    let mut parts = line.trim().splitn(2, |c: char| c.is_whitespace());
    let command = parts.next().unwrap_or("").to_ascii_uppercase();
    let args_str = parts.next().unwrap_or("");
    let parsed_args_result = if !args_str.is_empty() { parse_gcode_arguments(args_str) } else { Ok(ParsedArgs::new()) };

    match parsed_args_result {
        Ok(args) => {
            if command == "PING" { /* ... */ }
            else if command == "ID" { serial_write_line(serial, "KlipperRustRP2040_Stepper_v1\r\n"); }
            else if command == "ECHO" { /* ... */ }
            else if command == "SET_PIN" { /* ... */ }
            else if command == "GET_PIN" { /* ... */ }
            else if command == "QUERY_ADC" { /* ... */ }
            else if command == "SET_PWM" { /* ... */ }
            else if command == "DEBUG_STEPPER" {
                let steps_opt = args.get(&'S').and_then(|val| match val {
                    CommandArgValue::UInteger(u) => Some(*u),
                    CommandArg_Value::Integer(i) if *i > 0 => Some(*i as u32),
                    _ => None,
                });
                let dir_opt = args.get(&'D').and_then(|val| match val {
                    CommandArgValue::UInteger(u) => Some(*u == 1),
                    CommandArgValue::Integer(i) => Some(*i == 1),
                    _ => None,
                });
                let rate_hz_opt = args.get(&'R').and_then(|val| match val {
                    CommandArgValue::UInteger(u) => Some(*u),
                    CommandArgValue::Integer(i) if *i > 0 => Some(*i as u32),
                    _ => None,
                });
                let pulse_width_us_opt = args.get(&'W').and_then(|val| match val {
                    CommandArgValue::UInteger(u) => Some(*u),
                    CommandArgValue::Integer(i) if *i > 0 => Some(*i as u32),
                    _ => None,
                });

                if let Some(steps) = steps_opt {
                    interrupt_free(|cs| {
                        let mut scheduler_opt = SCHEDULER.borrow(cs).borrow_mut();
                        let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
                        let mut stepper_x_opt = STEPPER_X.borrow(cs).borrow_mut();

                        if let (Some(ref mut scheduler), Some(ref mut gpio_manager), Some(ref mut stepper_x)) =
                            (scheduler_opt.as_mut(), gpio_manager_opt.as_mut(), stepper_x_opt.as_mut()) {

                            // Check if stepper is already moving
                            if stepper_x.steps_to_move > 0 {
                                serial_write_line(serial, "Error: Stepper X is already moving\r\n");
                                return;
                            }

                            // 1. Set direction if provided
                            if let Some(dir_is_ccw) = dir_opt {
                                let dir = if dir_is_ccw { StepperDirection::CounterClockwise } else { StepperDirection::Clockwise };
                                if stepper_x.set_direction(dir, gpio_manager).is_err() {
                                    serial_write_line(serial, "Error: Failed to set stepper direction\r\n");
                                    return;
                                }
                            }

                            // 2. Set movement parameters
                            let rate_hz = rate_hz_opt.unwrap_or(100); // Default 100 Hz
                            if rate_hz == 0 { serial_write_line(serial, "Error: Rate R cannot be 0\r\n"); return; }
                            stepper_x.step_period_ticks = 1_000_000 / rate_hz; // Assumes 1MHz timer

                            let pulse_width_us = pulse_width_us_opt.unwrap_or(2); // Default 2us
                            stepper_x.pulse_duration_ticks = pulse_width_us; // Assumes 1MHz timer

                            if stepper_x.step_period_ticks <= stepper_x.pulse_duration_ticks {
                                serial_write_line(serial, "Error: Pulse width must be less than step period\r\n");
                                return;
                            }

                            stepper_x.steps_to_move = steps;
                            stepper_x.is_pulsing_high = false; // Ensure we start with a rising edge

                            // 3. Kick off the first step event with the scheduler
                            let now = scheduler.read_time();
                            stepper_x.next_step_waketime = now.wrapping_add(100); // Add small delay before first step
                            scheduler.schedule_task(stepper_x.timer_id_for_scheduler, stepper_x.next_step_waketime);

                            serial_write_line(serial, "ok\r\n");
                        } else {
                            serial_write_line(serial, "Error: Stepper/Scheduler/GPIO not initialized\r\n");
                        }
                    });
                } else {
                    serial_write_line(serial, "Error: Missing required S (steps) argument\r\n");
                }
            }
            else if command.is_empty() && args_str.is_empty() { /* Ignore */ }
            else { /* Unknown command */ }
        }
        Err(e) => { /* Arg parsing error */ }
    }
}
