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
    clocks::{init_clocks_and_plls, Clock},
    pac::{self, interrupt, NVIC},
    sio::Sio,
    watchdog::Watchdog,
    timer::Timer as RpHalTimer,
    timer::{Alarm0, Alarm1},
    usb::UsbBus,
    gpio::Pins as RpHalPins,
    adc::Adc as RpHalAdc, // ADC Peripheral
};
use rp2040_hal::clocks::UsbClock;

// Klipper HAL Traits and Implementations
use klipper_mcu_lib::{
    hal::{Timer as KlipperHalTimer, GpioOut, GpioIn, PullType, AdcChannel, AdcError, StepEventResult}, // Added AdcChannel, AdcError
    rp2040_hal_impl::{
        // timer::Rp2040Timer, // Rp2040Timer is used directly
        // adc::Rp2040AdcChannel, // Will be used by QUERY_ADC command
    },
    sched::{SchedulerState, KlipperSchedulerTrait},
    gpio_manager::{GpioManager, PinModeState},
};
use klipper_mcu_lib::rp2040_hal_impl::timer::Rp2040Timer;
// We will also need Rp2040AdcChannel when implementing QUERY_ADC
// use klipper_mcu_lib::rp2040_hal_impl::adc::Rp2040AdcChannel;


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
const LED_PIN_ID: u8 = 25;

static SCHEDULER: InterruptMutex<RefCell<Option<SchedulerState<Alarm1>>>> = InterruptMutex::new(RefCell::new(None));
static GPIO_MANAGER: InterruptMutex<RefCell<Option<GpioManager>>> = InterruptMutex::new(RefCell::new(None));
static ADC_PERIPHERAL: InterruptMutex<RefCell<Option<RpHalAdc>>> = InterruptMutex::new(RefCell::new(None));


// --- Callbacks and Dispatchers ---
fn test_timer0_callback(timer: &mut Rp2040Timer<Alarm0>) -> StepEventResult {
    info!("Test Timer0 (ID {}) Fired! Waketime: {}", TEST_TIMER0_ID, timer.get_waketime());
    let current_waketime = timer.get_waketime();
    let next_waketime = current_waketime.wrapping_add(1_000_000);

    timer.set_hw_irq_active(false);
    timer.set_waketime(next_waketime);

    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            scheduler.add_timer(timer);
        }
    });
    debug!("Test Timer0 (ID {}) Rescheduled. New target: {}. Notified scheduler.", TEST_TIMER0_ID, next_waketime);
    ()
}

fn master_scheduler_timer_callback(_master_timer_ref: &mut Rp2040Timer<Alarm1>) -> StepEventResult {
    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            scheduler.service_master_timer();
        }
    });
    ()
}

pub fn dispatch_klipper_task_from_scheduler(task_id: u32) {
    match task_id {
        TEST_TIMER0_ID => {
            interrupt_free(|cs| {
                if let Some(timer_instance) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() {
                    if let Some(callback_fn) = timer_instance.get_callback() {
                        (callback_fn)(timer_instance);
                    } else { warn!("dispatch: No callback for timer ID {}", task_id); }
                } else { warn!("dispatch: Timer instance not found for ID {}", task_id); }
            });
        }
        _ => warn!("Scheduler: Dispatch for unknown task ID {}", task_id),
    }
}

// --- Entry Point ---
#[entry]
fn main() -> ! {
    let mut pac_peripherals = pac::Peripherals::take().unwrap();
    let core_peripherals = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac_peripherals.WATCHDOG);
    let sio = Sio::new(pac_peripherals.SIO); // SIO is taken here

    let clocks = init_clocks_and_plls(
        12_000_000u32, pac_peripherals.XOSC, pac_peripherals.CLOCKS, pac_peripherals.PLL_SYS, pac_peripherals.PLL_USB,
        &mut pac_peripherals.RESETS, &mut watchdog,
    ).ok().unwrap();

    // USB Init
    let usb_bus_allocator = UsbBusAllocator::new(UsbBus::new(
        pac_peripherals.USBCTRL_REGS, pac_peripherals.USBCTRL_DPRAM, clocks.usb_clock, true, &mut pac_peripherals.RESETS,
    ));
    unsafe { USB_BUS = Some(usb_bus_allocator); }
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    unsafe {
        USB_SERIAL = Some(SerialPort::new(bus_ref));
        USB_DEVICE = Some(UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("KlipperMCU").product("Klipper Rust Firmware").serial_number("ADC_TEST")
            .device_class(usbd_serial::USB_CLASS_CDC).build());
    }

    // Initialize ADC Peripheral
    let adc_peripheral_hal = RpHalAdc::new(pac_peripherals.ADC, &mut pac_peripherals.RESETS);
    interrupt_free(|cs| {
        ADC_PERIPHERAL.borrow(cs).replace(Some(adc_peripheral_hal));
    });
    info!("ADC Peripheral Initialized.");

    // Initialize GpioManager
    // Pass the already taken `sio.gpio_bank0`
    let rp_hal_pins = RpHalPins::new(
        pac_peripherals.IO_BANK0,
        pac_peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac_peripherals.RESETS
    );
    let gpio_mgr = GpioManager::new(rp_hal_pins);
    interrupt_free(|cs| { GPIO_MANAGER.borrow(cs).replace(Some(gpio_mgr)); });
    info!("GPIO Manager Initialized.");

    // Configure LED Pin (GPIO25) using GpioManager
    interrupt_free(|cs| {
        if let Some(manager) = GPIO_MANAGER.borrow(cs).borrow_mut().as_mut() {
            if manager.configure_pin_as_output(LED_PIN_ID).is_ok() {
                info!("LED Pin {} configured as output.", LED_PIN_ID);
                let _ = manager.write_pin_output(LED_PIN_ID, false);
            } else {
                error!("Failed to configure LED Pin {} as output.", LED_PIN_ID);
            }
        }
    });

    // Timer and Scheduler Setup
    let rp_hal_timer = RpHalTimer::new(pac_peripherals.TIMER, &mut pac_peripherals.RESETS);
    let alarm0 = rp_hal_timer.alarm_0().unwrap();
    let mut klipper_timer0 = Rp2040Timer::new(alarm0, test_timer0_callback);
    klipper_timer0.set_hw_irq_active(false);
    interrupt_free(|cs| { TEST_TIMER0.borrow(cs).replace(Some(klipper_timer0)); });

    let alarm1_for_scheduler = rp_hal_timer.alarm_1().unwrap();
    let scheduler_raw_timer_ref = rp_hal_timer;
    let klipper_scheduler = SchedulerState::new(
        alarm1_for_scheduler, scheduler_raw_timer_ref,
        master_scheduler_timer_callback, dispatch_klipper_task_from_scheduler,
    );
    interrupt_free(|cs| { SCHEDULER.borrow(cs).replace(Some(klipper_scheduler)); });
    unsafe { NVIC::unmask(pac::Interrupt::TIMER_IRQ_1); }

    let now_ticks = rp_hal_timer.get_counter_low();
    let initial_waketime_for_test_timer0 = now_ticks.wrapping_add(2_000_000);
    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
             if let Some(timer0_ref) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() {
                timer0_ref.set_waketime(initial_waketime_for_test_timer0);
                scheduler.add_timer(timer0_ref);
                info!("TEST_TIMER0 (ID {}) initially scheduled for {}", TEST_TIMER0_ID, initial_waketime_for_test_timer0);
            }
        }
    });

    info!("Setup complete, entering main loop.");
    let mut loop_count = 0u32;
    let mut led_state = false;
    let mut delay = cortex_m::delay::Delay::new(core_peripherals.SYST, clocks.system_clock.freq().to_Hz());


    loop {
        interrupt_free(|cs| {
            if let Some(manager) = GPIO_MANAGER.borrow(cs).borrow_mut().as_mut() {
                // Toggle LED
                led_state = !led_state;
                let _ = manager.write_pin_output(LED_PIN_ID, led_state);
            }
        });
        delay.delay_ms(500);

        if loop_count % 2 == 0 {
            debug!("Main loop iteration: {}", loop_count);
        }
        loop_count = loop_count.wrapping_add(1);

        unsafe {
            if let Some(usb_dev) = USB_DEVICE.as_mut() {
                if let Some(serial) = USB_SERIAL.as_mut() {
                    usb_dev.poll(&mut [serial]);
                    let mut read_buf = [0u8; 64];
                    match serial.read(&mut read_buf) {
                        Ok(bytes_read) if bytes_read > 0 => {
                            let received_bytes = &read_buf[0..bytes_read];
                            for &byte in received_bytes {
                                if LINE_BUFFER.push(byte as char).is_err() {
                                    let _ = serial.write(b"Error: Line buffer full\r\n");
                                    LINE_BUFFER.clear(); break;
                                }
                                if byte == b'\n' {
                                    let mut line_to_process = LINE_BUFFER.clone();
                                    if line_to_process.ends_with('\n') { line_to_process.pop(); }
                                    if line_to_process.ends_with('\r') { line_to_process.pop(); }
                                    process_command(&line_to_process, serial);
                                    LINE_BUFFER.clear();
                                }
                            }
                        }
                        Ok(_) => {} Err(UsbError::WouldBlock) => {}
                        Err(e) => { warn!("USB read error: {:?}", Debug2Format(&e)); }
                    }
                }
            }
        }
    }
}

// --- Interrupt Handlers --- (TIMER_IRQ_0 is commented out)
#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_1() {
    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            scheduler.master_timer.on_interrupt();
        }
    });
}

// --- Command Processing & Helpers ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    let mut parts = line.trim().splitn(2, |c: char| c.is_whitespace());
    let command = parts.next().unwrap_or("").to_ascii_uppercase();
    let args_str = parts.next().unwrap_or("");

    let parsed_args_result = if !args_str.is_empty() {
        klipper_mcu_lib::command_parser::parse_gcode_arguments(args_str)
    } else {
        Ok(klipper_mcu_lib::command_parser::ParsedArgs::new())
    };

    match parsed_args_result {
        Ok(args) => {
            if command == "PING" { serial_write_line(serial, "PONG\r\n"); }
            else if command == "ID" { serial_write_line(serial, "KlipperRustRP2040_ADC_v1\r\n"); }
            else if command == "ECHO" {
                serial_write_line(serial, args_str); serial_write_line(serial, "\r\n");
            } else if command == "SET_PIN" {
                let pin_num_opt = args.get(&'P').and_then(|val| match val {
                    klipper_mcu_lib::command_parser::CommandArgValue::UInteger(u) => Some(*u as u8),
                    klipper_mcu_lib::command_parser::CommandArgValue::Integer(i) if *i >= 0 && *i <= 29 => Some(*i as u8),
                    _ => None,
                });
                let value_opt = args.get(&'S').and_then(|val| match val {
                    klipper_mcu_lib::command_parser::CommandArgValue::UInteger(u) => Some(*u == 1),
                    klipper_mcu_lib::command_parser::CommandArgValue::Integer(i) => Some(*i == 1),
                    _ => None,
                });

                if let (Some(pin_num), Some(value)) = (pin_num_opt, value_opt) {
                    interrupt_free(|cs| {
                        if let Some(manager) = GPIO_MANAGER.borrow(cs).borrow_mut().as_mut() {
                            if manager.configure_pin_as_output(pin_num).is_ok() {
                                if manager.write_pin_output(pin_num, value).is_ok() {
                                    serial_write_line(serial, "ok\r\n");
                                } else { serial_write_line(serial, "Error: Failed to write pin value\r\n"); }
                            } else { serial_write_line(serial, "Error: Failed to configure pin as output\r\n"); }
                        } else { serial_write_line(serial, "Error: GpioManager not initialized\r\n");}
                    });
                } else { serial_write_line(serial, "Error: Missing/invalid P (pin) or S (value) for SET_PIN\r\n"); }
            } else if command == "GET_PIN" {
                let pin_num_opt = args.get(&'P').and_then(|val| match val {
                    klipper_mcu_lib::command_parser::CommandArgValue::UInteger(u) => Some(*u as u8),
                    klipper_mcu_lib::command_parser::CommandArgValue::Integer(i) if *i >= 0 && *i <= 29 => Some(*i as u8),
                    _ => None,
                });
                if let Some(pin_num) = pin_num_opt {
                    interrupt_free(|cs| {
                        if let Some(manager) = GPIO_MANAGER.borrow(cs).borrow_mut().as_mut() {
                            if manager.configure_pin_as_input(pin_num, PullType::Floating).is_ok() {
                                match manager.read_pin_input(pin_num) {
                                    Ok(is_high) => {
                                        let mut response = heapless::String::<32>::new();
                                        use core::fmt::Write;
                                        write!(response, "PIN {} VALUE {}\r\n", pin_num, if is_high { 1 } else { 0 }).unwrap();
                                        serial_write_line(serial, response.as_str());
                                    }
                                    Err(e) => { serial_write_line(serial, "Error reading pin: "); serial_write_line(serial, e); serial_write_line(serial, "\r\n");}
                                }
                            } else { serial_write_line(serial, "Error: Failed to configure pin as input\r\n"); }
                        } else { serial_write_line(serial, "Error: GpioManager not initialized\r\n");}
                    });
                } else { serial_write_line(serial, "Error: Missing or invalid P (pin) for GET_PIN\r\n"); }
            }
            // QUERY_ADC command will be added in the next step
            else if command == "QUERY_ADC" {
                let pin_num_opt = args.get(&'P').and_then(|val| match val {
                    klipper_mcu_lib::command_parser::CommandArgValue::UInteger(u) => Some(*u as u8),
                    klipper_mcu_lib::command_parser::CommandArgValue::Integer(i) if *i >= 0 && *i <= 29 => Some(*i as u8),
                    _ => None,
                });

                if let Some(pin_num) = pin_num_opt {
                    if !(26..=29).contains(&pin_num) { // Validate if pin is ADC capable (GPIO 26-29)
                        serial_write_line(serial, "Error: Pin is not ADC capable (must be 26-29)\r\n");
                    } else {
                        let adc_read_result: Result<u16, &'static str> = interrupt_free(|cs| {
                            let mut manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
                            let mut adc_opt = ADC_PERIPHERAL.borrow(cs).borrow_mut();

                            if let (Some(manager), Some(adc_peripheral)) = (manager_opt.as_mut(), adc_opt.as_mut()) {
                                match manager.take_pin_for_adc(pin_num) {
                                    Ok(adc_capable_pin) => {
                                        // Convert AdcCapablePin enum to the specific Pin type needed by Rp2040AdcChannel
                                        // This is verbose. A helper method on AdcCapablePin could do this.
                                        let read_val = match adc_capable_pin {
                                            klipper_mcu_lib::gpio_manager::AdcCapablePin::Gpio26(pin_instance) => {
                                                match klipper_mcu_lib::rp2040_hal_impl::adc::Rp2040AdcChannel::new(adc_peripheral, pin_instance) {
                                                    Ok(mut adc_channel) => adc_channel.read_raw().map_err(|e| {
                                                        defmt::error!("ADC Read Error (Pin {}): {:?}", pin_num, defmt::Debug2Format(&e));
                                                        "ADC Read Error"
                                                    }),
                                                    Err(e) => {
                                                        defmt::error!("ADC Channel Config Error (Pin {}): {:?}", pin_num, defmt::Debug2Format(&e));
                                                        Err("ADC Channel Config Error")
                                                    }
                                                }
                                            }
                                            klipper_mcu_lib::gpio_manager::AdcCapablePin::Gpio27(pin_instance) => {
                                                match klipper_mcu_lib::rp2040_hal_impl::adc::Rp2040AdcChannel::new(adc_peripheral, pin_instance) {
                                                    Ok(mut adc_channel) => adc_channel.read_raw().map_err(|e| {
                                                        defmt::error!("ADC Read Error (Pin {}): {:?}", pin_num, defmt::Debug2Format(&e));
                                                        "ADC Read Error"
                                                    }),
                                                    Err(e) => {
                                                        defmt::error!("ADC Channel Config Error (Pin {}): {:?}", pin_num, defmt::Debug2Format(&e));
                                                        Err("ADC Channel Config Error")
                                                    }
                                                }
                                            }
                                            klipper_mcu_lib::gpio_manager::AdcCapablePin::Gpio28(pin_instance) => {
                                                match klipper_mcu_lib::rp2040_hal_impl::adc::Rp2040AdcChannel::new(adc_peripheral, pin_instance) {
                                                    Ok(mut adc_channel) => adc_channel.read_raw().map_err(|e| {
                                                        defmt::error!("ADC Read Error (Pin {}): {:?}", pin_num, defmt::Debug2Format(&e));
                                                        "ADC Read Error"
                                                    }),
                                                    Err(e) => {
                                                        defmt::error!("ADC Channel Config Error (Pin {}): {:?}", pin_num, defmt::Debug2Format(&e));
                                                        Err("ADC Channel Config Error")
                                                    }
                                                }
                                            }
                                            klipper_mcu_lib::gpio_manager::AdcCapablePin::Gpio29(pin_instance) => {
                                                match klipper_mcu_lib::rp2040_hal_impl::adc::Rp2040AdcChannel::new(adc_peripheral, pin_instance) {
                                                    Ok(mut adc_channel) => adc_channel.read_raw().map_err(|e| {
                                                        defmt::error!("ADC Read Error (Pin {}): {:?}", pin_num, defmt::Debug2Format(&e));
                                                        "ADC Read Error"
                                                    }),
                                                    Err(e) => {
                                                        defmt::error!("ADC Channel Config Error (Pin {}): {:?}", pin_num, defmt::Debug2Format(&e));
                                                        Err("ADC Channel Config Error")
                                                    }
                                                }
                                            }
                                        };
                                        // Release the pin back to GpioManager
                                        // We need to pass back the original AdcCapablePin to release_adc_pin
                                        // The current structure of take_pin_for_adc and release_adc_pin makes this awkward
                                        // because the pin is consumed by AdcChannel::new.
                                        // For now, let's assume release just updates state based on pin_id.
                                        // A better release would take back the `AdcCapablePin` to return its tokens.
                                        // The current `release_adc_pin` takes `_adc_capable_pin` but doesn't use it.
                                        // This is okay for the simplified unsafe `take_pin_for_adc`.
                                        let _ = manager.release_adc_pin(pin_num, klipper_mcu_lib::gpio_manager::AdcCapablePin::Gpio26(unsafe { rp2040_hal::gpio::Pin::new(rp2040_hal::gpio::bank0::Gpio26::ID) }.into_floating_input())); // Dummy pin for release
                                        read_val
                                    }
                                    Err(e) => Err(e), // Error from take_pin_for_adc
                                }
                            } else {
                                Err("Manager or ADC not initialized")
                            }
                        });

                        match adc_read_result {
                            Ok(raw_value) => {
                                let mut response = heapless::String::<64>::new();
                                use core::fmt::Write;
                                write!(response, "ADC PIN {} RAW_VALUE {}\r\n", pin_num, raw_value).unwrap();
                                serial_write_line(serial, response.as_str());
                            }
                            Err(e_str) => {
                                serial_write_line(serial, "Error: ");
                                serial_write_line(serial, e_str);
                                serial_write_line(serial, "\r\n");
                            }
                        }
                    }
                } else {
                    serial_write_line(serial, "Error: Missing or invalid P (pin) for QUERY_ADC\r\n");
                }
            }
            else if command.is_empty() && args_str.is_empty() { /* Ignore */ }
            else {
                serial_write_line(serial, "Error: Unknown command '");
                serial_write_line(serial, &command); serial_write_line(serial, "'\r\n");
            }
        }
        Err(e) => {
            let mut err_msg = heapless::String::<64>::new();
            use core::fmt::Write;
            write!(err_msg, "Error parsing arguments: {:?}\r\n", Debug2Format(&e)).unwrap();
            serial_write_line(serial, err_msg.as_str());
        }
    }
}

fn serial_write_line(serial: &mut SerialPort<UsbBus>, data: &str) {
    let bytes = data.as_bytes(); let mut written = 0;
    while written < bytes.len() {
        match serial.write(&bytes[written..]) {
            Ok(len) if len > 0 => { written += len; } Ok(_) => {}
            Err(UsbError::WouldBlock) => {} Err(_) => { break; }
        }
    }
}

fn u32_to_str<'a>(mut n: u32, buf: &'a mut [u8]) -> &'a [u8] {
    if n == 0 { buf[0] = b'0'; return &buf[0..1]; }
    let mut i = 0; let mut temp_buf = [0u8; 10];
    while n > 0 { temp_buf[i] = (n % 10) as u8 + b'0'; n /= 10; i += 1; }
    for j in 0..i { buf[j] = temp_buf[i - 1 - j]; }
    &buf[0..i]
}

fn parse_pin_value_args(args_str: &str) -> Result<(Option<u8>, Option<bool>), &'static str> {
    let mut pin_opt: Option<u8> = None;
    let mut value_opt: Option<bool> = None;
    for part in args_str.split_whitespace() {
        let mut kv_iter = part.splitn(2, '=');
        let key = kv_iter.next();
        let value_str = kv_iter.next();
        if key.is_none() || value_str.is_none() { return Err("Malformed K=V pair"); }
        let key = key.unwrap().trim();
        let value_str = value_str.unwrap().trim();
        match key.to_ascii_uppercase().as_str() {
            "PIN" => {
                if pin_opt.is_some() { return Err("PIN multiple times"); }
                match value_str.parse::<u8>() {
                    Ok(p) => pin_opt = Some(p),
                    Err(_) => return Err("Invalid PIN u8"),
                }
            }
            "VALUE" => {
                if value_opt.is_some() { return Err("VALUE multiple times"); }
                match value_str {
                    "0" => value_opt = Some(false), "1" => value_opt = Some(true),
                    _ => return Err("Invalid VALUE 0|1"),
                }
            }
            _ => { /* Ignore unknown keys */ }
        }
    }
    Ok((pin_opt, value_opt))
}
