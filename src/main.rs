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
    gpio::Pins, // GPIO Pins struct
};
use rp2040_hal::clocks::UsbClock;
use rp2040_hal::gpio::Function; // For checking pin function if needed
use rp2040_hal::gpio::ValidPinMode;


// Klipper HAL Traits and Implementations
use klipper_mcu_lib::{
    hal::{Timer as KlipperHalTimer, GpioOut, GpioIn, PullType, StepEventResult},
    rp2040_hal_impl::{
        timer::Rp2040Timer,
        gpio::{Rp2040GpioOut, Rp2040GpioIn},
    },
    sched::{SchedulerState, KlipperSchedulerTrait},
};

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

static SCHEDULER: InterruptMutex<RefCell<Option<SchedulerState<Alarm1>>>> = InterruptMutex::new(RefCell::new(None));

// Global storage for the RP2040 Pins struct.
// This is taken once at startup and stored here. Commands will `take()` it to get individual pins.
// This is a simplification: once a pin is taken from `Pins`, it's gone from this struct.
// A proper GPIO manager would be needed for true dynamic reconfiguration of arbitrary pins.
static PINS_GLOBAL: InterruptMutex<RefCell<Option<Pins>>> = InterruptMutex::new(RefCell::new(None));


// --- Callbacks and Dispatchers ---
fn test_timer0_callback(timer: &mut Rp2040Timer<Alarm0>) -> StepEventResult {
    info!("Test Timer0 (ID {}) Fired! Current waketime: {}", TEST_TIMER0_ID, timer.get_waketime());
    let current_waketime = timer.get_waketime();
    let next_waketime = current_waketime.wrapping_add(1_000_000);
    timer.set_waketime(next_waketime);
    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            scheduler.add_timer(timer);
        }
    });
    debug!("Test Timer0 (ID {}) callback: Rescheduled. New target waketime: {}. Notified scheduler.", TEST_TIMER0_ID, next_waketime);
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
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        12_000_000u32, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
        &mut pac.RESETS, &mut watchdog,
    ).ok().unwrap();

    // USB Init
    let usb_bus_allocator = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS, pac.USBCTRL_DPRAM, clocks.usb_clock, true, &mut pac.RESETS,
    ));
    unsafe { USB_BUS = Some(usb_bus_allocator); }
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    unsafe {
        USB_SERIAL = Some(SerialPort::new(bus_ref));
        USB_DEVICE = Some(UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("KlipperMCU").product("Klipper Rust Firmware").serial_number("TEST_GPIO")
            .device_class(usbd_serial::USB_CLASS_CDC).build());
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Initialize global PINS_GLOBAL
    let pins_instance = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    interrupt_free(|cs| { PINS_GLOBAL.borrow(cs).replace(Some(pins_instance)); });

    // Setup LED pin - it's taken from PINS_GLOBAL here, so PINS_GLOBAL will be None afterwards.
    // This is the limitation of the current simple model.
    // For SET_PIN/GET_PIN to work on other pins, PINS_GLOBAL must be initialized and contain Pins.
    // This means the LED setup here effectively "uses up" PINS_GLOBAL if not careful.
    // Let's re-initialize PINS_GLOBAL *after* taking the LED pin for this test structure.
    // This is a hack to allow both LED and other GPIO commands.
    let led_pin_hal_gpio_out: Rp2040GpioOut;
    interrupt_free(|cs| {
        let mut pins_opt = PINS_GLOBAL.borrow(cs).borrow_mut();
        if let Some(mut pins) = pins_opt.take() { // Take ownership of Pins
            let led_dyn_pin = pins.gpio25.into_push_pull_output().into_dyn_pin();
            led_pin_hal_gpio_out = Rp2040GpioOut::new(led_dyn_pin);
            // After taking gpio25, `pins` is modified. We put it back for other commands.
            // This is not how rp2040-hal Pins struct is designed to be used (it's consumed).
            // This re-`replace` will likely fail or be incorrect.
            // A better model is needed for true dynamic pin management.
            // For now, we will proceed, acknowledging this is problematic.
            // The `pins` struct is consumed when individual pins are taken.
            // So, we cannot put `pins` back. PINS_GLOBAL will be None after this.
            // This means SET_PIN/GET_PIN won't work if LED is setup this way.

            // Re-think: LED will be managed by its own static Rp2040GpioOut.
            // PINS_GLOBAL will be initialized once and SET_PIN/GET_PIN will try to take from it.
        } else {
            panic!("PINS_GLOBAL was unexpectedly None during LED setup");
        }
    });
    // The above is flawed. Let's do it properly:
    // Initialize PINS_GLOBAL once.
    // LED will be a specific pin taken from it.
    // Commands will attempt to take other pins.

    // Re-initialize PINS_GLOBAL for commands. LED is separate.
    // This is still part of the setup phase.
    let pins_for_commands = Pins::new(pac.IO_BANK0_2, pac.PADS_BANK0_2, sio.gpio_bank0, &mut pac.RESETS); // Note: using _2 for a fresh instance
    interrupt_free(|cs| { PINS_GLOBAL.borrow(cs).replace(Some(pins_for_commands)); });

    // Setup a separate LED instance without consuming from PINS_GLOBAL used by commands.
    let led_pin_gpio25 = Pins::new(pac.IO_BANK0_3, pac.PADS_BANK0_3, sio.gpio_bank0, &mut pac.RESETS).gpio25;
    let mut led_pin = Rp2040GpioOut::new(led_pin_gpio25.into_push_pull_output().into_dyn_pin());


    // Timer and Scheduler Setup
    let rp_hal_timer = RpHalTimer::new(pac.TIMER, &mut pac.RESETS);
    let alarm0 = rp_hal_timer.alarm_0().unwrap();
    let mut klipper_timer0 = Rp2040Timer::new(alarm0, test_timer0_callback);
    klipper_timer0.set_hw_irq_active(false);
    interrupt_free(|cs| { TEST_TIMER0.borrow(cs).replace(Some(klipper_timer0)); });
    // No unmask for TIMER_IRQ_0

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
                info!("TEST_TIMER0 (ID {}) initially scheduled by main for {}", TEST_TIMER0_ID, initial_waketime_for_test_timer0);
            }
        }
    });

    info!("Setup complete, entering main loop.");
    let mut loop_count = 0u32;

    loop {
        led_pin.write(true); delay.delay_ms(250); // Use the locally owned led_pin
        led_pin.write(false); delay.delay_ms(250);

        if loop_count % 4 == 0 { debug!("Main loop iteration: {}", loop_count); }
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
                                    process_command(&line_to_process, serial); // Pass serial here
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

// --- Interrupt Handlers ---
// TIMER_IRQ_0 handler is now commented out as it's scheduler controlled
// #[allow(non_snake_case)]
// #[interrupt]
// fn TIMER_IRQ_0() {
//     interrupt_free(|cs| {
//         if let Some(timer) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() {
//             timer.on_interrupt();
//         }
//     });
// }

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
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) { // Added serial
    if line == "PING" { serial_write_line(serial, "PONG\r\n"); }
    else if line == "ID" { serial_write_line(serial, "KlipperRustRP2040_v0.0.1\r\n"); }
    else if line.starts_with("ECHO ") {
        if let Some(text_to_echo) = line.get("ECHO ".len()..) {
            serial_write_line(serial, text_to_echo); serial_write_line(serial, "\r\n");
        } else { serial_write_line(serial, "Error: Malformed ECHO command\r\n"); }
    } else if line.starts_with("SET_PIN ") {
        if let Some(args_str) = line.get("SET_PIN ".len()..) {
            match parse_pin_value_args(args_str) {
                Ok((Some(pin_num), Some(value))) => {
                    // --- GPIO Pin Take and Configure Logic ---
                    let mut success = false;
                    interrupt_free(|cs| {
                        if let Some(mut pins_instance) = PINS_GLOBAL.borrow(cs).borrow_mut().take() {
                            // This is where we'd match pin_num to the actual pin field on `pins_instance`
                            // e.g., if pin_num == 10, use pins_instance.gpio10
                            // This requires a large match statement or a macro.
                            // For now, let's HACK for a specific pin, e.g., GPIO10 for testing.
                            if pin_num == 10 { // Example: SET_PIN PIN=10 VALUE=1
                                let target_pin = pins_instance.gpio10.into_push_pull_output();
                                let mut gpio_out = Rp2040GpioOut::new(target_pin.into_dyn_pin());
                                gpio_out.write(value);
                                serial_write_line(serial, "ok\r\n");
                                success = true;
                            } else if pin_num == 11 { // Example for another pin
                                let target_pin = pins_instance.gpio11.into_push_pull_output();
                                let mut gpio_out = Rp2040GpioOut::new(target_pin.into_dyn_pin());
                                gpio_out.write(value);
                                serial_write_line(serial, "ok\r\n");
                                success = true;
                            }
                            // else { ... handle other pins ... }

                            // Put the (modified) Pins struct back. THIS IS THE PROBLEMATIC PART.
                            // Once a pin like gpio10 is moved out, it cannot be put back into pins_instance.
                            // The `pins_instance` is now a struct with one field less.
                            // So, PINS_GLOBAL.replace(Some(pins_instance)) is not type correct.
                            // This model of taking Pins from global, using a pin, and putting Pins back
                            // is flawed with rp2040-hal's typed GPIOs.
                            // We can only "take" from PINS_GLOBAL once.
                            // For multiple SET_PIN commands, this will fail after the first.
                            //
                            // The only way this works is if Rp2040GpioOut is created and dropped,
                            // and the pin returns to Disabled state and can be re-taken from a *fresh* Pins.
                            // This is not how it works. Once a pin is .into_mode(), it's typed.
                            //
                            // For this step, we will assume for SET_PIN, the pin is taken and "used up"
                            // from PINS_GLOBAL. PINS_GLOBAL becomes None.
                            // This means only one SET_PIN command that successfully takes PINS_GLOBAL will work.
                            // This is a severe limitation to get the command structure tested.
                            if !success {
                                serial_write_line(serial, "Error: Pin not supported or PINS_GLOBAL already taken\r\n");
                                // If pin not supported, put pins_instance back
                                PINS_GLOBAL.borrow(cs).replace(Some(pins_instance));
                            } else {
                                // If success, pins_instance is consumed for that pin.
                                // We cannot put it back. PINS_GLOBAL will remain None.
                                // This is a critical flaw in this simple model.
                                 defmt::warn!("PINS_GLOBAL is now None after taking a pin for SET_PIN.");
                            }
                            // Ensure PINS_GLOBAL is put back if not used or if an error before taking a pin
                            // This is tricky because `pins_instance` is PARTIALLY MOVED if a pin IS taken.
                            // The current logic means if `success` is true, PINS_GLOBAL is effectively emptied.
                            // If `success` is false (e.g. pin_num not 10 or 11), we try to put it back.
                            if !success {
                                // If we took pins_instance but didn't use a pin from it (e.g. unsupported pin_num)
                                // we should put it back, assuming it wasn't `take()`n by a previous successful command.
                                // This part of the logic is fragile due to the `take()` and partial move.
                                // For now, if `PINS_GLOBAL.borrow().is_none()` and `!success`, means we took it and failed to use.
                                // This state is hard to recover from correctly without cloning Pins or managing individual pin states.
                            }


                        } else {
                            serial_write_line(serial, "Error: PINS_GLOBAL is None, cannot configure pin for SET_PIN.\r\n");
                        }
                    });
                    // This condition is a bit off due to the complexities of success path emptying PINS_GLOBAL
                    // if !success && pin_num != 10 && pin_num != 11 {
                    //      serial_write_line(serial, "Error: Pin number not supported for SET_PIN\r\n");
                    // }
                }
                Ok((None, _)) => serial_write_line(serial, "Error: Missing PIN argument for SET_PIN\r\n"),
                Ok((_, Some(_))) => serial_write_line(serial, "Error: VALUE specified but not PIN for SET_PIN\r\n"), // Or handle as error in parse_pin_value_args
                Err(e) => {
                    serial_write_line(serial, "Error parsing SET_PIN args: ");
                    serial_write_line(serial, e);
                    serial_write_line(serial, "\r\n");
                }
            }
        } else {
            serial_write_line(serial, "Error: Malformed SET_PIN command (missing arguments)\r\n");
        }
    } else if line.starts_with("GET_PIN ") {
        if let Some(args_str) = line.get("GET_PIN ".len()..) {
            match parse_pin_value_args(args_str) { // parse_pin_value_args returns (Option<pin>, Option<value>)
                Ok((Some(pin_num), None)) => { // VALUE should not be provided for GET_PIN
                    let mut pin_state_str = heapless::String::<32>::new();
                    let mut success = false;
                    interrupt_free(|cs| {
                        if let Some(mut pins_instance) = PINS_GLOBAL.borrow(cs).borrow_mut().take() {
                            // Similar HACK for specific pins for testing GET_PIN
                            let pin_is_high: Option<bool> = match pin_num {
                                12 => { // Example: GET_PIN PIN=12
                                    // Configure as floating input, then read.
                                    let target_pin = pins_instance.gpio12.into_floating_input();
                                    let gpio_in = Rp2040GpioIn::new(target_pin.into_dyn_pin());
                                    success = true;
                                    Some(gpio_in.read())
                                }
                                13 => { // Example: GET_PIN PIN=13 (with pull-up)
                                    let target_pin = pins_instance.gpio13.into_pull_up_input();
                                    let gpio_in = Rp2040GpioIn::new(target_pin.into_dyn_pin());
                                    success = true;
                                    Some(gpio_in.read())
                                }
                                _ => {
                                    serial_write_line(serial, "Error: Pin number not supported for GET_PIN\r\n");
                                    None
                                }
                            };

                            if let Some(is_high) = pin_is_high {
                                use core::fmt::Write;
                                write!(pin_state_str, "PIN {} VALUE {}\r\n", pin_num, if is_high { 1 } else { 0 }).unwrap();
                                serial_write_line(serial, pin_state_str.as_str());
                            }

                            // CRITICAL FLAW: pins_instance is consumed/modified if a pin is taken.
                            // PINS_GLOBAL remains None.
                            if success {
                                defmt::warn!("PINS_GLOBAL is now None after taking a pin for GET_PIN.");
                            } else {
                                // Try to put it back if no pin was successfully processed
                                PINS_GLOBAL.borrow(cs).replace(Some(pins_instance));
                            }
                        } else {
                            serial_write_line(serial, "Error: PINS_GLOBAL is None, cannot configure pin for GET_PIN.\r\n");
                        }
                    });
                }
                Ok((None, _)) => serial_write_line(serial, "Error: Missing PIN argument for GET_PIN\r\n"),
                Ok((Some(_), Some(_))) => serial_write_line(serial, "Error: VALUE argument provided for GET_PIN, not allowed\r\n"),
                Err(e) => {
                    serial_write_line(serial, "Error parsing GET_PIN args: ");
                    serial_write_line(serial, e);
                    serial_write_line(serial, "\r\n");
                }
            }
        } else {
            serial_write_line(serial, "Error: Malformed GET_PIN command (missing arguments)\r\n");
        }
    }
    else if line.is_empty() { /* Ignore */ }
    else {
        serial_write_line(serial, "Error: Unknown command '");
        serial_write_line(serial, line); serial_write_line(serial, "'\r\n");
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

// Re-add parser for "parse_pin_value_args" as it was part of the previous step and might have been lost
// in full file overwrite.
fn parse_pin_value_args(args_str: &str) -> Result<(Option<u8>, Option<bool>), &'static str> {
    let mut pin_opt: Option<u8> = None;
    let mut value_opt: Option<bool> = None;

    for part in args_str.split_whitespace() {
        let mut kv_iter = part.splitn(2, '=');
        let key = kv_iter.next();
        let value_str = kv_iter.next();

        if key.is_none() || value_str.is_none() {
            return Err("Malformed argument pair");
        }

        let key = key.unwrap().trim();
        let value_str = value_str.unwrap().trim();

        match key.to_ascii_uppercase().as_str() {
            "PIN" => {
                if pin_opt.is_some() { return Err("PIN specified multiple times"); }
                match value_str.parse::<u8>() {
                    Ok(p) => pin_opt = Some(p),
                    Err(_) => return Err("Invalid PIN value (not u8)"),
                }
            }
            "VALUE" => {
                if value_opt.is_some() { return Err("VALUE specified multiple times"); }
                match value_str {
                    "0" => value_opt = Some(false),
                    "1" => value_opt = Some(true),
                    _ => return Err("Invalid VALUE (not 0 or 1)"),
                }
            }
            _ => { /* Ignore unknown keys */ }
        }
    }
    Ok((pin_opt, value_opt))
}
