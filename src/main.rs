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
    timer::Timer as RpHalTimer, // RP2040 HAL Timer
    timer::{Alarm0, Alarm1},    // Specific Alarms
    usb::UsbBus,
};
use rp2040_hal::clocks::UsbClock;

// Klipper HAL Traits and Implementations
use klipper_mcu_lib::{
    hal::{Timer as KlipperHalTimer, GpioOut, StepEventResult},
    rp2040_hal_impl::{
        timer::Rp2040Timer,
        gpio::Rp2040GpioOut,
    },
    sched::{SchedulerState, KlipperSchedulerTrait}, // Klipper Scheduler
};

// USB
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

// --- Panic Handler ---
#[cfg(not(test))]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop { cortex_m::asm::nop(); }
}

// --- Globals ---
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;

const MAX_LINE_LENGTH: usize = 256;
static mut LINE_BUFFER: heapless::String<MAX_LINE_LENGTH> = heapless::String::new();

// Test Timer (Client Timer using Alarm0)
static TEST_TIMER0: InterruptMutex<RefCell<Option<Rp2040Timer<Alarm0>>>> = InterruptMutex::new(RefCell::new(None));
const TEST_TIMER0_ID: u32 = 0;

// Scheduler (uses Alarm1 for its master timer)
static SCHEDULER: InterruptMutex<RefCell<Option<SchedulerState<Alarm1>>>> = InterruptMutex::new(RefCell::new(None));

// --- Callbacks and Dispatchers ---

// Callback for TEST_TIMER0
fn test_timer0_callback(timer: &mut Rp2040Timer<Alarm0>) -> StepEventResult {
    info!("Test Timer0 (ID {}) Fired! Current waketime: {}", TEST_TIMER0_ID, timer.get_waketime());
    let current_waketime = timer.get_waketime();
    let next_waketime = current_waketime.wrapping_add(1_000_000); // Approx 1 sec

    timer.set_waketime(next_waketime); // Arm its own hardware alarm

    // Inform the scheduler of the new waketime for this timer
    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            // The scheduler's add_timer will internally call schedule_task
            scheduler.add_timer(timer);
        }
    });
    debug!("Test Timer0 (ID {}) rescheduled via callback to: {}", TEST_TIMER0_ID, next_waketime);
    ()
}

// Callback for the Scheduler's master timer (Alarm1)
fn master_scheduler_timer_callback(_master_timer_ref: &mut Rp2040Timer<Alarm1>) -> StepEventResult {
    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            scheduler.service_master_timer();
        }
    });
    ()
}

// Dispatch function called by the scheduler to run client timer tasks' logic
// This is passed to SchedulerState::new
pub fn dispatch_klipper_task_from_scheduler(task_id: u32) {
    match task_id {
        TEST_TIMER0_ID => {
            interrupt_free(|cs| {
                if let Some(timer) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() {
                    // This calls the timer's own callback (test_timer0_callback)
                    // because Rp2040Timer::on_interrupt checks alarm.finished()
                    // and then calls its stored callback.
                    // This assumes the scheduler is the one determining it's "finished" conceptually.
                    timer.on_interrupt();
                }
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
            .manufacturer("KlipperMCU").product("Klipper Rust Firmware").serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC).build());
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS,
    );
    let mut led_pin = Rp2040GpioOut::new(pins.gpio25.into_push_pull_output().into_dyn_pin());

    // --- Timer and Scheduler Setup ---
    let rp_hal_timer = RpHalTimer::new(pac.TIMER, &mut pac.RESETS);

    // Client Timer (TEST_TIMER0 using Alarm0)
    let alarm0 = rp_hal_timer.alarm_0().unwrap();
    let klipper_timer0 = Rp2040Timer::new(alarm0, test_timer0_callback);
    interrupt_free(|cs| { TEST_TIMER0.borrow(cs).replace(Some(klipper_timer0)); });
    // Enable TEST_TIMER0's own hardware interrupt (for when its alarm physically fires)
    unsafe { NVIC::unmask(pac::Interrupt::TIMER_IRQ_0); }


    // Scheduler (using Alarm1)
    let alarm1_for_scheduler = rp_hal_timer.alarm_1().unwrap();
    // Clone RpHalTimer for the scheduler to use for read_time().
    // RpHalTimer is Copy, so this is fine.
    let scheduler_raw_timer_ref = rp_hal_timer;

    let klipper_scheduler = SchedulerState::new(
        alarm1_for_scheduler,
        scheduler_raw_timer_ref,
        master_scheduler_timer_callback,
        dispatch_klipper_task_from_scheduler, // Pass the dispatcher function
    );
    interrupt_free(|cs| { SCHEDULER.borrow(cs).replace(Some(klipper_scheduler)); });
    // Enable Scheduler's master timer interrupt (Alarm1)
    unsafe { NVIC::unmask(pac::Interrupt::TIMER_IRQ_1); }

    // Initial scheduling of TEST_TIMER0 via the Scheduler
    let now_ticks = rp_hal_timer.get_counter_low();
    let initial_waketime_for_test_timer0 = now_ticks.wrapping_add(2_000_000); // Approx 2s

    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            // Use the scheduler's method to schedule the task.
            // This uses the HACK in add_timer for now.
            // A better way would be:
            // scheduler.schedule_task(TEST_TIMER0_ID, initial_waketime_for_test_timer0);
            // For now, we need to get a ref to TEST_TIMER0 and pass it to add_timer
            if let Some(timer0_ref) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() {
                timer0_ref.set_waketime(initial_waketime_for_test_timer0); // Set its own time first
                scheduler.add_timer(timer0_ref); // Then tell scheduler
                info!("TEST_TIMER0 (ID {}) initially scheduled by main for {}", TEST_TIMER0_ID, initial_waketime_for_test_timer0);
            }
        }
    });

    info!("Setup complete, entering main loop.");
    let mut loop_count = 0u32;

    loop {
        led_pin.write(true);
        delay.delay_ms(250);
        led_pin.write(false);
        delay.delay_ms(250);

        if loop_count % 4 == 0 {
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
                        Ok(_) => {}
                        Err(UsbError::WouldBlock) => {}
                        Err(e) => { warn!("USB read error: {:?}", Debug2Format(&e)); }
                    }
                }
            }
        }
    }
}

// --- Interrupt Handlers ---
#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_0() { // For TEST_TIMER0's own hardware alarm
    interrupt_free(|cs| {
        if let Some(timer) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() {
            timer.on_interrupt(); // Calls test_timer0_callback
        }
    });
}

#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_1() { // For Scheduler's master_timer (Alarm1)
    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            // The scheduler's master_timer.on_interrupt() will call master_scheduler_timer_callback,
            // which in turn calls scheduler.service_master_timer().
            scheduler.master_timer.on_interrupt();
        }
    });
}

// --- Command Processing & Helpers ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    // ... (process_command implementation as before) ...
    if line == "PING" {
        serial_write_line(serial, "PONG\r\n");
    } else if line == "ID" {
        serial_write_line(serial, "KlipperRustRP2040_v0.0.1\r\n");
    } else if line.starts_with("ECHO ") {
        if let Some(text_to_echo) = line.get("ECHO ".len()..) {
            serial_write_line(serial, text_to_echo);
            serial_write_line(serial, "\r\n");
        } else {
            serial_write_line(serial, "Error: Malformed ECHO command\r\n");
        }
    } else if line.is_empty() {
        // Ignore empty lines
    } else {
        serial_write_line(serial, "Error: Unknown command '");
        serial_write_line(serial, line);
        serial_write_line(serial, "'\r\n");
    }
}

fn serial_write_line(serial: &mut SerialPort<UsbBus>, data: &str) {
    // ... (serial_write_line implementation as before) ...
    let bytes = data.as_bytes();
    let mut written = 0;
    while written < bytes.len() {
        match serial.write(&bytes[written..]) {
            Ok(len) if len > 0 => { written += len; }
            Ok(_) => {}
            Err(UsbError::WouldBlock) => {}
            Err(_) => { break; }
        }
    }
}

fn u32_to_str<'a>(mut n: u32, buf: &'a mut [u8]) -> &'a [u8] {
    // ... (u32_to_str implementation as before) ...
    if n == 0 {
        buf[0] = b'0';
        return &buf[0..1];
    }
    let mut i = 0;
    let mut temp_buf = [0u8; 10];
    while n > 0 {
        temp_buf[i] = (n % 10) as u8 + b'0';
        n /= 10;
        i += 1;
    }
    for j in 0..i {
        buf[j] = temp_buf[i - 1 - j];
    }
    &buf[0..i]
}

// This is needed for sched.rs to compile, as it calls `crate::dispatch_klipper_task`.
// In a `bin` crate, `main.rs` items are not automatically part of `crate::` for other modules
// unless `main.rs` itself is structured as `mod main { ... }` and re-exported by `lib.rs`,
// or `lib.rs` is the crate root and `main.rs` uses items from it.
// Given our setup, `klipper_mcu_lib` is the library, and `main.rs` is the binary.
// `sched.rs` is part of `klipper_mcu_lib`.
// So, this function should ideally be in `lib.rs` or `main.rs` should call a method
// on the scheduler that takes the dispatcher.
// The current `sched.rs` now takes `task_dispatcher: fn(u32)` in `new()`.
// `dispatch_klipper_task_from_scheduler` is the function we pass.
// So the call `crate::dispatch_klipper_task(task_id);` in `sched.rs` should be removed
// as it's now `(self.task_dispatcher)(task_id);`. (This was corrected in the sched.rs overwrite).
