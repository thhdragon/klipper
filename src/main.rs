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

static TEST_TIMER0: InterruptMutex<RefCell<Option<Rp2040Timer<Alarm0>>>> = InterruptMutex::new(RefCell::new(None));
const TEST_TIMER0_ID: u32 = 0;

static SCHEDULER: InterruptMutex<RefCell<Option<SchedulerState<Alarm1>>>> = InterruptMutex::new(RefCell::new(None));

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
                        // Directly invoke the callback
                        // trace!("dispatch_klipper_task_from_scheduler: Calling callback for timer ID {}", task_id);
                        (callback_fn)(timer_instance);
                    } else {
                        warn!("dispatch_klipper_task_from_scheduler: No callback found for timer ID {}", task_id);
                    }
                } else {
                    warn!("dispatch_klipper_task_from_scheduler: Timer instance (TEST_TIMER0) not found for ID {}", task_id);
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

    let rp_hal_timer = RpHalTimer::new(pac.TIMER, &mut pac.RESETS);

    let alarm0 = rp_hal_timer.alarm_0().unwrap();
    let klipper_timer0 = Rp2040Timer::new(alarm0, test_timer0_callback);
    interrupt_free(|cs| { TEST_TIMER0.borrow(cs).replace(Some(klipper_timer0)); });
    unsafe { NVIC::unmask(pac::Interrupt::TIMER_IRQ_0); } // Keep this for now, will be subject of next step

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
        led_pin.write(true); delay.delay_ms(250);
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

#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_0() {
    interrupt_free(|cs| {
        if let Some(timer) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() {
            timer.on_interrupt();
        }
    });
}

#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_1() {
    interrupt_free(|cs| {
        if let Some(scheduler) = SCHEDULER.borrow(cs).borrow_mut().as_mut() {
            scheduler.master_timer.on_interrupt();
        }
    });
}

fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    if line == "PING" { serial_write_line(serial, "PONG\r\n"); }
    else if line == "ID" { serial_write_line(serial, "KlipperRustRP2040_v0.0.1\r\n"); }
    else if line.starts_with("ECHO ") {
        if let Some(text_to_echo) = line.get("ECHO ".len()..) {
            serial_write_line(serial, text_to_echo); serial_write_line(serial, "\r\n");
        } else { serial_write_line(serial, "Error: Malformed ECHO command\r\n"); }
    } else if line.is_empty() { /* Ignore */ }
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
