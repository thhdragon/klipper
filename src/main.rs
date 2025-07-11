#![no_std]
#![no_main]

// Defmt imports
use defmt::*; // Logging macros
use defmt_rtt as _; // Global logger + RTT transport + defmt panic handler

// Entry point
use cortex_m_rt::entry;

// Panic handler (original, likely superseded by defmt-rtt)
use core::panic::PanicInfo;
#[cfg(not(test))]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // This existing panic handler will likely NOT be called if defmt-rtt is linked.
    loop {
        cortex_m::asm::nop();
    }
}

// HAL and PAC for RP2040
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac::{self, interrupt, NVIC}, // Import NVIC for interrupt control
    sio::Sio,
    watchdog::Watchdog,
    timer::Timer as RpHalTimer, // Alias HAL's Timer to avoid conflict
    usb::UsbBus,
};
use rp2040_hal::clocks::UsbClock;

// Klipper HAL Timer trait and our implementation
use klipper_mcu_lib::hal::Timer as KlipperHalTimer; // Alias our HAL trait
use klipper_mcu_lib::rp2040_hal_impl::timer::Rp2040Timer;
use rp2040_hal::timer::Alarm0; // Specific alarm for testing

// For static storage of the timer instance
use core::cell::RefCell;
use cortex_m::interrupt::Mutex as InterruptMutex; // Alias to avoid conflict if `Mutex` is used elsewhere

// USB Device support
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

// Klipper GpioOut trait
use klipper_mcu_lib::{
    hal::GpioOut,
    rp2040_hal_impl::gpio::Rp2040GpioOut,
};

// Globals for USB
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;

// Define a line buffer for incoming serial commands
const MAX_LINE_LENGTH: usize = 256;
static mut LINE_BUFFER: heapless::String<MAX_LINE_LENGTH> = heapless::String::new();

// Static storage for our test timer instance (using Alarm0)
static TEST_TIMER0: InterruptMutex<RefCell<Option<Rp2040Timer<Alarm0>>>> = InterruptMutex::new(RefCell::new(None));

// Timer callback function
fn test_timer0_callback(timer: &mut Rp2040Timer<Alarm0>) -> klipper_mcu_lib::hal::StepEventResult {
    defmt::info!("Test Timer0 Fired! Current waketime: {}", timer.get_waketime());

    // Reschedule for 1 second later (approx)
    let current_waketime = timer.get_waketime();
    // Assuming 1MHz timer clock from rp2040-hal default setup
    let next_waketime = current_waketime.wrapping_add(1_000_000);
    timer.set_waketime(next_waketime);
    defmt::debug!("Timer0 rescheduled to: {}", next_waketime);

    () // StepEventResult is () for now
}

#[entry]
fn main() -> ! {
    // Setup peripherals
    let mut pac_peripherals = pac::Peripherals::take().unwrap();
    let core_peripherals = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac_peripherals.WATCHDOG);
    let sio = Sio::new(pac_peripherals.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac_peripherals.XOSC,
        pac_peripherals.CLOCKS,
        pac_peripherals.PLL_SYS,
        pac_peripherals.PLL_USB,
        &mut pac_peripherals.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Initialize USB
    let usb_bus_allocator = UsbBusAllocator::new(UsbBus::new(
        pac_peripherals.USBCTRL_REGS,
        pac_peripherals.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac_peripherals.RESETS,
    ));

    unsafe {
        USB_BUS = Some(usb_bus_allocator);
    }
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    unsafe {
        USB_SERIAL = Some(SerialPort::new(bus_ref));
        USB_DEVICE = Some(
            UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("KlipperMCU")
                .product("Klipper Rust Firmware")
                .serial_number("TEST_SERIAL")
                .device_class(usbd_serial::USB_CLASS_CDC)
                .build(),
        );
    }

    let mut delay = cortex_m::delay::Delay::new(core_peripherals.SYST, clocks.system_clock.freq().to_Hz());

    let pins = rp2040_hal::gpio::Pins::new(
        pac_peripherals.IO_BANK0,
        pac_peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac_peripherals.RESETS,
    );
    let mut led_pin = Rp2040GpioOut::new(pins.gpio25.into_push_pull_output().into_dyn_pin());

    // Initialize the RP2040 HAL Timer
    let rp_hal_timer = RpHalTimer::new(pac_peripherals.TIMER, &mut pac_peripherals.RESETS);
    let alarm0 = rp_hal_timer.alarm_0().unwrap(); // Take Alarm0

    // Create and store our Klipper HAL Timer instance for Alarm0
    let klipper_timer0 = Rp2040Timer::new(alarm0, test_timer0_callback);
    cortex_m::interrupt::free(|cs| {
        TEST_TIMER0.borrow(cs).replace(Some(klipper_timer0));
    });

    // Schedule the first timer event
    let now_ticks = rp_hal_timer.get_counter_low();
    let initial_waketime = now_ticks.wrapping_add(2_000_000); // Approx 2 seconds from now

    cortex_m::interrupt::free(|cs| {
        if let Some(timer) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() {
            timer.set_waketime(initial_waketime);
            defmt::info!("Test Timer0 initial waketime set to: {}", initial_waketime);
        }
    });

    // Unmask the interrupt for TIMER_IRQ_0 in the NVIC
    unsafe {
        NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let mut loop_count = 0u32;
    defmt::info!("Setup complete, entering main loop.");

    loop {
        led_pin.write(true);
        delay.delay_ms(250);
        led_pin.write(false);
        delay.delay_ms(250);

        if loop_count % 4 == 0 { // Log less frequently from main loop
            defmt::debug!("Main loop iteration: {}", loop_count);
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
                                    LINE_BUFFER.clear();
                                    break;
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
                        Err(e) => { defmt::warn!("USB read error: {:?}", defmt::Debug2Format(&e)); }
                    }
                }
            }
        }
    }
}

#[allow(non_snake_case)] // RP2040 PAC names interrupts in UPPER_CASE
#[interrupt]
fn TIMER_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(timer) = TEST_TIMER0.borrow(cs).borrow_mut().as_mut() {
            // Important: Check if *this specific alarm* is the source of interrupt
            // The on_interrupt method itself checks alarm.finished()
            timer.on_interrupt();
        } else {
            // This case should ideally not happen if setup is correct
            // and timer is not removed from static storage.
            // However, if it does, clear the interrupt for safety if possible,
            // though without the alarm object it's hard.
            // For now, just log.
            defmt::error!("TIMER_IRQ_0 fired but no timer instance found in static storage!");
            // Manually clear all alarm0 IRQs if possible (rp2040-hal specific)
            // This is a bit of a hack; proper design would ensure instance exists
            // or the IRQ is disabled when instance is None.
            // unsafe { (*rp2040_pac::TIMER::ptr()).intr.write(|w| w.alarm_0().set_bit()); }
        }
    });
}

fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
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
