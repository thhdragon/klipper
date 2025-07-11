#![no_std]
#![no_main]

// Defmt imports
use defmt::*; // Logging macros
use defmt_rtt as _; // Global logger + RTT transport + defmt panic handler

// Entry point
use cortex_m_rt::entry;

// NOTE: The custom panic_handler below is likely overridden by defmt_rtt's panic handler.
// We can remove it if defmt's output is preferred for panics.
// For now, let's keep it to see which one takes precedence or if there's a conflict.
// If defmt-rtt is linked, its panic handler should be used.

// Panic handler (original, may be superseded by defmt-rtt)
use core::panic::PanicInfo;
#[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // This existing panic handler will likely NOT be called if defmt-rtt is linked.
    // defmt::error!("Panic: {}", defmt::Debug2Format(info)); // Example if we wanted to use defmt here manually
    loop {
        cortex_m::asm::nop();
    }
}

// HAL and PAC for RP2040
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    usb::UsbBus, // USB Bus type from HAL
};
use rp2040_hal::clocks::UsbClock; // USB Clock struct

// USB Device support
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

// Use Gpio traits for pin operations
// use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin}; // Now used via our HAL trait

// Use our library's board pin definitions and HAL traits
use klipper_mcu_lib::{
    board_pins,
    hal::GpioOut, // Import our GpioOut trait
    rp2040_hal_impl::Rp2040GpioOut, // Import our RP2040 specific implementation
};


// Globals for USB
// We need a static allocator for the USB bus
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;

// Define a line buffer for incoming serial commands
const MAX_LINE_LENGTH: usize = 256;
static mut LINE_BUFFER: heapless::String<MAX_LINE_LENGTH> = heapless::String::new();

#[entry]
fn main() -> ! {
    // Setup peripherals
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the Raspberry Pi Pico board is 12 MHz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Initialize USB
    // Reset and enable the USB peripheral
    pac.RESETS.reset.modify(|_, w| w.usbctrl().set_bit());
    pac.RESETS.reset.modify(|_, w| w.usbctrl().clear_bit());
    while pac.RESETS.reset_done.read().usbctrl().bit_is_clear() {}

    // The RP2040 HAL's UsbBus::new requires the UsbClock, which should be part of the clocks struct.
    // Make sure the UsbClock is derived correctly. It might require PLL_USB to be configured.
    // The init_clocks_and_plls function should handle PLL_USB.
    let usb_bus = UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock, // Ensure usb_clock is valid and running at 48MHz
        true,
        &mut pac.RESETS,
    );

    // SAFETY: This is safe because we are in a single-threaded environment (main) before interrupts are enabled.
    unsafe {
        USB_BUS = Some(UsbBusAllocator::new(usb_bus));
    }
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Device Class (CDC) ACM serial port
    // SAFETY: Same as above.
    unsafe {
        USB_SERIAL = Some(SerialPort::new(bus_ref));
    }

    // Create a USB device with a vendor ID (VID) and product ID (PID)
    // Using a test VID/PID for now.
    // SAFETY: Same as above.
    unsafe {
        USB_DEVICE = Some(
            UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("KlipperMCU")
                .product("Klipper Rust Firmware")
                .serial_number("TEST_SERIAL")
                .device_class(usbd_serial::USB_CLASS_CDC) // Use the CDC class code
                .build(),
        );
    }


    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Initialize GPIO pins
    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure the LED pin as a push-pull output
    let led_pin_raw = pins.gpio25.into_push_pull_output();
    let mut led_pin = Rp2040GpioOut::new(led_pin_raw.into_dyn_pin());

    let mut count = 0u32;

    // Main loop
    loop {
        // Blink LED
        led_pin.write(true);
        delay.delay_ms(250);
        led_pin.write(false);
        delay.delay_ms(250);


        // Try to send data over USB serial
        // SAFETY: Accessing static mut. Ok for polling if USB IRQ doesn't interfere.
        // Use defmt for the counter message
        info!("Hello, Klipper! Count: {}", count);
        count = count.wrapping_add(1);


        // Poll the USB device for events
        // The serial port is still polled for incoming commands and for sending command responses.
        // This needs to be called regularly to process USB traffic
        // SAFETY: Accessing static mut globals. Guard with critical section if interrupts were involved.
        // For polling, this is okay as long as USB interrupt is not enabled and trying to access these.
        unsafe {
            if let Some(usb_dev) = USB_DEVICE.as_mut() {
                if let Some(serial) = USB_SERIAL.as_mut() {
                    usb_dev.poll(&mut [serial]);

                    // Check for new data received over USB
                    let mut read_buf = [0u8; 64]; // Buffer for reading serial data
                    match serial.read(&mut read_buf) {
                        Ok(count) if count > 0 => {
                            let received_bytes = &read_buf[0..count];
                            for &byte in received_bytes {
                                if LINE_BUFFER.push(byte as char).is_err() {
                                    // Line buffer is full before newline
                                    let full_msg = b"Error: Line buffer full\r\n";
                                    // Attempt to write error, ignore if it fails for now
                                    let _ = serial.write(full_msg);
                                    LINE_BUFFER.clear(); // Clear buffer and discard current line
                                    break; // Stop processing this chunk of received_bytes
                                }

                                if byte == b'\n' {
                                    // Newline received, process the line.
                                    // Trim the newline before processing.
                                    // LINE_BUFFER currently contains the line *with* the newline.
                                    let mut line_to_process = LINE_BUFFER.clone(); // Clone to trim
                                    if line_to_process.ends_with('\n') {
                                        line_to_process.pop(); // Remove \n
                                        if line_to_process.ends_with('\r') {
                                            line_to_process.pop(); // Remove \r if present
                                        }
                                    }

                                    // Call process_command with the trimmed line.
                                    // serial is mutably borrowed by process_command if it needs to write.
                                    // This is okay as USB_SERIAL is Option<SerialPort<...>>
                                    // and we're in an unsafe block accessing it.
                                    process_command(&line_to_process, serial);

                                    LINE_BUFFER.clear(); // Clear buffer for the next line
                                }
                            }
                        }
                        Ok(_) => {
                            // No data received or 0 bytes read
                        }
                        Err(UsbError::WouldBlock) => {
                            // No data available to read
                        }
                        Err(_err) => {
                            // An error occurred during read
                            // For example: log::error!("USB read error: {:?}", err);
                        }
                    }
                }
            }
        }
    }
}

/// Converts a u32 to a string representation in the provided buffer.
/// Returns a slice of the buffer that contains the number.
/// Buffer should be at least 10 bytes for u32.
fn u32_to_str<'a>(mut n: u32, buf: &'a mut [u8]) -> &'a [u8] {
    if n == 0 {
        buf[0] = b'0';
        return &buf[0..1];
    }

    let mut i = 0;
    let mut temp_buf = [0u8; 10]; // Max 10 digits for u32

    while n > 0 {
        temp_buf[i] = (n % 10) as u8 + b'0';
        n /= 10;
        i += 1;
    }

    // Reverse the digits into the output buffer
    for j in 0..i {
        buf[j] = temp_buf[i - 1 - j];
    }

    &buf[0..i]
}

// Function to process received commands
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    if line == "PING" {
        serial_write_line(serial, "PONG\r\n");
    } else if line == "ID" {
        serial_write_line(serial, "KlipperRustRP2040_v0.0.1\r\n");
    } else if line.starts_with("ECHO ") {
        // Extract the part after "ECHO "
        if let Some(text_to_echo) = line.get("ECHO ".len()..) {
            serial_write_line(serial, text_to_echo);
            serial_write_line(serial, "\r\n"); // Add newline after echoing
        } else {
            // Should not happen if starts_with is true and "ECHO " has non-zero length
            serial_write_line(serial, "Error: Malformed ECHO command\r\n");
        }
    } else if line.is_empty() {
        // Ignore empty lines (e.g. if just \r\n is sent)
    } else {
        serial_write_line(serial, "Error: Unknown command '");
        serial_write_line(serial, line);
        serial_write_line(serial, "'\r\n");
    }
}


// Helper function to write a string slice over serial.
// Handles potential partial writes.
#[allow(dead_code)] // Will be used by process_command
fn serial_write_line(serial: &mut SerialPort<UsbBus>, data: &str) {
    let bytes = data.as_bytes();
    let mut written = 0;
    while written < bytes.len() {
        match serial.write(&bytes[written..]) {
            Ok(len) if len > 0 => {
                written += len;
            }
            Ok(_) => { /* 0 bytes written, buffer likely full, try again */ }
            Err(UsbError::WouldBlock) => { /* Try again later */ }
            Err(_) => { /* Other error, stop trying */ break; }
        }
        // It's important that UsbDevice.poll is called frequently enough
        // for the host to actually pick up the data.
        // If this function blocks for too long (e.g. spinning on WouldBlock),
        // it can starve USB polling. For now, this simple loop is okay
        // as process_command is called within the main USB polling loop.
    }
}
