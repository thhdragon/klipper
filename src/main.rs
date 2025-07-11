#![no_std]
#![no_main]

// Panic handler
use core::panic::PanicInfo;
#[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Log the panic info if a logger is available
    // For example, using RTT or semihosting if enabled.
    // log::error!("{}", info);

    // On panic, loop indefinitely. Blink an LED or signal error in some way.
    // Attempt to blink the LED rapidly as an error signal if possible.
    // This part is tricky without a fully initialized HAL.
    // For now, just a simple loop.
    loop {
        // Use a volatile write to prevent the compiler from optimizing this loop away.
        cortex_m::asm::nop();
    }
}

// Entry point
use cortex_m_rt::entry;

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
        unsafe {
            if let Some(serial) = USB_SERIAL.as_mut() {
                // Create a message with a counter
                // Note: heapless::String or arrayvec could be used for formatting without std.
                // For simplicity, we'll use a fixed buffer and manual formatting.
                let mut message_buf = [0u8; 64]; // Buffer for our message
                let greeting = b"Hello, Klipper! Count: ";
                let count_str = u32_to_str(count, &mut message_buf[greeting.len()..]);
                let newline = b"\r\n";

                // Combine parts into the buffer
                message_buf[..greeting.len()].copy_from_slice(greeting);
                // count_str is already in message_buf
                let end_of_count = greeting.len() + count_str.len();
                message_buf[end_of_count..end_of_count + newline.len()].copy_from_slice(newline);

                let final_message = &message_buf[..end_of_count + newline.len()];

                // Write the message. Ignore errors and partial writes for this simple example.
                let _ = serial.write(final_message);
            }
        }
        count = count.wrapping_add(1);


        // Poll the USB device for events
        // This needs to be called regularly to process USB traffic
        // SAFETY: Accessing static mut globals. Guard with critical section if interrupts were involved.
        // For polling, this is okay as long as USB interrupt is not enabled and trying to access these.
        unsafe {
            if let Some(usb_dev) = USB_DEVICE.as_mut() {
                if let Some(serial) = USB_SERIAL.as_mut() {
                    usb_dev.poll(&mut [serial]);
                    // Read/Write operations would go here - Step 4
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
