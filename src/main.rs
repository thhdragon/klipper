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
};

// Use Gpio traits for pin operations
// use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin}; // Now used via our HAL trait

// Use our library's board pin definitions and HAL traits
use klipper_mcu_lib::{
    board_pins,
    hal::GpioOut, // Import our GpioOut trait
    rp2040_hal_impl::Rp2040GpioOut, // Import our RP2040 specific implementation
};


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

    // Create our HAL abstraction for the LED pin.
    // The `into_dyn_pin()` converts from a specific pin type (e.g., Gpio25)
    // to a type-erased `DynPin` which our `Rp2040GpioOut` struct expects.
    let mut led_pin = Rp2040GpioOut::new(led_pin_raw.into_dyn_pin());

    // Blink the LED using our GpioOut trait
    loop {
        led_pin.write(true); // Use the trait method
        delay.delay_ms(500);
        led_pin.write(false); // Use the trait method
        delay.delay_ms(500);

        // Example of using toggle:
        // led_pin.toggle(); // Use the trait method
        // delay.delay_ms(500);
    }
}
