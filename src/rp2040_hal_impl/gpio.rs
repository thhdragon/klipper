// src/rp2040_hal_impl/gpio.rs

#![cfg_attr(not(test), no_std)]

use rp2040_hal::gpio::dynpin::{DynPin, DynPinModeError};
use rp2040_hal::gpio::{DynPinId, FunctionSioInput, FunctionSioOutput, Pin, PullDown, PullUp, Floating, Input, SioOutput};
use embedded_hal::digital::v2::{InputPin as EhInputPin, OutputPin, ToggleableOutputPin};
use crate::hal::{GpioOut, GpioIn as HalGpioIn, PullType}; // Use crate::hal for traits
use defmt;

// --- Rp2040GpioOut Implementation ---

/// A wrapper around an RP2040 GPIO pin configured as an output.
pub struct Rp2040GpioOut {
    pin: Pin<DynPinId, FunctionSioOutput, PullDown>, // Assuming PullDown for SioOutput, might need more flexibility
}

impl Rp2040GpioOut {
    /// Creates a new `Rp2040GpioOut` from an RP2040 HAL pin already configured as PushPullOutput
    /// and degraded to a DynPin.
    pub fn new(dynamic_push_pull_output_pin: Pin<DynPinId, FunctionSioOutput, PullDown>) -> Self {
        Rp2040GpioOut { pin: dynamic_push_pull_output_pin }
    }
}

impl GpioOut for Rp2040GpioOut {
    /// Conceptual setup. In practice, pin configuration and degradation to DynPin
    /// should happen in a context that owns the global `Pins` struct (e.g., in `main.rs`).
    /// This instance should then be created using `Rp2040GpioOut::new()`.
    fn setup(_pin_id: u8, _inverted: bool) -> Self {
        defmt::panic!("Rp2040GpioOut::setup(pin_id, inverted) is conceptual. Use Rp2040GpioOut::new(DynPin) after configuring the pin as output using rp2040-hal's Pins struct and degrading it.");
        // The following is unreachable but satisfies the compiler for return type Self
        // loop {}
    }

    fn write(&mut self, high: bool) {
        if high {
            self.pin.set_high().unwrap_or_else(|_| defmt::error!("Rp2040GpioOut: Error setting pin high"));
        } else {
            self.pin.set_low().unwrap_or_else(|_| defmt::error!("Rp2040GpioOut: Error setting pin low"));
        }
    }

    fn toggle(&mut self) {
        self.pin.toggle().unwrap_or_else(|_| defmt::error!("Rp2040GpioOut: Error toggling pin"));
    }
}


// --- Rp2040GpioIn Implementation ---

/// A wrapper around an RP2040 GPIO pin configured as an input.
pub struct Rp2040GpioIn {
    // Store a type-erased dynamic pin. We expect it to be configured as an input
    // by the code that creates this struct instance using `Rp2040GpioIn::new()`.
    pin: DynPin,
}

impl Rp2040GpioIn {
    /// Creates a new `Rp2040GpioIn` from an RP2040 HAL `DynPin` already configured as an input
    /// with the desired pull type.
    ///
    /// The caller is responsible for:
    /// 1. Obtaining the specific `GpioXY` pin from the `rp2040_hal::gpio::Pins` struct.
    /// 2. Configuring it as an input with the correct pull resistor (e.g., `into_floating_input()`, `into_pull_up_input()`).
    /// 3. Degrading it to a `DynPin` (e.g., using `.into_dyn_pin()`).
    /// 4. Passing the resulting `DynPin` to this constructor.
    pub fn new(dynamic_input_pin: DynPin) -> Self {
        // It's assumed dynamic_input_pin is already in an SIO input mode.
        // The DynPin type itself doesn't statically enforce this beyond Function::Sio,
        // but runtime checks in `read()` via `is_high()`/`is_low()` will fail if not.
        Self { pin: dynamic_input_pin }
    }
}

impl HalGpioIn for Rp2040GpioIn {
    /// Conceptual setup for the `GpioIn` trait.
    ///
    /// **Important:** Due to `rp2040-hal`'s ownership model for the global `Pins` struct,
    /// this function cannot safely or practically perform the initial pin acquisition and configuration
    /// based on a raw `pin_id`.
    ///
    /// Instead, the user of this HAL should:
    /// 1. In `main.rs` or similar setup code, get the `rp2040_hal::gpio::Pins` instance.
    /// 2. Use the `pin_id` to get the specific typed pin (e.g., `pins.gpio10`).
    /// 3. Configure this typed pin with the desired `PullType` (e.g., `into_pull_up_input()`).
    /// 4. Degrade the configured typed pin to a `DynPin` (e.g., `pin.into_dyn_pin()`).
    /// 5. Construct `Rp2040GpioIn` using `Rp2040GpioIn::new(dyn_pin)`.
    ///
    /// This trait method panics to indicate this division of responsibility.
    fn new_input(pin_id: u8, pull: PullType) -> Self where Self: Sized {
        defmt::panic!(
            "Rp2040GpioIn::new_input(pin_id={}, pull={:?}) is conceptual and should not be called directly. \
            The pin must be configured using rp2040-hal's Pins struct and then passed to Rp2040GpioIn::new(DynPin).",
            pin_id, defmt::Debug2Format(&pull)
        );
        // loop {} // Unreachable, but to satisfy compiler if needed.
    }

    /// Reads the current state of the input pin.
    /// Returns `true` if the pin is high, `false` if low.
    fn read(&self) -> bool {
        // DynPin provides is_high() and is_low() which return Result<bool, DynPinModeError>
        // DynPinModeError occurs if the pin is not in an SIO mode compatible with input reading.
        match self.pin.is_high() {
            Ok(state) => state,
            Err(_e) => {
                defmt::error!(
                    "Rp2040GpioIn: Failed to read pin state (is_high failed). Error: {:?}",
                    defmt::Debug2Format(&_e)
                );
                // Default to a safe value (e.g., false) on error.
                // Consider if panicking is more appropriate depending on use case.
                false
            }
        }
    }
}
