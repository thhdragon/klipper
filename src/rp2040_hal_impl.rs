#![cfg_attr(not(test), no_std)]

use rp2040_hal::gpio::{DynPinId, FunctionSioOutput, Pin, PullDown, SioOutput};
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use klipper_mcu_lib::hal::GpioOut; // Use the GpioOut trait from our library

/// A wrapper around an RP2040 GPIO pin configured as an output.
pub struct Rp2040GpioOut {
    // The Pin type from rp2040-hal is generic over the PinId and Function.
    // To store it in a struct field without making the struct generic,
    // we can erase the pin type to a dynamic pin if needed, or use a specific type.
    // For simplicity, let's assume we are dealing with a known pin function (SioOutput).
    // If we need to handle various pin states or IDs generically, DynPin might be better.
    // Pin<DynPinId, FunctionSioOutput, PullDown>
    // However, into_push_pull_output gives specific types like Pin<Gpio25, FunctionSioOutput, PullDown>
    // Let's make the struct generic for now to hold the concrete pin type.
    pin: Pin<DynPinId, FunctionSioOutput, PullDown>,
}

impl Rp2040GpioOut {
    /// Creates a new `Rp2040GpioOut` from a HAL pin already configured as PushPullOutput.
    /// The `setup` in the trait is more about initial configuration.
    /// This `new` is for when the pin is already set up by the main init code.
    pub fn new(pin: Pin<DynPinId, FunctionSioOutput, PullDown>) -> Self {
        Rp2040GpioOut { pin }
    }
}

impl GpioOut for Rp2040GpioOut {
    /// Sets up a GPIO pin. In a real scenario, this would take a pin ID from `board_pins`
    /// and perform the full `Pins::new().gpioX.into_mode()` sequence.
    /// For this first pass, we assume the pin is already configured and passed to `new`.
    /// The trait's `setup` might need rethinking or be used differently.
    /// For now, this is a placeholder or simplified version.
    fn setup(_pin_id: u8, _inverted: bool) -> Self {
        // This is problematic because HAL initialization (Pins::new) usually happens once.
        // We can't easily call it here per pin without access to PAC and RESETS.
        // A better approach: `main` gets all pins, then passes the specific configured pin
        // to a constructor for this type.
        // For this example, let's panic, as this setup is not really usable standalone yet.
        unimplemented!("Proper GpioOut::setup needs access to HAL peripherals or a pre-configured pin.");
    }

    fn write(&mut self, high: bool) {
        if high {
            self.pin.set_high().unwrap();
        } else {
            self.pin.set_low().unwrap();
        }
    }

    fn toggle(&mut self) {
        self.pin.toggle().unwrap();
    }
}

// To make this usable, we need a way to get a `Pin<DynPinId, FunctionSioOutput, PullDown>`
// from the specific `Pin<GpioX, _, _>` types.
// rp2040-hal pins have an `into_dyn_pin()` method.

// Example of a constructor that might be more practical:
// pub fn from_pin<Id, Mode, Pull>(pin: Pin<Id, Mode, Pull>) -> Self
// where
//     Id: PinId,
//     Mode: PinMode,
//     Pin<Id, Mode, Pull>: OutputPin + ToggleableOutputPin,
// { ... }
// But our GpioOut trait is simpler. We'll adapt its usage in main.rs.
