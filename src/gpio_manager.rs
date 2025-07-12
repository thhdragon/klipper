// src/gpio_manager.rs
#![cfg_attr(not(test), no_std)]

use rp2040_hal::gpio::{self, Pin, FunctionSio, SioInput, PullDown};

// Define aliases for the pin types to make the struct definition cleaner.
// This defines the "resting state" of a pin when it's stored in the manager.
// The default reset state for pins in rp2040-hal is FunctionSio<SioInput> with PullDown.
type Gpio0Pin = Pin<gpio::bank0::Gpio0, FunctionSio<SioInput>, PullDown>;
type Gpio1Pin = Pin<gpio::bank0::Gpio1, FunctionSio<SioInput>, PullDown>;
type Gpio2Pin = Pin<gpio::bank0::Gpio2, FunctionSio<SioInput>, PullDown>;
type Gpio3Pin = Pin<gpio::bank0::Gpio3, FunctionSio<SioInput>, PullDown>;
type Gpio4Pin = Pin<gpio::bank0::Gpio4, FunctionSio<SioInput>, PullDown>;
type Gpio5Pin = Pin<gpio::bank0::Gpio5, FunctionSio<SioInput>, PullDown>;
type Gpio6Pin = Pin<gpio::bank0::Gpio6, FunctionSio<SioInput>, PullDown>;
type Gpio7Pin = Pin<gpio::bank0::Gpio7, FunctionSio<SioInput>, PullDown>;
type Gpio8Pin = Pin<gpio::bank0::Gpio8, FunctionSio<SioInput>, PullDown>;
type Gpio9Pin = Pin<gpio::bank0::Gpio9, FunctionSio<SioInput>, PullDown>;
type Gpio10Pin = Pin<gpio::bank0::Gpio10, FunctionSio<SioInput>, PullDown>;
type Gpio11Pin = Pin<gpio::bank0::Gpio11, FunctionSio<SioInput>, PullDown>;
type Gpio12Pin = Pin<gpio::bank0::Gpio12, FunctionSio<SioInput>, PullDown>;
type Gpio13Pin = Pin<gpio::bank0::Gpio13, FunctionSio<SioInput>, PullDown>;
type Gpio14Pin = Pin<gpio::bank0::Gpio14, FunctionSio<SioInput>, PullDown>;
type Gpio15Pin = Pin<gpio::bank0::Gpio15, FunctionSio<SioInput>, PullDown>;
type Gpio16Pin = Pin<gpio::bank0::Gpio16, FunctionSio<SioInput>, PullDown>;
type Gpio17Pin = Pin<gpio::bank0::Gpio17, FunctionSio<SioInput>, PullDown>;
type Gpio18Pin = Pin<gpio::bank0::Gpio18, FunctionSio<SioInput>, PullDown>;
type Gpio19Pin = Pin<gpio::bank0::Gpio19, FunctionSio<SioInput>, PullDown>;
type Gpio20Pin = Pin<gpio::bank0::Gpio20, FunctionSio<SioInput>, PullDown>;
type Gpio21Pin = Pin<gpio::bank0::Gpio21, FunctionSio<SioInput>, PullDown>;
type Gpio22Pin = Pin<gpio::bank0::Gpio22, FunctionSio<SioInput>, PullDown>;
type Gpio23Pin = Pin<gpio::bank0::Gpio23, FunctionSio<SioInput>, PullDown>;
type Gpio24Pin = Pin<gpio::bank0::Gpio24, FunctionSio<SioInput>, PullDown>;
type Gpio25Pin = Pin<gpio::bank0::Gpio25, FunctionSio<SioInput>, PullDown>;
type Gpio26Pin = Pin<gpio::bank0::Gpio26, FunctionSio<SioInput>, PullDown>;
type Gpio27Pin = Pin<gpio::bank0::Gpio27, FunctionSio<SioInput>, PullDown>;
type Gpio28Pin = Pin<gpio::bank0::Gpio28, FunctionSio<SioInput>, PullDown>;
type Gpio29Pin = Pin<gpio::bank0::Gpio29, FunctionSio<SioInput>, PullDown>;


/// Manages the state and ownership of all GPIO pins.
/// Each pin is stored as an Option wrapping its typed HAL representation.
/// When a pin is used for a specific function, its Option is `take()`n,
/// and after use, it must be converted back to its default state and `replace()`d.
#[allow(non_snake_case)]
pub struct GpioManager {
    // Each field corresponds to a GPIO pin.
    pub Gpio0: Option<Gpio0Pin>,
    pub Gpio1: Option<Gpio1Pin>,
    pub Gpio2: Option<Gpio2Pin>,
    pub Gpio3: Option<Gpio3Pin>,
    pub Gpio4: Option<Gpio4Pin>,
    pub Gpio5: Option<Gpio5Pin>,
    pub Gpio6: Option<Gpio6Pin>,
    pub Gpio7: Option<Gpio7Pin>,
    pub Gpio8: Option<Gpio8Pin>,
    pub Gpio9: Option<Gpio9Pin>,
    pub Gpio10: Option<Gpio10Pin>,
    pub Gpio11: Option<Gpio11Pin>,
    pub Gpio12: Option<Gpio12Pin>,
    pub Gpio13: Option<Gpio13Pin>,
    pub Gpio14: Option<Gpio14Pin>,
    pub Gpio15: Option<Gpio15Pin>,
    pub Gpio16: Option<Gpio16Pin>,
    pub Gpio17: Option<Gpio17Pin>,
    pub Gpio18: Option<Gpio18Pin>,
    pub Gpio19: Option<Gpio19Pin>,
    pub Gpio20: Option<Gpio20Pin>,
    pub Gpio21: Option<Gpio21Pin>,
    pub Gpio22: Option<Gpio22Pin>,
    pub Gpio23: Option<Gpio23Pin>,
    pub Gpio24: Option<Gpio24Pin>,
    pub Gpio25: Option<Gpio25Pin>,
    pub Gpio26: Option<Gpio26Pin>,
    pub Gpio27: Option<Gpio27Pin>,
    pub Gpio28: Option<Gpio28Pin>,
    pub Gpio29: Option<Gpio29Pin>,
}

impl GpioManager {
    /// Creates a new GpioManager, taking ownership of all GPIO pins from the HAL `Pins` struct.
    /// Each pin is stored in its corresponding `Option` field.
    pub fn new(hal_pins: gpio::Pins) -> Self {
        Self {
            Gpio0: Some(hal_pins.gpio0),
            Gpio1: Some(hal_pins.gpio1),
            Gpio2: Some(hal_pins.gpio2),
            Gpio3: Some(hal_pins.gpio3),
            Gpio4: Some(hal_pins.gpio4),
            Gpio5: Some(hal_pins.gpio5),
            Gpio6: Some(hal_pins.gpio6),
            Gpio7: Some(hal_pins.gpio7),
            Gpio8: Some(hal_pins.gpio8),
            Gpio9: Some(hal_pins.gpio9),
            Gpio10: Some(hal_pins.gpio10),
            Gpio11: Some(hal_pins.gpio11),
            Gpio12: Some(hal_pins.gpio12),
            Gpio13: Some(hal_pins.gpio13),
            Gpio14: Some(hal_pins.gpio14),
            Gpio15: Some(hal_pins.gpio15),
            Gpio16: Some(hal_pins.gpio16),
            Gpio17: Some(hal_pins.gpio17),
            Gpio18: Some(hal_pins.gpio18),
            Gpio19: Some(hal_pins.gpio19),
            Gpio20: Some(hal_pins.gpio20),
            Gpio21: Some(hal_pins.gpio21),
            Gpio22: Some(hal_pins.gpio22),
            Gpio23: Some(hal_pins.gpio23),
            Gpio24: Some(hal_pins.gpio24),
            Gpio25: Some(hal_pins.gpio25),
            Gpio26: Some(hal_pins.gpio26),
            Gpio27: Some(hal_pins.gpio27),
            Gpio28: Some(hal_pins.gpio28),
            Gpio29: Some(hal_pins.gpio29),
        }
    }
}
// All other methods (`take_pin_for_...`, `release_pin...`, `configure_pin...`, etc.)
// and helper enums (`ManagedPin`, `PinModeState`, `...CapablePin`) are now removed.
// All logic for pin manipulation will now reside in `main.rs::process_command`.
