// src/endstop.rs
#![cfg_attr(not(test), no_std)]

use defmt::Format;

/// Represents the configuration for a single endstop switch.
#[derive(Debug, Format, Copy, Clone)]
pub struct Endstop {
    /// The GPIO pin number for this endstop.
    pub(crate) pin_id: u8,

    /// The logical state that indicates the endstop is triggered.
    /// `true` if triggered when HIGH, `false` if triggered when LOW.
    /// E.g., for a normally-open switch connected to GND with a pull-up,
    /// the triggered state is LOW (`false`).
    pub(crate) triggered_state: bool,
}

impl Endstop {
    /// Creates a new Endstop configuration.
    pub fn new(pin_id: u8, triggered_when_high: bool) -> Self {
        Self {
            pin_id,
            triggered_state: triggered_when_high,
        }
    }
}
