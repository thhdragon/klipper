// src/toolhead.rs
#![cfg_attr(not(test), no_std)]

use defmt::Format;

/// Defines whether G-code coordinates are interpreted as absolute or relative.
#[derive(Debug, Format, Copy, Clone, PartialEq, Eq)]
pub enum PositioningMode {
    Absolute,
    Relative,
}

/// Holds the state of the machine's toolhead, including position and settings.
#[derive(Debug, Format)]
pub struct Toolhead {
    /// The current logical position in millimeters (X, Y, Z).
    pub current_position: (f32, f32, f32),

    /// The current positioning mode (`G90` or `G91`).
    pub positioning_mode: PositioningMode,

    /// The last commanded feedrate (velocity) in mm/minute.
    /// G-code typically specifies feedrate in mm/min, so we store it that way.
    pub feedrate: f32,
}

impl Toolhead {
    /// Creates a new Toolhead instance with default values.
    pub fn new() -> Self {
        Self {
            current_position: (0.0, 0.0, 0.0),
            positioning_mode: PositioningMode::Absolute, // G90 is the default
            feedrate: 1500.0, // A reasonable default feedrate, e.g., 25 mm/s * 60
        }
    }
}

impl Default for Toolhead {
    fn default() -> Self {
        Self::new()
    }
}
