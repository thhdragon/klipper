// src/rp2040_hal_impl/gpio.rs
#![cfg_attr(not(test), no_std)]

// This file previously contained Rp2040GpioOut and Rp2040GpioIn structs
// and their implementations of the GpioOut and GpioIn HAL traits.

// With the introduction of the GpioManager in `src/gpio_manager.rs`,
// the GpioManager now centralizes GPIO control and directly performs operations
// on DynPin objects. The commands in `main.rs` interact with GpioManager's
// methods (e.g., `configure_pin_as_output`, `write_pin_output`, etc.)
// rather than instantiating and using Rp2040GpioOut/Rp2040GpioIn objects directly.

// Therefore, the Rp2040GpioOut and Rp2040GpioIn structs as previously defined here
// are no longer the primary interface for GPIO commands and are considered obsolete
// in the context of the GpioManager handling these operations.

// This file can be left empty or removed if no other RP2040-specific GPIO HAL
// helper types (that are not the GpioManager itself) are needed.
// For now, leaving it empty to signify the shift in responsibility.

// The GpioOut and GpioIn traits still exist in `src/hal.rs` as abstract definitions
// of capability. The GpioManager fulfills these capabilities through its own API.
// A future refinement could have GpioManager return temporary types that implement
// these traits, but that's not the current design.
