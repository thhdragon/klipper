// src/rp2040_hal_impl/mod.rs

// This module provides RP2040-specific implementations of the HAL traits
// defined in `crate::hal`.

pub mod gpio;
pub mod timer;
pub mod adc;

// Re-export specific types if needed, e.g.:
// pub use gpio::Rp2040GpioOut;
// pub use timer::Rp2040Timer;
// For now, users can access them via `klipper_mcu_lib::rp2040_hal_impl::timer::Rp2040Timer`
// or `klipper_mcu_lib::rp2040_hal_impl::gpio::Rp2040GpioOut`.
// We can refine re-exports later.
