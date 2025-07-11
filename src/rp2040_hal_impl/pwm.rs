// src/rp2040_hal_impl/pwm.rs
#![cfg_attr(not(test), no_std)]

use crate::hal::{PwmChannel as KlipperPwmChannel, PwmError};
use embedded_hal::PwmPin; // Using embedded-hal's PwmPin trait
use rp2040_hal::pwm::Channel as RpChannel; // The concrete Channel type from rp2040-hal
use rp2040_hal::pwm::{SliceMode, ValidSliceMode}; // For generic bounds
use rp2040_hal::pwm::{ChannelId, SliceId}; // For generic bounds

use defmt;

/// Wrapper for an RP2040 PWM channel that implements Klipper's PwmChannel trait.
///
/// `C` is a concrete rp2040_hal::pwm::Channel type,
/// e.g., `rp2040_hal::pwm::Channel<rp2040_hal::pwm::Pwm0, rp2040_hal::pwm::A, rp2040_hal::pwm::FreeRunning>`
/// Such a channel `C` must implement `embedded_hal::PwmPin<Duty = u16>`.
pub struct Rp2040PwmChannel<C>
where
    C: PwmPin<Duty = u16>,
{
    hal_channel: C,
    // Max duty cycle value for this channel. Determined by slice's TOP value.
    // This is usually passed in or calculated when the channel is configured.
    max_duty_val: u16,
}

impl<C> Rp2040PwmChannel<C>
where
    C: PwmPin<Duty = u16>,
{
    /// Creates a new Rp2040PwmChannel wrapper.
    ///
    /// - `hal_channel`: The configured `rp2040_hal::pwm::Channel` instance.
    /// - `max_duty_val`: The maximum raw duty cycle value (TOP value of the slice).
    ///
    /// The frequency/period of the PWM slice must be configured *before* this
    /// wrapper is created, as this wrapper does not control slice-level settings.
    pub fn new(hal_channel: C, max_duty_val: u16) -> Self {
        Self { hal_channel, max_duty_val }
    }
}

impl<C> KlipperPwmChannel for Rp2040PwmChannel<C>
where
    C: PwmPin<Duty = u16>,
{
    type Error = PwmError;

    fn enable(&mut self) -> Result<(), Self::Error> {
        self.hal_channel.enable();
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.hal_channel.disable();
        Ok(())
    }

    fn set_duty_cycle_percent(&mut self, duty_percent: f32) -> Result<(), Self::Error> {
        if !(0.0..=1.0).contains(&duty_percent) {
            return Err(PwmError::InvalidDutyCycle);
        }
        let raw_duty = (duty_percent * self.max_duty_val as f32) as u16;
        self.set_duty_cycle_raw(raw_duty) // Delegate to raw
    }

    fn set_duty_cycle_raw(&mut self, duty_raw: u16) -> Result<(), Self::Error> {
        if duty_raw > self.max_duty_val {
            // defmt::warn!("Attempted duty {} > max_duty {}", duty_raw, self.max_duty_val);
            return Err(PwmError::InvalidDutyCycle);
        }
        self.hal_channel.set_duty(duty_raw);
        Ok(())
    }

    fn get_max_duty_raw(&self) -> u16 {
        self.max_duty_val
        // Or, if the underlying HAL channel provides it directly:
        // self.hal_channel.get_max_duty()
        // `embedded_hal::PwmPin` requires `get_max_duty()`.
        // So, self.hal_channel.get_max_duty() should be used if available and correct.
        // The `max_duty_val` passed at construction should be this value.
    }

    /// Sets the frequency of the PWM signal in Hz.
    ///
    /// **Note:** For `rp2040-hal`, frequency is typically set at the `Slice` level,
    /// not per-channel after the channel is created from a slice. This method
    /// is therefore difficult to implement robustly without access to the `Slice`
    /// and the global `pwm::Slices` peripheral, plus system clock.
    ///
    /// This implementation will return an error, indicating frequency should be
    /// configured when the pin is first set up for PWM.
    fn set_frequency_hz(&mut self, _freq_hz: u32) -> Result<(), Self::Error> {
        defmt::error!("Rp2040PwmChannel: set_frequency_hz is not supported directly on a configured channel. Frequency is set at the Slice level during initial PWM setup.");
        Err(PwmError::ConfigurationError)
        // A more advanced implementation might involve this Rp2040PwmChannel struct
        // holding references or tokens to reconfigure its parent Slice, but that adds complexity.
    }
}

// This also needs to be added to `rp2040_hal_impl/mod.rs`
// pub mod pwm;
