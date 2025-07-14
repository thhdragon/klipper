// src/rp2040_hal_impl/spi.rs
#![cfg_attr(not(test), no_std)]

use crate::hal::{Spi as KlipperSpi, SpiError};
use rp2040_hal::spi::{Spi as RpHalSpi, Enabled};
use rp2040_hal::pac::SPI0; // Example, needs to be generic or specific
use rp2040_hal::gpio::Pin;
use rp2040_hal::gpio::FunctionSpi;
use embedded_hal::spi::{FullDuplex, Read, Write}; // embedded-hal traits
use nb; // For non-blocking error handling from embedded-hal

// The concrete HAL SPI type is very complex, e.g.:
// Spi<Enabled, pac::SPI0, (Pin<Gpio18, FunctionSpi>, Pin<Gpio19, FunctionSpi>, Pin<Gpio20, FunctionSpi>)>
// To make our wrapper usable, it must be generic over the SPI instance and the Pins tuple.

/// Wrapper for a configured RP2040 SPI peripheral that implements Klipper's Spi trait.
/// `SPI` is the concrete `rp2040_hal::spi::Spi<...>` instance.
pub struct Rp2040Spi<SPI> {
    hal_spi: SPI,
}

impl<SPI> Rp2040Spi<SPI> {
    /// Creates a new Rp2040Spi wrapper.
    /// The `hal_spi` instance must already be fully configured and enabled.
    pub fn new(hal_spi: SPI) -> Self {
        Self { hal_spi }
    }
}

// Implement our KlipperSpi trait for any type that implements the embedded-hal traits.
// This makes our wrapper very flexible. It can wrap any HAL-provided SPI object
// that conforms to embedded-hal.
impl<SPI> KlipperSpi for Rp2040Spi<SPI>
where
    SPI: Write<u8, Error = rp2040_hal::spi::Error>
         + Read<u8, Error = rp2040_hal::spi::Error>
         + FullDuplex<u8, Error = rp2040_hal::spi::Error>,
{
    type Error = SpiError;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.hal_spi.write(words).map_err(|_e| {
            defmt::error!("SPI Write Error: {:?}", defmt::Debug2Format(&_e));
            SpiError::TransferError
        })
    }

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        // FullDuplex requires sending and reading one word at a time in a loop.
        for byte in words.iter_mut() {
            // nb::block! macro handles non-blocking operations, waiting until they complete.
            nb::block!(self.hal_spi.send(*byte)).map_err(|_e| SpiError::TransferError)?;
            *byte = nb::block!(self.hal_spi.read()).map_err(|_e| SpiError::TransferError)?;
        }
        Ok(words)
    }

    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        // To read, we must send dummy bytes (e.g., 0x00).
        for byte in words.iter_mut() {
            nb::block!(self.hal_spi.send(0x00)).map_err(|_e| SpiError::TransferError)?;
            *byte = nb::block!(self.hal_spi.read()).map_err(|_e| SpiError::TransferError)?;
        }
        Ok(())
    }
}

// This also needs to be added to `rp2040_hal_impl/mod.rs`
// pub mod spi;
