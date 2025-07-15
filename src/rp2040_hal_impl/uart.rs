// src/rp2040_hal_impl/uart.rs
#![cfg_attr(not(test), no_std)]

use crate::hal::{Uart as KlipperUart, UartError};
use rp2040_hal::uart::{Enabled, UartPeripheral};
use rp2040_hal::pac::UART0;
use rp2040_hal::gpio::Pin;
use rp2040_hal::gpio::FunctionUart;
use embedded_hal::serial::{Read, Write};

/// Wrapper for a configured RP2040 UART peripheral that implements Klipper's Uart trait.
/// `UART` is the concrete `rp2040_hal::uart::UartPeripheral<...>` instance.
pub struct Rp2040Uart<UART> {
    hal_uart: UART,
}

impl<UART> Rp2040Uart<UART> {
    /// Creates a new Rp2040Uart wrapper.
    /// The `hal_uart` instance must already be fully configured and enabled.
    pub fn new(hal_uart: UART) -> Self {
        Self { hal_uart }
    }

    /// Consumes the wrapper and returns the inner HAL UART peripheral.
    pub fn into_inner(self) -> UART {
        self.hal_uart
    }
}

// Implement our KlipperUart trait for any type that implements the embedded-hal traits.
impl<UART> KlipperUart for Rp2040Uart<UART>
where
    UART: Read<u8, Error = rp2040_hal::uart::Error>
          + Write<u8, Error = rp2040_hal::uart::Error>,
{
    type Error = UartError;

    fn write_blocking(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.hal_uart.write_full_blocking(words).map_err(|_e| {
            defmt::error!("UART Write Error: {:?}", defmt::Debug2Format(&_e));
            UartError::WriteError
        })
    }

    fn read_blocking(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.hal_uart.read_full_blocking(buffer).map_err(|_e| {
            defmt::error!("UART Read Error: {:?}", defmt::Debug2Format(&_e));
            UartError::ReadError
        })
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.hal_uart.flush().map_err(|_e| {
            defmt::error!("UART Flush Error: {:?}", defmt::Debug2Format(&_e));
            UartError::WriteError // Typically flush errors are write-related
        })
    }
}
