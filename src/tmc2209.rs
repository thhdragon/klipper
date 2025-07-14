// src/tmc2209.rs
#![cfg_attr(not(test), no_std)]

use defmt::Format;
use crate::hal::{Uart, UartError};

// Register addresses (as defined before)
#[allow(dead_code)]
#[allow(non_camel_case_types)]
pub mod registers { /* ... as before ... */ }

/// Struct representing a TMC2209 driver.
/// It is a zero-sized-type; all operations are done on the passed-in UART bus.
#[derive(Copy, Clone)]
pub struct TMC2209;

impl TMC2209 {
    /// Creates a new handle for a TMC2209 driver.
    pub const fn new() -> Self {
        Self
    }

    /// Writes a value to a register on the TMC2209.
    /// `uart`: A mutable reference to a Uart HAL object.
    /// `slave_addr`: The address of the TMC device (0-3).
    /// `reg_addr`: The address of the register to write to.
    /// `value`: The 32-bit data to write.
    pub fn write_register(
        &self,
        uart: &mut impl Uart,
        slave_addr: u8,
        reg_addr: u8,
        value: u32,
    ) -> Result<(), UartError> {
        let mut datagram = [0u8; 8];
        datagram[0] = 0x55; // Sync byte
        datagram[1] = slave_addr;
        datagram[2] = reg_addr | 0x80; // Set write bit
        datagram[3..7].copy_from_slice(&value.to_be_bytes());
        datagram[7] = Self::calculate_crc(&datagram[0..7]);

        uart.write_blocking(&datagram)
    }

    /// Reads a value from a register on the TMC2209.
    /// This is the complex method due to single-wire half-duplex communication.
    /// It requires the ability to deconstruct/reconstruct the UART peripheral
    /// to switch the pin mode, which is not handled here but must be done by the caller.
    /// For now, this method will just send the read request. The calling code in main.rs
    /// will be responsible for the pin mode switching and reading the response.
    pub fn send_read_request(
        &self,
        uart: &mut impl Uart,
        slave_addr: u8,
        reg_addr: u8,
    ) -> Result<(), UartError> {
        let mut datagram = [0u8; 4];
        datagram[0] = 0x55; // Sync byte
        datagram[1] = slave_addr;
        datagram[2] = reg_addr; // Write bit is 0 for read
        datagram[3] = Self::calculate_crc(&datagram[0..3]);

        uart.write_blocking(&datagram)?;
        uart.flush() // Ensure the request is fully sent before pin mode switching
    }

    /// Calculates the CRC-8 for a TMC2209 datagram.
    fn calculate_crc(data: &[u8]) -> u8 {
        let mut crc: u8 = 0;
        for &byte in data {
            crc ^= byte;
            for _ in 0..8 {
                if (crc & 0x80) != 0 {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }
}
