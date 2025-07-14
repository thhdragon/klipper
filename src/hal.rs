// ... (existing content of hal.rs: GpioOut, GpioIn, Timer, Scheduler, AdcChannel, PwmChannel, Spi) ...

/// Errors that can occur during UART operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq, defmt::Format)]
pub enum UartError {
    InvalidPin,         // A pin cannot be used for the requested UART function.
    PinUnavailable,     // A pin is in use by another peripheral.
    ConfigurationError, // Error during UART peripheral or pin configuration.
    ReadError,          // An error occurred during a read operation (e.g., overrun, parity).
    WriteError,         // An error occurred during a write operation.
}

/// Trait for a configured UART (serial) peripheral.
pub trait Uart {
    /// Associated error type for this UART implementation.
    type Error: core::fmt::Debug + defmt::Format;

    /// Writes a slice of bytes to the UART bus, blocking until the entire slice is sent.
    fn write_blocking(&mut self, words: &[u8]) -> Result<(), Self::Error>;

    /// Reads bytes from the UART bus into the provided buffer, blocking until the buffer is full.
    fn read_blocking(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Flushes the write buffer, ensuring all data has been sent.
    fn flush(&mut self) -> Result<(), Self::Error>;
}
