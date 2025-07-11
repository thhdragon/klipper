#![cfg_attr(not(test), no_std)]

// Placeholder for StepEventResult until stepper module is more defined.
// For now, a simple unit type will suffice for timer callbacks.
pub type StepEventResult = ();

// --- Placeholder for Hardware Abstractions (from existing src/lib.rs) ---
pub trait GpioOut {
    fn setup(pin_id: u8, inverted: bool) -> Self; // Simplified, HALs have more complex setup
    fn write(&mut self, high: bool);
    fn toggle(&mut self);
    // gpio_out_toggle_noirq in C implies direct register access,
    // HALs might provide this or it might require unsafe code.
    // For now, a simple toggle is provided.
}

/// Enum to specify internal pull resistor configuration for input pins.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PullType {
    Up,
    Down,
    Floating,
}

/// Trait for digital input pins.
pub trait GpioIn {
    /// Configures a pin as an input with the specified pull resistor.
    /// `pin_id`: The MCU-specific identifier for the pin.
    /// `pull`: The desired pull resistor configuration.
    /// Returns an instance of the GpioIn implementor.
    /// This might panic or return a Result in a more robust implementation if pin_id is invalid.
    fn new_input(pin_id: u8, pull: PullType) -> Self where Self: Sized; // Sized because new returns Self

    /// Reads the current state of the input pin.
    /// Returns `true` if the pin is high, `false` if low.
    fn read(&self) -> bool;
}

pub trait Timer {
    // Represents a timer that can be scheduled by a scheduler
    // The callback will likely need to be more flexible, perhaps taking a context.
    // For now, using the existing signature from src/lib.rs which implies Stepper implements Timer
    // or a similar dispatch mechanism.
    fn new(callback: fn(&mut Self) -> StepEventResult) -> Self; // Simplified
    fn get_waketime(&self) -> u32;
    fn set_waketime(&mut self, waketime: u32);
    // func pointer is handled by the callback in new()
}

pub trait Scheduler {
    // Simplified scheduler interface
    fn add_timer(&mut self, timer: &mut impl Timer);
    fn delete_timer(&mut self, timer: &mut impl Timer);
    fn read_time(&self) -> u32; // Equivalent to timer_read_time()
    fn get_clock_freq(&self) -> u32; // To be used by timer_from_us
    // timer_is_before can be implemented using direct comparison of u32 times
}

/// Errors that can occur during ADC operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq, defmt::Format)]
pub enum AdcError {
    InvalidPin,         // The specified pin cannot be used for ADC or is invalid.
    PinUnavailable,     // The pin is currently in use by another function.
    ReadError,          // A hardware error occurred during ADC read.
    ConfigurationError, // Error during ADC or pin configuration.
}

/// Trait for Analog to Digital Converter (ADC) pins.
pub trait AdcChannel {
    /// Associated error type for this ADC implementation.
    type Error: core::fmt::Debug + defmt::Format; // Ensure error can be logged/formatted

    // Constructor might be part of a higher-level AdcController trait,
    // or each pin could be individually convertible.
    // For now, let's assume a method to create an ADC channel from a pin ID.
    // This is difficult to make generic here without knowing the ADC controller.
    // Let's simplify: the AdcChannel is ALREADY an ADC pin.

    /// Reads the raw ADC value from the channel.
    /// Typically a 10-bit or 12-bit value depending on the MCU.
    fn read_raw(&mut self) -> Result<u16, Self::Error>;

    /// (Optional) Reads the voltage from the ADC channel.
    /// `vref` is the reference voltage for the ADC (e.g., 3.3f32).
    fn read_voltage(&mut self, vref: f32) -> Result<f32, Self::Error> {
        // Default implementation assuming max raw value for 12-bit ADC.
        // Implementers should override if their ADC has different resolution.
        const ADC_MAX_RAW_12BIT: u32 = (1 << 12) - 1; // 4095
        let raw = self.read_raw()? as u32;
        Ok((raw as f32 / ADC_MAX_RAW_12BIT as f32) * vref)
    }
}

// It might be better to have an AdcController trait that vends AdcChannel instances.
// pub trait AdcController {
//     type Channel: AdcChannel;
//     fn acquire_channel(&mut self, pin_id: u8) -> Result<Self::Channel, AdcError>;
// }
// For now, AdcChannel itself is the main focus.

/// Errors that can occur during PWM operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq, defmt::Format)]
pub enum PwmError {
    InvalidPin,         // The specified pin cannot be used for PWM or is invalid.
    PinUnavailable,     // The pin is currently in use by another function.
    InvalidDutyCycle,   // Duty cycle value is out of range (e.g., > 1.0 or > max_raw).
    InvalidFrequency,   // Frequency value is out of achievable range.
    InvalidPeriod,      // Period value is out of achievable range.
    ConfigurationError, // Error during PWM or pin configuration.
}

/// Trait for Pulse Width Modulation (PWM) channels.
/// Assumes a single PWM channel is being controlled.
pub trait PwmChannel {
    /// Associated error type for this PWM implementation.
    type Error: core::fmt::Debug + defmt::Format;

    // Constructor is typically handled by a higher-level PWM peripheral manager
    // that vends PwmChannel instances for specific pins/channels.
    // fn new_pwm_channel(pin_id: u8, pwm_peripheral_resources: ???) -> Result<Self, Self::Error> where Self: Sized;

    /// Enables the PWM channel output.
    fn enable(&mut self) -> Result<(), Self::Error>;

    /// Disables the PWM channel output.
    /// (e.g., sets duty to 0 or tristates the pin, depending on hardware).
    fn disable(&mut self) -> Result<(), Self::Error>;

    /// Sets the duty cycle as a percentage (0.0 to 1.0).
    /// 0.0 means always off, 1.0 means always on (within the PWM period).
    fn set_duty_cycle_percent(&mut self, duty_percent: f32) -> Result<(), Self::Error>;

    /// Sets the duty cycle using a raw hardware-specific value.
    /// `duty_raw` typically ranges from 0 to `get_max_duty_raw()`.
    fn set_duty_cycle_raw(&mut self, duty_raw: u16) -> Result<(), Self::Error>;

    /// Gets the maximum possible raw duty cycle value for this channel.
    /// This often depends on the PWM period/frequency and hardware counter size.
    fn get_max_duty_raw(&self) -> u16;

    /// Sets the frequency of the PWM signal in Hz.
    /// Note: Setting frequency may also affect the resolution of the duty cycle (max_duty_raw).
    fn set_frequency_hz(&mut self, freq_hz: u32) -> Result<(), Self::Error>;

    // /// Sets the period of the PWM signal in raw timer ticks.
    // /// This is often a more direct way to control frequency for some HALs.
    // fn set_period_ticks(&mut self, period_ticks: u32) -> Result<(), Self::Error>;
}
