// src/gpio_manager.rs
#![cfg_attr(not(test), no_std)]

use rp2040_hal::gpio::dynpin::{DynPin, DynPinModeError};
use rp2040_hal::gpio::{self, Pins, FunctionSio, SioConfig, PullType as HalPullType, FunctionPwm, PinId};
use crate::hal::PullType as KlipperPullType;
use defmt::Format;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum PinModeState {
    Disabled,
    InputFloating,
    InputPullUp,
    InputPullDown,
    OutputPushPull,
    FunctionAdc,
    FunctionPwm,
}

impl Default for PinModeState {
    fn default() -> Self {
        PinModeState::Disabled
    }
}

#[derive(Format)]
pub struct ManagedPin {
    #[defmt(Debug2Format)]
    pub(crate) pin: DynPin,
    pub(crate) current_mode: PinModeState,
    pub(crate) id: u8,
}

impl ManagedPin {
    pub fn new(pin: DynPin, id: u8) -> Self {
        Self {
            pin,
            current_mode: PinModeState::Disabled,
            id,
        }
    }
    pub fn id(&self) -> u8 { self.id }
}

pub const NUM_GPIO_PINS: usize = 30;

pub struct GpioManager {
    pins: [Option<ManagedPin>; NUM_GPIO_PINS],
}

impl GpioManager {
    pub fn new(hal_pins: Pins) -> Self {
        let mut pins_array: [Option<ManagedPin>; NUM_GPIO_PINS] = core::array::from_fn(|_| None);
        pins_array[0]  = Some(ManagedPin::new(hal_pins.gpio0.into_dyn_pin(), 0));
        pins_array[1]  = Some(ManagedPin::new(hal_pins.gpio1.into_dyn_pin(), 1));
        pins_array[2]  = Some(ManagedPin::new(hal_pins.gpio2.into_dyn_pin(), 2));
        pins_array[3]  = Some(ManagedPin::new(hal_pins.gpio3.into_dyn_pin(), 3));
        pins_array[4]  = Some(ManagedPin::new(hal_pins.gpio4.into_dyn_pin(), 4));
        pins_array[5]  = Some(ManagedPin::new(hal_pins.gpio5.into_dyn_pin(), 5));
        pins_array[6]  = Some(ManagedPin::new(hal_pins.gpio6.into_dyn_pin(), 6));
        pins_array[7]  = Some(ManagedPin::new(hal_pins.gpio7.into_dyn_pin(), 7));
        pins_array[8]  = Some(ManagedPin::new(hal_pins.gpio8.into_dyn_pin(), 8));
        pins_array[9]  = Some(ManagedPin::new(hal_pins.gpio9.into_dyn_pin(), 9));
        pins_array[10] = Some(ManagedPin::new(hal_pins.gpio10.into_dyn_pin(), 10));
        pins_array[11] = Some(ManagedPin::new(hal_pins.gpio11.into_dyn_pin(), 11));
        pins_array[12] = Some(ManagedPin::new(hal_pins.gpio12.into_dyn_pin(), 12));
        pins_array[13] = Some(ManagedPin::new(hal_pins.gpio13.into_dyn_pin(), 13));
        pins_array[14] = Some(ManagedPin::new(hal_pins.gpio14.into_dyn_pin(), 14));
        pins_array[15] = Some(ManagedPin::new(hal_pins.gpio15.into_dyn_pin(), 15));
        pins_array[16] = Some(ManagedPin::new(hal_pins.gpio16.into_dyn_pin(), 16));
        pins_array[17] = Some(ManagedPin::new(hal_pins.gpio17.into_dyn_pin(), 17));
        pins_array[18] = Some(ManagedPin::new(hal_pins.gpio18.into_dyn_pin(), 18));
        pins_array[19] = Some(ManagedPin::new(hal_pins.gpio19.into_dyn_pin(), 19));
        pins_array[20] = Some(ManagedPin::new(hal_pins.gpio20.into_dyn_pin(), 20));
        pins_array[21] = Some(ManagedPin::new(hal_pins.gpio21.into_dyn_pin(), 21));
        pins_array[22] = Some(ManagedPin::new(hal_pins.gpio22.into_dyn_pin(), 22));
        pins_array[23] = Some(ManagedPin::new(hal_pins.gpio23.into_dyn_pin(), 23));
        pins_array[24] = Some(ManagedPin::new(hal_pins.gpio24.into_dyn_pin(), 24));
        pins_array[25] = Some(ManagedPin::new(hal_pins.gpio25.into_dyn_pin(), 25));
        pins_array[26] = Some(ManagedPin::new(hal_pins.gpio26.into_dyn_pin(), 26));
        pins_array[27] = Some(ManagedPin::new(hal_pins.gpio27.into_dyn_pin(), 27));
        pins_array[28] = Some(ManagedPin::new(hal_pins.gpio28.into_dyn_pin(), 28));
        pins_array[29] = Some(ManagedPin::new(hal_pins.gpio29.into_dyn_pin(), 29));
        Self { pins: pins_array }
    }

    fn with_pin_mut<F, R>(&mut self, pin_id: u8, func: F) -> Result<R, &'static str>
    where F: FnOnce(&mut ManagedPin) -> Result<R, DynPinModeError>,
    {
        if pin_id as usize >= NUM_GPIO_PINS { return Err("Invalid pin_id"); }
        match self.pins[pin_id as usize].as_mut() {
            Some(managed_pin) => func(managed_pin).map_err(|_e| "Pin operation failed (DynPinModeError)"),
            None => Err("Pin not available (should not happen if initialized)"),
        }
    }

    pub fn configure_pin_as_output(&mut self, pin_id: u8) -> Result<(), &'static str> { /* ... as before ... */ }
    pub fn write_pin_output(&mut self, pin_id: u8, high: bool) -> Result<(), &'static str> { /* ... as before ... */ }
    pub fn configure_pin_as_input(&mut self, pin_id: u8, pull: KlipperPullType) -> Result<(), &'static str> { /* ... as before ... */ }
    pub fn read_pin_input(&mut self, pin_id: u8) -> Result<bool, &'static str> { /* ... as before ... */ }
    pub fn release_pin(&mut self, pin_id: u8) -> Result<(), &'static str> { /* ... as before, sets to Disabled/FloatingInput ... */ }

    // Re-implementations from previous step to ensure they are present
    pub fn configure_pin_as_output(&mut self, pin_id: u8) -> Result<(), &'static str> {
        self.with_pin_mut(pin_id, |mp| {
            let temp_dyn_pin = core::mem::replace(&mut mp.pin, unsafe { core::mem::zeroed() });
            mp.pin = temp_dyn_pin.into_push_pull_output().into_dyn_pin();
            mp.current_mode = PinModeState::OutputPushPull;
            Ok(())
        })?;
        Ok(())
    }

    pub fn write_pin_output(&mut self, pin_id: u8, high: bool) -> Result<(), &'static str> {
        self.with_pin_mut(pin_id, |mp| {
            if mp.current_mode != PinModeState::OutputPushPull {
                return Err(DynPinModeError::InvalidState);
            }
            if high { mp.pin.set_high() } else { mp.pin.set_low() }
        })
    }

    pub fn configure_pin_as_input(&mut self, pin_id: u8, pull: KlipperPullType) -> Result<(), &'static str> {
        self.with_pin_mut(pin_id, |mp| {
            let temp_dyn_pin = core::mem::replace(&mut mp.pin, unsafe { core::mem::zeroed() });
            mp.pin = match pull {
                KlipperPullType::Up => temp_dyn_pin.into_pull_up_input().into_dyn_pin(),
                KlipperPullType::Down => temp_dyn_pin.into_pull_down_input().into_dyn_pin(),
                KlipperPullType::Floating => temp_dyn_pin.into_floating_input().into_dyn_pin(),
            };
            mp.current_mode = match pull {
                KlipperPullType::Up => PinModeState::InputPullUp,
                KlipperPullType::Down => PinModeState::InputPullDown,
                KlipperPullType::Floating => PinModeState::InputFloating,
            };
            Ok(())
        })?;
        Ok(())
    }

    pub fn read_pin_input(&mut self, pin_id: u8) -> Result<bool, &'static str> {
        self.with_pin_mut(pin_id, |mp| {
            match mp.current_mode {
                PinModeState::InputFloating | PinModeState::InputPullUp | PinModeState::InputPullDown => {
                    mp.pin.is_high()
                }
                _ => Err(DynPinModeError::InvalidState),
            }
        })
    }

    pub fn release_pin(&mut self, pin_id: u8) -> Result<(), &'static str> {
        self.with_pin_mut(pin_id, |mp| {
            let temp_dyn_pin = core::mem::replace(&mut mp.pin, unsafe { core::mem::zeroed() });
            mp.pin = temp_dyn_pin.into_floating_input().into_dyn_pin();
            mp.current_mode = PinModeState::Disabled;
            Ok(())
        })?;
        Ok(())
    }


    // --- ADC Pin Handling ---
    // AdcCapablePin enum definition remains as before.
    pub enum AdcCapablePin {
        Gpio26(gpio::Pin<gpio::bank0::Gpio26, gpio::FloatingInput>),
        Gpio27(gpio::Pin<gpio::bank0::Gpio27, gpio::FloatingInput>),
        Gpio28(gpio::Pin<gpio::bank0::Gpio28, gpio::FloatingInput>),
        Gpio29(gpio::Pin<gpio::bank0::Gpio29, gpio::FloatingInput>),
    }

    /// Attempts to take an ADC-capable pin from the manager.
    /// If successful, the pin's slot in the manager becomes `None`, and the typed pin is returned.
    /// The caller is responsible for calling `release_adc_pin_by_id` to return it.
    /// The `unsafe` block for pin reconstruction remains for this phase.
    pub fn take_pin_for_adc(&mut self, pin_id: u8) -> Result<AdcCapablePin, &'static str> {
        if !(26..=29).contains(&pin_id) {
            return Err("Pin is not ADC capable (must be GPIO26-29)");
        }

        let managed_pin_slot = &mut self.pins[pin_id as usize];

        if let Some(managed_pin) = managed_pin_slot.as_ref() {
            // Check current mode before taking
            match managed_pin.current_mode {
                PinModeState::Disabled | PinModeState::InputFloating => {
                    // Mode is compatible, proceed to take.
                }
                PinModeState::FunctionAdc => {
                    // This could mean it's already taken for ADC by another part of the code,
                    // or it was released without state reset. For safety, error out.
                    return Err("Pin already marked as FunctionAdc; release first or check for concurrent use.");
                }
                _ => {
                    return Err("Pin in incompatible mode for ADC");
                }
            }
        } else {
            return Err("Pin slot is already empty (pin taken and not released?)");
        }

        // If we reach here, the pin exists and is in a compatible mode. Now take it.
        // The `take()` here removes the ManagedPin from the array, leaving None.
        if let Some(mut taken_managed_pin) = managed_pin_slot.take() {
            // Perform the unsafe reconstruction to get the typed pin.
            let typed_adc_pin_result = match pin_id {
                26 => Ok(AdcCapablePin::Gpio26(unsafe { gpio::Pin::new(gpio::bank0::Gpio26::ID) }.into_floating_input())),
                27 => Ok(AdcCapablePin::Gpio27(unsafe { gpio::Pin::new(gpio::bank0::Gpio27::ID) }.into_floating_input())),
                28 => Ok(AdcCapablePin::Gpio28(unsafe { gpio::Pin::new(gpio::bank0::Gpio28::ID) }.into_floating_input())),
                29 => Ok(AdcCapablePin::Gpio29(unsafe { gpio::Pin::new(gpio::bank0::Gpio29::ID) }.into_floating_input())),
                _ => unreachable!("ADC pin_id validation failed earlier or logic error"),
            };

            match typed_adc_pin_result {
                Ok(typed_pin) => {
                    // Update the state of the (now conceptually separate) ManagedPin before it's dropped.
                    // Or, rather, the GpioManager no longer holds this ManagedPin.
                    // The state tracking of "FunctionAdc" is implicit by it being taken.
                    // When release_adc_pin_by_id is called, a *new* ManagedPin in Disabled state is created.
                    defmt::debug!("GpioManager: Pin {} taken for ADC. Slot is now None.", pin_id);
                    Ok(typed_pin)
                }
                Err(e) => {
                    // If reconstruction failed (though unsafe new shouldn't directly error here like this)
                    // put the original ManagedPin back.
                    *managed_pin_slot = Some(taken_managed_pin);
                    Err(e) // Should be a specific error from the match if one branch failed.
                }
            }
        } else {
            // This case should have been caught by the `is_some()` check at the start of the function,
            // but as a fallback.
            Err("Failed to take ManagedPin from slot (was None unexpectedly)")
        }
    }

    // release_adc_pin is renamed to release_adc_pin_by_id and modified later.
    // For now, ensure the old release_adc_pin is correctly updated if it was used.
    // The plan is to use release_adc_pin_by_id.
    // Removing the old `release_adc_pin` that took `AdcCapablePin`.
    // pub fn release_adc_pin(&mut self, pin_id: u8, _adc_capable_pin: AdcCapablePin) -> Result<(), &'static str> { /* ... old ... */ }

    /// Releases a pin previously taken for ADC use by its ID, returning it to the manager in a Disabled state.
    /// This reconstructs a default DynPin for the slot.
    pub fn release_adc_pin_by_id(&mut self, pin_id: u8) -> Result<(), &'static str> {
        if !(26..=29).contains(&pin_id) { // Ensure it's an ADC pin ID being released this way
            return Err("Invalid pin_id for ADC release (not 26-29)");
        }
        if pin_id as usize >= NUM_GPIO_PINS { // General bounds check
            return Err("Invalid pin_id (out of bounds)");
        }

        if self.pins[pin_id as usize].is_some() {
            defmt::warn!("GpioManager: Attempted to release pin {} (ADC) which was not None (already present). State: {:?}", pin_id, self.pins[pin_id as usize].as_ref().unwrap().current_mode);
            return Err("Pin slot not empty; was it actually taken by take_pin_for_adc, or already released?");
        }

        // Reconstruct a default DynPin for this pin_id and set its state to Disabled.
        let default_dyn_pin = match pin_id {
            26 => unsafe { gpio::Pin::new(gpio::bank0::Gpio26::ID) }.into_floating_input().into_dyn_pin(),
            27 => unsafe { gpio::Pin::new(gpio::bank0::Gpio27::ID) }.into_floating_input().into_dyn_pin(),
            28 => unsafe { gpio::Pin::new(gpio::bank0::Gpio28::ID) }.into_floating_input().into_dyn_pin(),
            29 => unsafe { gpio::Pin::new(gpio::bank0::Gpio29::ID) }.into_floating_input().into_dyn_pin(),
            _ => unreachable!("ADC pin_id validation failed earlier or logic error during release"),
        };

        self.pins[pin_id as usize] = Some(ManagedPin {
            pin: default_dyn_pin,
            current_mode: PinModeState::Disabled,
            id: pin_id,
        });
        defmt::debug!("GpioManager: Pin {} released from ADC and set to Disabled.", pin_id);
        Ok(())
    }


    // --- PWM Pin Handling ---
    // PwmCapablePin enum definition remains as before.
    #[allow(missing_docs)]
    pub enum PwmCapablePin {
        Gpio0(gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionPwm>), Gpio1(gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionPwm>),
        Gpio2(gpio::Pin<gpio::bank0::Gpio2, gpio::FunctionPwm>), Gpio3(gpio::Pin<gpio::bank0::Gpio3, gpio::FunctionPwm>),
        Gpio4(gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionPwm>), Gpio5(gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionPwm>),
        Gpio6(gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionPwm>), Gpio7(gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionPwm>),
        Gpio8(gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionPwm>), Gpio9(gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionPwm>),
        Gpio10(gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionPwm>), Gpio11(gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionPwm>),
        Gpio12(gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionPwm>), Gpio13(gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionPwm>),
        Gpio14(gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionPwm>), Gpio15(gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionPwm>),
        Gpio16(gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionPwm>), Gpio17(gpio::Pin<gpio::bank0::Gpio17, gpio::FunctionPwm>),
        Gpio18(gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionPwm>), Gpio19(gpio::Pin<gpio::bank0::Gpio19, gpio::FunctionPwm>),
        Gpio20(gpio::Pin<gpio::bank0::Gpio20, gpio::FunctionPwm>), Gpio21(gpio::Pin<gpio::bank0::Gpio21, gpio::FunctionPwm>),
        Gpio22(gpio::Pin<gpio::bank0::Gpio22, gpio::FunctionPwm>), Gpio23(gpio::Pin<gpio::bank0::Gpio23, gpio::FunctionPwm>),
        Gpio24(gpio::Pin<gpio::bank0::Gpio24, gpio::FunctionPwm>), Gpio25(gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionPwm>),
        Gpio26(gpio::Pin<gpio::bank0::Gpio26, gpio::FunctionPwm>), Gpio27(gpio::Pin<gpio::bank0::Gpio27, gpio::FunctionPwm>),
        Gpio28(gpio::Pin<gpio::bank0::Gpio28, gpio::FunctionPwm>), Gpio29(gpio::Pin<gpio::bank0::Gpio29, gpio::FunctionPwm>),
    }

    pub fn configure_pin_for_pwm(&mut self, pin_id: u8) -> Result<(), &'static str> { /* ... as before ... */ }
    // Implementation of configure_pin_for_pwm (copied from previous state)
    pub fn configure_pin_for_pwm(&mut self, pin_id: u8) -> Result<(), &'static str> {
        if pin_id as usize >= NUM_GPIO_PINS { return Err("Invalid pin_id for PWM"); }
        self.with_pin_mut(pin_id, |mp| {
            match mp.current_mode {
                PinModeState::Disabled | PinModeState::InputFloating | PinModeState::OutputPushPull => {
                    mp.current_mode = PinModeState::FunctionPwm;
                    defmt::debug!("GpioManager: Pin {} state set for PWM.", pin_id);
                    Ok(())
                }
                PinModeState::FunctionPwm => Ok(()),
                _ => { defmt::warn!("GpioManager: Pin {} in incompatible mode {:?} for PWM.", pin_id, mp.current_mode); Err(DynPinModeError::InvalidState) }
            }
        })
    }

    /// Attempts to take a pin and configure it for PWM.
    /// Uses unsafe new to reconstruct the typed pin.
    pub fn take_pin_for_pwm(&mut self, pin_id: u8) -> Result<PwmCapablePin, &'static str> {
        if pin_id as usize >= NUM_GPIO_PINS { return Err("Invalid pin_id"); }

        let managed_pin_entry = &mut self.pins[pin_id as usize];
        if managed_pin_entry.is_none() { return Err("ManagedPin is None, cannot take for PWM"); }

        let mut managed_pin = managed_pin_entry.take().unwrap(); // Take ownership of ManagedPin

        match managed_pin.current_mode {
            PinModeState::Disabled | PinModeState::InputFloating | PinModeState::OutputPushPull | PinModeState::FunctionPwm => {
                // It's okay to reconfigure from these states, or if already PWM.
                // The `unsafe` reconstruction of the typed pin.
                // This assumes that the underlying PAC tokens are implicitly available again.
                // This is a significant HACK for rp2040-hal's typed GPIO system.
                let typed_pwm_pin_result = match pin_id {
                    0  => Ok(PwmCapablePin::Gpio0(unsafe { gpio::Pin::new(gpio::bank0::Gpio0::ID) }.into_function())),
                    1  => Ok(PwmCapablePin::Gpio1(unsafe { gpio::Pin::new(gpio::bank0::Gpio1::ID) }.into_function())),
                    2  => Ok(PwmCapablePin::Gpio2(unsafe { gpio::Pin::new(gpio::bank0::Gpio2::ID) }.into_function())),
                    3  => Ok(PwmCapablePin::Gpio3(unsafe { gpio::Pin::new(gpio::bank0::Gpio3::ID) }.into_function())),
                    4  => Ok(PwmCapablePin::Gpio4(unsafe { gpio::Pin::new(gpio::bank0::Gpio4::ID) }.into_function())),
                    5  => Ok(PwmCapablePin::Gpio5(unsafe { gpio::Pin::new(gpio::bank0::Gpio5::ID) }.into_function())),
                    6  => Ok(PwmCapablePin::Gpio6(unsafe { gpio::Pin::new(gpio::bank0::Gpio6::ID) }.into_function())),
                    7  => Ok(PwmCapablePin::Gpio7(unsafe { gpio::Pin::new(gpio::bank0::Gpio7::ID) }.into_function())),
                    8  => Ok(PwmCapablePin::Gpio8(unsafe { gpio::Pin::new(gpio::bank0::Gpio8::ID) }.into_function())),
                    9  => Ok(PwmCapablePin::Gpio9(unsafe { gpio::Pin::new(gpio::bank0::Gpio9::ID) }.into_function())),
                    10 => Ok(PwmCapablePin::Gpio10(unsafe { gpio::Pin::new(gpio::bank0::Gpio10::ID) }.into_function())),
                    11 => Ok(PwmCapablePin::Gpio11(unsafe { gpio::Pin::new(gpio::bank0::Gpio11::ID) }.into_function())),
                    12 => Ok(PwmCapablePin::Gpio12(unsafe { gpio::Pin::new(gpio::bank0::Gpio12::ID) }.into_function())),
                    13 => Ok(PwmCapablePin::Gpio13(unsafe { gpio::Pin::new(gpio::bank0::Gpio13::ID) }.into_function())),
                    14 => Ok(PwmCapablePin::Gpio14(unsafe { gpio::Pin::new(gpio::bank0::Gpio14::ID) }.into_function())),
                    15 => Ok(PwmCapablePin::Gpio15(unsafe { gpio::Pin::new(gpio::bank0::Gpio15::ID) }.into_function())),
                    16 => Ok(PwmCapablePin::Gpio16(unsafe { gpio::Pin::new(gpio::bank0::Gpio16::ID) }.into_function())),
                    17 => Ok(PwmCapablePin::Gpio17(unsafe { gpio::Pin::new(gpio::bank0::Gpio17::ID) }.into_function())),
                    18 => Ok(PwmCapablePin::Gpio18(unsafe { gpio::Pin::new(gpio::bank0::Gpio18::ID) }.into_function())),
                    19 => Ok(PwmCapablePin::Gpio19(unsafe { gpio::Pin::new(gpio::bank0::Gpio19::ID) }.into_function())),
                    20 => Ok(PwmCapablePin::Gpio20(unsafe { gpio::Pin::new(gpio::bank0::Gpio20::ID) }.into_function())),
                    21 => Ok(PwmCapablePin::Gpio21(unsafe { gpio::Pin::new(gpio::bank0::Gpio21::ID) }.into_function())),
                    22 => Ok(PwmCapablePin::Gpio22(unsafe { gpio::Pin::new(gpio::bank0::Gpio22::ID) }.into_function())),
                    23 => Ok(PwmCapablePin::Gpio23(unsafe { gpio::Pin::new(gpio::bank0::Gpio23::ID) }.into_function())),
                    24 => Ok(PwmCapablePin::Gpio24(unsafe { gpio::Pin::new(gpio::bank0::Gpio24::ID) }.into_function())),
                    25 => Ok(PwmCapablePin::Gpio25(unsafe { gpio::Pin::new(gpio::bank0::Gpio25::ID) }.into_function())),
                    26 => Ok(PwmCapablePin::Gpio26(unsafe { gpio::Pin::new(gpio::bank0::Gpio26::ID) }.into_function())),
                    27 => Ok(PwmCapablePin::Gpio27(unsafe { gpio::Pin::new(gpio::bank0::Gpio27::ID) }.into_function())),
                    28 => Ok(PwmCapablePin::Gpio28(unsafe { gpio::Pin::new(gpio::bank0::Gpio28::ID) }.into_function())),
                    29 => Ok(PwmCapablePin::Gpio29(unsafe { gpio::Pin::new(gpio::bank0::Gpio29::ID) }.into_function())),
                    _ => Err("Invalid pin_id for PwmCapablePin reconstruction"), // Should be caught by initial check
                };

                match typed_pwm_pin_result {
                    Ok(typed_pin) => {
                        managed_pin.current_mode = PinModeState::FunctionPwm;
                        // The original DynPin in managed_pin is now conceptually "gone" because we've
                        // unsafely reconstructed a typed pin. We put the ManagedPin back with updated state.
                        // The DynPin field in it is stale if we don't update it from typed_pin.
                        // For this take model, the caller gets the typed_pin, and this slot becomes empty.
                        // So, *managed_pin_entry remains None (because we .take() earlier).
                        defmt::debug!("GpioManager: Pin {} taken for PWM. Manager slot is now None.", pin_id);
                        Ok(typed_pin)
                    }
                    Err(e) => {
                        *managed_pin_entry = Some(managed_pin); // Put original back on error
                        Err(e)
                    }
                }
            }
            _ => {
                *managed_pin_entry = Some(managed_pin); // Put original back
                Err("Pin in incompatible mode for PWM")
            }
        }
    }

    /// Releases a pin previously taken for PWM, returning it to the manager in a Disabled state.
    /// The `_pwm_pin` argument is consumed by the HAL when setting up PWM, so it's mostly a type-state token here.
    pub fn release_pwm_pin_by_id(&mut self, pin_id: u8) -> Result<(), &'static str> {
        if pin_id as usize >= NUM_GPIO_PINS {
            return Err("Invalid pin_id for release");
        }

        // We expect the slot to be None because take_pin_for_pwm should have emptied it.
        if self.pins[pin_id as usize].is_some() {
            // This might indicate the pin was taken, but then the consuming operation failed
            // before it could be "emptied" from the manager's perspective, or it was never taken.
            // Or, it means take_pin_for_pwm logic needs adjustment to always None-out the slot.
            // For now, if it's Some, we'll assume we are resetting an existing ManagedPin.
            defmt::warn!("release_pwm_pin_by_id: Pin {} slot was not empty. Resetting existing.", pin_id);
        }

        // Reconstruct a default DynPin for this pin_id and set its state to Disabled.
        // This uses the same unsafe hack as take_pin_for_pwm to get a typed pin,
        // then immediately degrades it to DynPin in a default state.
        let default_dyn_pin = match pin_id {
            0  => unsafe { gpio::Pin::new(gpio::bank0::Gpio0::ID) }.into_floating_input().into_dyn_pin(),
            1  => unsafe { gpio::Pin::new(gpio::bank0::Gpio1::ID) }.into_floating_input().into_dyn_pin(),
            // ... Add all GPIOs 2-29 similarly ...
            2  => unsafe { gpio::Pin::new(gpio::bank0::Gpio2::ID) }.into_floating_input().into_dyn_pin(),
            3  => unsafe { gpio::Pin::new(gpio::bank0::Gpio3::ID) }.into_floating_input().into_dyn_pin(),
            4  => unsafe { gpio::Pin::new(gpio::bank0::Gpio4::ID) }.into_floating_input().into_dyn_pin(),
            5  => unsafe { gpio::Pin::new(gpio::bank0::Gpio5::ID) }.into_floating_input().into_dyn_pin(),
            6  => unsafe { gpio::Pin::new(gpio::bank0::Gpio6::ID) }.into_floating_input().into_dyn_pin(),
            7  => unsafe { gpio::Pin::new(gpio::bank0::Gpio7::ID) }.into_floating_input().into_dyn_pin(),
            8  => unsafe { gpio::Pin::new(gpio::bank0::Gpio8::ID) }.into_floating_input().into_dyn_pin(),
            9  => unsafe { gpio::Pin::new(gpio::bank0::Gpio9::ID) }.into_floating_input().into_dyn_pin(),
            10 => unsafe { gpio::Pin::new(gpio::bank0::Gpio10::ID) }.into_floating_input().into_dyn_pin(),
            11 => unsafe { gpio::Pin::new(gpio::bank0::Gpio11::ID) }.into_floating_input().into_dyn_pin(),
            12 => unsafe { gpio::Pin::new(gpio::bank0::Gpio12::ID) }.into_floating_input().into_dyn_pin(),
            13 => unsafe { gpio::Pin::new(gpio::bank0::Gpio13::ID) }.into_floating_input().into_dyn_pin(),
            14 => unsafe { gpio::Pin::new(gpio::bank0::Gpio14::ID) }.into_floating_input().into_dyn_pin(),
            15 => unsafe { gpio::Pin::new(gpio::bank0::Gpio15::ID) }.into_floating_input().into_dyn_pin(),
            16 => unsafe { gpio::Pin::new(gpio::bank0::Gpio16::ID) }.into_floating_input().into_dyn_pin(),
            17 => unsafe { gpio::Pin::new(gpio::bank0::Gpio17::ID) }.into_floating_input().into_dyn_pin(),
            18 => unsafe { gpio::Pin::new(gpio::bank0::Gpio18::ID) }.into_floating_input().into_dyn_pin(),
            19 => unsafe { gpio::Pin::new(gpio::bank0::Gpio19::ID) }.into_floating_input().into_dyn_pin(),
            20 => unsafe { gpio::Pin::new(gpio::bank0::Gpio20::ID) }.into_floating_input().into_dyn_pin(),
            21 => unsafe { gpio::Pin::new(gpio::bank0::Gpio21::ID) }.into_floating_input().into_dyn_pin(),
            22 => unsafe { gpio::Pin::new(gpio::bank0::Gpio22::ID) }.into_floating_input().into_dyn_pin(),
            23 => unsafe { gpio::Pin::new(gpio::bank0::Gpio23::ID) }.into_floating_input().into_dyn_pin(),
            24 => unsafe { gpio::Pin::new(gpio::bank0::Gpio24::ID) }.into_floating_input().into_dyn_pin(),
            25 => unsafe { gpio::Pin::new(gpio::bank0::Gpio25::ID) }.into_floating_input().into_dyn_pin(),
            26 => unsafe { gpio::Pin::new(gpio::bank0::Gpio26::ID) }.into_floating_input().into_dyn_pin(),
            27 => unsafe { gpio::Pin::new(gpio::bank0::Gpio27::ID) }.into_floating_input().into_dyn_pin(),
            28 => unsafe { gpio::Pin::new(gpio::bank0::Gpio28::ID) }.into_floating_input().into_dyn_pin(),
            29 => unsafe { gpio::Pin::new(gpio::bank0::Gpio29::ID) }.into_floating_input().into_dyn_pin(),
            _ => return Err("Invalid pin_id for default DynPin reconstruction"),
        };

        self.pins[pin_id as usize] = Some(ManagedPin {
            pin: default_dyn_pin,
            current_mode: PinModeState::Disabled, // Mark as disabled and ready for reuse
            id: pin_id,
        });
        defmt::debug!("GpioManager: Pin {} released (by ID) from PWM and set to Disabled.", pin_id);
        Ok(())
    }
}
