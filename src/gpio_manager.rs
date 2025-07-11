// src/gpio_manager.rs
#![cfg_attr(not(test), no_std)]

use rp2040_hal::gpio::dynpin::{DynPin, DynPinModeError};
use rp2040_hal::gpio::{self, Pins, FunctionSio, SioConfig, PullType as HalPullType};
use crate::hal::PullType as KlipperPullType; // Our HAL enum
use defmt::Format;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum PinModeState {
    Disabled,
    InputFloating,
    InputPullUp,
    InputPullDown,
    OutputPushPull,
}

impl Default for PinModeState {
    fn default() -> Self {
        PinModeState::Disabled
    }
}

#[derive(Format)]
pub struct ManagedPin {
    #[defmt(Debug2Format)]
    pub(crate) pin: DynPin, // The actual DynPin object
    pub(crate) current_mode: PinModeState,
    pub(crate) id: u8,
}

impl ManagedPin {
    pub fn new(pin: DynPin, id: u8) -> Self {
        Self {
            pin, // Initially, this DynPin is in its reset state (e.g. FloatingInput or Disabled)
            current_mode: PinModeState::Disabled, // Manager considers it Disabled until configured
            id,
        }
    }
    pub fn id(&self) -> u8 { self.id }
}

pub const NUM_GPIO_PINS: usize = 30;

pub struct GpioManager {
    pins: [Option<ManagedPin>; NUM_GPIO_PINS],
    // We need SIO and RESETS to reconfigure pins if we want to change their function fundamentally
    // beyond what DynPin.into_mode() offers. However, DynPin.into_mode() should be sufficient
    // for SIO functions (input/output).
    // For now, let's assume DynPin's own methods are enough.
}

impl GpioManager {
    pub fn new(hal_pins: Pins) -> Self {
        let mut pins_array: [Option<ManagedPin>; NUM_GPIO_PINS] = core::array::from_fn(|_| None);
        // Manually convert each pin. This is verbose but explicit.
        // The into_dyn_pin() consumes the typed pin.
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
    where
        F: FnOnce(&mut ManagedPin) -> Result<R, DynPinModeError>,
    {
        if pin_id as usize >= NUM_GPIO_PINS {
            return Err("Invalid pin_id");
        }
        match self.pins[pin_id as usize].as_mut() {
            Some(managed_pin) => {
                func(managed_pin).map_err(|_e| {
                    defmt::error!("Pin {} DynPin op failed: {:?}", pin_id, defmt::Debug2Format(&_e));
                    "Pin operation failed (DynPinModeError)"
                })
            }
            None => Err("Pin not available in manager (should not happen if initialized correctly)"),
        }
    }

    pub fn configure_pin_as_output(&mut self, pin_id: u8) -> Result<(), &'static str> {
        self.with_pin_mut(pin_id, |mp| {
            // into_push_pull_output consumes and returns a new DynPin.
            // We need to replace the one in ManagedPin.
            // This requires taking pin out, transforming, putting back.
            // This implies ManagedPin.pin should be Option<DynPin> or we use unsafe.
            // Or, DynPin's mode change methods return () and modify in place, if available.
            // rp2040-hal DynPin `into_X_output()` consumes `self` and returns `Pin<Self::Id, OutputMode>`
            // which can then be `into_dyn_pin()` again.

            // Temporarily take ownership of the DynPin to reconfigure it
            let temp_dyn_pin = core::mem::replace(&mut mp.pin, unsafe { core::mem::zeroed() }); // Placeholder
            mp.pin = temp_dyn_pin.into_push_pull_output().into_dyn_pin(); // New DynPin in output mode
            mp.current_mode = PinModeState::OutputPushPull;
            Ok(())
        })?; // The ? operator propagates the error if with_pin_mut or func fails
        Ok(())
    }

    pub fn write_pin_output(&mut self, pin_id: u8, high: bool) -> Result<(), &'static str> {
        self.with_pin_mut(pin_id, |mp| {
            if mp.current_mode != PinModeState::OutputPushPull {
                return Err(DynPinModeError::InvalidState); // Or our own error type
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
                _ => Err(DynPinModeError::InvalidState), // Not configured as input
            }
        })
    }

    pub fn release_pin(&mut self, pin_id: u8) -> Result<(), &'static str> {
        self.with_pin_mut(pin_id, |mp| {
            // To "release" a pin, we put it into a default safe state, e.g., Disabled/FloatingInput.
            // This involves reconfiguring it.
            let temp_dyn_pin = core::mem::replace(&mut mp.pin, unsafe { core::mem::zeroed() });
            mp.pin = temp_dyn_pin.into_floating_input().into_dyn_pin(); // Example default state
            // Or, if DynPin has a .disable() or .into_disabled_pin()
            // mp.pin = temp_dyn_pin.into_disabled().into_dyn_pin(); // Hypothetical
            mp.current_mode = PinModeState::Disabled; // Or InputFloating if that's the reset state
            Ok(())
        })?;
        Ok(())
    }

    // --- ADC Pin Handling ---

    // Helper enum to return a specifically typed GPIO pin for ADC conversion.
    // This is needed because Rp2040AdcChannel::new expects a typed Pin, not a DynPin.
    // Only ADC-capable pins are included.
    pub enum AdcCapablePin {
        Gpio26(gpio::Pin<gpio::bank0::Gpio26, gpio::FloatingInput>),
        Gpio27(gpio::Pin<gpio::bank0::Gpio27, gpio::FloatingInput>),
        Gpio28(gpio::Pin<gpio::bank0::Gpio28, gpio::FloatingInput>),
        Gpio29(gpio::Pin<gpio::bank0::Gpio29, gpio::FloatingInput>),
    }

    /// Attempts to configure and temporarily take an ADC-capable pin.
    /// The pin's state in GpioManager is set to FunctionAdc, and its DynPin is temporarily removed.
    /// The caller is responsible for calling `release_adc_pin` to return it.
    pub fn take_pin_for_adc(&mut self, pin_id: u8) -> Result<AdcCapablePin, &'static str> {
        if !(26..=29).contains(&pin_id) {
            return Err("Pin is not ADC capable");
        }

        let managed_pin_entry = &mut self.pins[pin_id as usize];
        if managed_pin_entry.is_none() {
            // This should not happen if GpioManager is initialized correctly,
            // means the ManagedPin was taken and not returned.
            return Err("Pin previously taken and not released (ManagedPin is None)");
        }

        // Take the ManagedPin out of the Option to work with it.
        // We'll put it back (or a new one if pin was consumed) later.
        let mut managed_pin = managed_pin_entry.take().unwrap();

        match managed_pin.current_mode {
            PinModeState::Disabled | PinModeState::InputFloating => {
                // Ok to configure for ADC.
                // We need to convert the DynPin back to its specific GpioX type, then into_floating_input().
                // This is the hard part with DynPin. DynPin.try_into_ συγκεκριμένος_pin() might work.
                // `DynPin::try_into_pin<GpioX, FloatingInput>()` is not a method.
                // `DynPin::into_floating_input()` returns a new DynPin.
                // We need `Pin<GpioX, FloatingInput>`.

                // This requires unsafe or a different way of storing pins if we need to recover typed pins.
                // For now, this is a conceptual block. The actual conversion is non-trivial.
                // defmt::error!("take_pin_for_adc: Converting DynPin back to typed GpioX for ADC is complex and not fully implemented.");
                // return Err("DynPin to typed GpioX conversion for ADC not implemented");

                // Simplification: Assume the DynPin can be directly used to create a typed pin for ADC
                // This is a placeholder for the complex conversion or a HAL feature.
                // The `into_floating_input()` method on `DynPin` returns another `DynPin`.
                // We need the specific `Pin<Gpio26, FloatingInput>`.
                // This means the original `Pins` struct is needed, or unsafe transmutation.

                // Let's assume we can "re-take" the specific pin from *somewhere* if we know its ID.
                // This completely bypasses the `DynPin` stored in `managed_pin.pin` for this operation,
                // which is not ideal but a pragmatic way forward for the ADC step if GpioManager
                // cannot easily yield the *typed* pin required by Rp2040AdcChannel.
                // This implies `GpioManager` is more of a state tracker than a resource owner here.

                // To make progress, we will assume that if a pin is marked `Disabled` or `InputFloating`,
                // we can "construct" the typed pin again. This is only safe if nothing else has
                // grabbed the raw PAC tokens for that pin.
                // This part of the code will be UNSAFE or require a different HAL abstraction.
                // For now, let's return a specific pin based on ID, assuming it's constructible.
                // This is a MAJOR simplification / HACK.

                let result_pin = match pin_id {
                    26 => Ok(AdcCapablePin::Gpio26(unsafe { gpio::Pin::new(gpio::bank0::Gpio26::ID) }.into_floating_input())),
                    27 => Ok(AdcCapablePin::Gpio27(unsafe { gpio::Pin::new(gpio::bank0::Gpio27::ID) }.into_floating_input())),
                    28 => Ok(AdcCapablePin::Gpio28(unsafe { gpio::Pin::new(gpio::bank0::Gpio28::ID) }.into_floating_input())),
                    29 => Ok(AdcCapablePin::Gpio29(unsafe { gpio::Pin::new(gpio::bank0::Gpio29::ID) }.into_floating_input())),
                    _ => unreachable!(), // Already checked by initial if
                };

                if result_pin.is_ok() {
                    managed_pin.current_mode = PinModeState::FunctionAdc;
                    // Put the managed_pin (with updated state but original DynPin) back.
                    // The DynPin is not actually used by this hacked path.
                    *managed_pin_entry = Some(managed_pin);
                } else {
                    // Failed to create, put original managed_pin back
                    *managed_pin_entry = Some(managed_pin);
                }
                result_pin
            }
            PinModeState::FunctionAdc => {
                 // Already in ADC mode, this might be an error or allow re-taking.
                 // For now, let's treat as error to prevent concurrent access issues.
                *managed_pin_entry = Some(managed_pin); // Put it back
                Err("Pin already configured for ADC and potentially in use")
            }
            _ => { // Pin is in another mode (Output, PullUp/Down Input, etc.)
                *managed_pin_entry = Some(managed_pin); // Put it back
                Err("Pin in incompatible mode for ADC")
            }
        }
    }

    /// Releases a pin that was taken for ADC use, returning it to a Disabled state.
    pub fn release_adc_pin(&mut self, pin_id: u8, _adc_capable_pin: AdcCapablePin) -> Result<(), &'static str> {
        // The `_adc_capable_pin` is consumed, its underlying raw pin parts are now "back" with the PAC tokens conceptually.
        // We just need to update the state in GpioManager.
        if pin_id as usize >= NUM_GPIO_PINS {
            return Err("Invalid pin_id");
        }

        match self.pins[pin_id as usize].as_mut() {
            Some(managed_pin) => {
                if managed_pin.current_mode == PinModeState::FunctionAdc {
                    // Revert the DynPin to a default state (e.g., floating input)
                    // This assumes the original DynPin in managed_pin is still valid for this.
                    let temp_dyn_pin = core::mem::replace(&mut managed_pin.pin, unsafe { core::mem::zeroed() });
                    managed_pin.pin = temp_dyn_pin.into_floating_input().into_dyn_pin();
                    managed_pin.current_mode = PinModeState::Disabled; // Or InputFloating
                    Ok(())
                } else {
                    Err("Pin was not in FunctionAdc mode when release_adc_pin called")
                }
            }
            None => Err("Pin not found in manager to release (was it taken for ADC?)"),
        }
    }
}
