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
}
