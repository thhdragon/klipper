// src/rp2040_hal_impl/timer.rs
#![cfg_attr(not(test), no_std)]

use rp2040_hal::timer::Alarm;
use crate::hal::{StepEventResult, Timer as HalTimer}; // Klipper HAL Timer trait
use defmt; // For logging errors

/// A timer implementation for RP2040 using one of its hardware alarms.
///
/// `A` is a specific alarm type from `rp2040_hal::timer` (e.g., `Alarm0`, `Alarm1`).
pub struct Rp2040Timer<A: Alarm> {
    /// The hardware alarm instance.
    pub(super) alarm: A,

    /// The callback function to invoke when the timer expires.
    pub(super) callback: Option<fn(&mut Self) -> StepEventResult>,

    /// The currently scheduled waketime (raw timer value).
    pub(super) scheduled_waketime: u32,
    /// Flag to control if this timer's own hardware interrupt should be enabled when armed.
    /// If false, it's expected to be driven by an external scheduler.
    pub(super) hw_irq_enabled: bool,
}

impl<A: Alarm> Rp2040Timer<A> {
    /// Sets whether this timer's dedicated hardware interrupt should be active.
    /// If `false`, the timer relies on an external scheduler to call its callback.
    pub fn set_hw_irq_active(&mut self, active: bool) {
        self.hw_irq_enabled = active;
        if !active {
            // If disabling, immediately try to disable the hardware interrupt too.
            self.alarm.disable_interrupt();
        }
        // If enabling, arm_hardware_alarm will handle enabling it when next scheduled.
    }

    /// Retrieves the stored callback function pointer.
    /// This is intended for use by a scheduler that needs to invoke the callback directly.
    pub fn get_callback(&self) -> Option<fn(&mut Self) -> StepEventResult> {
        self.callback
    }

    /// Helper to arm the timer's hardware alarm and enable its interrupt.
    /// This is called by `set_waketime`.
    fn arm_hardware_alarm(&mut self, waketime: u32) {
        self.alarm.clear_interrupt();
        match self.alarm.schedule(waketime) {
            Ok(()) => {
                if self.hw_irq_enabled {
                    self.alarm.enable_interrupt();
                    // defmt::trace!("Rp2040Timer: Armed HW alarm (IRQ enabled) for {}", waketime);
                } else {
                    self.alarm.disable_interrupt(); // Ensure it's disabled if not active
                    // defmt::trace!("Rp2040Timer: Armed HW alarm (IRQ disabled) for {}", waketime);
                }
                self.scheduled_waketime = waketime;
            }
            Err(_e) => {
                // If schedule fails (e.g. time in past, though rp2040-hal might handle this),
                // then the interrupt won't be enabled for this attempt.
                // Log the error. The scheduled_waketime will reflect the intended time.
                defmt::error!("Rp2040Timer: Failed to schedule hardware alarm for waketime: {}. Error: {:?}", waketime, defmt::Debug2Format(&_e));
                // We still update scheduled_waketime to reflect the *intent* to wake at this time,
                // even if the hardware arming failed. The caller (scheduler or direct user)
                // might retry or handle this.
                self.scheduled_waketime = waketime;
            }
        }
    }

    /// Helper to disarm the timer's hardware alarm.
    #[allow(dead_code)] // May be useful later
    fn disarm_hardware_alarm(&mut self) {
        self.alarm.disable_interrupt();
        self.alarm.clear_interrupt();
        // defmt::trace!("Rp2040Timer: Disarmed hardware alarm.");
    }

    /// Called from this timer's specific hardware IRQ handler (e.g., TIMER_IRQ_0 for Alarm0).
    /// This method checks if its specific alarm fired and, if so, invokes the callback.
    pub fn on_interrupt(&mut self) {
        if self.alarm.finished() {
            self.alarm.clear_interrupt();
            // defmt::debug!("Rp2040Timer: Hardware interrupt handled for waketime: {}", self.scheduled_waketime);
            if let Some(callback_fn) = self.callback {
                (callback_fn)(self);
            }
        }
    }
}

impl<A: Alarm> HalTimer for Rp2040Timer<A> {
    fn new(mut alarm: A, callback: fn(&mut Self) -> StepEventResult) -> Self {
        alarm.disable_interrupt(); // Ensure interrupt is initially disabled by default
        alarm.clear_interrupt();   // Clear any pending state
        Self {
            alarm,
            callback: Some(callback),
            scheduled_waketime: 0,
            hw_irq_enabled: true, // Default to hardware IRQ being active
        }
    }

    fn get_waketime(&self) -> u32 {
        self.scheduled_waketime
    }

    fn set_waketime(&mut self, waketime: u32) {
        // This method will arm the underlying hardware alarm.
        // If this timer is managed by a scheduler, the scheduler might decide
        // not to enable the hardware interrupt at the NVIC level, relying on its
        // own master timer to dispatch this timer's callback.
        self.arm_hardware_alarm(waketime);
    }
}
