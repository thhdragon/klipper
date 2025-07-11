// src/rp2040_hal_impl/timer.rs
#![cfg_attr(not(test), no_std)]

use rp2040_hal::timer::Alarm;
use crate::hal::StepEventResult; // Assuming StepEventResult is in crate::hal

/// A timer implementation for RP2040 using one of its hardware alarms.
///
/// `A` is a specific alarm type from `rp2040_hal::timer` (e.g., `Alarm0`, `Alarm1`).
pub struct Rp2040Timer<A: Alarm> {
    /// The hardware alarm instance.
    pub(super) alarm: A, // Made pub(super) for access from interrupt setup if needed in rp2040_hal_impl

    /// The callback function to invoke when the timer expires.
    /// The callback receives a mutable reference to this timer instance.
    pub(super) callback: Option<fn(&mut Self) -> StepEventResult>,

    /// The currently scheduled waketime (raw timer value).
    pub(super) scheduled_waketime: u32,
}

use crate::hal::Timer as HalTimer;
use rp2040_hal::pac; // For Interrupt enum
use cortex_m::peripheral::NVIC;

// For simplicity in this first pass, we are not using global static Mutex<RefCell<Option<...>>>
// storage for the Rp2040Timer instances within this file. Instead, the `new` method will return
// an instance, and the caller (e.g., main.rs) will be responsible for managing its lifetime
// and accessibility if it needs to be called from an interrupt handler that it sets up.
// This means the interrupt handler setup will be somewhat manual for the user of this Timer.
// A more integrated solution would manage this internally.

impl<A: Alarm> Rp2040Timer<A> {
    /// Helper to arm the timer. Not part of the trait.
    fn arm(&mut self, waketime: u32) {
        self.alarm.clear_interrupt(); // Clear any pending interrupt
        self.alarm.schedule(waketime).unwrap(); // Use unwrap for simplicity, real code might handle error
        self.alarm.enable_interrupt();
        self.scheduled_waketime = waketime;
    }

    /// Helper to disarm the timer. Not part of the trait.
    #[allow(dead_code)] // May be useful later
    fn disarm(&mut self) {
        self.alarm.disable_interrupt();
        self.alarm.clear_interrupt();
    }

    // This method would be called from the specific timer IRQ handler.
    // The handler needs a way to get `&mut self`.
    // This is the part that requires careful global/static management if we have multiple timers.
    // For now, this function is here, but linking it to an IRQ that can call it is complex.
    pub fn on_interrupt(&mut self) {
        if self.alarm.finished() { // Check if alarm actually fired
            self.alarm.clear_interrupt(); // Important: clear IRQ before callback

            if let Some(callback_fn) = self.callback {
                // Potentially re-arm or stop based on callback result.
                // For now, we assume callback might re-schedule by calling self.set_waketime().
                // If not, the timer remains disarmed until set_waketime is called again.
                // The trait doesn't specify auto-rearming.
                (callback_fn)(self);
            }
        }
    }
}

impl<A: Alarm> HalTimer for Rp2040Timer<A> {
    /// Creates a new timer instance.
    ///
    /// The caller is responsible for:
    /// 1. Ensuring the associated interrupt (e.g., TIMER_IRQ_0 for Alarm0) is unmasked in the NVIC.
    /// 2. Setting up the interrupt handler function to call `timer_instance.on_interrupt()`.
    ///
    /// `alarm`: The specific RP2040 HAL Alarm instance (e.g., `timer.alarm_0()`).
    /// `callback`: The function to call when the timer expires.
    fn new(mut alarm: A, callback: fn(&mut Self) -> StepEventResult) -> Self {
        // Ensure interrupt is disabled initially; set_waketime will enable it.
        alarm.disable_interrupt();
        alarm.clear_interrupt();

        Self {
            alarm,
            callback: Some(callback),
            scheduled_waketime: 0, // Initialized to 0, effectively unscheduled
        }
    }

    fn get_waketime(&self) -> u32 {
        self.scheduled_waketime
    }

    fn set_waketime(&mut self, waketime: u32) {
        self.arm(waketime);
    }
}

// To make this work with interrupts, we need:
// 1. Static storage for Rp2040Timer instances (or some way for ISR to get a mutable ref).
// 2. Interrupt handler functions that call the `on_interrupt` method of the correct instance.

// Example for Alarm0 - this would typically be in main.rs or a higher level setup code
// static TIMER0_INSTANCE: Mutex<RefCell<Option<Rp2040Timer<Alarm0>>>> = Mutex::new(RefCell::new(None));
//
// #[interrupt]
// fn TIMER_IRQ_0() {
//     interrupt::free(|cs| {
//         if let Some(timer) = TIMER0_INSTANCE.borrow(cs).borrow_mut().as_mut() {
//             timer.on_interrupt();
//         }
//     });
// }
//
// In main setup:
// let alarm0 = hal_timer.alarm_0().unwrap();
// let mut timer0 = Rp2040Timer::new(alarm0, my_callback_for_timer0);
// unsafe { NVIC::unmask(pac::Interrupt::TIMER_IRQ_0); }
// interrupt::free(|cs| {
//     TIMER0_INSTANCE.borrow(cs).replace(Some(timer0));
// });
// timer0_ref_mut_from_mutex.set_waketime(...); // Need a way to access it after moving to mutex

// The trait `fn new(...) -> Self` makes this pattern awkward if `new` is supposed to set up the static.
// If `new` returns Self, the caller owns it. The caller must then place it in the Mutex
// and ensure the ISR can find it.

// For this step, I will implement the trait methods.
// The setup of statics and ISRs will be part of the "Integrate a Test Timer in main.rs" step,
// as it involves main.rs code.
