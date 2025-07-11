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
