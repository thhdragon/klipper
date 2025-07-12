// klipper_host_rust/src/reactor.rs
// Corresponds to klippy/reactor.py - Event reactor.

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct TimerHandle(pub usize); // Made field public

// Callback now returns Option<f64>: Some(next_event_time) to reschedule, None to unschedule.
pub type TimerCallback = Box<dyn FnMut(f64) -> Option<f64>>;

pub trait Reactor {
    fn monotonic(&self) -> f64;
    fn now(&self) -> f64 { // Default implementation
        self.monotonic()
    }
    fn register_timer(&mut self, eventtime: f64, callback: TimerCallback) -> TimerHandle;
    fn unregister_timer(&mut self, handle: TimerHandle);
    fn update_timer(&mut self, handle: TimerHandle, eventtime: f64);

    // For FD handling (placeholders for now)
    // These would return handles similar to TimerHandle if they need to be unregistered individually.
    // For simplicity, using usize as a placeholder handle ID.
    fn register_fd(&mut self, fd: i32, callback: Box<dyn FnMut(f64)>) -> usize;
    fn unregister_fd(&mut self, handle_id: usize);

    fn pause(&self, waketime: f64); // Changed to &self
    fn is_shutdown(&self) -> bool;
    fn run(&mut self);
    fn _check_timers(&mut self, eventtime: f64, idle: bool);
}
