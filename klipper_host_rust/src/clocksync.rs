// klipper_host_rust/src/clocksync.rs
// Corresponds to klippy/clocksync.py - Logic for clock synchronization with MCU.

use crate::reactor::{Reactor, TimerHandle};
use crate::serialhdl::{SerialHdl, Command, CommandQueue, MessageParams, ParamValue};
use std::sync::Arc;
use log::{debug, info, error}; // Import log macros

// Constants from clocksync.py
const RTT_AGE: f64 = 0.000010 / (60.0 * 60.0);
const DECAY: f64 = 1.0 / 30.0;
const TRANSMIT_EXTRA: f64 = 0.001; // Time offset for transmit

#[derive(Debug)]
pub enum ClockSyncError {
    MissingParam(String),
    InvalidParamType(String),
    SerialNotInitialized,
    CommandNotInitialized,
    QueueNotInitialized,
}

impl std::fmt::Display for ClockSyncError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ClockSyncError::MissingParam(param) => write!(f, "Missing parameter: {}", param),
            ClockSyncError::InvalidParamType(param) => write!(f, "Invalid parameter type: {}", param),
            ClockSyncError::SerialNotInitialized => write!(f, "Serial interface not initialized"),
            ClockSyncError::CommandNotInitialized => write!(f, "Get clock command not initialized"),
            ClockSyncError::QueueNotInitialized => write!(f, "Command queue not initialized"),
        }
    }
}

impl std::error::Error for ClockSyncError {}

// Note: Callbacks like _get_clock_event and _handle_clock are tricky in Rust
// when they need to modify `self`. Python methods implicitly capture `self`.
// In Rust, `reactor.register_timer` and `serial.register_response` would ideally
// take closures `Box<dyn FnMut(...)>` or similar to capture the `ClockSync` instance.
// For this porting step, we might define them as static methods or free functions
// that take `&mut ClockSync` if the placeholder signatures allow, or we might
// need to adjust the placeholder signatures in reactor/serialhdl.
// For now, let's assume we can make them static methods and pass `self` explicitly if needed,
// or that the reactor/serial parts will eventually support `FnMut` closures.

pub struct ClockSync {
    // reactor: Reactor, // Reactor is now typically managed externally and accessed via Arc<Mutex<>> or similar if shared.
                         // For simplicity, if ClockSync needs to own its timer handles and interact,
                         // it might need a way to communicate back to the reactor owner, or the reactor itself needs to be passed around.
                         // Let's assume for now that register_timer returns a handle, and ClockSync stores it.
                         // The actual timer execution is managed by the reactor that `new` receives.
    serial: Option<Arc<SerialHdl>>, // Using Arc if SerialHdl might be shared or to simplify lifetimes.
                                 // Option because it's set in `connect`.
    get_clock_timer: Option<TimerHandle>, // Handle for the timer from reactor.register_timer
    get_clock_cmd: Option<Command>, // Command for "get_clock"
    cmd_queue: Option<CommandQueue>, // Command queue from serial

    queries_pending: u32,
    mcu_freq: f64,
    last_clock: u64,            // Last known 64-bit MCU clock value
    clock_est: (f64, f64, f64), // (host_time_avg, mcu_clock_avg, mcu_freq_estimate)
    min_half_rtt: f64,          // Minimum half round trip time seen
    min_rtt_time: f64,          // Host time when min_half_rtt was recorded

    // Linear regression variables
    time_avg: f64,              // Averaged host send time
    time_variance: f64,         // Variance of host send time
    clock_avg: f64,             // Averaged MCU clock time
    clock_covariance: f64,      // Covariance of host send time and MCU clock

    prediction_variance: f64,   // Variance of the clock prediction error
    last_prediction_time: f64,  // Host time of the last valid prediction update
}

impl ClockSync {
    // Removed static _get_clock_event_callback and _handle_clock_callback

    pub fn new() -> Self {
        // Timer registration is deferred to `connect` or a method that has access to `self`
        // and the reactor instance.
        // The `reactor` instance is no longer stored directly if we use closures
        // that capture `Arc<Mutex<Self>>` or similar.
        // For now, let's assume `new` doesn't register the timer yet, or
        // it needs the reactor passed in to store the handle.
        // If `Reactor` is owned by `ClockSync`, it would be `mut reactor: Reactor`.
        // If `Reactor` is external, `new` might not need it, but `connect` will.
        Self {
            // reactor, // Removed for now, assuming reactor is passed to methods needing it or handled via closures
            serial: None,
            get_clock_timer: None, // Timer handle will be set later
            get_clock_cmd: None,
            cmd_queue: None,
            queries_pending: 0,
            mcu_freq: 1.0, // Default, will be overridden in connect
            last_clock: 0,
            clock_est: (0.0, 0.0, 0.0), // (time_avg, clock_avg, freq)
            min_half_rtt: f64::INFINITY, // Initialize with a very large number
            min_rtt_time: 0.0,
            time_avg: 0.0,
            time_variance: 0.0,
            clock_avg: 0.0,
            clock_covariance: 0.0,
            prediction_variance: 0.0,
            last_prediction_time: 0.0,
        }
    }

    // `reactor` is passed here as `&mut Reactor` assuming ClockSync might own it,
    // or it's passed from an owner who also owns ClockSync.
    // If Reactor is shared (e.g. Arc<Mutex<Reactor>>), the signature would change.
    pub fn connect(
        &mut self,
        reactor: &mut dyn Reactor,
        serial: Arc<SerialHdl>,
        // To enable closures that capture `self`, we'd typically need ClockSync to be
        // wrapped in Arc<Mutex<>>. For now, we'll make the methods on ClockSync `&mut self`
        // and assume the callback mechanism can somehow call them.
        // The closures passed to reactor/serial will be `move || { self_arc_mutex.lock().unwrap().method() }`
        // This requires `self` to be `Arc<Mutex<ClockSync>>` when creating those closures.
        // This initial step focuses on changing reactor/serial to accept `Box<dyn FnMut>`,
        // the actual closure creation will be tricky.
        // For now, let's assume `_get_clock_event` and `_handle_clock` are instance methods
        // and we can somehow get the reactor to call them.
    ) -> Result<(), ClockSyncError> {
        self.serial = Some(Arc::clone(&serial));
        let serial_ref = self.serial.as_ref().ok_or_else(|| ClockSyncError::SerialNotInitialized)?;

        self.mcu_freq = serial_ref.get_msgparser().get_constant_float("CLOCK_FREQ");

        let params = serial_ref.send_with_response("get_uptime", "uptime");

        let high = params.get("high")
            .and_then(|v| match v { ParamValue::Int(i) => Some(*i as u64), _ => None })
            .ok_or_else(|| ClockSyncError::MissingParam("high".to_string()))?;
        let clock_val = params.get("clock")
            .and_then(|v| match v { ParamValue::Int(i) => Some(*i as u64), _ => None })
            .ok_or_else(|| ClockSyncError::MissingParam("clock".to_string()))?;
        self.last_clock = (high << 32) | clock_val;
        self.clock_avg = self.last_clock as f64;

        self.time_avg = params.get("#sent_time")
            .and_then(|v| match v { ParamValue::Float(f) => Some(*f), _ => None })
            .ok_or_else(|| ClockSyncError::MissingParam("#sent_time".to_string()))?;

        self.clock_est = (self.time_avg, self.clock_avg, self.mcu_freq);
        self.prediction_variance = (0.001 * self.mcu_freq).powi(2);

        for _i in 0..8 {
            reactor.pause(reactor.monotonic() + 0.050);
            self.last_prediction_time = -9999.0;
            let clock_params = serial_ref.send_with_response("get_clock", "clock");
            self._handle_clock(clock_params)?; // Direct call to instance method, propagate error
        }

        self.get_clock_cmd = Some(serial_ref.get_msgparser().create_command("get_clock"));
        self.cmd_queue = Some(serial_ref.alloc_command_queue());

        // Timer registration and response handler registration will require `self` to be captured.
        // This is where Arc<Mutex<Self>> would typically be used.
        // For this step, we'll just define the instance methods _get_clock_event and _handle_clock.
        // The actual registration in `connect` is deferred or needs a different structure.

        // Example of how it *might* look if ClockSync is Arc<Mutex<ClockSync>> (conceptual)
        // let self_arc = ...; // Assume self is Arc<Mutex<ClockSync>>
        // let self_clone_for_timer = Arc::clone(&self_arc);
        // let timer_handle = reactor.register_timer(Box::new(move |eventtime| {
        //     self_clone_for_timer.lock().unwrap()._get_clock_event(eventtime)
        // }));
        // self.get_clock_timer = Some(timer_handle);
        //
        // let self_clone_for_handler = Arc::clone(&self_arc);
        // serial_ref.register_response(Box::new(move |params| {
        //     self_clone_for_handler.lock().unwrap()._handle_clock(params)
        // }), "clock");


        // For now, we'll assume the timer is registered externally or that `ClockSync` itself
        // isn't directly responsible for creating the Box<dyn FnMut> that needs to capture itself.
        // Let's leave `get_clock_timer` as None and assume it's set up by the caller of `connect`
        // or by a method on ClockSync that takes an Arc<Mutex<Self>>.
        // To make progress, we will just call `update_timer` with a dummy handle if it was set.
        // This part needs a more complete architectural decision on ownership/callbacks.

        // If a timer was previously registered (e.g. by an owner), update it.
        // This is a bit of a placeholder action for now.
        if let Some(timer_handle) = self.get_clock_timer {
            reactor.update_timer(timer_handle, reactor.now());
        }
    }

    // Renamed from _handle_clock_instance
    fn _handle_clock(&mut self, params: MessageParams) -> Result<(), ClockSyncError> {
        self.queries_pending = 0;

        let clock_param = params.get("clock")
            .and_then(|v| match v { ParamValue::Int(i) => Some(*i as u32), _ => None })
            .ok_or_else(|| ClockSyncError::MissingParam("clock".to_string()))?;

        // Extend clock to 64bit
        let clock_delta = (clock_param.wrapping_sub(self.last_clock as u32)) as u64; // Handle wrap around
        let current_mcu_clock = self.last_clock + clock_delta;
        self.last_clock = current_mcu_clock;

        let sent_time = params.get("#sent_time")
            .and_then(|v| match v { ParamValue::Float(f) => Some(*f), _ => None })
            .ok_or_else(|| ClockSyncError::MissingParam("#sent_time".to_string()))?;
        if sent_time == 0.0 { // Python `if not sent_time:`
            return Ok(()); // Not an error, just skip processing
        }
        let receive_time = params.get("#receive_time")
            .and_then(|v| match v { ParamValue::Float(f) => Some(*f), _ => None })
            .ok_or_else(|| ClockSyncError::MissingParam("#receive_time".to_string()))?;

        let half_rtt = 0.5 * (receive_time - sent_time);
        let aged_rtt_allowance = (sent_time - self.min_rtt_time) * RTT_AGE;

        if half_rtt < self.min_half_rtt + aged_rtt_allowance {
            self.min_half_rtt = half_rtt;
            self.min_rtt_time = sent_time;
            debug!("ClockSync: new minimum rtt {:.3}: hrtt={:.6} freq={}",
                      sent_time, half_rtt, self.clock_est.2);
        }

        // Filter out samples that are extreme outliers
        let expected_clock_at_sent_time = (sent_time - self.time_avg) * self.clock_est.2 + self.clock_avg;
        let clock_diff_sq = (current_mcu_clock as f64 - expected_clock_at_sent_time).powi(2);

        if clock_diff_sq > 25.0 * self.prediction_variance
           && clock_diff_sq > (0.000500 * self.mcu_freq).powi(2) {
            if current_mcu_clock as f64 > expected_clock_at_sent_time && sent_time < self.last_prediction_time + 10.0 {
                debug!("ClockSync: Ignoring clock sample {:.3}: freq={} diff={} stddev={:.3}",
                          sent_time, self.clock_est.2, (current_mcu_clock as f64 - expected_clock_at_sent_time),
                          self.prediction_variance.sqrt());
                return Ok(()); // Not an error, just skip processing
            }
            info!("ClockSync: Resetting prediction variance {:.3}: freq={} diff={} stddev={:.3}",
                     sent_time, self.clock_est.2, (current_mcu_clock as f64 - expected_clock_at_sent_time),
                     self.prediction_variance.sqrt());
            self.prediction_variance = (0.001 * self.mcu_freq).powi(2);
        } else {
            self.last_prediction_time = sent_time;
            self.prediction_variance = (1.0 - DECAY) * (self.prediction_variance + clock_diff_sq * DECAY);
        }

        // Add clock and sent_time to linear regression
        let diff_sent_time = sent_time - self.time_avg;
        self.time_avg += DECAY * diff_sent_time;
        self.time_variance = (1.0 - DECAY) * (self.time_variance + diff_sent_time.powi(2) * DECAY);

        let diff_clock = current_mcu_clock as f64 - self.clock_avg;
        self.clock_avg += DECAY * diff_clock;
        self.clock_covariance = (1.0 - DECAY) * (self.clock_covariance + diff_sent_time * diff_clock * DECAY);

        // Update prediction from linear regression
        let new_freq = if self.time_variance == 0.0 { self.mcu_freq } else { self.clock_covariance / self.time_variance };
        let pred_stddev = self.prediction_variance.sqrt();

        let serial_ref = self.serial.as_ref().ok_or_else(|| ClockSyncError::SerialNotInitialized)?;
        serial_ref.set_clock_est(
            new_freq,
            self.time_avg + TRANSMIT_EXTRA,
            (self.clock_avg - 3.0 * pred_stddev) as i64, // Note: conversion to i64
            current_mcu_clock
        );

        self.clock_est = (self.time_avg + self.min_half_rtt, self.clock_avg, new_freq);
        debug!("ClockSync: regr {:.3}: freq={:.3} d={}({:.3})",
                 sent_time, new_freq,
                 (current_mcu_clock as f64 - expected_clock_at_sent_time), // d is float in py
                 pred_stddev);
        Ok(())
    }

    // Renamed from _get_clock_event_instance
    // This method is intended to be called by the reactor when the timer fires.
    // It returns the next event time. Errors during its operation are logged but do not stop the timer.
    pub fn _get_clock_event(&mut self, eventtime: f64) -> f64 {
        let result: Result<(), ClockSyncError> = (|| {
            let serial_ref = self.serial.as_ref().ok_or_else(|| ClockSyncError::SerialNotInitialized)?;
            let cmd = self.get_clock_cmd.as_ref().ok_or_else(|| ClockSyncError::CommandNotInitialized)?;
            let queue = self.cmd_queue.as_ref().ok_or_else(|| ClockSyncError::QueueNotInitialized)?;

            serial_ref.raw_send(cmd.clone(), 0, 0, queue.clone());
            self.queries_pending += 1;
            Ok(())
        })();

        if let Err(e) = result {
            error!("Error in _get_clock_event: {}", e);
        }

        // Use an unusual time for the next event so clock messages
        // don't resonate with other periodic events.
        eventtime + 0.9839
    }

    // clock frequency conversions
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 {
        // Python returns int(), which can be large. Rust i64 should suffice.
        (print_time * self.mcu_freq).round() as i64
    }

    pub fn clock_to_print_time(&self, clock: u64) -> f64 {
        clock as f64 / self.mcu_freq
    }

    // system time conversions
    pub fn get_clock(&self, eventtime: f64) -> u64 {
        let (sample_time, clock_val, freq) = self.clock_est;
        // Python's int() truncates towards zero. In Rust, f64 as u64 truncates.
        (clock_val + (eventtime - sample_time) * freq).round() as u64
    }

    pub fn estimate_clock_systime(&self, reqclock: u64) -> f64 {
        let (sample_time, clock_val, freq) = self.clock_est;
        if freq == 0.0 { return sample_time; } // Avoid division by zero, return sample_time
        (reqclock as f64 - clock_val) / freq + sample_time
    }

    pub fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.clock_to_print_time(self.get_clock(eventtime))
    }

    // misc commands
    pub fn clock32_to_clock64(&self, clock32: u32) -> u64 {
        let last_clk = self.last_clock;
        let mut clock_diff = clock32 as i64 - (last_clk & 0xffffffff) as i64;
        // Equivalent to: clock_diff -= (clock_diff & 0x80000000) << 1
        // This is to handle the wrap-around correctly, choosing the closest clock.
        if clock_diff > 0x7fffffff {
            clock_diff -= 0x100000000;
        } else if clock_diff < -0x7fffffff {
            clock_diff += 0x100000000;
        }
        (last_clk as i64 + clock_diff) as u64
    }

    pub fn is_active(&self) -> bool {
        // In Klipper, if queries_pending > 4, it might indicate MCU has reset or is unresponsive.
        self.queries_pending <= 4
    }

    pub fn dump_debug(&self) -> String {
        let (sample_time, clock, freq) = self.clock_est;
        format!(
            "clocksync state: mcu_freq={} last_clock={}\n \
             clock_est=({:.3} {:.0} {:.3}) min_half_rtt={:.6} min_rtt_time={:.3}\n \
             time_avg={:.3}({:.3}) clock_avg={:.3}({:.3})\n \
             pred_variance={:.3}",
            self.mcu_freq, self.last_clock, sample_time, clock, freq,
            self.min_half_rtt, self.min_rtt_time,
            self.time_avg, self.time_variance,
            self.clock_avg, self.clock_covariance, // In Python this was clock_covariance, not clock_variance
            self.prediction_variance
        )
    }

    pub fn stats(&self, _eventtime: f64) -> String {
        // Original Python: `return "freq=%d" % (freq,)`
        // freq here refers to self.clock_est.2
        format!("freq={:.0}", self.clock_est.2)
    }

    // Default implementation, overridden in SecondarySync
    pub fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> (f64, f64) {
        (0.0, self.mcu_freq)
    }

    // Port of connect_file
    pub fn connect_file(&mut self, reactor: &mut dyn Reactor, serial: Arc<SerialHdl>, pace: bool) { // Changed to dyn Reactor
        self.serial = Some(Arc::clone(&serial)); // Use Arc::clone
        let serial_ref = self.serial.as_ref().unwrap();

        self.mcu_freq = serial_ref.get_msgparser().get_constant_float("CLOCK_FREQ");
        self.clock_est = (0.0, 0.0, self.mcu_freq);

        let mut freq_for_set = 1_000_000_000_000.0; // Large number, effectively making clock advance very slowly relative to host time
        if pace {
            freq_for_set = self.mcu_freq;
        }
        serial_ref.set_clock_est(freq_for_set, reactor.monotonic(), 0, 0); // Use passed reactor
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Use mock versions of Reactor and SerialHdl
    use crate::reactor::{TimerCallback as ReactorTimerCallback, TimerHandle as ReactorTimerHandle};
    use crate::serialhdl::{ResponseCallback as SerialResponseCallback, MsgParser as SerialMsgParser, Command as SerialCommand, CommandQueue as SerialCommandQueue, MessageParams as SerialMessageParams, ParamValue as SerialParamValue};
    use std::sync::{Arc, Mutex};
    use std::collections::HashMap;
    use std::cell::RefCell;

    // --- Mock Reactor ---
    struct MockReactor {
        monotonic_time: Mutex<f64>,
        timers: RefCell<HashMap<ReactorTimerHandle, (ReactorTimerCallback, f64)>>,
        next_handle: Mutex<usize>,
    }

    impl MockReactor {
        fn new() -> Self {
            MockReactor {
                monotonic_time: Mutex::new(0.0),
                timers: RefCell::new(HashMap::new()),
                next_handle: Mutex::new(0),
            }
        }
        #[allow(dead_code)]
        fn set_monotonic_time(&self, time: f64) {
            *self.monotonic_time.lock().unwrap() = time;
        }
        #[allow(dead_code)]
        fn advance_time(&self, duration: f64) {
            *self.monotonic_time.lock().unwrap() += duration;
        }
        // Implement methods needed by ClockSync, matching Reactor's public API
        pub fn monotonic(&self) -> f64 { *self.monotonic_time.lock().unwrap() }
        pub fn pause(&self, waketime: f64) {
            let mut current_time = self.monotonic_time.lock().unwrap();
            if waketime > *current_time { *current_time = waketime; }
        }
        pub fn now(&self) -> f64 { self.monotonic() }
        pub fn register_timer(&self, _callback: ReactorTimerCallback) -> ReactorTimerHandle {
            let mut handle_val = self.next_handle.lock().unwrap();
            let handle = ReactorTimerHandle(*handle_val);
            *handle_val += 1;
            // In a real mock, we might store and allow triggering this callback.
            // For now, just returning a handle is enough for some tests.
            handle
        }
        pub fn update_timer(&self, _handle: ReactorTimerHandle, _eventtime: f64) {
            // Can be expanded if timer update logic needs testing
        }
    }

    // --- Mock SerialHdl ---
    struct MockMsgParser;
    impl MockMsgParser {
        fn new() -> Self { MockMsgParser }
        fn get_constant_float(&self, key: &str) -> f64 {
            if key == "CLOCK_FREQ" { 25_000_000.0 } else { 0.0 }
        }
        fn create_command(&self, _command_name: &str) -> SerialCommand { SerialCommand }
    }

    struct MockSerialHdl {
        msg_parser: MockMsgParser,
        responses: RefCell<HashMap<String, SerialMessageParams>>,
        sent_commands: RefCell<Vec<String>>,
        clock_est_calls: RefCell<Vec<(f64, f64, i64, u64)>>,
    }

    impl MockSerialHdl {
        fn new() -> Arc<Self> {
            Arc::new(MockSerialHdl {
                msg_parser: MockMsgParser::new(),
                responses: RefCell::new(HashMap::new()),
                sent_commands: RefCell::new(Vec::new()),
                clock_est_calls: RefCell::new(Vec::new()),
            })
        }
        // Method to stage a response for send_with_response
        #[allow(dead_code)]
        fn stage_response(&self, command: &str, response: SerialMessageParams) {
            self.responses.borrow_mut().insert(command.to_string(), response);
        }

        // Implement methods needed by ClockSync, matching SerialHdl's public API
        pub fn get_msgparser(&self) -> &MockMsgParser { &self.msg_parser }
        pub fn send_with_response(&self, command: &str, _command_id: &str) -> SerialMessageParams {
            self.sent_commands.borrow_mut().push(command.to_string());
            self.responses.borrow_mut().remove(command).unwrap_or_default()
        }
        pub fn alloc_command_queue(&self) -> SerialCommandQueue { SerialCommandQueue }
        pub fn raw_send(&self, _command: SerialCommand, _cmd_id: u32, _data: u32, _queue: SerialCommandQueue) {
            // Could track raw sends if needed
        }
        pub fn set_clock_est(&self, new_freq: f64, time_avg: f64, clock_avg_adj: i64, cur_clock: u64) {
            self.clock_est_calls.borrow_mut().push((new_freq, time_avg, clock_avg_adj, cur_clock));
        }
        #[allow(dead_code)]
        pub fn register_response(&self, _callback: SerialResponseCallback, _message_type: &str) {
            // Can be expanded if response registration needs testing
        }
    }


    // Helper to create a default ClockSync for testing certain methods
    fn create_test_clocksync() -> ClockSync {
        let mut cs = ClockSync::new();
        cs.mcu_freq = 25_000_000.0;
        cs.clock_est = (100.0, 2_500_000_000.0, 25_000_000.0);
        cs.last_clock = 2_500_000_000;
        cs
    }

    #[test]
    fn test_clocksync_new() {
        let cs = ClockSync::new();
        assert_eq!(cs.mcu_freq, 1.0);
        assert!(cs.min_half_rtt.is_infinite());
    }

    #[test]
    fn test_time_conversions() {
        let cs = create_test_clocksync();

        assert_eq!(cs.print_time_to_clock(1.0), 25_000_000);
        assert_eq!(cs.print_time_to_clock(0.5), 12_500_000);
        assert_eq!(cs.print_time_to_clock(100.0), 2_500_000_000);


        // clock_to_print_time
        assert_eq!(cs.clock_to_print_time(25_000_000), 1.0);
        assert_eq!(cs.clock_to_print_time(12_500_000), 0.5);
        assert_eq!(cs.clock_to_print_time(2_500_000_000), 100.0);

        // get_clock (eventtime = sample_time)
        // clock_est = (sample_time=100.0, clock_val=2.5G, freq=25M)
        assert_eq!(cs.get_clock(100.0), 2_500_000_000); // eventtime is sample_time
        // get_clock (eventtime after sample_time)
        assert_eq!(cs.get_clock(101.0), 2_500_000_000 + 25_000_000); // 1s later
        // get_clock (eventtime before sample_time)
        assert_eq!(cs.get_clock(99.0), 2_500_000_000 - 25_000_000); // 1s earlier

        // estimate_clock_systime
        // (reqclock - clock_val) / freq + sample_time
        // (2.5G - 2.5G) / 25M + 100.0 = 100.0
        assert_eq!(cs.estimate_clock_systime(2_500_000_000), 100.0);
        // ( (2.5G + 25M) - 2.5G ) / 25M + 100.0 = 1.0 + 100.0 = 101.0
        assert_eq!(cs.estimate_clock_systime(2_500_000_000 + 25_000_000), 101.0);

        // estimated_print_time
        assert_eq!(cs.estimated_print_time(100.0), 100.0);
        assert_eq!(cs.estimated_print_time(101.0), 101.0);
    }

    #[test]
    fn test_clock32_to_clock64() {
        let mut cs = create_test_clocksync();

        // Test case 1: clock32 is slightly ahead of last_clock's lower 32 bits
        cs.last_clock = 0x1_000000F0;
        let clock32_ahead = 0x000000F5; // 5 ticks ahead
        assert_eq!(cs.clock32_to_clock64(clock32_ahead), 0x1_000000F5);

        // Test case 2: clock32 has wrapped around (is smaller than last_clock's lower 32 bits)
        cs.last_clock = 0x1_FFFFFFF0; // Lower part is near wrap
        let clock32_wrapped = 0x00000010; // Wrapped around
        assert_eq!(cs.clock32_to_clock64(clock32_wrapped), 0x2_00000010); // Should increment upper part

        // Test case 3: clock32 is behind, but not by much (no wrap anticipated backwards)
        cs.last_clock = 0x1_000000F0;
        let clock32_behind = 0x000000E0;
        assert_eq!(cs.clock32_to_clock64(clock32_behind), 0x1_000000E0);

        // Test case 4: clock32 significantly behind, suggesting it's from previous wrap cycle (should choose closest)
        // This implies clock32 is much smaller than (last_clock & 0xFFFFFFFF)
        // Python: last_clock=1000, clock32=10 -> diff = -990. if clock_diff < -0x7fffffff: clock_diff += 0x100000000
        // Here, if last_clock = 0x1_00000200 (512), clock32 = 10. diff = 10 - 512 = -502. This is fine.
        // If last_clock = 0x1_80000000, clock32 = 10. diff = 10 - 0x80000000 = small - large_pos = large_neg.
        // clock_diff = 10 - 2147483648 = -2147483638. This is < -0x7fffffff (-2147483647)
        // So clock_diff += 0x100000000 = -2147483638 + 4294967296 = 2147483658
        // result = last_clock + clock_diff = 0x1_80000000 + 2147483658 = (0x1_80000000 - 0x80000000) + 0x80000000 + 2147483658
        // = 0x1_00000000 + 2147483658. This logic seems to ensure it picks the "forward" clock.
        cs.last_clock = 0x80000000; // Lower part is 0x80000000
        let clock32_far_behind_wrap = 0x00000010; // e.g. 16
        // clock_diff = 16 - 2147483648 = -2147483632.
        // clock_diff is < -0x7fffffff, so clock_diff += 0x100000000
        // clock_diff = -2147483632 + 4294967296 = 2147483664
        // result = 0x80000000 + 2147483664 = 2147483648 + 2147483664 = 4294967312 = 0x100000010
        assert_eq!(cs.clock32_to_clock64(clock32_far_behind_wrap), 0x1_00000010);

        // Test case 5: clock32 significantly ahead, suggesting it's from next wrap cycle (should choose closest)
        // Python: last_clock=10, clock32=1000. diff = 990. if clock_diff > 0x7fffffff: clock_diff -= 0x100000000
        cs.last_clock = 0x00000010; // Lower part is 16
        let clock32_far_ahead_wrap = 0xF0000000; // e.g. 4026531840
        // clock_diff = 0xF0000000 - 0x10 = 4026531840 - 16 = 4026531824
        // This is > 0x7FFFFFFF (2147483647), so clock_diff -= 0x100000000
        // clock_diff = 4026531824 - 4294967296 = -268435472
        // result = 0x10 + (-268435472) = 16 - 268435472 = -268435456.
        // This means the 64-bit clock would go backward if last_clock had no upper bits.
        // If last_clock = 0x1_00000010, then result = 0x1_00000010 - 268435472 = 0x0_F0000000
        assert_eq!(cs.clock32_to_clock64(clock32_far_ahead_wrap), 0x0_F0000000);
    }

    #[test]
    fn test_secondary_sync_new() {
        //let reactor = MockReactor::new(); // Not needed for SecondarySync::new
        let main_cs = Arc::new(create_test_clocksync());
        let secondary_cs = SecondarySync::new(main_cs); // MockReactor not passed
        assert_eq!(secondary_cs.clock_adj, (0.0, 1.0));
    }

    #[test]
    fn test_handle_clock_basic() { // Renamed from test_handle_clock_instance_basic
        let mut cs = ClockSync::new();
        cs.mcu_freq = 25_000_000.0;
        cs.time_avg = 10.0;
        cs.clock_avg = 250_000_000.0;
        cs.clock_est = (cs.time_avg, cs.clock_avg, cs.mcu_freq);
        cs.prediction_variance = (0.001 * cs.mcu_freq).powi(2);
        cs.last_clock = 250_000_000;
        cs.min_half_rtt = 0.0005;
        cs.min_rtt_time = 9.0;

        let mock_serial = MockSerialHdl::new();
        cs.serial = Some(mock_serial.clone()); // Use mock serial


        let mut params = HashMap::new();
        params.insert("clock".to_string(), SerialParamValue::Int(12_500_000));
        params.insert("#sent_time".to_string(), SerialParamValue::Float(10.5));
        params.insert("#receive_time".to_string(), SerialParamValue::Float(10.501));

        let result = cs._handle_clock(params);
        assert!(result.is_ok());

        assert_eq!(cs.queries_pending, 0);
        assert_eq!(cs.last_clock, 262_500_000);

        assert_eq!(cs.min_half_rtt, 0.0005);
        assert_eq!(cs.min_rtt_time, 10.5);

        assert!((cs.time_avg - (10.0 + DECAY * 0.5)).abs() < 1e-9);
        assert!((cs.clock_avg - (250_000_000.0 + DECAY * 12_500_000.0)).abs() < 1.0);

        assert!((cs.clock_est.0 - (cs.time_avg + cs.min_half_rtt)).abs() < 1e-9);
        assert!((cs.clock_est.1 - cs.clock_avg).abs() < 1.0);
        assert!((cs.clock_est.2 - cs.mcu_freq).abs() < cs.mcu_freq * 0.01);
        assert_eq!(mock_serial.clock_est_calls.borrow().len(), 1); // Check set_clock_est was called
    }

    #[test]
    fn test_connect_missing_params() {
        let mut cs = ClockSync::new();
        let mut mock_reactor = MockReactor::new();
        let mock_serial = MockSerialHdl::new(); // Empty responses

        // Missing 'high'
        let mut params_no_high = HashMap::new();
        params_no_high.insert("clock".to_string(), SerialParamValue::Int(1000));
        params_no_high.insert("#sent_time".to_string(), SerialParamValue::Float(0.1));
        mock_serial.stage_response("get_uptime", params_no_high);
        let result = cs.connect(&mut mock_reactor, mock_serial.clone());
        assert!(matches!(result, Err(ClockSyncError::MissingParam(p)) if p == "high"));

        // Missing 'clock'
        let mut params_no_clock = HashMap::new();
        params_no_clock.insert("high".to_string(), SerialParamValue::Int(0));
        params_no_clock.insert("#sent_time".to_string(), SerialParamValue::Float(0.1));
        mock_serial.stage_response("get_uptime", params_no_clock);
        let result = cs.connect(&mut mock_reactor, mock_serial.clone());
        assert!(matches!(result, Err(ClockSyncError::MissingParam(p)) if p == "clock"));

        // Missing '#sent_time'
        let mut params_no_sent_time = HashMap::new();
        params_no_sent_time.insert("high".to_string(), SerialParamValue::Int(0));
        params_no_sent_time.insert("clock".to_string(), SerialParamValue::Int(1000));
        mock_serial.stage_response("get_uptime", params_no_sent_time);
        let result = cs.connect(&mut mock_reactor, mock_serial.clone());
        assert!(matches!(result, Err(ClockSyncError::MissingParam(p)) if p == "#sent_time"));
    }

    #[test]
    fn test_handle_clock_missing_params() {
        let mut cs = create_test_clocksync();
        let mock_serial = MockSerialHdl::new();
        cs.serial = Some(mock_serial);

        let params_empty = HashMap::new();
        let result = cs._handle_clock(params_empty);
        assert!(matches!(result, Err(ClockSyncError::MissingParam(p)) if p == "clock"));

        let mut params_no_sent_time = HashMap::new();
        params_no_sent_time.insert("clock".to_string(), SerialParamValue::Int(123));
        let result = cs._handle_clock(params_no_sent_time);
        assert!(matches!(result, Err(ClockSyncError::MissingParam(p)) if p == "#sent_time"));

        let mut params_no_receive_time = HashMap::new();
        params_no_receive_time.insert("clock".to_string(), SerialParamValue::Int(123));
        params_no_receive_time.insert("#sent_time".to_string(), SerialParamValue::Float(0.1));
        let result = cs._handle_clock(params_no_receive_time);
        assert!(matches!(result, Err(ClockSyncError::MissingParam(p)) if p == "#receive_time"));
    }

    #[test]
    fn test_handle_clock_sent_time_zero() {
        let mut cs = create_test_clocksync();
        let mock_serial = MockSerialHdl::new();
        cs.serial = Some(mock_serial);

        let mut params = HashMap::new();
        params.insert("clock".to_string(), SerialParamValue::Int(123));
        params.insert("#sent_time".to_string(), SerialParamValue::Float(0.0));
        params.insert("#receive_time".to_string(), SerialParamValue::Float(0.1));

        let result = cs._handle_clock(params);
        assert!(result.is_ok()); // Should return Ok, but not process further
    }

    #[test]
    fn test_get_clock_event_no_serial() {
        let mut cs = ClockSync::new(); // No serial, no cmd, no queue
        let eventtime = 123.0;
        let next_eventtime = cs._get_clock_event(eventtime);
        assert_eq!(next_eventtime, eventtime + 0.9839);
        assert_eq!(cs.queries_pending, 0); // Should not increment
        // Check logs for error (manual check or with logging test framework)
    }

    #[test]
    fn test_get_clock_event_no_cmd_queue() {
        let mut cs = ClockSync::new();
        let mock_serial = MockSerialHdl::new();
        cs.serial = Some(mock_serial); // Has serial, but no cmd/queue set yet
        let eventtime = 123.0;
        let next_eventtime = cs._get_clock_event(eventtime);
        assert_eq!(next_eventtime, eventtime + 0.9839);
        assert_eq!(cs.queries_pending, 0);
    }
     #[test]
    fn test_connect_successful_path() {
        let mut cs = ClockSync::new();
        let mut mock_reactor = MockReactor::new();
        let mock_serial = MockSerialHdl::new();

        // Stage get_uptime response
        let mut uptime_response = HashMap::new();
        uptime_response.insert("high".to_string(), SerialParamValue::Int(0));
        uptime_response.insert("clock".to_string(), SerialParamValue::Int(1_000_000));
        uptime_response.insert("#sent_time".to_string(), SerialParamValue::Float(0.001));
        mock_serial.stage_response("get_uptime", uptime_response);

        // Stage get_clock responses for the loop
        for i in 0..8 {
            let mut clock_response = HashMap::new();
            clock_response.insert("clock".to_string(), SerialParamValue::Int(1_000_000 + (i + 1) * 50000));
            clock_response.insert("#sent_time".to_string(), SerialParamValue::Float(0.001 + (i as f64 * 0.05) + 0.01));
            clock_response.insert("#receive_time".to_string(), SerialParamValue::Float(0.001 + (i as f64 * 0.05) + 0.011));
            mock_serial.stage_response("get_clock", clock_response);
        }

        mock_reactor.set_monotonic_time(0.0);
        let result = cs.connect(&mut mock_reactor, mock_serial.clone());
        assert!(result.is_ok());
        assert_eq!(cs.mcu_freq, 25_000_000.0);
        assert!(cs.get_clock_cmd.is_some());
        assert!(cs.cmd_queue.is_some());
        assert_eq!(mock_serial.sent_commands.borrow().len(), 1 + 8); // 1 get_uptime + 8 get_clock
    }

    #[test]
    fn test_secondary_sync_connect_success() {
        let main_cs_data = create_test_clocksync(); // Main CS has clock_est = (100.0, 2.5G, 25M)
        let main_cs = Arc::new(main_cs_data);
        let mut secondary_cs = SecondarySync::new(Arc::clone(&main_cs));

        let mut mock_reactor = MockReactor::new();
        // Set reactor time for call to reactor.monotonic() in SecondarySync::connect
        // This is the time at which connect is called.
        mock_reactor.set_monotonic_time(100.0);

        let mock_serial_secondary = MockSerialHdl::new();

        // Stage responses for secondary's internal ClockSync's connect method
        let mut uptime_response = HashMap::new();
        uptime_response.insert("high".to_string(), SerialParamValue::Int(0));
        // Let secondary MCU start at a different clock and perceived host time
        uptime_response.insert("clock".to_string(), SerialParamValue::Int(1_000_000)); // e.g., 1M ticks at its own CLOCK_FREQ (which will be 25M by mock)
        uptime_response.insert("#sent_time".to_string(), SerialParamValue::Float(99.0)); // It thinks host time was 99.0 when it sent this
        mock_serial_secondary.stage_response("get_uptime", uptime_response);

        for i in 0..8 { // 8 priming calls to _handle_clock
            let mut clock_response = HashMap::new();
            // Clock advances based on its 25MHz freq.
            // sent_time also advances based on reactor.monotonic() + 0.050 then +0.01 from send_with_response
            let base_clock = 1_000_000 + (i * 25_000_000 / 20); // Approx 0.05s worth of ticks
            clock_response.insert("clock".to_string(), SerialParamValue::Int(base_clock + 50000)); // some small advance
            let sent_time = 99.0 + (i as f64 * 0.050) + 0.01; // approximate sent time for this loop iteration
            clock_response.insert("#sent_time".to_string(), SerialParamValue::Float(sent_time));
            clock_response.insert("#receive_time".to_string(), SerialParamValue::Float(sent_time + 0.001)); // 1ms RTT
            mock_serial_secondary.stage_response("get_clock", clock_response);
        }

        let result = secondary_cs.connect(&mut mock_reactor, mock_serial_secondary.clone());
        assert!(result.is_ok(), "SecondarySync connect failed: {:?}", result.err());

        // After connect, secondary_cs.clock_sync will have its own clock_est.
        // main_cs.estimated_print_time(100.0) is 100.0.
        // secondary_cs.clock_sync.estimated_print_time(100.0) will be based on its own sync.
        // Example: if secondary's clock_est became (99.0, 1M, 25M)
        // secondary_cs.clock_sync.estimated_print_time(100.0) = (1M + (100.0 - 99.0)*25M) / 25M = (1M + 25M) / 25M = 26/25 = 1.04
        // Initial clock_adj offset before calibrate_clock: 100.0 - 1.04 = 98.96
        // This is a rough check; the actual value depends on the 8 priming calls.
        // The key is that it's calculated and calibrate_clock is called.
        assert_ne!(secondary_cs.clock_adj.0, 0.0, "clock_adj.0 should have been updated from 0.0");
    }

    #[test]
    fn test_secondary_sync_connect_error_propagation() {
        let main_cs = Arc::new(create_test_clocksync());
        let mut secondary_cs = SecondarySync::new(Arc::clone(&main_cs));
        let mut mock_reactor = MockReactor::new();
        let mock_serial_secondary = MockSerialHdl::new();

        let mut uptime_response_missing_high = HashMap::new();
        uptime_response_missing_high.insert("clock".to_string(), SerialParamValue::Int(1000));
        mock_serial_secondary.stage_response("get_uptime", uptime_response_missing_high);

        let result = secondary_cs.connect(&mut mock_reactor, mock_serial_secondary);
        assert!(matches!(result, Err(ClockSyncError::MissingParam(p)) if p == "high"));
    }

    #[test]
    fn test_secondary_calibrate_clock_basic() {
        // Main clock sync: time=100, clock=2.5G, freq=25MHz. So print_time = 100.0
        let main_cs_data = ClockSync {
            mcu_freq: 25_000_000.0,
            clock_est: (100.0, 2_500_000_000.0, 25_000_000.0),
            ..create_test_clocksync() // Fills other fields like last_clock etc.
        };
        let main_cs = Arc::new(main_cs_data);

        let mut secondary_cs = SecondarySync::new(Arc::clone(&main_cs));
        // Secondary clock sync: time=100, clock=5G, freq=50MHz. So print_time = 100.0
        // This means at host time 100.0, both MCUs agree their print time is 100.0
        secondary_cs.clock_sync.mcu_freq = 50_000_000.0;
        secondary_cs.clock_sync.clock_est = (100.0, 5_000_000_000.0, 50_000_000.0);
        secondary_cs.clock_sync.last_clock = 5_000_000_000;
         // Initial adjustment: secondary thinks its print time = host time (offset 0), freq = own
        secondary_cs.clock_adj = (0.0, 50_000_000.0);

        // current host eventtime = 100.0
        // current secondary's "known" print_time (e.g. from a command) = 100.0
        let (offset, freq) = secondary_cs.calibrate_clock(100.0, 100.0);

        // main_sync.estimated_print_time(eventtime=100.0) = 100.0
        // est_print_time (main's view of current print time) = 100.0
        // sync1_print_time = max(known_print_time=100.0, est_print_time=100.0) = 100.0

        // sync2_print_time = max(sync1_print_time + 4. = 104.0,
        //                        last_sync_time=0,
        //                        known_print_time + 2.5 * (known_print_time - est_print_time) = 100.0 + 2.5 * (0) = 100.0)
        //                  = 104.0

        // sync2_main_clock = sync2_print_time * main_mcu_freq = 104.0 * 25M = 2.6G ticks
        // sync2_sys_time (host time when main MCU reaches sync2_main_clock):
        //   main_cs.clock_est = (sample_time=100.0, clock=2.5G, freq=25M)
        //   sync2_sys_time = 100.0 + (2.6G - 2.5G) / 25M = 100.0 + 0.1G / 25M = 100.0 + 4.0 = 104.0

        // sync1_clock_val (secondary's clock ticks for sync1_print_time=100.0, using current clock_adj(0, 50M)):
        //   secondary.print_time_to_clock(100.0) = (100.0 - 0.0) * 50M = 5G ticks

        // sync2_clock_val (secondary's internal clock ticks at host time sync2_sys_time=104.0):
        //   secondary.clock_sync.get_clock(104.0)
        //   secondary.clock_sync.clock_est = (sample_time=100.0, clock=5G, freq=50M)
        //   = 5G + (104.0 - 100.0) * 50M = 5G + 4.0 * 50M = 5G + 0.2G = 5.2G ticks

        // adjusted_freq = (sync2_clock_val - sync1_clock_val) / (sync2_print_time - sync1_print_time)
        //               = (5.2G - 5G) / (104.0 - 100.0) = 0.2G / 4.0 = 50M

        // adjusted_offset = sync1_print_time - (sync1_clock_val / adjusted_freq)
        //                 = 100.0 - (5G / 50M) = 100.0 - 100.0 = 0.0

        assert!((offset - 0.0).abs() < 1e-9, "Offset was {}", offset);
        assert!((freq - 50_000_000.0).abs() < 1e-9, "Freq was {}", freq);
        assert_eq!(secondary_cs.last_sync_time, 104.0);
    }

}


pub struct SecondarySync {
    clock_sync: ClockSync,
    main_sync: Arc<ClockSync>,
    clock_adj: (f64, f64),
    last_sync_time: f64,
}

impl SecondarySync {
    pub fn new(main_sync: Arc<ClockSync>) -> Self {
        Self {
            clock_sync: ClockSync::new(),
            main_sync,
            clock_adj: (0.0, 1.0), // Initialize with a default freq multiplier of 1.0
            last_sync_time: 0.0,
        }
    }

    // connect is overridden from ClockSync
    pub fn connect(&mut self, reactor: &mut dyn Reactor, serial: Arc<SerialHdl>) -> Result<(), ClockSyncError> { // Changed to dyn Reactor
        self.clock_sync.connect(reactor, Arc::clone(&serial))?; // Propagate error
        self.clock_adj = (0.0, self.clock_sync.mcu_freq);

        let curtime = reactor.monotonic();
        let main_print_time = self.main_sync.estimated_print_time(curtime);
        let local_print_time = self.clock_sync.estimated_print_time(curtime);

        self.clock_adj = (main_print_time - local_print_time, self.clock_sync.mcu_freq);
        self.calibrate_clock(0.0, curtime);
        Ok(())
    }

    // connect_file is also overridden
    // Assuming connect_file in the base class doesn't return Result, or we'd match that.
    // For now, keeping it simple as it mostly sets up initial state.
    pub fn connect_file(&mut self, reactor: &mut dyn Reactor, serial: Arc<SerialHdl>, pace: bool) { // Changed to dyn Reactor
        self.clock_sync.connect_file(reactor, Arc::clone(&serial), pace); // Pass reactor
        self.clock_adj = (0.0, self.clock_sync.mcu_freq);
    }

    // clock frequency conversions are overridden
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 {
        let (adjusted_offset, adjusted_freq) = self.clock_adj;
        if adjusted_freq == 0.0 { return 0; } // Avoid division by zero if freq is somehow zero
        ((print_time - adjusted_offset) * adjusted_freq).round() as i64
    }

    pub fn clock_to_print_time(&self, clock: u64) -> f64 {
        let (adjusted_offset, adjusted_freq) = self.clock_adj;
        if adjusted_freq == 0.0 { return adjusted_offset; } // Avoid division by zero
        clock as f64 / adjusted_freq + adjusted_offset
    }

    // misc commands are overridden
    pub fn dump_debug(&self) -> String {
        let (adjusted_offset, adjusted_freq) = self.clock_adj;
        format!("{} clock_adj=({:.3} {:.3})",
                self.clock_sync.dump_debug(), adjusted_offset, adjusted_freq)
    }

    pub fn stats(&self, eventtime: f64) -> String {
        let (_adjusted_offset, adjusted_freq) = self.clock_adj;
        format!("{} adj={:.0}",
                self.clock_sync.stats(eventtime), adjusted_freq)
    }

    // calibrate_clock is the core logic for SecondarySync
    pub fn calibrate_clock(&mut self, print_time: f64, eventtime: f64) -> (f64, f64) {
        // Calculate: est_print_time = main_sync.estimated_print_time()
        let (ser_time, ser_clock, ser_freq) = self.main_sync.clock_est;
        let main_mcu_freq = self.main_sync.mcu_freq;
        if main_mcu_freq == 0.0 {
            debug!("SecondarySync::calibrate_clock: main_mcu_freq is 0, cannot calibrate.");
            return self.clock_adj;
        }

        let est_main_clock = (eventtime - ser_time) * ser_freq + ser_clock;
        let est_print_time = est_main_clock / main_mcu_freq;

        // Determine sync1_print_time and sync2_print_time
        let sync1_print_time = print_time.max(est_print_time);
        // Python: sync2_print_time = max(sync1_print_time + 4., self.last_sync_time, print_time + 2.5 * (print_time - est_print_time))
        let mut sync2_print_time = sync1_print_time + 4.0;
        sync2_print_time = sync2_print_time.max(self.last_sync_time);
        sync2_print_time = sync2_print_time.max(print_time + 2.5 * (print_time - est_print_time));

        if (sync2_print_time - sync1_print_time).abs() < f64::EPSILON { // Avoid division by zero if times are too close
            debug!("SecondarySync::calibrate_clock: sync1 and sync2 print times are too close, cannot calibrate.");
            return self.clock_adj;
        }

        // Calc sync2_sys_time (inverse of main_sync.estimated_print_time)
        let sync2_main_clock = sync2_print_time * main_mcu_freq;
        let sync2_sys_time = if ser_freq == 0.0 {
            ser_time
        } else {
            ser_time + (sync2_main_clock - ser_clock) / ser_freq
        };

        // Adjust freq so estimated print_time will match at sync2_print_time
        let sync1_clock_val = self.print_time_to_clock(sync1_print_time); // Uses self.clock_adj
        // For get_clock, we need the underlying ClockSync's get_clock
        let sync2_clock_val = self.clock_sync.get_clock(sync2_sys_time);

        let adjusted_freq = (sync2_clock_val as f64 - sync1_clock_val as f64)
                            / (sync2_print_time - sync1_print_time);

        let adjusted_offset = if adjusted_freq.abs() < f64::EPSILON {
            sync1_print_time // Avoid division by zero if adjusted_freq is zero
        } else {
            sync1_print_time - (sync1_clock_val as f64 / adjusted_freq)
        };

        // Apply new values
        self.clock_adj = (adjusted_offset, adjusted_freq);
        self.last_sync_time = sync2_print_time;
        self.clock_adj
    }
}
