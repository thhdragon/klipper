// klipper_host_rust/src/clocksync.rs
// Corresponds to klippy/clocksync.py - Logic for clock synchronization with MCU.

use crate::reactor::{Reactor, TimerHandle};
use crate::serialhdl::{SerialHdl, Command, CommandQueue, MessageParams, ParamValue};
use std::sync::Arc; // For Arc<Mutex<ClockSync>> if callbacks need mutable access.
                    // For now, we'll try to manage with &mut or by passing necessary state.

// Constants from clocksync.py
const RTT_AGE: f64 = 0.000010 / (60.0 * 60.0);
const DECAY: f64 = 1.0 / 30.0;
const TRANSMIT_EXTRA: f64 = 0.001; // Time offset for transmit


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
    reactor: Reactor, // In Python, this is passed in. Here, we might store a copy or ref.
                      // If Reactor methods need &mut self, ClockSync might need to hold Reactor by value
                      // or use interior mutability (RefCell/Mutex) if Reactor is shared.
                      // For now, assume ClockSync owns its Reactor instance for simplicity.
    serial: Option<Arc<SerialHdl>>, // Using Arc if SerialHdl might be shared or to simplify lifetimes.
                                 // Option because it's set in `connect`.
    get_clock_timer: TimerHandle, // Handle for the timer from reactor.register_timer
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
    // This is a static method because the callback signature in reactor.rs is `fn(f64)->f64`
    // A real implementation would likely use closures to capture `self`.
    // For now, this function won't be able to modify ClockSync state directly
    // without passing it in, which the current reactor signature doesn't support.
    // This highlights a future refactoring need for the callback mechanism.
    fn _get_clock_event_callback(_eventtime: f64) -> f64 {
        // This function needs access to self.serial, self.get_clock_cmd, self.cmd_queue
        // and to increment self.queries_pending.
        // This static version CANNOT DO THAT.
        // This is a major simplification for now.
        //
        // Original Python:
        // self.serial.raw_send(self.get_clock_cmd, 0, 0, self.cmd_queue)
        // self.queries_pending += 1
        // return eventtime + .9839
        println!("ClockSync Static: _get_clock_event_callback called. Needs self access!");
        _eventtime + 0.9839 // Placeholder return
    }

    // Similar issue for _handle_clock, it needs `&mut self`.
    // The serialhdl placeholder ResponseCallback is `fn(MessageParams)`.
    // This static version also cannot modify ClockSync state.
    fn _handle_clock_callback(_params: MessageParams) {
        // This function needs access to &mut self.
        println!("ClockSync Static: _handle_clock_callback called. Needs self access!");
        // Actual logic will be in the instance method `_handle_clock_instance`
    }

    pub fn new(mut reactor: Reactor) -> Self {
        // In Python, the timer is registered with a method reference.
        // In Rust, if reactor.register_timer takes `fn(f64)->f64`, we pass a static/free fn.
        let get_clock_timer_handle = reactor.register_timer(Self::_get_clock_event_callback);

        Self {
            reactor, // Reactor is moved in
            serial: None,
            get_clock_timer: get_clock_timer_handle,
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

    pub fn connect(&mut self, serial: Arc<SerialHdl>) {
        self.serial = Some(Arc::clone(&serial));
        let serial_ref = self.serial.as_ref().unwrap(); // Now we have a reference to serial

        self.mcu_freq = serial_ref.get_msgparser().get_constant_float("CLOCK_FREQ");

        // Load initial clock and frequency
        // Python: params = serial.send_with_response('get_uptime', 'uptime')
        let params = serial_ref.send_with_response("get_uptime", "uptime");

        let high = match params.get("high").and_then(|v| match v { ParamValue::Int(i) => Some(*i as u64), _ => None }) {
            Some(val) => val,
            None => { /* error handling, e.g., log and default */ println!("Error: 'high' not found or not Int in get_uptime response"); 0 }
        };
        let clock_val = match params.get("clock").and_then(|v| match v { ParamValue::Int(i) => Some(*i as u64), _ => None }) {
            Some(val) => val,
            None => { /* error handling */ println!("Error: 'clock' not found or not Int in get_uptime response"); 0 }
        };
        self.last_clock = (high << 32) | clock_val;

        self.clock_avg = self.last_clock as f64;

        self.time_avg = match params.get("#sent_time").and_then(|v| match v { ParamValue::Float(f) => Some(*f), _ => None }) {
            Some(val) => val,
            None => { /* error handling */ println!("Error: '#sent_time' not found or not Float in get_uptime response"); 0.0 }
        };

        self.clock_est = (self.time_avg, self.clock_avg, self.mcu_freq);
        self.prediction_variance = (0.001 * self.mcu_freq).powi(2);

        // Enable periodic get_clock timer by making some initial calls
        // Python:
        // for i in range(8):
        //     self.reactor.pause(self.reactor.monotonic() + 0.050)
        //     self.last_prediction_time = -9999.
        //     params = serial.send_with_response('get_clock', 'clock')
        //     self._handle_clock(params)
        for _i in 0..8 {
            self.reactor.pause(self.reactor.monotonic() + 0.050);
            self.last_prediction_time = -9999.0; // Using f64 for time
            // In Python, send_with_response for 'get_clock' is not typical; 'get_clock' responses
            // usually come via registered handlers from raw_send.
            // For this initial priming, Klipper *does* use send_with_response.
            let clock_params = serial_ref.send_with_response("get_clock", "clock");
            self._handle_clock_instance(clock_params); // Call the instance method
        }

        self.get_clock_cmd = Some(serial_ref.get_msgparser().create_command("get_clock"));
        self.cmd_queue = Some(serial_ref.alloc_command_queue());

        // Register response handler. This is problematic with current static callback.
        // serial_ref.register_response(Self::_handle_clock_callback, "clock");
        // TODO: This needs to be an instance method or closure. For now, we'll call _handle_clock_instance manually where needed.
        // Or, the serial module needs to be adapted to take e.g. an Arc<Mutex<ClockSync>> and a method pointer.
        // For now, we'll assume that when a 'clock' message would arrive, _handle_clock_instance is called.

        self.reactor.update_timer(self.get_clock_timer, self.reactor.now());
    }

    // Instance version of _handle_clock, to be called internally or by a future closure-based callback system.
    fn _handle_clock_instance(&mut self, params: MessageParams) {
        self.queries_pending = 0; // Reset pending queries count

        let clock_param = match params.get("clock").and_then(|v| match v { ParamValue::Int(i) => Some(*i as u32), _ => None }) {
            Some(val) => val,
            None => { println!("Error: 'clock' not found in _handle_clock_instance params"); return; }
        };

        // Extend clock to 64bit
        let clock_delta = (clock_param.wrapping_sub(self.last_clock as u32)) as u64; // Handle wrap around
        let current_mcu_clock = self.last_clock + clock_delta;
        self.last_clock = current_mcu_clock;

        let sent_time = match params.get("#sent_time").and_then(|v| match v { ParamValue::Float(f) => Some(*f), _ => None }) {
            Some(val) => val,
            None => { println!("Error: '#sent_time' not found in _handle_clock_instance params"); return; }
        };
        if sent_time == 0.0 { // Python `if not sent_time:`
            return;
        }
        let receive_time = match params.get("#receive_time").and_then(|v| match v { ParamValue::Float(f) => Some(*f), _ => None }) {
            Some(val) => val,
            None => { println!("Error: '#receive_time' not found in _handle_clock_instance params"); return; }
        };

        let half_rtt = 0.5 * (receive_time - sent_time);
        let aged_rtt_allowance = (sent_time - self.min_rtt_time) * RTT_AGE;

        if half_rtt < self.min_half_rtt + aged_rtt_allowance {
            self.min_half_rtt = half_rtt;
            self.min_rtt_time = sent_time;
            // logging.debug in Python
            println!("ClockSync: new minimum rtt {:.3}: hrtt={:.6} freq={}",
                      sent_time, half_rtt, self.clock_est.2);
        }

        // Filter out samples that are extreme outliers
        let expected_clock_at_sent_time = (sent_time - self.time_avg) * self.clock_est.2 + self.clock_avg;
        let clock_diff_sq = (current_mcu_clock as f64 - expected_clock_at_sent_time).powi(2);

        if clock_diff_sq > 25.0 * self.prediction_variance
           && clock_diff_sq > (0.000500 * self.mcu_freq).powi(2) {
            if current_mcu_clock as f64 > expected_clock_at_sent_time && sent_time < self.last_prediction_time + 10.0 {
                println!("ClockSync: Ignoring clock sample {:.3}: freq={} diff={} stddev={:.3}",
                          sent_time, self.clock_est.2, (current_mcu_clock as f64 - expected_clock_at_sent_time),
                          self.prediction_variance.sqrt());
                return;
            }
            println!("ClockSync: Resetting prediction variance {:.3}: freq={} diff={} stddev={:.3}",
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

        if let Some(serial_ref) = self.serial.as_ref() {
            serial_ref.set_clock_est(
                new_freq,
                self.time_avg + TRANSMIT_EXTRA,
                (self.clock_avg - 3.0 * pred_stddev) as i64, // Note: conversion to i64
                current_mcu_clock
            );
        }

        self.clock_est = (self.time_avg + self.min_half_rtt, self.clock_avg, new_freq);
        // logging.debug("regr %.3f: freq=%.3f d=%d(%.3f)", sent_time, new_freq, clock - exp_clock, pred_stddev)
        println!("ClockSync: regr {:.3}: freq={:.3} d={}({:.3})",
                 sent_time, new_freq,
                 (current_mcu_clock as f64 - expected_clock_at_sent_time), // d is float in py
                 pred_stddev);
    }

    // This is the instance method part of _get_clock_event.
    // The callback itself is static due to current reactor limitations.
    // This method would be called by that static callback if it could get `&mut self`.
    pub fn _get_clock_event_instance(&mut self, eventtime: f64) -> f64 {
        if let (Some(serial_ref), Some(cmd), Some(queue)) =
            (self.serial.as_ref(), self.get_clock_cmd.as_ref(), self.cmd_queue.as_ref()) {
            // TODO: The raw_send in serialhdl.rs is a placeholder and doesn't actually send.
            // It also doesn't trigger any response that would call _handle_clock_instance.
            // This needs to be simulated or handled when serialhdl is more complete.
            serial_ref.raw_send(cmd.clone(), 0, 0, queue.clone());
            self.queries_pending += 1;
        } else {
            println!("ClockSync Error: Serial, get_clock_cmd, or cmd_queue not initialized for _get_clock_event_instance");
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
    pub fn connect_file(&mut self, serial: Arc<SerialHdl>, pace: bool) {
        self.serial = Some(serial); // Arc::clone(&serial) is done by passing Arc
        let serial_ref = self.serial.as_ref().unwrap();

        self.mcu_freq = serial_ref.get_msgparser().get_constant_float("CLOCK_FREQ");
        self.clock_est = (0.0, 0.0, self.mcu_freq);

        let mut freq_for_set = 1_000_000_000_000.0; // Large number, effectively making clock advance very slowly relative to host time
        if pace {
            freq_for_set = self.mcu_freq;
        }
        // serial.set_clock_est(freq, self.reactor.monotonic(), 0, 0)
        // The reactor.monotonic() call implies current host time.
        // The 0s for clock_avg_adj and cur_clock are mcu clock values.
        serial_ref.set_clock_est(freq_for_set, self.reactor.monotonic(), 0, 0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reactor::Reactor; // For creating dummy reactor
    use crate::serialhdl::{SerialHdl, MessageParams, ParamValue}; // For dummy serial and params
    use std::sync::Arc;
    use std::collections::HashMap;

    // Helper to create a default ClockSync for testing certain methods
    fn create_test_clocksync() -> ClockSync {
        let reactor = Reactor::new(); // Using placeholder new
        let mut cs = ClockSync::new(reactor);
        cs.mcu_freq = 25_000_000.0; // Common Klipper MCU frequency
        cs.clock_est = (100.0, 2_500_000_000.0, 25_000_000.0); // sample_time, clock, freq
        cs.last_clock = 2_500_000_000; // Example last_clock
        cs
    }

    #[test]
    fn test_clocksync_new() {
        let reactor = Reactor::new();
        let cs = ClockSync::new(reactor);
        assert_eq!(cs.mcu_freq, 1.0); // Default before connect
        assert!(cs.min_half_rtt.is_infinite());
    }

    #[test]
    fn test_time_conversions() {
        let cs = create_test_clocksync();

        // print_time_to_clock
        assert_eq!(cs.print_time_to_clock(1.0), 25_000_000); // 1 second = 25M ticks
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
        assert_eq!(cs.estimated_print_time(100.0), 100.0); // clock_to_print_time(get_clock(100.0))
        assert_eq!(cs.estimated_print_time(101.0), 101.0);
    }

    #[test]
    fn test_clock32_to_clock64() {
        let mut cs = create_test_clocksync();

        // Test case 1: clock32 is slightly ahead of last_clock's lower 32 bits
        cs.last_clock = 0x1_000000F0; // Upper part is 1, lower is 0xF0
        let clock32_ahead = 0x000000F5; // 5 ticks ahead
        assert_eq!(cs.clock32_to_clock64(clock32_ahead), 0x1_000000F5);

        // Test case 2: clock32 has wrapped around (is smaller than last_clock's lower 32 bits)
        cs.last_clock = 0x1_FFFFFF潛行0; // Lower part is near wrap
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
        let reactor = Reactor::new();
        let main_cs = Arc::new(create_test_clocksync());
        let secondary_cs = SecondarySync::new(reactor, main_cs);
        assert_eq!(secondary_cs.clock_adj, (0.0, 1.0)); // Default before connect
    }

    #[test]
    fn test_handle_clock_instance_basic() {
        let reactor = Reactor::new();
        let mut cs = ClockSync::new(reactor); // Use default new
        cs.mcu_freq = 25_000_000.0;
        cs.time_avg = 10.0;
        cs.clock_avg = 250_000_000.0; // 10s * 25M/s
        cs.clock_est = (cs.time_avg, cs.clock_avg, cs.mcu_freq);
        cs.prediction_variance = (0.001 * cs.mcu_freq).powi(2); // (25000)^2 = 625,000,000
        cs.last_clock = 250_000_000; // Aligned with clock_avg for simplicity
        cs.min_half_rtt = 0.0005; // 0.5ms
        cs.min_rtt_time = 9.0;

        // Mock serial for set_clock_est
        let serial_hdl = Arc::new(SerialHdl::new()); // Placeholder new
        cs.serial = Some(serial_hdl);


        let mut params = HashMap::new();
        // MCU clock is 250_000_000 + 12_500_000 = 262_500_000 (0.5s later)
        // last_clock was 250_000_000. clock param is u32.
        // So, clock param should be 12_500_000.
        params.insert("clock".to_string(), ParamValue::Int(12_500_000));
        params.insert("#sent_time".to_string(), ParamValue::Float(10.5)); // 0.5s after time_avg
        params.insert("#receive_time".to_string(), ParamValue::Float(10.501)); // RTT = 1ms, half_rtt = 0.5ms

        cs._handle_clock_instance(params);

        assert_eq!(cs.queries_pending, 0);
        assert_eq!(cs.last_clock, 262_500_000);

        // Check if min_rtt is updated (it should be, 0.0005 is good)
        assert_eq!(cs.min_half_rtt, 0.0005); // Should remain same or update if this one is better
        assert_eq!(cs.min_rtt_time, 10.5); // Updated to current sent_time

        // Check if averages moved slightly towards the new sample
        // Initial: time_avg = 10.0, clock_avg = 250M
        // Sample: sent_time = 10.5, mcu_clock = 262.5M
        // DECAY = 1/30
        // diff_sent_time = 10.5 - 10.0 = 0.5
        // new time_avg = 10.0 + (1/30)*0.5 = 10.0 + 0.01666... = 10.01666...
        assert!((cs.time_avg - (10.0 + DECAY * 0.5)).abs() < 1e-9);
        // diff_clock = 262.5M - 250M = 12.5M
        // new clock_avg = 250M + (1/30)*12.5M = 250M + 416666.66... = 250416666.66...
        assert!((cs.clock_avg - (250_000_000.0 + DECAY * 12_500_000.0)).abs() < 1.0); // Check within 1 tick

        // clock_est should be updated
        // clock_est = (time_avg + min_half_rtt, clock_avg, new_freq)
        // new_freq calculation is complex, but it should be close to mcu_freq
        assert!((cs.clock_est.0 - (cs.time_avg + cs.min_half_rtt)).abs() < 1e-9);
        assert!((cs.clock_est.1 - cs.clock_avg).abs() < 1.0);
        assert!((cs.clock_est.2 - cs.mcu_freq).abs() < cs.mcu_freq * 0.01); // within 1%
    }
}


pub struct SecondarySync {
    clock_sync: ClockSync,
    main_sync: Arc<ClockSync>, // Requires ClockSync to be Send+Sync if used across threads
    clock_adj: (f64, f64),
    last_sync_time: f64,
}

impl SecondarySync {
    pub fn new(reactor: Reactor, main_sync: Arc<ClockSync>) -> Self {
        Self {
            clock_sync: ClockSync::new(reactor),
            main_sync,
            clock_adj: (0.0, 1.0), // (offset, frequency_multiplier)
            last_sync_time: 0.0,
        }
    }

    // connect is overridden from ClockSync
    pub fn connect(&mut self, serial: Arc<SerialHdl>) {
        self.clock_sync.connect(serial); // Call embedded ClockSync's connect
        self.clock_adj = (0.0, self.clock_sync.mcu_freq); // Initialize with own mcu_freq

        let curtime = self.clock_sync.reactor.monotonic();
        let main_print_time = self.main_sync.estimated_print_time(curtime);
        let local_print_time = self.clock_sync.estimated_print_time(curtime);

        self.clock_adj = (main_print_time - local_print_time, self.clock_sync.mcu_freq);
        self.calibrate_clock(0.0, curtime); // Initial calibration
    }

    // connect_file is also overridden
    pub fn connect_file(&mut self, serial: Arc<SerialHdl>, pace: bool) {
        self.clock_sync.connect_file(serial, pace);
        // Unlike Python's ClockSync.connect_file, SecondarySync's version does this:
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
        if main_mcu_freq == 0.0 { /* error or default */ return self.clock_adj; }

        let est_main_clock = (eventtime - ser_time) * ser_freq + ser_clock;
        let est_print_time = est_main_clock / main_mcu_freq;

        // Determine sync1_print_time and sync2_print_time
        let sync1_print_time = print_time.max(est_print_time);
        // Python: sync2_print_time = max(sync1_print_time + 4., self.last_sync_time, print_time + 2.5 * (print_time - est_print_time))
        let mut sync2_print_time = sync1_print_time + 4.0;
        sync2_print_time = sync2_print_time.max(self.last_sync_time);
        sync2_print_time = sync2_print_time.max(print_time + 2.5 * (print_time - est_print_time));

        if (sync2_print_time - sync1_print_time).abs() < f64::EPSILON { // Avoid division by zero if times are too close
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
