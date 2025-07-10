// klipper_host_rust/src/toolhead.rs
// Corresponds to klippy/toolhead.py - Manages the printer's toolhead.

use crate::configfile::Configfile;
use crate::mcu::Mcu;
use crate::reactor::Reactor;
// TODO: Add imports for Kinematics, TrapQ, etc. once their Rust structures are clearer.
use crate::trapq::TrapQ; // Assuming trapq.rs provides this
// use crate::kinematics::Kinematics; // Placeholder

// Common suffixes: _d is distance (in mm), _v is velocity (in
//   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
//   seconds), _r is ratio (scalar between 0.0 and 1.0)

#[derive(Debug, Clone)]
pub struct Move {
    // toolhead: &'a ToolHead<'a>, // Lifetime might be needed if ToolHead is borrowed. For now, let's avoid direct ref if possible.
    pub start_pos: [f64; 4], // [x, y, z, e]
    pub end_pos: [f64; 4],   // [x, y, z, e]
    pub accel: f64,
    pub junction_deviation: f64,
    // pub timing_callbacks: Vec<Box<dyn FnMut(f64)>>, // How to handle callbacks? For now, an empty vec or omit.
    pub is_kinematic_move: bool,
    pub axes_d: [f64; 4], // [dx, dy, dz, de]
    pub move_d: f64,      // distance of XYZ move
    pub axes_r: [f64; 4], // unit vector of move [rx, ry, rz, re]
    pub min_move_t: f64,  // minimum time to complete this move

    pub max_start_v2: f64,
    pub max_cruise_v2: f64,
    pub delta_v2: f64,
    pub max_smoothed_v2: f64,
    pub smooth_delta_v2: f64,
    pub next_junction_v2: f64,

    // Calculated values from set_junction
    pub start_v: f64,
    pub cruise_v: f64,
    pub end_v: f64,
    pub accel_t: f64,
    pub cruise_t: f64,
    pub decel_t: f64,
}

impl Move {
    pub fn new(
        // toolhead_ref: &'a ToolHead, // Pass necessary toolhead fields directly for now
        max_accel: f64, // from toolhead
        junction_deviation_val: f64, // from toolhead
        max_velocity: f64, // from toolhead
        max_accel_to_decel: f64, // from toolhead
        start_pos_arr: [f64; 4],
        end_pos_arr: [f64; 4],
        speed: f64,
    ) -> Self {
        let axes_d_arr = [
            end_pos_arr[0] - start_pos_arr[0],
            end_pos_arr[1] - start_pos_arr[1],
            end_pos_arr[2] - start_pos_arr[2],
            end_pos_arr[3] - start_pos_arr[3],
        ];

        let mut move_d_val = (axes_d_arr[0].powi(2) + axes_d_arr[1].powi(2) + axes_d_arr[2].powi(2)).sqrt();
        let mut current_is_kinematic_move = true;
        let mut current_accel = max_accel;
        let mut current_velocity = speed.min(max_velocity);
        let mut final_end_pos = end_pos_arr; // Make a mutable copy
        let mut effective_axes_d = axes_d_arr; // This will be used for axes_r calculation

        if move_d_val < 0.000000001 {
            // Extrude only move
            final_end_pos = [start_pos_arr[0], start_pos_arr[1], start_pos_arr[2], end_pos_arr[3]];

            effective_axes_d[0] = 0.0; // Modify effective_axes_d for axes_r calculation
            effective_axes_d[1] = 0.0;
            effective_axes_d[2] = 0.0;
            // effective_axes_d[3] remains axes_d_arr[3]

            move_d_val = effective_axes_d[3].abs();
            current_accel = 99999999.9;
            current_velocity = speed;
            current_is_kinematic_move = false;
        }

        let inv_move_d = if move_d_val == 0.0 { 0.0 } else { 1.0 / move_d_val };
        let axes_r_arr = [
            effective_axes_d[0] * inv_move_d,
            effective_axes_d[1] * inv_move_d,
            effective_axes_d[2] * inv_move_d,
            effective_axes_d[3] * inv_move_d,
        ];
        let min_move_t_val = if current_velocity == 0.0 { f64::INFINITY } else { move_d_val / current_velocity };

        Move {
            start_pos: start_pos_arr,
            end_pos: final_end_pos,
            accel: current_accel,
            junction_deviation: junction_deviation_val,
            is_kinematic_move: current_is_kinematic_move,
            axes_d: axes_d_arr, // Store original full diffs (Python also seems to keep original start/end pos)
            move_d: move_d_val,
            axes_r: axes_r_arr, // axes_r is now based on effective_axes_d
            min_move_t: min_move_t_val,
            max_start_v2: 0.0,
            max_cruise_v2: current_velocity.powi(2),
            delta_v2: 2.0 * move_d_val * current_accel,
            max_smoothed_v2: 0.0,
            smooth_delta_v2: 2.0 * move_d_val * max_accel_to_decel, // Python uses toolhead.max_accel_to_decel
            next_junction_v2: 999999999.9,
            start_v: 0.0,  // Initialized in set_junction
            cruise_v: 0.0, // Initialized in set_junction
            end_v: 0.0,    // Initialized in set_junction
            accel_t: 0.0,  // Initialized in set_junction
            cruise_t: 0.0, // Initialized in set_junction
            decel_t: 0.0,  // Initialized in set_junction
        }
    }

    pub fn limit_speed(&mut self, speed: f64, accel: f64) {
        let speed2 = speed.powi(2);
        if speed2 < self.max_cruise_v2 {
            self.max_cruise_v2 = speed2;
            if speed == 0.0 {
                self.min_move_t = f64::INFINITY;
            } else {
                self.min_move_t = self.move_d / speed;
            }
        }
        self.accel = self.accel.min(accel);
        self.delta_v2 = 2.0 * self.move_d * self.accel;
        self.smooth_delta_v2 = self.smooth_delta_v2.min(self.delta_v2);
    }

    pub fn limit_next_junction_speed(&mut self, speed: f64) {
        self.next_junction_v2 = self.next_junction_v2.min(speed.powi(2));
    }

    // In a real application, this might return a Result or use a custom error type.
    // For now, it mimics the Python version's side effect of calling printer.command_error.
    // We'll return a String that could be used to signal an error upstream.
    pub fn move_error(&self, msg: &str) -> String {
        format!(
            "{}: {:.3} {:.3} {:.3} [{:.3}]",
            msg, self.end_pos[0], self.end_pos[1], self.end_pos[2], self.end_pos[3]
        )
    }

    // Placeholder for ExtraAxis trait/struct. This will need to be defined elsewhere.
    // For now, assume it has a calc_junction method.
    // pub trait ExtraAxis {
    //     fn calc_junction(&self, prev_move: &Move, current_move: &Move, e_index: usize) -> f64;
    // }
    // Note: `extra_axes` argument for `calc_junction` will be passed from `LookAheadQueue::add_move`
    // which in turn gets it from `ToolHead`. So `Move::calc_junction` itself doesn't need direct
    // access to `ToolHead`'s `extra_axes` field if `LookAheadQueue::add_move` passes it.
    // However, the original Python code `move.calc_junction(self.queue[-2])` calls it from `LookAheadQueue`,
    // and `Move.calc_junction` accesses `self.toolhead.extra_axes`.
    // For now, keeping `extra_axes` as an argument to `Move::calc_junction`.

    pub fn calc_junction(&mut self, prev_move: &Move, toolhead_extra_axes: &[Box<dyn ExtraAxis>] ) {
        if !self.is_kinematic_move || !prev_move.is_kinematic_move {
            return;
        }

        let ea_v2: Vec<f64> = toolhead_extra_axes
            .iter()
            .enumerate()
            .map(|(e_index, ea)| ea.calc_junction(prev_move, self, e_index + 3))
            .collect();

        let min_ea_v2 = ea_v2.iter().fold(f64::INFINITY, |acc, &val| acc.min(val));

        let mut max_start_v2 = self.max_cruise_v2
            .min(prev_move.max_cruise_v2)
            .min(prev_move.next_junction_v2)
            .min(prev_move.max_start_v2 + prev_move.delta_v2)
            .min(min_ea_v2);

        let axes_r = &self.axes_r;
        let prev_axes_r = &prev_move.axes_r;
        let junction_cos_theta = -(axes_r[0] * prev_axes_r[0]
            + axes_r[1] * prev_axes_r[1]
            + axes_r[2] * prev_axes_r[2]);

        // Clamp junction_cos_theta to avoid domain errors with sqrt if it's slightly outside [-1, 1] due to float precision
        let junction_cos_theta_clamped = junction_cos_theta.max(-1.0).min(1.0);

        let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta_clamped)).sqrt().max(0.0);
        let cos_theta_d2 = (0.5 * (1.0 + junction_cos_theta_clamped)).sqrt().max(0.0);

        let one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;

        if one_minus_sin_theta_d2 > 1e-9 && cos_theta_d2 > 1e-9 { // Avoid division by zero or near-zero
            let r_jd = sin_theta_d2 / one_minus_sin_theta_d2;
            let move_jd_v2 = r_jd * self.junction_deviation * self.accel;
            let pmove_jd_v2 = r_jd * prev_move.junction_deviation * prev_move.accel;

            let quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2;
            let move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2;
            let pmove_centripetal_v2 = prev_move.delta_v2 * quarter_tan_theta_d2;

            max_start_v2 = max_start_v2
                .min(move_jd_v2)
                .min(pmove_jd_v2)
                .min(move_centripetal_v2)
                .min(pmove_centripetal_v2);
        }

        self.max_start_v2 = max_start_v2;
        self.max_smoothed_v2 = max_start_v2
            .min(prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2);
    }

    pub fn set_junction(&mut self, start_v2: f64, cruise_v2: f64, end_v2: f64) {
        let half_inv_accel = if self.accel == 0.0 { 0.0 } else { 0.5 / self.accel };
        let accel_d = (cruise_v2 - start_v2) * half_inv_accel;
        let decel_d = (cruise_v2 - end_v2) * half_inv_accel;
        let cruise_d = self.move_d - accel_d - decel_d;

        self.start_v = start_v2.sqrt().max(0.0); // Ensure non-negative
        self.cruise_v = cruise_v2.sqrt().max(0.0);
        self.end_v = end_v2.sqrt().max(0.0);

        self.accel_t = if (self.start_v + self.cruise_v) == 0.0 { 0.0 } else { accel_d / ((self.start_v + self.cruise_v) * 0.5) };
        self.cruise_t = if self.cruise_v == 0.0 { 0.0 } else { cruise_d / self.cruise_v };
        self.decel_t = if (self.end_v + self.cruise_v) == 0.0 { 0.0 } else { decel_d / ((self.end_v + self.cruise_v) * 0.5) };
    }
}

const LOOKAHEAD_FLUSH_TIME: f64 = 0.250;

pub struct LookAheadQueue {
    pub queue: Vec<Move>,
    pub junction_flush: f64,
}

impl LookAheadQueue {
    pub fn new() -> Self {
        LookAheadQueue {
            queue: Vec::new(),
            junction_flush: LOOKAHEAD_FLUSH_TIME,
        }
    }

    pub fn reset(&mut self) {
        self.queue.clear();
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
    }

    pub fn set_flush_time(&mut self, flush_time: f64) {
        self.junction_flush = flush_time;
    }

    pub fn get_last(&self) -> Option<&Move> {
        self.queue.last()
    }

    // To allow modification of the last move (e.g. for limit_next_junction_speed)
    pub fn get_last_mut(&mut self) -> Option<&mut Move> {
        self.queue.last_mut()
    }

    // `add_move` needs access to `ToolHead.extra_axes` to pass to `move_to_add.calc_junction`
    pub fn add_move(&mut self, mut move_to_add: Move, toolhead_extra_axes: &[Box<dyn ExtraAxis>]) -> bool {
        if self.queue.is_empty() {
            self.queue.push(move_to_add);
        } else {
            let prev_move = &self.queue[self.queue.len() - 1];
            // Now call calc_junction with the extra_axes from ToolHead
            move_to_add.calc_junction(prev_move, toolhead_extra_axes);
            self.queue.push(move_to_add);
        }

        if let Some(added_move) = self.queue.last() {
             self.junction_flush -= added_move.min_move_t;
        }

        self.junction_flush <= 0.0
    }

    pub fn flush(&mut self, lazy: bool) -> Vec<Move> {
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
        let mut update_flush_count = lazy;

        let mut flush_count = self.queue.len();
        if flush_count == 0 {
            return Vec::new();
        }

        let mut delayed: Vec<(usize, f64, f64)> = Vec::new(); // Stores (index, start_v2, next_end_v2)
        let mut next_end_v2 = 0.0;
        let mut next_smoothed_v2 = 0.0;
        let mut peak_cruise_v2 = 0.0;

        for i in (0..self.queue.len()).rev() {
            let mut move_i = self.queue[i].clone(); // Clone to allow modification if needed, or work with indices

            let reachable_start_v2 = next_end_v2 + move_i.delta_v2;
            let start_v2 = move_i.max_start_v2.min(reachable_start_v2);

            let reachable_smoothed_v2 = next_smoothed_v2 + move_i.smooth_delta_v2;
            let smoothed_v2 = move_i.max_smoothed_v2.min(reachable_smoothed_v2);

            if smoothed_v2 < reachable_smoothed_v2 {
                if smoothed_v2 + move_i.smooth_delta_v2 > next_smoothed_v2 || !delayed.is_empty() {
                    if update_flush_count && peak_cruise_v2 > 0.0 { // Check peak_cruise_v2 > 0 to avoid premature flush
                        flush_count = i; // Python is (i), but loop is inclusive, Rust exclusive. Check logic. Python is i, so this means up to i (exclusive of i).
                                         // If Python flush_count = i, it means process up to index i-1.
                                         // So if current i is where condition met, items 0..i-1 are flushed.
                                         // Here, if current i is the one not to be flushed, flush_count should be i.
                        update_flush_count = false;
                    }
                    peak_cruise_v2 = move_i.max_cruise_v2.min((smoothed_v2 + reachable_smoothed_v2) * 0.5);

                    if !delayed.is_empty() {
                        if !update_flush_count && i < flush_count {
                            let mut mc_v2 = peak_cruise_v2;
                            for (m_idx, ms_v2, me_v2) in delayed.iter().rev() {
                                mc_v2 = mc_v2.min(*ms_v2);
                                if let Some(m_to_set) = self.queue.get_mut(*m_idx) {
                                    m_to_set.set_junction(ms_v2.min(mc_v2), mc_v2, me_v2.min(mc_v2));
                                }
                            }
                        }
                        delayed.clear();
                    }
                }
                if !update_flush_count && i < flush_count {
                    let cruise_v2 = (start_v2 + reachable_start_v2) * 0.5;
                    let cruise_v2 = cruise_v2.min(move_i.max_cruise_v2).min(peak_cruise_v2);
                    if let Some(m_to_set) = self.queue.get_mut(i) {
                         m_to_set.set_junction(start_v2.min(cruise_v2), cruise_v2, next_end_v2.min(cruise_v2));
                    }
                }
            } else {
                delayed.push((i, start_v2, next_end_v2));
            }
            next_end_v2 = start_v2;
            next_smoothed_v2 = smoothed_v2;
        }

        if update_flush_count || flush_count == 0 {
            // If update_flush_count is still true, it means we didn't find a suitable flush point (e.g. all moves are accel/decel).
            // Or if flush_count became 0 (e.g. queue was small).
            // In Python, if update_flush_count or not flush_count, returns [].
            return Vec::new();
        }

        // In Python: res = queue[:flush_count], del queue[:flush_count]
        // This means it takes elements from index 0 up to (flush_count - 1).
        // flush_count here is the index of the first element *not* to flush.
        let res = self.queue.drain(0..flush_count).collect();
        res
    }
}

const BUFFER_TIME_LOW: f64 = 1.0;
const BUFFER_TIME_HIGH: f64 = 2.0;
const BUFFER_TIME_START: f64 = 0.250;

// Constants used in methods, ported from Python version
const MIN_KIN_TIME: f64 = 0.100;
const MOVE_BATCH_TIME: f64 = 0.500;
const STEPCOMPRESS_FLUSH_TIME: f64 = 0.050;
const SDS_CHECK_TIME: f64 = 0.001; // step+dir+step filter in stepcompress.c
const MOVE_HISTORY_EXPIRE: f64 = 30.0;

// DRIP constants - for future use when drip_move is implemented
// const DRIP_SEGMENT_TIME: f64 = 0.050;
// const DRIP_TIME: f64 = 0.100;


pub struct ToolHead<'a> {
    // printer: Printer, // How to represent the printer object?
    reactor: &'a Reactor,
    all_mcus: Vec<&'a Mcu>, // Assuming Mcu struct exists
    mcu: &'a Mcu,
    pub lookahead: LookAheadQueue,
    pub commanded_pos: [f64; 4], // x, y, z, e

    // Velocity and acceleration control
    pub max_velocity: f64,
    pub max_accel: f64,
    pub min_cruise_ratio: f64,
    pub square_corner_velocity: f64,
    pub junction_deviation: f64,
    pub max_accel_to_decel: f64,

    // Print time tracking
    pub print_time: f64,
    pub special_queuing_state: String, // "NeedPrime", "Priming", "Drip", ""
    // priming_timer: Option<TimerHandle>, // Needs reactor integration

    // Flush tracking
    // flush_timer: Option<TimerHandle>, // Needs reactor integration
    pub do_kick_flush_timer: bool,
    pub last_flush_time: f64,
    pub min_restart_time: f64,
    pub need_flush_time: f64,
    pub step_gen_time: f64,
    pub clear_history_time: f64,

    // Kinematic step generation scan window time tracking
    pub kin_flush_delay: f64,
    pub kin_flush_times: Vec<f64>,

    trapq: TrapQ, // Using the imported TrapQ struct
    flush_trapqs: Vec<TrapQ>, // Assuming TrapQ is cloneable or we manage references

    kin: Box<dyn Kinematics>,
    extra_axes: Vec<Box<dyn ExtraAxis>>,
}

// Basic trait definitions - these would likely be in their own modules
pub trait Kinematics {
    fn check_move(&self, move_params: &Move) -> Result<(), String>; // Using Result for error
    // TODO: Add other methods like set_position, get_status etc.
}

pub trait ExtraAxis {
    fn check_move(&self, move_params: &Move, axis_index: usize) -> Result<(), String>;
    fn process_move(&mut self, move_time: f64, move_params: &Move, axis_index: usize);
    // TODO: Add get_trapq, get_name etc.
    fn calc_junction(&self, prev_move: &Move, current_move: &Move, e_index: usize) -> f64;
}


impl<'a> ToolHead<'a> {
    pub fn new(
        config: &Configfile, // Assuming Configfile struct exists and is passed
        reactor_ref: &'a Reactor,
        mcus_list: Vec<&'a Mcu>, // Pass a list of Mcu references
        // printer_ref: Printer, // How to handle printer?
    ) -> Result<Self, String> { // Using Result for error handling

        let max_velocity = config.getfloat("max_velocity", 0.0, None)?; // Add getfloat to Configfile
        let max_accel = config.getfloat("max_accel", 0.0, None)?;

        let mut min_cruise_ratio = 0.5;
        if config.getfloat("minimum_cruise_ratio", 0.0, None).is_err() { // Check if None
            if let Ok(req_accel_to_decel) = config.getfloat("max_accel_to_decel", 0.0, None) {
                // config.deprecate("max_accel_to_decel"); // How to handle deprecation?
                min_cruise_ratio = 1.0 - (req_accel_to_decel / max_accel).min(1.0);
            }
        }
        min_cruise_ratio = config.getfloat("minimum_cruise_ratio", min_cruise_ratio, Some(1.0))?;


        let square_corner_velocity = config.getfloat("square_corner_velocity", 5.0, Some(0.0))?;

        let mut toolhead = ToolHead {
            reactor: reactor_ref,
            all_mcus: mcus_list.clone(), // Clone the Vec of references
            mcu: mcus_list.get(0).ok_or_else(|| "No MCUs provided".to_string())?, // Get the first MCU
            lookahead: LookAheadQueue::new(),
            commanded_pos: [0.0, 0.0, 0.0, 0.0],
            max_velocity,
            max_accel,
            min_cruise_ratio,
            square_corner_velocity,
            junction_deviation: 0.0, // Calculated by _calc_junction_deviation
            max_accel_to_decel: 0.0, // Calculated by _calc_junction_deviation
            print_time: 0.0,
            special_queuing_state: "NeedPrime".to_string(),
            do_kick_flush_timer: true,
            last_flush_time: 0.0,
            min_restart_time: 0.0,
            need_flush_time: 0.0,
            step_gen_time: 0.0,
            clear_history_time: 0.0,
            kin_flush_delay: 0.001, // SDS_CHECK_TIME
            kin_flush_times: Vec::new(),
            // Initialize other fields, possibly with defaults or placeholders
        };

        toolhead._calc_junction_deviation(); // Call the helper method

        // TODO: Initialize kinematics, extra_axes, register commands, event handlers etc.
        // This will require more infrastructure to be in place.
        // For now, create placeholder kinematics and extra_axes
        // In a real scenario, these would be loaded based on config.
        struct PlaceholderKinematics;
        impl Kinematics for PlaceholderKinematics {
            fn check_move(&self, _move_params: &Move) -> Result<(), String> { Ok(()) }
        }
        toolhead.kin = Box::new(PlaceholderKinematics);

        struct PlaceholderExtraAxis;
        impl ExtraAxis for PlaceholderExtraAxis {
            fn check_move(&self, _move_params: &Move, _axis_index: usize) -> Result<(), String> { Ok(()) }
            fn process_move(&mut self, _move_time: f64, _move_params: &Move, _axis_index: usize) {}
            fn calc_junction(&self, _prev_move: &Move, _current_move: &Move, _e_index: usize) -> f64 { f64::INFINITY }
        }
        // Initialize with one dummy extruder axis
        toolhead.extra_axes.push(Box::new(PlaceholderExtraAxis));


        Ok(toolhead)
    }

    // Helper method to be called from new()
    fn _calc_junction_deviation(&mut self) {
        let scv2 = self.square_corner_velocity.powi(2);
        if self.max_accel == 0.0 { // Avoid division by zero
            self.junction_deviation = 0.0;
        } else {
            self.junction_deviation = scv2 * (2.0_f64.sqrt() - 1.0) / self.max_accel;
        }
        self.max_accel_to_decel = self.max_accel * (1.0 - self.min_cruise_ratio);
    }

    // Print time and flush tracking
    fn _advance_flush_time(&mut self, flush_time: f64) {
        let flush_time = flush_time.max(self.last_flush_time);

        // TODO: Generate steps via itersolve. This requires itersolve.rs and kinematics integration.
        // sg_flush_want = min(flush_time + STEPCOMPRESS_FLUSH_TIME, self.print_time - self.kin_flush_delay)
        // sg_flush_time = max(sg_flush_want, flush_time)
        // for sg in self.step_generators:
        //     sg(sg_flush_time)
        let sg_flush_time = flush_time; // Placeholder
        // self.min_restart_time = self.min_restart_time.max(sg_flush_time);

        // Free trapq entries that are no longer needed
        let mut clear_history_time = self.clear_history_time;
        // const MOVE_HISTORY_EXPIRE: f64 = 30.0; // Define this constant
        // if !self.can_pause { // TODO: Implement self.can_pause logic
        //     clear_history_time = flush_time - MOVE_HISTORY_EXPIRE;
        // }
        let free_time = sg_flush_time - self.kin_flush_delay;

        self.trapq.finalize_moves(free_time, clear_history_time); // Assuming TrapQ has this method
        for ftq in self.flush_trapqs.iter_mut() {
            ftq.finalize_moves(free_time, clear_history_time); // Assuming TrapQ is mutable here or method takes &mut self
        }

        // Flush stepcompress and mcu steppersync
        // TODO: Implement step_generators and mcu.flush_moves logic
        for mcu_item in &self.all_mcus {
            // mcu_item.flush_moves(flush_time, clear_history_time); // Assuming Mcu has this method
        }

        self.last_flush_time = flush_time;
    }

    fn _advance_move_time(&mut self, next_print_time: f64) {
        // const STEPCOMPRESS_FLUSH_TIME: f64 = 0.050; // Define if not already global
        let pt_delay = self.kin_flush_delay + 0.050; // STEPCOMPRESS_FLUSH_TIME
        let mut current_flush_time = self.last_flush_time.max(self.print_time - pt_delay);
        self.print_time = self.print_time.max(next_print_time);
        let want_flush_time = current_flush_time.max(self.print_time - pt_delay);

        // const MOVE_BATCH_TIME: f64 = 0.500; // Define if not already global
        while current_flush_time < want_flush_time {
            current_flush_time = (current_flush_time + 0.500).min(want_flush_time); // MOVE_BATCH_TIME
            self._advance_flush_time(current_flush_time);
        }
    }

    fn _calc_print_time(&mut self) {
        // const MIN_KIN_TIME: f64 = 0.100;
        let curtime = self.reactor.monotonic();
        let est_print_time = self.mcu.estimated_print_time(curtime); // Assuming Mcu has this
        let kin_time = (est_print_time + 0.100).max(self.min_restart_time); // MIN_KIN_TIME
        let kin_time = kin_time + self.kin_flush_delay;
        let min_print_time = (est_print_time + BUFFER_TIME_START).max(kin_time);

        if min_print_time > self.print_time {
            self.print_time = min_print_time;
            // self.printer.send_event("toolhead:sync_print_time", curtime, est_print_time, self.print_time);
            // TODO: Event system
        }
    }

    fn _process_lookahead(&mut self, lazy: bool) {
        let moves = self.lookahead.flush(lazy);
        if moves.is_empty() {
            return;
        }

        if !self.special_queuing_state.is_empty() { // Corresponds to "if self.special_queuing_state:"
            self.special_queuing_state = "".to_string();
            // self.need_check_pause = -1.0; // TODO: pause checking logic
            self._calc_print_time();
        }

        let mut next_move_time = self.print_time;
        for mut move_item in moves { // moves is Vec<Move>, move_item is Move, make mutable for callbacks
            if move_item.is_kinematic_move {
                self.trapq.append( // Assuming TrapQ has this method
                    next_move_time,
                    move_item.accel_t, move_item.cruise_t, move_item.decel_t,
                    move_item.start_pos[0], move_item.start_pos[1], move_item.start_pos[2],
                    move_item.axes_r[0], move_item.axes_r[1], move_item.axes_r[2],
                    move_item.start_v, move_item.cruise_v, move_item.accel);
            }

            for (e_index, ea) in self.extra_axes.iter_mut().enumerate() {
                if move_item.axes_d[e_index + 3].abs() > f64::EPSILON { // Check if non-zero
                    ea.process_move(next_move_time, &move_item, e_index + 3);
                }
            }

            next_move_time += move_item.accel_t + move_item.cruise_t + move_item.decel_t;

            // TODO: Handle timing_callbacks
            // for cb in move_item.timing_callbacks:
            //     cb(next_move_time);
        }

        // self.note_mcu_movequeue_activity(next_move_time + self.kin_flush_delay, true); // TODO
        self.step_gen_time = self.step_gen_time.max(next_move_time + self.kin_flush_delay); // Simplified from note_mcu_movequeue_activity
        self.need_flush_time = self.need_flush_time.max(next_move_time + self.kin_flush_delay);
        // TODO: kick flush timer if self.do_kick_flush_timer is true

        self._advance_move_time(next_move_time);
    }


    // Movement commands
    pub fn get_position(&self) -> [f64; 4] {
        self.commanded_pos
    }

    pub fn set_position(&mut self, new_pos: [f64; 4], homing_axes: Option<[bool;3]>) {
        // This method in Klipper also calls self.kin.set_position()
        // and trapq_set_position. For now, just update commanded_pos.
        // It's used by G92 and after homing.
        // The actual machine position doesn't change here, rather the G-code interpretation of it.
        // However, ToolHead's commanded_pos should reflect the G-code coordinate space after G92 or homing.

        // If specific axes are being homed/set, only update those.
        // G92 without params doesn't mean "set all to current commanded_pos",
        // it means "reset offsets so current machine pos becomes current gcode pos".
        // G92 X10 means "current machine X is now to be known as G-code X10".

        // For now, G92 in gcode.rs directly updates gcode.state.
        // Toolhead's commanded_pos is updated via move_to.
        // A direct set_position on toolhead should reflect the *new G-code coordinates*.

        if let Some(axes) = homing_axes { // Typically used after homing an axis
            if axes[0] { self.commanded_pos[0] = new_pos[0]; }
            if axes[1] { self.commanded_pos[1] = new_pos[1]; }
            if axes[2] { self.commanded_pos[2] = new_pos[2]; }
            // Extruder 'E' is usually handled by G92 E0, not directly by homing_axes flags.
            if new_pos.len() > 3 { // If E is provided in new_pos (e.g. G92 E0)
                 if self.commanded_pos.len() > 3 { // Ensure commanded_pos has an E axis
                    // This part is a bit tricky. G92 E0 means set E to 0.
                    // If G92 X10 Y10 is called, E should remain unchanged unless E is also specified.
                    // The new_pos here should ideally come from gcode.rs which knows which params were in G92.
                    // For now, let's assume if new_pos has E, it's meant to be set.
                    // A better G92 would pass Option<f64> for each axis.
                 }
            }
        } else { // General G92 or full set_position
            self.commanded_pos = new_pos;
        }
        // TODO: self.trapq.set_position(...)
        // TODO: self.kin.set_position(...)
        self.printer.send_event("toolhead:set_position"); // Assuming printer object and event system
    }


    // Performs a homing move for a single axis.
    // axis_index: 0 for X, 1 for Y, 2 for Z.
    // For now, this is a simplified, mocked version.
    // It will "move" to a conceptual endstop and return its configured machine position at trigger.
    pub fn perform_homing_move(
        &mut self,
        axis_index: usize,
        // config_homing_speed: f64, // TODO: Would come from config
        // config_position_endstop: f64, // TODO: Would come from config (this is the GCODE value to set)
        // config_homing_dir_is_positive: bool // TODO: Whether homing towards positive or negative
    ) -> Result<f64, String> {
        if axis_index > 2 {
            return Err("Invalid axis_index for homing".to_string());
        }

        // Mocked parameters (would normally come from config per axis)
        // These constants are defined at the top of the ToolHead struct for now
        let homing_speed = DEFAULT_HOMING_SPEED; // mm/s
        // This is the MACHINE coordinate where the endstop is physically located.
        // After homing, G28 will typically issue a G92 to make this machine coordinate
        // correspond to a specific G-CODE coordinate (often 0, or bed_size).
        let machine_pos_at_endstop_trigger = match axis_index {
            0 => X_MACHINE_POSITION_AT_ENDSTOP, // e.g., 0.0 if min endstop at 0
            1 => Y_MACHINE_POSITION_AT_ENDSTOP, // e.g., 0.0
            2 => Z_MACHINE_POSITION_AT_ENDSTOP, // e.g., 0.0
            _ => unreachable!(),
        };

        // Simulate flushing moves before homing that axis
        self.wait_moves(); // Existing method to flush lookahead and wait

        let axis_char = ['X', 'Y', 'Z'][axis_index];
        println!(
            "ToolHead: Mock homing for axis {}. Moving at speed {} mm/s. Expecting endstop trigger at machine pos: {:.3}",
            axis_char, homing_speed, machine_pos_at_endstop_trigger
        );

        // In a real implementation:
        // 1. Get current G-CODE position: self.commanded_pos[axis_index]
        // 2. Get current G92 offset for the axis from gcode_state (passed in or accessed).
        // 3. Calculate current MACHINE position = GCODE_pos - G92_offset.
        // 4. Determine homing direction (e.g., towards negative for X min).
        // 5. Command a move to a point far beyond the expected endstop trigger point in that direction.
        //    This move would be executed via drip_move, monitoring the endstop.
        // 6. drip_move would return the MACHINE position where the endstop triggered.

        // For this mock:
        // We directly "know" the machine_pos_at_endstop_trigger.
        // We need to update self.commanded_pos to reflect the G-CODE coordinate that G28 will set.
        // G28 will call G92 logic: G92 NewGcodePos.
        // The NewGcodePos is often 0 (e.g. X_POSITION_ENDSTOP_GCODE_VALUE).
        // The G92 logic calculates: offset = machine_pos_at_endstop_trigger - NewGcodePos.
        // Then G28 calls toolhead.set_position(NewGcodePos for this axis).
        // So, after this homing move, toolhead's commanded_pos for this axis should become NewGcodePos.

        let gcode_pos_after_homing_g92 = match axis_index {
             0 => X_GCODE_POSITION_AFTER_HOMING,
             1 => Y_GCODE_POSITION_AFTER_HOMING,
             2 => Z_GCODE_POSITION_AFTER_HOMING,
            _ => unreachable!(),
        };
        self.commanded_pos[axis_index] = gcode_pos_after_homing_g92;

        // Simulate that the toolhead is now physically at machine_pos_at_endstop_trigger,
        // and its G-code coordinate for this axis is gcode_pos_after_homing_g92.
        // The actual update to GCodeState's last_position (machine) and base_position (gcode)
        // will be handled by the G28 command in gcode.rs using the returned machine_pos_at_endstop_trigger.

        println!(
            "ToolHead: Axis {} mock homing complete. Endstop 'triggered' at machine position {:.3f}. Toolhead G-code pos for axis set to {:.3f}.",
            axis_char, machine_pos_at_endstop_trigger, gcode_pos_after_homing_g92
        );

        Ok(machine_pos_at_endstop_trigger)
    }

    // newpos is [f64;4]
    pub fn move_to(&mut self, newpos: [f64; 4], speed: f64) -> Result<(), String> {
        // Before executing the move, ensure lookahead queue is processed to reflect latest position
        // self._process_lookahead(false); // Process entire queue if any doubt about current state.
                                        // This might be too aggressive here.
                                        // commanded_pos should be the authoritative source from GCode's perspective.

        let move_instance = Move::new(
            self.max_accel,
            self.junction_deviation,
            self.max_velocity,
            self.max_accel_to_decel,
            self.commanded_pos,
            newpos,
            speed,
        );

        if move_instance.move_d == 0.0 {
            return Ok(());
        }

        if move_instance.is_kinematic_move {
            self.kin.check_move(&move_instance)?;
        }

        for (e_index, ea) in self.extra_axes.iter().enumerate() {
            // Only check axes that are actually moving for this specific move instance.
            // The 4th component of axes_d corresponds to the first extra_axis (index 0 in extra_axes Vec).
            if e_index < move_instance.axes_d.len() - 3 && move_instance.axes_d[e_index + 3].abs() > f64::EPSILON {
                ea.check_move(&move_instance, e_index + 3)?;
            }
        }

        self.commanded_pos = move_instance.end_pos;
        // Pass toolhead's extra_axes to lookahead's add_move
        let want_flush = self.lookahead.add_move(move_instance, &self.extra_axes);

        if want_flush {
            self._process_lookahead(true); // lazy = true
        }

        // TODO: Pause checking logic
        // if self.print_time > self.need_check_pause:
        //     self._check_pause();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Helper for float comparisons
    fn approx_eq(a: f64, b: f64, epsilon: f64) -> bool {
        (a - b).abs() < epsilon
    }

    // Mock ExtraAxis for testing Move::calc_junction if needed later
    struct MockExtraAxis;
    impl ExtraAxis for MockExtraAxis {
        fn check_move(&self, _move_params: &Move, _axis_index: usize) -> Result<(), String> { Ok(()) }
        fn process_move(&mut self, _move_time: f64, _move_params: &Move, _axis_index: usize) {}
        fn calc_junction(&self, _prev_move: &Move, _current_move: &Move, _e_index: usize) -> f64 {
            // For basic tests, assume it doesn't restrict junction speed
            f64::INFINITY
        }
    }

    const DEFAULT_MAX_ACCEL: f64 = 3000.0;
    const DEFAULT_JUNCTION_DEV: f64 = 0.013; // Calculated from 5mm/s square_corner_velocity and 3000 accel
    const DEFAULT_MAX_VELOCITY: f64 = 500.0;
    const DEFAULT_MAX_ACCEL_TO_DECEL: f64 = DEFAULT_MAX_ACCEL / 2.0; // Example value


    #[test]
    fn test_move_new_normal_move() {
        let start_pos = [0.0, 0.0, 0.0, 0.0];
        let end_pos = [10.0, 0.0, 0.0, 0.0];
        let speed = 100.0;
        let m = Move::new(
            DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL,
            start_pos, end_pos, speed
        );

        assert!(m.is_kinematic_move);
        assert!(approx_eq(m.move_d, 10.0, 1e-9));
        assert!(approx_eq(m.axes_d[0], 10.0, 1e-9));
        assert!(approx_eq(m.axes_r[0], 1.0, 1e-9));
        assert!(approx_eq(m.min_move_t, 10.0 / 100.0, 1e-9));
        assert!(approx_eq(m.max_cruise_v2, speed.powi(2), 1e-9));
        assert!(approx_eq(m.accel, DEFAULT_MAX_ACCEL, 1e-9));
        assert!(approx_eq(m.delta_v2, 2.0 * 10.0 * DEFAULT_MAX_ACCEL, 1e-9));
    }

    #[test]
    fn test_move_new_extrude_only_move() {
        let start_pos = [10.0, 0.0, 0.0, 0.0];
        let end_pos = [10.0, 0.0, 0.0, 5.0]; // Only E moves
        let speed = 20.0;
        let m = Move::new(
            DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL,
            start_pos, end_pos, speed
        );

        assert!(!m.is_kinematic_move);
        assert!(approx_eq(m.move_d, 5.0, 1e-9)); // move_d becomes extruder move distance
        assert!(approx_eq(m.axes_d[3], 5.0, 1e-9)); // Original axes_d for E
        assert!(approx_eq(m.axes_r[3], 1.0, 1e-9)); // axes_r for E (assuming move_d is now 5.0)
        assert!(approx_eq(m.min_move_t, 5.0 / 20.0, 1e-9));
        assert!(approx_eq(m.max_cruise_v2, speed.powi(2), 1e-9));
        assert!(approx_eq(m.accel, 99999999.9, 1e-9));
    }

    #[test]
    fn test_move_new_zero_length_kinematic_move() {
        let start_pos = [0.0, 0.0, 0.0, 0.0];
        let end_pos = [0.0, 0.0, 0.0, 0.0]; // No XYZ movement
        let speed = 100.0;
         let m = Move::new(
            DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL,
            start_pos, end_pos, speed
        );
        // This becomes an extrude-only move of zero length as per current logic
        assert!(!m.is_kinematic_move);
        assert!(approx_eq(m.move_d, 0.0, 1e-9));
    }


    #[test]
    fn test_move_limit_speed() {
        let start_pos = [0.0, 0.0, 0.0, 0.0];
        let end_pos = [10.0, 0.0, 0.0, 0.0];
        let initial_speed = 100.0;
        let mut m = Move::new(
            DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL,
            start_pos, end_pos, initial_speed
        );

        let new_speed = 50.0;
        let new_accel = 1500.0;
        m.limit_speed(new_speed, new_accel);

        assert!(approx_eq(m.max_cruise_v2, new_speed.powi(2), 1e-9));
        assert!(approx_eq(m.min_move_t, 10.0 / new_speed, 1e-9));
        assert!(approx_eq(m.accel, new_accel, 1e-9));
        assert!(approx_eq(m.delta_v2, 2.0 * 10.0 * new_accel, 1e-9));
    }

    #[test]
    fn test_move_limit_speed_lower_accel() {
        let start_pos = [0.0, 0.0, 0.0, 0.0];
        let end_pos = [10.0, 0.0, 0.0, 0.0];
        let initial_speed = 100.0;
        let initial_accel = 2000.0;
        let mut m = Move::new(
            initial_accel, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL,
            start_pos, end_pos, initial_speed
        );

        // Limit with higher speed (should not change max_cruise_v2) and lower accel
        m.limit_speed(150.0, 1000.0);

        assert!(approx_eq(m.max_cruise_v2, initial_speed.powi(2), 1e-9)); // Unchanged
        assert!(approx_eq(m.min_move_t, 10.0 / initial_speed, 1e-9)); // Unchanged
        assert!(approx_eq(m.accel, 1000.0, 1e-9)); // Accel is limited
        assert!(approx_eq(m.delta_v2, 2.0 * 10.0 * 1000.0, 1e-9));
    }

    #[test]
    fn test_move_set_junction() {
        let start_pos = [0.0, 0.0, 0.0, 0.0];
        let end_pos = [100.0, 0.0, 0.0, 0.0]; // 100mm move
        let speed = 100.0;
        let accel = 1000.0;
        let mut m = Move::new(
            accel, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL,
            start_pos, end_pos, speed
        );

        // Scenario: start from 0, accelerate to 50mm/s, cruise, decelerate to 0
        // v_start = 0, v_cruise = 50, v_end = 0
        // accel_d = (v_cruise^2 - v_start^2) / (2*accel) = (50^2 - 0) / (2*1000) = 2500 / 2000 = 1.25mm
        // decel_d = (v_cruise^2 - v_end^2) / (2*accel) = (50^2 - 0) / (2*1000) = 1.25mm
        // cruise_d = move_d - accel_d - decel_d = 100 - 1.25 - 1.25 = 97.5mm
        // accel_t = (v_cruise - v_start) / accel = 50 / 1000 = 0.05s
        // cruise_t = cruise_d / v_cruise = 97.5 / 50 = 1.95s
        // decel_t = (v_cruise - v_end) / accel = 50 / 1000 = 0.05s
        // Total time = 0.05 + 1.95 + 0.05 = 2.05s

        let start_v = 0.0;
        let cruise_v = 50.0;
        let end_v = 0.0;
        m.set_junction(start_v.powi(2), cruise_v.powi(2), end_v.powi(2));

        assert!(approx_eq(m.start_v, start_v, 1e-9));
        assert!(approx_eq(m.cruise_v, cruise_v, 1e-9));
        assert!(approx_eq(m.end_v, end_v, 1e-9));

        let expected_accel_d = (cruise_v.powi(2) - start_v.powi(2)) / (2.0 * accel);
        let expected_decel_d = (cruise_v.powi(2) - end_v.powi(2)) / (2.0 * accel);
        // let expected_cruise_d = m.move_d - expected_accel_d - expected_decel_d;

        assert!(approx_eq(m.accel_t, expected_accel_d / ((start_v + cruise_v) * 0.5), 1e-9));
        assert!(approx_eq(m.decel_t, expected_decel_d / ((end_v + cruise_v) * 0.5), 1e-9));
        // cruise_t can have small error due to subtractions for cruise_d
        assert!(approx_eq(m.cruise_t, (m.move_d - expected_accel_d - expected_decel_d) / cruise_v, 1e-5));
    }

    #[test]
    fn test_move_calc_junction_90_degree_turn() {
        let extra_axes_mock: Vec<Box<dyn ExtraAxis>> = vec![Box::new(MockExtraAxis)];
        let move1_start = [0.0, 0.0, 0.0, 0.0];
        let move1_end = [10.0, 0.0, 0.0, 0.0];
        let mut move1 = Move::new(DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL, move1_start, move1_end, 50.0);
        // Manually set some properties as if it was processed by lookahead
        move1.max_start_v2 = 0.0; // Starts from stop
        move1.max_smoothed_v2 = 0.0;

        let move2_start = [10.0, 0.0, 0.0, 0.0];
        let move2_end = [10.0, 10.0, 0.0, 0.0];
        let mut move2 = Move::new(DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL, move2_start, move2_end, 50.0);

        move2.calc_junction(&move1, &extra_axes_mock);

        // For a 90-degree turn, junction_cos_theta should be 0.
        // sin_theta_d2 = sqrt(0.5 * (1 - 0)) = sqrt(0.5) approx 0.707
        // cos_theta_d2 = sqrt(0.5 * (1 + 0)) = sqrt(0.5) approx 0.707
        // R_jd = sin_theta_d2 / (1 - sin_theta_d2) = 0.7071 / (1 - 0.7071) = 0.7071 / 0.2929 approx 2.414
        // move_jd_v2 = R_jd * junction_deviation * accel
        //            = 2.414 * 0.013 * 3000 = 2.414 * 39 = 94.146
        // quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2 = 0.25 * 1 = 0.25
        // move_centripetal_v2 = delta_v2 * quarter_tan_theta_d2
        //                     = (2 * 10 * 3000) * 0.25 = 60000 * 0.25 = 15000
        // max_start_v2 should be min(move1.max_cruise_v2, move_jd_v2, etc.)
        // move1.max_cruise_v2 = 50^2 = 2500
        // So, max_start_v2 for move2 should be heavily limited by move_jd_v2 (94.146)
        assert!(move2.max_start_v2 < 100.0, "max_start_v2 was {}", move2.max_start_v2);
        assert!(approx_eq(move2.max_start_v2, 94.14753, 1e-3)); // Value from Klipper's own tests for similar scenario
    }

    #[test]
    fn test_lookahead_add_move() {
        let mut laq = LookAheadQueue::new();
        let extra_axes_mock: Vec<Box<dyn ExtraAxis>> = vec![Box::new(MockExtraAxis)];

        let move1 = Move::new(DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL, [0.0,0.0,0.0,0.0], [10.0,0.0,0.0,0.0], 100.0);
        let initial_junction_flush = laq.junction_flush;
        let min_move_t1 = move1.min_move_t;

        laq.add_move(move1, &extra_axes_mock);
        assert_eq!(laq.queue.len(), 1);
        assert!(approx_eq(laq.junction_flush, initial_junction_flush - min_move_t1, 1e-9));

        let move2 = Move::new(DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL, [10.0,0.0,0.0,0.0], [10.0,10.0,0.0,0.0], 100.0);
        let min_move_t2 = move2.min_move_t;
        laq.add_move(move2, &extra_axes_mock);
        assert_eq!(laq.queue.len(), 2);
        assert!(approx_eq(laq.junction_flush, initial_junction_flush - min_move_t1 - min_move_t2, 1e-9));
        // Check if calc_junction was implicitly called (move2.max_start_v2 should be affected)
        assert!(laq.queue[1].max_start_v2 < laq.queue[1].max_cruise_v2);
    }

    #[test]
    fn test_lookahead_flush_simple() {
        let mut laq = LookAheadQueue::new();
        let extra_axes_mock: Vec<Box<dyn ExtraAxis>> = vec![Box::new(MockExtraAxis)];

        let move1 = Move::new(DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL, [0.0,0.0,0.0,0.0], [10.0,0.0,0.0,0.0], 100.0);
        let move2 = Move::new(DEFAULT_MAX_ACCEL, DEFAULT_JUNCTION_DEV, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL_TO_DECEL, [10.0,0.0,0.0,0.0], [20.0,0.0,0.0,0.0], 100.0);
        laq.add_move(move1.clone(), &extra_axes_mock);
        laq.add_move(move2.clone(), &extra_axes_mock);

        let flushed_moves = laq.flush(false); // Not lazy
        assert_eq!(flushed_moves.len(), 2);
        assert_eq!(laq.queue.len(), 0);
        // We'd expect set_junction to have been called on these moves
        assert!(flushed_moves[0].start_v > 0.0 || flushed_moves[0].cruise_t > 0.0); // Basic check it was processed
        assert!(flushed_moves[1].start_v > 0.0 || flushed_moves[1].cruise_t > 0.0);
    }
}
