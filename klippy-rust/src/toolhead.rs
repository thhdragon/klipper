use crate::configfile::Config;
use crate::kinematics::cartesian::CartKinematics;
use crate::mcu::MCU;
use crate::trapq::TrapQ;
use crate::Printer;
use std::collections::VecDeque;

// Placeholders
pub mod gcode {
    pub struct GCode;
    pub struct GCodeMove;
    #[derive(Clone, Copy)]
    pub struct Coord;
}
pub mod chelper {
    pub fn trapq_alloc() -> *mut crate::trapq::TrapQ {
        std::ptr::null_mut()
    }
    pub fn trapq_append(
        _trapq: *mut crate::trapq::TrapQ,
        _print_time: f64,
        _accel_t: f64,
        _cruise_t: f64,
        _decel_t: f64,
        _start_pos_x: f64,
        _start_pos_y: f64,
        _start_pos_z: f64,
        _axes_r_x: f64,
        _axes_r_y: f64,
        _axes_r_z: f64,
        _start_v: f64,
        _cruise_v: f64,
        _accel: f64,
    ) {
    }
    pub fn trapq_finalize_moves(
        _trapq: *mut crate::trapq::TrapQ,
        _print_time: f64,
        _clear_history_time: f64,
    ) {
    }
}
pub mod kinematics {
    pub mod extruder {
        #[derive(Clone, Copy, PartialEq)]
        pub struct DummyExtruder;
        impl DummyExtruder {
            pub fn new() -> Self {
                DummyExtruder
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct Move {
    start_pos: [f64; 4],
    end_pos: [f64; 4],
    speed: f64,
    accel: f64,
    junction_deviation: f64,
    is_kinematic_move: bool,
    axes_d: [f64; 4],
    move_d: f64,
    axes_r: [f64; 4],
    min_move_t: f64,
    max_start_v2: f64,
    max_cruise_v2: f64,
    delta_v2: f64,
    max_smoothed_v2: f64,
    smooth_delta_v2: f64,
    next_junction_v2: f64,
    start_v: f64,
    cruise_v: f64,
    end_v: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
}

impl Move {
    pub fn new(
        max_velocity: f64,
        max_accel: f64,
        junction_deviation: f64,
        max_accel_to_decel: f64,
        start_pos: &[f64],
        end_pos: &[f64],
        speed: f64,
    ) -> Self {
        let mut move_d = 0.0;
        let mut axes_d = vec![0.0; start_pos.len()];
        for i in 0..3 {
            axes_d[i] = end_pos[i] - start_pos[i];
            move_d += axes_d[i] * axes_d[i];
        }
        move_d = move_d.sqrt();

        let mut is_kinematic_move = true;
        let mut velocity = speed.min(max_velocity);
        let mut accel = max_accel;
        let mut end_pos = end_pos.to_vec();

        if move_d < 0.000000001 {
            // Extrude only move
            end_pos[0] = start_pos[0];
            end_pos[1] = start_pos[1];
            end_pos[2] = start_pos[2];
            axes_d[0] = 0.0;
            axes_d[1] = 0.0;
            axes_d[2] = 0.0;
            move_d = (end_pos[3] - start_pos[3]).abs();
            velocity = speed;
            accel = 99999999.9;
            is_kinematic_move = false;
        }

        let inv_move_d = if move_d > 0.0 { 1.0 / move_d } else { 0.0 };
        let axes_r = axes_d.iter().map(|d| d * inv_move_d).collect::<Vec<_>>();

        Move {
            start_pos: start_pos.try_into().unwrap(),
            end_pos: end_pos.try_into().unwrap(),
            speed,
            accel,
            junction_deviation,
            is_kinematic_move,
            axes_d: axes_d.try_into().unwrap(),
            move_d,
            axes_r: axes_r.try_into().unwrap(),
            min_move_t: move_d / velocity,
            max_start_v2: 0.0,
            max_cruise_v2: velocity * velocity,
            delta_v2: 2.0 * move_d * accel,
            max_smoothed_v2: 0.0,
            smooth_delta_v2: 2.0 * move_d * max_accel_to_decel,
            next_junction_v2: 999999999.9,
            start_v: 0.0,
            cruise_v: 0.0,
            end_v: 0.0,
            accel_t: 0.0,
            cruise_t: 0.0,
            decel_t: 0.0,
        }
    }

    pub fn calc_junction(&mut self, prev_move: &Move) {
        if !self.is_kinematic_move || !prev_move.is_kinematic_move {
            return;
        }
        // Allow extra axes to calculate maximum junction
        // let ea_v2 = self
        //     .toolhead
        //     .extra_axes
        //     .iter()
        //     .enumerate()
        //     .map(|(i, ea)| ea.calc_junction(prev_move, self, i + 3))
        //     .collect::<Vec<_>>();
        let mut max_start_v2 = self
            .max_cruise_v2
            .min(prev_move.max_cruise_v2)
            .min(prev_move.next_junction_v2)
            .min(prev_move.max_start_v2 + prev_move.delta_v2);
        // .min(ea_v2.iter().fold(std::f64::INFINITY, |a, &b| a.min(b)));

        // Find max velocity using "approximated centripetal velocity"
        let axes_r = self.axes_r;
        let prev_axes_r = prev_move.axes_r;
        let junction_cos_theta = -(axes_r[0] * prev_axes_r[0]
            + axes_r[1] * prev_axes_r[1]
            + axes_r[2] * prev_axes_r[2]);
        let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta)).max(0.0).sqrt();
        let cos_theta_d2 = (0.5 * (1.0 + junction_cos_theta)).max(0.0).sqrt();
        let one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;
        if one_minus_sin_theta_d2 > 0.0 && cos_theta_d2 > 0.0 {
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
        // Apply limits
        self.max_start_v2 = max_start_v2;
        self.max_smoothed_v2 =
            max_start_v2.min(prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2);
    }

    pub fn set_junction(&mut self, start_v2: f64, cruise_v2: f64, end_v2: f64) {
        let half_inv_accel = 0.5 / self.accel;
        let accel_d = (cruise_v2 - start_v2) * half_inv_accel;
        let decel_d = (cruise_v2 - end_v2) * half_inv_accel;
        let cruise_d = self.move_d - accel_d - decel_d;
        self.start_v = start_v2.sqrt();
        self.cruise_v = cruise_v2.sqrt();
        self.end_v = end_v2.sqrt();
        self.accel_t = accel_d / ((self.start_v + self.cruise_v) * 0.5);
        self.cruise_t = cruise_d / self.cruise_v;
        self.decel_t = decel_d / ((self.end_v + self.cruise_v) * 0.5);
    }

    pub fn limit_speed(&mut self, speed: f64, accel: f64) {
        let speed2 = speed * speed;
        if speed2 < self.max_cruise_v2 {
            self.max_cruise_v2 = speed2;
            self.min_move_t = self.move_d / speed;
        }
        self.accel = self.accel.min(accel);
        self.delta_v2 = 2.0 * self.move_d * self.accel;
        self.smooth_delta_v2 = self.smooth_delta_v2.min(self.delta_v2);
    }

    pub fn limit_next_junction_speed(&mut self, speed: f64) {
        self.next_junction_v2 = self.next_junction_v2.min(speed * speed);
    }

    pub fn move_error(&self, msg: &str) {
        let ep = self.end_pos;
        let _m = format!(
            "{}: {:.3} {:.3} {:.3} [{:.3}]",
            msg, ep[0], ep[1], ep[2], ep[3]
        );
        // unsafe { (*self.toolhead).printer.command_error(m) };
    }
}

pub struct LookAheadQueue {
    queue: VecDeque<Move>,
    junction_flush: f64,
}

impl LookAheadQueue {
    pub fn new() -> Self {
        LookAheadQueue {
            queue: VecDeque::new(),
            junction_flush: 0.250,
        }
    }

    pub fn reset(&mut self) {
        self.queue.clear();
        self.junction_flush = 0.250;
    }

    pub fn set_flush_time(&mut self, flush_time: f64) {
        self.junction_flush = flush_time;
    }

    pub fn get_last(&mut self) -> Option<&mut Move> {
        self.queue.back_mut()
    }

    pub fn add_move(&mut self, mut move_to_add: Move) -> bool {
        if let Some(prev_move) = self.queue.back() {
            move_to_add.calc_junction(prev_move);
        }
        self.junction_flush -= move_to_add.min_move_t;
        self.queue.push_back(move_to_add);
        self.junction_flush <= 0.0
    }

    pub fn flush(&mut self, lazy: bool) -> Vec<Move> {
        self.junction_flush = 0.250;
        let mut update_flush_count = lazy;
        let queue = &mut self.queue;
        let mut flush_count = queue.len();

        let mut delayed: Vec<(Move, f64, f64)> = vec![];
        let mut next_end_v2 = 0.0;
        let mut next_smoothed_v2 = 0.0;
        let mut peak_cruise_v2 = 0.0;

        for i in (0..flush_count).rev() {
            let mut move_ = queue[i];
            let reachable_start_v2 = next_end_v2 + move_.delta_v2;
            let start_v2 = move_.max_start_v2.min(reachable_start_v2);
            let reachable_smoothed_v2 = next_smoothed_v2 + move_.smooth_delta_v2;
            let smoothed_v2 = move_.max_smoothed_v2.min(reachable_smoothed_v2);

            if smoothed_v2 < reachable_smoothed_v2 {
                if smoothed_v2 + move_.smooth_delta_v2 > next_smoothed_v2 || !delayed.is_empty() {
                    if update_flush_count && peak_cruise_v2 > 0.0 {
                        flush_count = i;
                        update_flush_count = false;
                    }
                    peak_cruise_v2 = move_
                        .max_cruise_v2
                        .min((smoothed_v2 + reachable_smoothed_v2) * 0.5);
                    if !delayed.is_empty() {
                        if !update_flush_count && i < flush_count {
                            let mut mc_v2 = peak_cruise_v2;
                            for (m, ms_v2, me_v2) in delayed.iter_mut().rev() {
                                mc_v2 = mc_v2.min(*ms_v2);
                                m.set_junction(ms_v2.min(mc_v2), mc_v2, me_v2.min(mc_v2));
                            }
                        }
                        delayed.clear();
                    }
                }
                if !update_flush_count && i < flush_count {
                    let cruise_v2 = (start_v2 + reachable_start_v2) * 0.5;
                    let cruise_v2 = cruise_v2.min(move_.max_cruise_v2).min(peak_cruise_v2);
                    move_.set_junction(
                        start_v2.min(cruise_v2),
                        cruise_v2,
                        next_end_v2.min(cruise_v2),
                    );
                }
            } else {
                delayed.push((move_, start_v2, next_end_v2));
            }
            next_end_v2 = start_v2;
            next_smoothed_v2 = smoothed_v2;
        }

        if update_flush_count || flush_count == 0 {
            return vec![];
        }

        queue.drain(..flush_count).collect()
    }
}

pub struct ToolHead {
    printer: *mut Printer,
    reactor: *mut (), // Placeholder for reactor::Reactor
    all_mcus: Vec<*mut MCU>,
    mcu: *mut MCU,
    lookahead: LookAheadQueue,
    commanded_pos: Vec<f64>,
    pub max_velocity: f64,
    pub max_accel: f64,
    min_cruise_ratio: f64,
    pub square_corner_velocity: f64,
    pub junction_deviation: f64,
    pub max_accel_to_decel: f64,
    check_stall_time: f64,
    print_stall: u32,
    can_pause: bool,
    need_check_pause: f64,
    print_time: f64,
    special_queuing_state: String,
    priming_timer: *mut (), // Placeholder for reactor::Timer
    flush_timer: *mut (),   // Placeholder for reactor::Timer
    do_kick_flush_timer: bool,
    last_flush_time: f64,
    min_restart_time: f64,
    need_flush_time: f64,
    step_gen_time: f64,
    clear_history_time: f64,
    kin_flush_delay: f64,
    kin_flush_times: Vec<f64>,
    trapq: *mut TrapQ,
    trapq_append: fn(
        *mut TrapQ,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
    ),
    trapq_finalize_moves: fn(*mut TrapQ, f64, f64),
    step_generators: Vec<fn(f64)>,
    flush_trapqs: Vec<*mut TrapQ>,
    kin: CartKinematics,
    coord: gcode::Coord,
    pub extra_axes: Vec<kinematics::extruder::DummyExtruder>,
}

impl ToolHead {
    pub fn new(config: &Config) -> Self {
        // let printer = config.get_printer();
        // let reactor = printer.get_reactor();
        // let all_mcus = printer.lookup_objects("mcu");
        // let mcu = all_mcus[0].1;
        let lookahead = LookAheadQueue::new();
        // lookahead.set_flush_time(2.0);
        let commanded_pos = vec![0.0, 0.0, 0.0, 0.0];
        // let max_velocity = config.get_float("max_velocity", None, Some(0.0), None);
        // let max_accel = config.get_float("max_accel", None, Some(0.0), None);
        // let min_cruise_ratio = 0.5;
        // let square_corner_velocity = config.get_float("square_corner_velocity", Some(5.0), Some(0.0), None);
        let trapq = chelper::trapq_alloc();
        ToolHead {
            printer: std::ptr::null_mut(),
            reactor: std::ptr::null_mut(),
            all_mcus: vec![],
            mcu: std::ptr::null_mut(),
            lookahead,
            commanded_pos,
            max_velocity: 0.0,
            max_accel: 0.0,
            min_cruise_ratio: 0.0,
            square_corner_velocity: 0.0,
            junction_deviation: 0.0,
            max_accel_to_decel: 0.0,
            check_stall_time: 0.0,
            print_stall: 0,
            can_pause: true,
            need_check_pause: -1.0,
            print_time: 0.0,
            special_queuing_state: "NeedPrime".to_string(),
            priming_timer: std::ptr::null_mut(),
            flush_timer: std::ptr::null_mut(),
            do_kick_flush_timer: true,
            last_flush_time: 0.0,
            min_restart_time: 0.0,
            need_flush_time: 0.0,
            step_gen_time: 0.0,
            clear_history_time: 0.0,
            kin_flush_delay: 0.001,
            kin_flush_times: vec![],
            trapq,
            trapq_append: chelper::trapq_append,
            trapq_finalize_moves: chelper::trapq_finalize_moves,
            step_generators: vec![],
            flush_trapqs: vec![trapq],
            kin: CartKinematics::new(
                &crate::kinematics::cartesian::Toolhead,
                config,
            ),
            coord: gcode::Coord,
            extra_axes: vec![kinematics::extruder::DummyExtruder::new()],
        }
    }

    pub fn move_(&mut self, newpos: &[f64], speed: f64) {
        let mut move_ = Move::new(
            self.max_velocity,
            self.max_accel,
            self.junction_deviation,
            self.max_accel_to_decel,
            &self.commanded_pos,
            newpos,
            speed,
        );
        if move_.move_d == 0.0 {
            return;
        }
        if move_.is_kinematic_move {
            self.kin.check_move(&mut move_);
        }
        // for e_index, ea in enumerate(self.extra_axes) {
        //     if move_.axes_d[e_index + 3] {
        //         ea.check_move(move_, e_index + 3);
        //     }
        // }
        self.commanded_pos = newpos.to_vec();
        let want_flush = self.lookahead.add_move(move_);
        if want_flush {
            // self.process_lookahead(true);
        }
        // if self.print_time > self.need_check_pause {
        //     self.check_pause();
        // }
    }

    pub fn drip_move(&mut self, newpos: &[f64], speed: f64, _drip_completion: &DripCompletion) {
        let mut newpos = newpos.to_vec();
        newpos[3] = self.commanded_pos[3];
        let mut move_ = Move::new(
            self.max_velocity,
            self.max_accel,
            self.junction_deviation,
            self.max_accel_to_decel,
            &self.commanded_pos,
            &newpos,
            speed,
        );
        if move_.move_d > 0.0 {
            self.kin.check_move(&mut move_);
        }
        // self.dwell(self.kin_flush_delay);
        // self.process_lookahead();
        // let next_move_time = self.drip_load_trapq(move_);
        // self.drip_update_time(next_move_time, drip_completion, &[]);
        // self.trapq_finalize_moves(self.trapq, std::f64::INFINITY, 0.0);
    }

    fn _process_lookahead(&mut self, lazy: bool) {
        let moves = self.lookahead.flush(lazy);
        if moves.is_empty() {
            return;
        }
        if !self.special_queuing_state.is_empty() {
            self.special_queuing_state = "".to_string();
            self.need_check_pause = -1.0;
            self._calc_print_time();
        }
        let mut next_move_time = self.print_time;
        for move_ in moves {
            if move_.is_kinematic_move {
                (self.trapq_append)(
                    self.trapq,
                    next_move_time,
                    move_.accel_t,
                    move_.cruise_t,
                    move_.decel_t,
                    move_.start_pos[0],
                    move_.start_pos[1],
                    move_.start_pos[2],
                    move_.axes_r[0],
                    move_.axes_r[1],
                    move_.axes_r[2],
                    move_.start_v,
                    move_.cruise_v,
                    move_.accel,
                );
            }
            // for (e_index, ea) in self.extra_axes.iter().enumerate() {
            //     if move_.axes_d[e_index + 3] != 0.0 {
            //         ea.process_move(next_move_time, &move_, e_index + 3);
            //     }
            // }
            next_move_time += move_.accel_t + move_.cruise_t + move_.decel_t;
            // for cb in move_.timing_callbacks {
            //     cb(next_move_time);
            // }
        }
        // self.note_mcu_movequeue_activity(next_move_time + self.kin_flush_delay, true);
        self._advance_move_time(next_move_time);
    }

    fn _advance_move_time(&mut self, next_print_time: f64) {
        let pt_delay = self.kin_flush_delay + 0.050; // STEPCOMPRESS_FLUSH_TIME
        let flush_time = self.last_flush_time.max(self.print_time - pt_delay);
        self.print_time = self.print_time.max(next_print_time);
        let want_flush_time = flush_time.max(self.print_time - pt_delay);
        let mut flush_time = flush_time;
        while flush_time < want_flush_time {
            flush_time = (flush_time + 0.500).min(want_flush_time); // MOVE_BATCH_TIME
                                                                    // self.advance_flush_time(flush_time);
        }
    }

    fn _calc_print_time(&mut self) {
        // let curtime = self.reactor.monotonic();
        // let est_print_time = self.mcu.estimated_print_time(curtime);
        // let kin_time = (est_print_time + 0.100).max(self.min_restart_time); // MIN_KIN_TIME
        // let kin_time = kin_time + self.kin_flush_delay;
        // let min_print_time = (est_print_time + 0.250).max(kin_time); // BUFFER_TIME_START
        // if min_print_time > self.print_time {
        //     self.print_time = min_print_time;
        //     // self.printer.send_event("toolhead:sync_print_time", curtime, est_print_time, self.print_time);
        // }
    }

    fn _check_pause(&mut self) {
        // let eventtime = self.reactor.monotonic();
        // let est_print_time = self.mcu.estimated_print_time(eventtime);
        // let buffer_time = self.print_time - est_print_time;
        // if !self.special_queuing_state.is_empty() {
        //     if self.check_stall_time > 0.0 {
        //         if est_print_time < self.check_stall_time {
        //             self.print_stall += 1;
        //         }
        //         self.check_stall_time = 0.0;
        //     }
        //     self.special_queuing_state = "Priming".to_string();
        //     self.need_check_pause = -1.0;
        //     if self.priming_timer.is_null() {
        //         self.priming_timer = self.reactor.register_timer(self._priming_handler);
        //     }
        //     let wtime = eventtime + (0.100).max(buffer_time - 1.0); // BUFFER_TIME_LOW
        //     self.reactor.update_timer(self.priming_timer, wtime);
        // }
        // while {
        //     let pause_time = buffer_time - 2.0; // BUFFER_TIME_HIGH
        //     pause_time > 0.0
        // } {
        //     if !self.can_pause {
        //         self.need_check_pause = std::f64::INFINITY;
        //         return;
        //     }
        //     eventtime = self.reactor.pause(eventtime + (1.0).min(pause_time));
        //     est_print_time = self.mcu.estimated_print_time(eventtime);
        //     buffer_time = self.print_time - est_print_time;
        // }
        // if self.special_queuing_state.is_empty() {
        //     self.need_check_pause = est_print_time + 2.0 + 0.100; // BUFFER_TIME_HIGH
        // }
    }

    fn _priming_handler(&mut self, _eventtime: f64) {
        // self.reactor.unregister_timer(self.priming_timer);
        // self.priming_timer = std::ptr::null_mut();
        // if self.special_queuing_state == "Priming" {
        //     self._flush_lookahead();
        //     self.check_stall_time = self.print_time;
        // }
    }

    fn _flush_handler(&mut self, _eventtime: f64) {
        // let est_print_time = self.mcu.estimated_print_time(eventtime);
        // if self.special_queuing_state.is_empty() {
        //     let print_time = self.print_time;
        //     let buffer_time = print_time - est_print_time;
        //     if buffer_time > 1.0 {
        //         // BUFFER_TIME_LOW
        //         return eventtime + buffer_time - 1.0;
        //     }
        //     self._flush_lookahead();
        //     if print_time != self.print_time {
        //         self.check_stall_time = self.print_time;
        //     }
        // }
        // loop {
        //     let end_flush = self.need_flush_time + 0.250; // BGFLUSH_EXTRA_TIME
        //     if self.last_flush_time >= end_flush {
        //         self.do_kick_flush_timer = true;
        //         return std::f64::INFINITY;
        //     }
        //     let buffer_time = self.last_flush_time - est_print_time;
        //     if buffer_time > 0.200 {
        //         // BGFLUSH_LOW_TIME
        //         return eventtime + buffer_time - 0.200;
        //     }
        //     let ftime = est_print_time + 0.200 + 0.200; // BGFLUSH_LOW_TIME + BGFLUSH_BATCH_TIME
        //     self._advance_flush_time(end_flush.min(ftime));
        // }
    }

    fn dwell(&mut self, _delay: f64) {
        // let next_print_time = self.get_last_move_time() + delay.max(0.0);
        // self._advance_move_time(next_print_time);
        // self._check_pause();
    }

    fn wait_moves(&mut self) {
        // self._flush_lookahead();
        // let mut eventtime = self.reactor.monotonic();
        // while !self.special_queuing_state.is_empty() || self.print_time >= self.mcu.estimated_print_time(eventtime) {
        //     if !self.can_pause {
        //         break;
        //     }
        //     eventtime = self.reactor.pause(eventtime + 0.100);
        // }
    }

    fn set_extruder(&mut self, extruder: kinematics::extruder::DummyExtruder, extrude_pos: f64) {
        // let prev_ea_trapq = self.extra_axes[0].get_trapq();
        // if self.flush_trapqs.contains(&prev_ea_trapq) {
        //     self.flush_trapqs.remove_item(&prev_ea_trapq);
        // }
        self.extra_axes[0] = extruder;
        self.commanded_pos[3] = extrude_pos;
        // let ea_trapq = extruder.get_trapq();
        // if !ea_trapq.is_null() {
        //     self.flush_trapqs.push(ea_trapq);
        // }
    }

    fn get_extruder(&self) -> &kinematics::extruder::DummyExtruder {
        &self.extra_axes[0]
    }

    pub fn add_extra_axis(&mut self, ea: kinematics::extruder::DummyExtruder) {
        // self._flush_lookahead();
        self.extra_axes.push(ea);
        self.commanded_pos.push(0.0);
        // let ea_trapq = ea.get_trapq();
        // if !ea_trapq.is_null() {
        //     self.flush_trapqs.push(ea_trapq);
        // }
        // self.printer.send_event("toolhead:update_extra_axes");
    }

    pub fn remove_extra_axis(&mut self, _ea: kinematics::extruder::DummyExtruder) {
        // self._flush_lookahead();
        // if let Some(ea_index) = self.extra_axes.iter().position(|&x| x == ea) {
        //     let ea_trapq = self.extra_axes[ea_index].get_trapq();
        //     if self.flush_trapqs.contains(&ea_trapq) {
        //         self.flush_trapqs.remove_item(&ea_trapq);
        //     }
        //     self.commanded_pos.remove(ea_index + 3);
        //     self.extra_axes.remove(ea_index);
        //     // self.printer.send_event("toolhead:update_extra_axes");
        // }
    }

    pub fn get_extra_axes(&self) -> Vec<Option<kinematics::extruder::DummyExtruder>> {
        let mut axes = vec![None, None, None];
        axes.extend(self.extra_axes.iter().map(|ea| Some(*ea)));
        axes
    }

    fn note_mcu_movequeue_activity(&mut self, _wake_time: f64, _triggered: bool) {
        // self.need_flush_time = self.need_flush_time.max(wake_time);
        // if self.do_kick_flush_timer {
        //     self.do_kick_flush_timer = false;
        //     let curtime = self.reactor.monotonic();
        //     self.reactor.update_timer(self.flush_timer, curtime);
        // }
    }

    pub fn get_max_velocity(&self) -> (f64, f64) {
        (self.max_velocity, self.max_accel)
    }

    pub fn _calc_junction_deviation(&mut self) {
        let scv2 = self.square_corner_velocity * self.square_corner_velocity;
        self.junction_deviation = scv2 * (2.0f64.sqrt() - 1.0) / self.max_accel;
        self.max_accel_to_decel = self.max_accel;
        // self.max_accel_to_decel = self.config.get_float(
        //     "max_accel_to_decel",
        //     self.max_accel,
        //     0.0,
        //     None,
        // );
    }
}

pub struct DripCompletion;

pub fn add_printer_objects(_config: &()) {
    // config.get_printer().add_object("toolhead", ToolHead::new(config));
    // kinematics::extruder::add_printer_objects(config);
}
