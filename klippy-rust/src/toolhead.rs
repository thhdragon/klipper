use crate::kinematics::cartesian::{CartKinematics, Toolhead};
use crate::mcu::MCU;
use crate::trapq::{self, TrapQ};
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
        pub struct DummyExtruder;
        impl DummyExtruder {
            pub fn new() -> Self {
                DummyExtruder
            }
        }
    }
}

pub struct Move {
    toolhead: *mut ToolHead,
    start_pos: [f64; 4],
    end_pos: [f64; 4],
    speed: f64,
    accel: f64,
    junction_deviation: f64,
    timing_callbacks: Vec<fn(f64)>,
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
    pub fn new(toolhead: &mut ToolHead, start_pos: [f64; 4], end_pos: [f64; 4], speed: f64) -> Self {
        let mut move_d = 0.0;
        let mut axes_d = [0.0; 4];
        for i in 0..3 {
            axes_d[i] = end_pos[i] - start_pos[i];
            move_d += axes_d[i] * axes_d[i];
        }
        move_d = move_d.sqrt();

        let mut is_kinematic_move = true;
        let mut velocity = speed.min(toolhead.max_velocity);
        let mut accel = toolhead.max_accel;
        let mut end_pos = end_pos;

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
        let axes_r = [
            axes_d[0] * inv_move_d,
            axes_d[1] * inv_move_d,
            axes_d[2] * inv_move_d,
            axes_d[3] * inv_move_d,
        ];

        Move {
            toolhead,
            start_pos,
            end_pos,
            speed,
            accel,
            junction_deviation: toolhead.junction_deviation,
            timing_callbacks: vec![],
            is_kinematic_move,
            axes_d,
            move_d,
            axes_r,
            min_move_t: move_d / velocity,
            max_start_v2: 0.0,
            max_cruise_v2: velocity * velocity,
            delta_v2: 2.0 * move_d * accel,
            max_smoothed_v2: 0.0,
            smooth_delta_v2: 2.0 * move_d * toolhead.max_accel_to_decel,
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
        let max_start_v2 = self
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
            let max_start_v2 = max_start_v2
                .min(move_jd_v2)
                .min(pmove_jd_v2)
                .min(move_centripetal_v2)
                .min(pmove_centripetal_v2);
        }
        // Apply limits
        self.max_start_v2 = max_start_v2;
        self.max_smoothed_v2 = max_start_v2.min(prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2);
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
        let m = format!(
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

    pub fn add_move(&mut self, move_to_add: Move) -> bool {
        if let Some(_prev_move) = self.queue.back() {
            // move_to_add.calc_junction(prev_move);
        }
        self.junction_flush -= move_to_add.min_move_t;
        self.queue.push_back(move_to_add);
        self.junction_flush <= 0.0
    }
}

pub struct ToolHead {
    printer: *mut Printer,
    reactor: *mut (), // Placeholder for reactor::Reactor
    all_mcus: Vec<*mut MCU>,
    mcu: *mut MCU,
    lookahead: LookAheadQueue,
    commanded_pos: [f64; 4],
    max_velocity: f64,
    max_accel: f64,
    min_cruise_ratio: f64,
    square_corner_velocity: f64,
    junction_deviation: f64,
    max_accel_to_decel: f64,
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
    extra_axes: Vec<kinematics::extruder::DummyExtruder>,
}

impl ToolHead {
    pub fn new(config: &crate::configfile::Config) -> Self {
        // let printer = config.get_printer();
        // let reactor = printer.get_reactor();
        // let all_mcus = printer.lookup_objects("mcu");
        // let mcu = all_mcus[0].1;
        let lookahead = LookAheadQueue {
            queue: VecDeque::new(),
            junction_flush: 0.250,
        };
        // lookahead.set_flush_time(2.0);
        let commanded_pos = [0.0, 0.0, 0.0, 0.0];
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
            kin: CartKinematics::new(&Toolhead, config),
            coord: gcode::Coord,
            extra_axes: vec![kinematics::extruder::DummyExtruder::new()],
        }
    }

    pub fn move_(&mut self, newpos: [f64; 4], speed: f64) {
        let mut move_ = Move::new(self, self.commanded_pos, newpos, speed);
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
        self.commanded_pos = newpos;
        let want_flush = self.lookahead.add_move(move_);
        if want_flush {
            // self.process_lookahead(true);
        }
        // if self.print_time > self.need_check_pause {
        //     self.check_pause();
        // }
    }

    pub fn drip_move(&mut self, newpos: [f64; 4], speed: f64, _drip_completion: &DripCompletion) {
        let mut newpos = newpos;
        newpos[3] = self.commanded_pos[3];
        let mut move_ = Move::new(self, self.commanded_pos, newpos, speed);
        if move_.move_d > 0.0 {
            self.kin.check_move(&mut move_);
        }
        // self.dwell(self.kin_flush_delay);
        // self.process_lookahead();
        // let next_move_time = self.drip_load_trapq(move_);
        // self.drip_update_time(next_move_time, drip_completion, &[]);
        // self.trapq_finalize_moves(self.trapq, std::f64::INFINITY, 0.0);
    }
}

pub struct DripCompletion;

pub fn add_printer_objects(config: &()) {
    // config.get_printer().add_object("toolhead", ToolHead::new(config));
    // kinematics::extruder::add_printer_objects(config);
}
