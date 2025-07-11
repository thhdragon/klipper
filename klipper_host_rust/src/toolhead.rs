// klipper_host_rust/src/toolhead.rs
// Corresponds to klippy/toolhead.py - Manages the printer's toolhead.

use crate::configfile::Configfile;

#[derive(Debug, Clone)]
pub struct Endstop {
    pub name: String,
    pub triggered: bool,
    pub trigger_position_machine: f64,
    // New fields for homing parameters
    pub homing_speed: f64,
    pub homing_retract_dist: f64,
    pub second_homing_speed: f64,
}

impl Endstop {
    fn new(
        name: String,
        trigger_position_machine: f64,
        homing_speed: f64,
        homing_retract_dist: f64,
        second_homing_speed: f64,
    ) -> Self {
        Endstop {
            name,
            triggered: false,
            trigger_position_machine,
            homing_speed,
            homing_retract_dist,
            second_homing_speed,
        }
    }

    #[allow(dead_code)]
    fn is_triggered_by_pos(&self, current_machine_pos: f64, homing_positive_dir: bool) -> bool {
        if homing_positive_dir {
            current_machine_pos >= self.trigger_position_machine
        } else {
            current_machine_pos <= self.trigger_position_machine
        }
    }
}


use crate::mcu::Mcu;
use crate::reactor::Reactor;
use crate::heaters::Heater;
use crate::kinematics::cartesian::{CartesianKinematics, DEFAULT_ROTATION_DISTANCE, DEFAULT_FULL_STEPS_PER_ROTATION, DEFAULT_MICROSTEPS};
use crate::trapq::TrapQ;
use crate::extras::fan::Fan;

pub const DEFAULT_HOMING_SPEED: f64 = 5.0; // Default from Klipper stepper.py GenericPrinterRail
pub const DEFAULT_SECOND_HOMING_SPEED_RATIO: f64 = 0.5; // Klipper defaults second_homing_speed to homing_speed / 2.0
pub const DEFAULT_HOMING_RETRACT_DIST: f64 = 5.0; // Default from Klipper stepper.py GenericPrinterRail

// Machine position where endstop triggers (these are defaults if not in config)
pub const X_MACHINE_POSITION_AT_ENDSTOP: f64 = 0.0;
pub const Y_MACHINE_POSITION_AT_ENDSTOP: f64 = 0.0;
pub const Z_MACHINE_POSITION_AT_ENDSTOP: f64 = 0.0;
// G-code position to set after homing (via G92 logic)
pub const X_GCODE_POSITION_AFTER_HOMING: f64 = 0.0;
pub const Y_GCODE_POSITION_AFTER_HOMING: f64 = 0.0;
pub const Z_GCODE_POSITION_AFTER_HOMING: f64 = 0.0;


#[derive(Debug, Clone)]
pub struct Move {
    pub start_pos: [f64; 4],
    pub end_pos: [f64; 4],
    pub accel: f64,
    pub junction_deviation: f64,
    pub is_kinematic_move: bool,
    pub axes_d: [f64; 4],
    pub move_d: f64,
    pub axes_r: [f64; 4],
    pub min_move_t: f64,

    pub max_start_v2: f64,
    pub max_cruise_v2: f64,
    pub delta_v2: f64,
    pub max_smoothed_v2: f64,
    pub smooth_delta_v2: f64,
    pub next_junction_v2: f64,

    pub start_v: f64,
    pub cruise_v: f64,
    pub end_v: f64,
    pub accel_t: f64,
    pub cruise_t: f64,
    pub decel_t: f64,
}

impl Move {
    pub fn new(
        max_accel: f64,
        junction_deviation_val: f64,
        max_velocity: f64,
        max_accel_to_decel: f64,
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
        let mut final_end_pos = end_pos_arr;
        let mut effective_axes_d = axes_d_arr;

        if move_d_val < 0.000000001 {
            final_end_pos = [start_pos_arr[0], start_pos_arr[1], start_pos_arr[2], end_pos_arr[3]];
            effective_axes_d[0] = 0.0;
            effective_axes_d[1] = 0.0;
            effective_axes_d[2] = 0.0;
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
            axes_d: axes_d_arr,
            move_d: move_d_val,
            axes_r: axes_r_arr,
            min_move_t: min_move_t_val,
            max_start_v2: 0.0,
            max_cruise_v2: current_velocity.powi(2),
            delta_v2: 2.0 * move_d_val * current_accel,
            max_smoothed_v2: 0.0,
            smooth_delta_v2: 2.0 * move_d_val * max_accel_to_decel,
            next_junction_v2: 999999999.9,
            start_v: 0.0,
            cruise_v: 0.0,
            end_v: 0.0,
            accel_t: 0.0,
            cruise_t: 0.0,
            decel_t: 0.0,
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

    pub fn move_error(&self, msg: &str) -> String {
        format!(
            "{}: {:.3} {:.3} {:.3} [{:.3}]",
            msg, self.end_pos[0], self.end_pos[1], self.end_pos[2], self.end_pos[3]
        )
    }

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

        let junction_cos_theta_clamped = junction_cos_theta.max(-1.0).min(1.0);

        let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta_clamped)).sqrt().max(0.0);
        let cos_theta_d2 = (0.5 * (1.0 + junction_cos_theta_clamped)).sqrt().max(0.0);

        let one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;

        if one_minus_sin_theta_d2 > 1e-9 && cos_theta_d2 > 1e-9 {
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

        self.start_v = start_v2.sqrt().max(0.0);
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

    pub fn get_last_mut(&mut self) -> Option<&mut Move> {
        self.queue.last_mut()
    }

    pub fn add_move(&mut self, mut move_to_add: Move, toolhead_extra_axes: &[Box<dyn ExtraAxis>]) -> bool {
        if self.queue.is_empty() {
            self.queue.push(move_to_add);
        } else {
            let prev_move = &self.queue[self.queue.len() - 1];
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

        let mut delayed: Vec<(usize, f64, f64)> = Vec::new();
        let mut next_end_v2 = 0.0;
        let mut next_smoothed_v2 = 0.0;
        let mut peak_cruise_v2 = 0.0;

        for i in (0..self.queue.len()).rev() {
            let mut move_i = self.queue[i].clone();

            let reachable_start_v2 = next_end_v2 + move_i.delta_v2;
            let start_v2 = move_i.max_start_v2.min(reachable_start_v2);

            let reachable_smoothed_v2 = next_smoothed_v2 + move_i.smooth_delta_v2;
            let smoothed_v2 = move_i.max_smoothed_v2.min(reachable_smoothed_v2);

            if smoothed_v2 < reachable_smoothed_v2 {
                if smoothed_v2 + move_i.smooth_delta_v2 > next_smoothed_v2 || !delayed.is_empty() {
                    if update_flush_count && peak_cruise_v2 > 0.0 {
                        flush_count = i;
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
            return Vec::new();
        }
        let res = self.queue.drain(0..flush_count).collect();
        res
    }
}

const BUFFER_TIME_LOW: f64 = 1.0;
const BUFFER_TIME_HIGH: f64 = 2.0;
const BUFFER_TIME_START: f64 = 0.250;

const MIN_KIN_TIME: f64 = 0.100;
const MOVE_BATCH_TIME: f64 = 0.500;
const STEPCOMPRESS_FLUSH_TIME: f64 = 0.050;
const SDS_CHECK_TIME: f64 = 0.001;
const MOVE_HISTORY_EXPIRE: f64 = 30.0;

pub struct ToolHead<'a> {
    reactor: &'a Reactor,
    printer: &'a dyn PrinterUtility,
    all_mcus: Vec<&'a Mcu>,
    mcu: &'a Mcu,
    pub lookahead: LookAheadQueue,
    pub commanded_pos: [f64; 4],

    pub max_velocity: f64,
    pub max_accel: f64,
    pub min_cruise_ratio: f64,
    pub square_corner_velocity: f64,
    pub junction_deviation: f64,
    pub max_accel_to_decel: f64,

    pub print_time: f64,
    pub special_queuing_state: String,

    pub do_kick_flush_timer: bool,
    pub last_flush_time: f64,
    pub min_restart_time: f64,
    pub need_flush_time: f64,
    pub step_gen_time: f64,
    pub clear_history_time: f64,

    pub kin_flush_delay: f64,
    pub kin_flush_times: Vec<f64>,

    trapq: TrapQ,
    flush_trapqs: Vec<TrapQ>,

    pub kin: Box<dyn Kinematics>,
    extra_axes: Vec<Box<dyn ExtraAxis>>,

    pub extruder_heater: Heater,
    pub bed_heater: Heater,

    x_endstop: Endstop,
    y_endstop: Endstop,
    z_endstop: Endstop,

    pub part_cooling_fan: Fan,
}

pub trait Kinematics {
    fn check_move(&self, move_params: &mut Move) -> Result<(), String>;
    fn set_kinematics_position(&mut self, new_pos: &[f64; 3], homed_axes_mask: [bool; 3]);
    fn calc_position_from_steppers(&self) -> Result<[f64; 3], String>;
    fn set_steppers_to_gcode_target(&mut self, target_gcode_pos: [f64;3]) -> Result<(), String>;
    #[cfg(test)]
    fn get_axis_limits_for_test(&self) -> [(f64, f64); 3];
}

pub trait ExtraAxis {
    fn check_move(&self, move_params: &Move, axis_index: usize) -> Result<(), String>;
    fn process_move(&mut self, move_time: f64, move_params: &Move, axis_index: usize);
    fn calc_junction(&self, prev_move: &Move, current_move: &Move, e_index: usize) -> f64;
}


impl<'a> ToolHead<'a> {
    pub fn new(
        config: &Configfile,
        reactor_ref: &'a Reactor,
        printer_ref: &'a dyn PrinterUtility,
        mcus_list: Vec<&'a Mcu>,
    ) -> Result<Self, String> {
        let cfg_err = |e: configfile::ConfigError| e.to_string();

        let max_velocity = config.getfloat("printer", "max_velocity", None, Some(0.0), None).map_err(cfg_err)?;
        let max_accel = config.getfloat("printer", "max_accel", None, Some(0.0), None).map_err(cfg_err)?;

        let mut min_cruise_ratio = config.getfloat("printer", "minimum_cruise_ratio", Some(0.5), Some(0.0), Some(1.0))
            .map_err(cfg_err)?;
        if config.get_str("printer", "minimum_cruise_ratio").is_err() {
            if let Ok(req_accel_to_decel) = config.getfloat("printer", "max_accel_to_decel", None, Some(0.0), None) {
                min_cruise_ratio = 1.0 - (req_accel_to_decel / max_accel).min(1.0);
            }
        }
        let square_corner_velocity = config.getfloat("printer", "square_corner_velocity", Some(5.0), Some(0.0), None).map_err(cfg_err)?;

        // Stepper and Rail configurations
        let mut rail_configs = Vec::new();
        for axis_char in ['x', 'y', 'z'] {
            let sec_name = format!("stepper_{}", axis_char);
            let pos_min = config.getfloat(&sec_name, "position_min", Some(0.0), None, None).map_err(cfg_err)?;
            let pos_max = config.getfloat(&sec_name, "position_max", None, Some(pos_min), None).map_err(cfg_err)?;
            let pos_endstop = config.getfloat(&sec_name, "position_endstop", Some(0.0), None, None).map_err(cfg_err)?;

            let rotation_distance = config.getfloat(&sec_name, "rotation_distance", Some(DEFAULT_ROTATION_DISTANCE), Some(0.0), None).map_err(cfg_err)?;
            let full_steps = config.getint(&sec_name, "full_steps_per_rotation", Some(DEFAULT_FULL_STEPS_PER_ROTATION as i64), Some(1), None).map_err(cfg_err)? as u32;
            let microsteps = config.getint(&sec_name, "microsteps", Some(DEFAULT_MICROSTEPS as i64), Some(1), None).map_err(cfg_err)? as u32;
            let gear_ratio = config.getfloat(&sec_name, "gear_ratio", Some(1.0), Some(0.0), None).map_err(cfg_err)?;

            let stepper_configs_for_rail = vec![(
                config.get(&sec_name, "name", Some(sec_name.clone())).unwrap_or_else(|_| sec_name.clone()),
                rotation_distance,
                full_steps,
                microsteps,
                gear_ratio
            )];
            rail_configs.push((stepper_configs_for_rail, pos_min, pos_max, pos_endstop));
        }

        let default_z_velo = max_velocity / 20.0;
        let default_z_accel = max_accel / 20.0;
        let max_z_velocity = config.getfloat("printer", "max_z_velocity", Some(default_z_velo), Some(0.0), Some(max_velocity)).map_err(cfg_err)?;
        let max_z_accel = config.getfloat("printer", "max_z_accel", Some(default_z_accel), Some(0.0), Some(max_accel)).map_err(cfg_err)?;

        // Heater configurations
        let extruder_heater_name = config.get("extruder", "heater_name", Some("extruder".to_string())).unwrap_or_else(|_| "extruder".to_string());
        let extruder_min_temp = config.getfloat("extruder", "min_temp", Some(0.0), None, None).map_err(cfg_err)?;
        let extruder_max_temp = config.getfloat("extruder", "max_temp", Some(280.0), Some(extruder_min_temp), None).map_err(cfg_err)?;

        let bed_heater_name = config.get("heater_bed", "heater_name", Some("heater_bed".to_string())).unwrap_or_else(|_| "heater_bed".to_string());
        let bed_min_temp = config.getfloat("heater_bed", "min_temp", Some(0.0), None, None).map_err(cfg_err)?;
        let bed_max_temp = config.getfloat("heater_bed", "max_temp", Some(120.0), Some(bed_min_temp), None).map_err(cfg_err)?;

        // Fan configuration
        let fan_section_name = "fan";
        let fan_name = config.get(fan_section_name, "name", Some("part_cooling_fan".to_string()))
            .unwrap_or_else(|_| "part_cooling_fan".to_string());
        let fan_max_power = config.getfloat(fan_section_name, "max_power", Some(1.0), Some(0.0), Some(1.0))
            .map_err(cfg_err)? as f32;
        let fan_off_below = config.getfloat(fan_section_name, "off_below", Some(0.0), Some(0.0), Some(1.0))
            .map_err(cfg_err)? as f32;
        let fan_kick_start_time = config.getfloat(fan_section_name, "kick_start_time", Some(0.1), Some(0.0), None)
            .map_err(cfg_err)? as f32;

        // Endstop configurations (using already fetched pos_endstop and new homing params)
        let x_homing_speed = config.getfloat("stepper_x", "homing_speed", Some(DEFAULT_HOMING_SPEED), Some(0.0), None).map_err(cfg_err)?;
        let x_homing_retract_dist = config.getfloat("stepper_x", "homing_retract_dist", Some(DEFAULT_HOMING_RETRACT_DIST), Some(0.0), None).map_err(cfg_err)?;
        let x_second_homing_speed = config.getfloat("stepper_x", "second_homing_speed", Some(x_homing_speed * DEFAULT_SECOND_HOMING_SPEED_RATIO), Some(0.0), None).map_err(cfg_err)?;

        let y_homing_speed = config.getfloat("stepper_y", "homing_speed", Some(DEFAULT_HOMING_SPEED), Some(0.0), None).map_err(cfg_err)?;
        let y_homing_retract_dist = config.getfloat("stepper_y", "homing_retract_dist", Some(DEFAULT_HOMING_RETRACT_DIST), Some(0.0), None).map_err(cfg_err)?;
        let y_second_homing_speed = config.getfloat("stepper_y", "second_homing_speed", Some(y_homing_speed * DEFAULT_SECOND_HOMING_SPEED_RATIO), Some(0.0), None).map_err(cfg_err)?;

        let z_homing_speed = config.getfloat("stepper_z", "homing_speed", Some(DEFAULT_HOMING_SPEED / 2.0), Some(0.0), None).map_err(cfg_err)?; // Z often slower
        let z_homing_retract_dist = config.getfloat("stepper_z", "homing_retract_dist", Some(DEFAULT_HOMING_RETRACT_DIST / 2.0), Some(0.0), None).map_err(cfg_err)?;
        let z_second_homing_speed = config.getfloat("stepper_z", "second_homing_speed", Some(z_homing_speed * DEFAULT_SECOND_HOMING_SPEED_RATIO), Some(0.0), None).map_err(cfg_err)?;


        let toolhead = ToolHead {
            reactor: reactor_ref,
            printer: printer_ref,
            all_mcus: mcus_list.clone(),
            mcu: mcus_list.get(0).ok_or_else(|| "No MCUs provided".to_string())?,
            lookahead: LookAheadQueue::new(),
            commanded_pos: [0.0, 0.0, 0.0, 0.0],
            max_velocity,
            max_accel,
            min_cruise_ratio,
            square_corner_velocity,
            junction_deviation: 0.0,
            max_accel_to_decel: 0.0,
            print_time: 0.0,
            special_queuing_state: "NeedPrime".to_string(),
            do_kick_flush_timer: true,
            last_flush_time: 0.0,
            min_restart_time: 0.0,
            need_flush_time: 0.0,
            step_gen_time: 0.0,
            clear_history_time: 0.0,
            kin_flush_delay: SDS_CHECK_TIME,
            kin_flush_times: Vec::new(),
            trapq: TrapQ::new_for_test(),
            flush_trapqs: vec![TrapQ::new_for_test()],
            extruder_heater: Heater::new(extruder_heater_name, extruder_min_temp, extruder_max_temp),
            bed_heater: Heater::new(bed_heater_name, bed_min_temp, bed_max_temp),
            x_endstop: Endstop::new("x_endstop".to_string(), rail_configs[0].3, x_homing_speed, x_homing_retract_dist, x_second_homing_speed),
            y_endstop: Endstop::new("y_endstop".to_string(), rail_configs[1].3, y_homing_speed, y_homing_retract_dist, y_second_homing_speed),
            z_endstop: Endstop::new("z_endstop".to_string(), rail_configs[2].3, z_homing_speed, z_homing_retract_dist, z_second_homing_speed),
            part_cooling_fan: Fan::new(fan_name, fan_max_power, fan_off_below, fan_kick_start_time),
            kin: Box::new(CartesianKinematics::new(
                rail_configs[0].clone(), rail_configs[1].clone(), rail_configs[2].clone(), max_z_velocity, max_z_accel
            )),
        };

        // toolhead._calc_junction_deviation(); // Already done with the temporary toolhead instance

        // No need to re-assign kin, it's done in the struct initialization.
        // The previous awkward double-init of kin is removed.

        struct PlaceholderExtraAxis;
        impl ExtraAxis for PlaceholderExtraAxis {
            fn check_move(&self, _move_params: &Move, _axis_index: usize) -> Result<(), String> { Ok(()) }
            fn process_move(&mut self, _move_time: f64, _move_params: &Move, _axis_index: usize) {}
            fn calc_junction(&self, _prev_move: &Move, _current_move: &Move, _e_index: usize) -> f64 { f64::INFINITY }
        }
        toolhead.extra_axes.push(Box::new(PlaceholderExtraAxis));

        Ok(toolhead)
    }

    fn _calc_junction_deviation(&mut self) {
        let scv2 = self.square_corner_velocity.powi(2);
        if self.max_accel == 0.0 {
            self.junction_deviation = 0.0;
        } else {
            self.junction_deviation = scv2 * (2.0_f64.sqrt() - 1.0) / self.max_accel;
        }
        self.max_accel_to_decel = self.max_accel * (1.0 - self.min_cruise_ratio);
    }

    fn _advance_flush_time(&mut self, flush_time: f64) {
        let flush_time = flush_time.max(self.last_flush_time);
        let sg_flush_time = flush_time;
        let free_time = sg_flush_time - self.kin_flush_delay;

        self.trapq.finalize_moves(free_time, self.clear_history_time);
        for ftq in self.flush_trapqs.iter_mut() {
            ftq.finalize_moves(free_time, self.clear_history_time);
        }
        for mcu_item in &self.all_mcus {
            // mcu_item.flush_moves(flush_time, clear_history_time);
        }
        self.last_flush_time = flush_time;
    }

    fn _advance_move_time(&mut self, next_print_time: f64) {
        let pt_delay = self.kin_flush_delay + STEPCOMPRESS_FLUSH_TIME;
        let mut current_flush_time = self.last_flush_time.max(self.print_time - pt_delay);
        self.print_time = self.print_time.max(next_print_time);
        let want_flush_time = current_flush_time.max(self.print_time - pt_delay);

        while current_flush_time < want_flush_time {
            current_flush_time = (current_flush_time + MOVE_BATCH_TIME).min(want_flush_time);
            self._advance_flush_time(current_flush_time);
        }
    }

    fn _calc_print_time(&mut self) {
        let curtime = self.reactor.monotonic();
        let est_print_time = self.mcu.estimated_print_time(curtime);
        let kin_time = (est_print_time + MIN_KIN_TIME).max(self.min_restart_time);
        let kin_time = kin_time + self.kin_flush_delay;
        let min_print_time = (est_print_time + BUFFER_TIME_START).max(kin_time);

        if min_print_time > self.print_time {
            self.print_time = min_print_time;
            self.printer.send_event("toolhead:sync_print_time".to_string(), format!("curtime:{}, est_print_time:{}, print_time:{}", curtime, est_print_time, self.print_time));
        }
    }

    fn _process_lookahead(&mut self, lazy: bool) {
        let moves = self.lookahead.flush(lazy);
        if moves.is_empty() {
            return;
        }

        if !self.special_queuing_state.is_empty() {
            self.special_queuing_state = "".to_string();
            self._calc_print_time();
        }

        let mut next_move_time = self.print_time;
        for mut move_item in moves {
            if move_item.is_kinematic_move {
                self.trapq.append(
                    next_move_time,
                    move_item.accel_t, move_item.cruise_t, move_item.decel_t,
                    move_item.start_pos[0], move_item.start_pos[1], move_item.start_pos[2],
                    move_item.axes_r[0], move_item.axes_r[1], move_item.axes_r[2],
                    move_item.start_v, move_item.cruise_v, move_item.accel);
            }

            for (e_index, ea) in self.extra_axes.iter_mut().enumerate() {
                if move_item.axes_d[e_index + 3].abs() > f64::EPSILON {
                    ea.process_move(next_move_time, &move_item, e_index + 3);
                }
            }
            next_move_time += move_item.accel_t + move_item.cruise_t + move_item.decel_t;
        }
        self.step_gen_time = self.step_gen_time.max(next_move_time + self.kin_flush_delay);
        self.need_flush_time = self.need_flush_time.max(next_move_time + self.kin_flush_delay);
        self._advance_move_time(next_move_time);
    }

    pub fn get_position(&self) -> [f64; 4] {
        self.commanded_pos
    }

    pub fn set_position(&mut self, new_pos: [f64; 4], homing_axes: Option<[bool;3]>) {
        if let Some(axes) = homing_axes {
            if axes[0] { self.commanded_pos[0] = new_pos[0]; }
            if axes[1] { self.commanded_pos[1] = new_pos[1]; }
            if axes[2] { self.commanded_pos[2] = new_pos[2]; }
        } else {
            self.commanded_pos = new_pos;
        }

        if let Some(mask) = homing_axes {
            let kin_new_pos = [new_pos[0], new_pos[1], new_pos[2]];
            self.kin.set_kinematics_position(&kin_new_pos, mask);
        }
        self.printer.send_event("toolhead:set_position".to_string(), "".to_string());
    }

    pub fn get_reactor(&self) -> &dyn Reactor {
        self.reactor
    }

    pub fn perform_homing_move(
        &mut self,
        axis_index: usize,
    ) -> Result<f64, String> {
        if axis_index > 2 {
            return Err("Invalid axis_index for homing".to_string());
        }

        let (endstop, gcode_pos_after_homing, axis_char) = match axis_index {
            0 => (&self.x_endstop, X_GCODE_POSITION_AFTER_HOMING, 'X'),
            1 => (&self.y_endstop, Y_GCODE_POSITION_AFTER_HOMING, 'Y'),
            2 => (&self.z_endstop, Z_GCODE_POSITION_AFTER_HOMING, 'Z'),
            _ => unreachable!(),
        };
        let homing_speed = endstop.homing_speed; // Use configured homing speed

        let homing_positive_dir = false;
        let step_size = if homing_positive_dir { 1.0 } else { -1.0 };
        let max_homing_travel = 300.0;
        let mut travel_so_far = 0.0;

        self.wait_moves();

        println!(
            "ToolHead: Starting iterative homing for axis {}. Speed: {:.1} Target machine_pos: {:.3f}",
            axis_char, homing_speed, endstop.trigger_position_machine
        );

        loop {
            let current_simulated_machine_pos = self.commanded_pos[axis_index];
            if endstop.is_triggered_by_pos(current_simulated_machine_pos, homing_positive_dir) {
                println!(
                    "ToolHead: Axis {} endstop triggered at machine_pos {:.3f} (simulated, target was {:.3f})",
                    axis_char, current_simulated_machine_pos, endstop.trigger_position_machine
                );
                self.commanded_pos[axis_index] = gcode_pos_after_homing;
                return Ok(endstop.trigger_position_machine);
            }

            if travel_so_far >= max_homing_travel {
                return Err(format!(
                    "Homing failed for axis {}: Max travel {:.1f}mm reached without trigger",
                    axis_char, max_homing_travel
                ));
            }
            let mut next_pos_gcode = self.commanded_pos;
            next_pos_gcode[axis_index] += step_size;

            if let Err(e) = self.move_to(next_pos_gcode, homing_speed) {
                return Err(format!("Error during homing step for axis {}: {}", axis_char, e));
            }
            self.wait_moves();
            travel_so_far += step_size.abs();
        }
    }

    pub fn move_to(&mut self, newpos: [f64; 4], speed: f64) -> Result<(), String> {
        let mut move_instance = Move::new(
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
            self.kin.check_move(&mut move_instance)?;
        }

        for (e_index, ea) in self.extra_axes.iter().enumerate() {
            if e_index < move_instance.axes_d.len() - 3 && move_instance.axes_d[e_index + 3].abs() > f64::EPSILON {
                ea.check_move(&move_instance, e_index + 3)?;
            }
        }

        self.commanded_pos = move_instance.end_pos;
        let want_flush = self.lookahead.add_move(move_instance, &self.extra_axes);

        if want_flush {
            self._process_lookahead(true);
        }
        Ok(())
    }

    pub fn dwell(&mut self, delay: f64) -> Result<(), String> {
        let next_print_time = self.get_last_move_time() + delay.max(0.0);
        self._advance_move_time(next_print_time);
        Ok(())
    }

    pub fn wait_moves(&mut self) {
        self._flush_lookahead();
        println!("ToolHead: wait_moves called (lookahead flushed).");
    }

    pub fn get_kinematics_calculated_position(&self) -> Result<[f64; 3], String> {
        self.kin.calc_position_from_steppers()
    }

    pub fn move_kinematics_steppers_to_gcode_target(&mut self, target_gcode_pos: [f64;3]) -> Result<(), String> {
        self.kin.set_steppers_to_gcode_target(target_gcode_pos)
    }
}

pub trait PrinterUtility {
    fn send_event(&self, event: &str, params_str: String);
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::configfile::Configfile;
    use crate::reactor::Reactor;
    use crate::mcu::Mcu;

    struct MockPrinterUtility;
    impl PrinterUtility for MockPrinterUtility {
        fn send_event(&self, event: &str, _params: String) {
            println!("MockPrinterUtility: Event: {}", event);
        }
    }
    struct MockReactor;
    impl Reactor for MockReactor {
        fn monotonic(&self) -> f64 { 0.0 }
        fn register_timer(&mut self, _time: f64, _callback: Box<dyn FnMut(f64) -> Option<f64>>) -> usize { 0 }
        fn register_fd(&mut self, _fd: i32, _callback: Box<dyn FnMut(f64)>) -> usize {0}
        fn unregister_fd(&mut self, _handle_id: usize) {}
        fn unregister_timer(&mut self, _handle_id: usize) {}
        fn is_shutdown(&self) -> bool {false}
        fn run(&mut self) {}
        fn pause(&mut self, _waketime: f64) {}
        fn _check_timers(&mut self, _eventtime: f64, _idle: bool) {}
    }
    struct MockMcu;
    impl Mcu for MockMcu {
        fn estimated_print_time(&self, _curtime: f64) -> f64 {0.0}
    }

    fn approx_eq(a: f64, b: f64, epsilon: f64) -> bool {
        (a - b).abs() < epsilon
    }

    struct MockExtraAxis;
    impl ExtraAxis for MockExtraAxis {
        fn check_move(&self, _move_params: &Move, _axis_index: usize) -> Result<(), String> { Ok(()) }
        fn process_move(&mut self, _move_time: f64, _move_params: &Move, _axis_index: usize) {}
        fn calc_junction(&self, _prev_move: &Move, _current_move: &Move, _e_index: usize) -> f64 {
            f64::INFINITY
        }
    }

    const DEFAULT_MAX_ACCEL_TEST: f64 = 3000.0; // Renamed to avoid conflict
    const DEFAULT_JUNCTION_DEV_TEST: f64 = 0.013;
    const DEFAULT_MAX_VELOCITY_TEST: f64 = 500.0;
    const DEFAULT_MAX_ACCEL_TO_DECEL_TEST: f64 = DEFAULT_MAX_ACCEL_TEST / 2.0;

    fn create_test_toolhead_with_reactor_printer() -> (ToolHead<'static>, Box<MockReactor>, Box<MockPrinterUtility>) {
        let mut config = Configfile::new(None);
        config.add_section("printer");
        config.set("printer", "max_velocity", &DEFAULT_MAX_VELOCITY_TEST.to_string());
        config.set("printer", "max_accel", &DEFAULT_MAX_ACCEL_TEST.to_string());
        config.set("printer", "square_corner_velocity", &"5.0".to_string());

        for axis in ["x", "y", "z"].iter() {
            let sec_name = format!("stepper_{}", axis);
            config.add_section(&sec_name);
            config.set(&sec_name, "position_min", "0");
            config.set(&sec_name, "position_max", if *axis == "z" {"180"} else {"200"});
            config.set(&sec_name, "position_endstop", "0");
            config.set(&sec_name, "rotation_distance", if *axis == "z" {"8"} else {"40"});
            config.set(&sec_name, "full_steps_per_rotation", "200");
            config.set(&sec_name, "microsteps", "16");
            config.set(&sec_name, "gear_ratio", "1.0");
            config.set(&sec_name, "homing_speed", if *axis == "z" {"10"} else {"50"});
            config.set(&sec_name, "homing_retract_dist", if *axis == "z" {"2"} else {"5"});
            config.set(&sec_name, "second_homing_speed", if *axis == "z" {"5"} else {"25"});
        }
        config.add_section("extruder");
        config.set("extruder", "min_temp", "0");
        config.set("extruder", "max_temp", "280");
        config.add_section("heater_bed");
        config.set("heater_bed", "min_temp", "0");
        config.set("heater_bed", "max_temp", "120");
        config.add_section("fan");


        let reactor = Box::new(MockReactor);
        let printer_util = Box::new(MockPrinterUtility);
        let mcu = Box::new(MockMcu);

        let static_reactor: &'static MockReactor = Box::leak(reactor);
        let static_printer_util: &'static MockPrinterUtility = Box::leak(printer_util);
        let static_mcu: &'static MockMcu = Box::leak(mcu);

        let th = ToolHead::new(&config, static_reactor, static_printer_util, vec![static_mcu]).unwrap();

        (th, Box::new(MockReactor), Box::new(MockPrinterUtility))
    }


    #[test]
    fn test_move_new_normal_move() {
        let start_pos = [0.0, 0.0, 0.0, 0.0];
        let end_pos = [10.0, 0.0, 0.0, 0.0];
        let speed = 100.0;
        let m = Move::new(
            DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST,
            start_pos, end_pos, speed
        );

        assert!(m.is_kinematic_move);
        assert!(approx_eq(m.move_d, 10.0, 1e-9));
        assert!(approx_eq(m.axes_d[0], 10.0, 1e-9));
        assert!(approx_eq(m.axes_r[0], 1.0, 1e-9));
        assert!(approx_eq(m.min_move_t, 10.0 / 100.0, 1e-9));
        assert!(approx_eq(m.max_cruise_v2, speed.powi(2), 1e-9));
        assert!(approx_eq(m.accel, DEFAULT_MAX_ACCEL_TEST, 1e-9));
        assert!(approx_eq(m.delta_v2, 2.0 * 10.0 * DEFAULT_MAX_ACCEL_TEST, 1e-9));
    }

    #[test]
    fn test_move_new_extrude_only_move() {
        let start_pos = [10.0, 0.0, 0.0, 0.0];
        let end_pos = [10.0, 0.0, 0.0, 5.0];
        let speed = 20.0;
        let m = Move::new(
            DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST,
            start_pos, end_pos, speed
        );

        assert!(!m.is_kinematic_move);
        assert!(approx_eq(m.move_d, 5.0, 1e-9));
        assert!(approx_eq(m.axes_d[3], 5.0, 1e-9));
        assert!(approx_eq(m.axes_r[3], 1.0, 1e-9));
        assert!(approx_eq(m.min_move_t, 5.0 / 20.0, 1e-9));
        assert!(approx_eq(m.max_cruise_v2, speed.powi(2), 1e-9));
        assert!(approx_eq(m.accel, 99999999.9, 1e-9));
    }

    #[test]
    fn test_move_new_zero_length_kinematic_move() {
        let start_pos = [0.0, 0.0, 0.0, 0.0];
        let end_pos = [0.0, 0.0, 0.0, 0.0];
        let speed = 100.0;
         let m = Move::new(
            DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST,
            start_pos, end_pos, speed
        );
        assert!(!m.is_kinematic_move);
        assert!(approx_eq(m.move_d, 0.0, 1e-9));
    }


    #[test]
    fn test_move_limit_speed() {
        let start_pos = [0.0, 0.0, 0.0, 0.0];
        let end_pos = [10.0, 0.0, 0.0, 0.0];
        let initial_speed = 100.0;
        let mut m = Move::new(
            DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST,
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
            initial_accel, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST,
            start_pos, end_pos, initial_speed
        );
        m.limit_speed(150.0, 1000.0);

        assert!(approx_eq(m.max_cruise_v2, initial_speed.powi(2), 1e-9));
        assert!(approx_eq(m.min_move_t, 10.0 / initial_speed, 1e-9));
        assert!(approx_eq(m.accel, 1000.0, 1e-9));
        assert!(approx_eq(m.delta_v2, 2.0 * 10.0 * 1000.0, 1e-9));
    }

    #[test]
    fn test_move_set_junction() {
        let start_pos = [0.0, 0.0, 0.0, 0.0];
        let end_pos = [100.0, 0.0, 0.0, 0.0];
        let speed = 100.0;
        let accel = 1000.0;
        let mut m = Move::new(
            accel, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST,
            start_pos, end_pos, speed
        );
        let start_v = 0.0;
        let cruise_v = 50.0;
        let end_v = 0.0;
        m.set_junction(start_v.powi(2), cruise_v.powi(2), end_v.powi(2));

        assert!(approx_eq(m.start_v, start_v, 1e-9));
        assert!(approx_eq(m.cruise_v, cruise_v, 1e-9));
        assert!(approx_eq(m.end_v, end_v, 1e-9));

        let expected_accel_d = (cruise_v.powi(2) - start_v.powi(2)) / (2.0 * accel);
        let expected_decel_d = (cruise_v.powi(2) - end_v.powi(2)) / (2.0 * accel);

        assert!(approx_eq(m.accel_t, expected_accel_d / ((start_v + cruise_v) * 0.5), 1e-9));
        assert!(approx_eq(m.decel_t, expected_decel_d / ((end_v + cruise_v) * 0.5), 1e-9));
        assert!(approx_eq(m.cruise_t, (m.move_d - expected_accel_d - expected_decel_d) / cruise_v, 1e-5));
    }

    #[test]
    fn test_move_calc_junction_90_degree_turn() {
        let extra_axes_mock: Vec<Box<dyn ExtraAxis>> = vec![Box::new(MockExtraAxis)];
        let move1_start = [0.0, 0.0, 0.0, 0.0];
        let move1_end = [10.0, 0.0, 0.0, 0.0];
        let mut move1 = Move::new(DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST, move1_start, move1_end, 50.0);
        move1.max_start_v2 = 0.0;
        move1.max_smoothed_v2 = 0.0;

        let move2_start = [10.0, 0.0, 0.0, 0.0];
        let move2_end = [10.0, 10.0, 0.0, 0.0];
        let mut move2 = Move::new(DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST, move2_start, move2_end, 50.0);

        move2.calc_junction(&move1, &extra_axes_mock);

        assert!(move2.max_start_v2 < 100.0, "max_start_v2 was {}", move2.max_start_v2);
        assert!(approx_eq(move2.max_start_v2, 94.14753, 1e-3));
    }

    #[test]
    fn test_lookahead_add_move() {
        let mut laq = LookAheadQueue::new();
        let extra_axes_mock: Vec<Box<dyn ExtraAxis>> = vec![Box::new(MockExtraAxis)];

        let move1 = Move::new(DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST, [0.0,0.0,0.0,0.0], [10.0,0.0,0.0,0.0], 100.0);
        let initial_junction_flush = laq.junction_flush;
        let min_move_t1 = move1.min_move_t;

        laq.add_move(move1, &extra_axes_mock);
        assert_eq!(laq.queue.len(), 1);
        assert!(approx_eq(laq.junction_flush, initial_junction_flush - min_move_t1, 1e-9));

        let move2 = Move::new(DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST, [10.0,0.0,0.0,0.0], [10.0,10.0,0.0,0.0], 100.0);
        let min_move_t2 = move2.min_move_t;
        laq.add_move(move2, &extra_axes_mock);
        assert_eq!(laq.queue.len(), 2);
        assert!(approx_eq(laq.junction_flush, initial_junction_flush - min_move_t1 - min_move_t2, 1e-9));
        assert!(laq.queue[1].max_start_v2 < laq.queue[1].max_cruise_v2);
    }

    #[test]
    fn test_lookahead_flush_simple() {
        let mut laq = LookAheadQueue::new();
        let extra_axes_mock: Vec<Box<dyn ExtraAxis>> = vec![Box::new(MockExtraAxis)];

        let move1 = Move::new(DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST, [0.0,0.0,0.0,0.0], [10.0,0.0,0.0,0.0], 100.0);
        let move2 = Move::new(DEFAULT_MAX_ACCEL_TEST, DEFAULT_JUNCTION_DEV_TEST, DEFAULT_MAX_VELOCITY_TEST, DEFAULT_MAX_ACCEL_TO_DECEL_TEST, [10.0,0.0,0.0,0.0], [20.0,0.0,0.0,0.0], 100.0);
        laq.add_move(move1.clone(), &extra_axes_mock);
        laq.add_move(move2.clone(), &extra_axes_mock);

        let flushed_moves = laq.flush(false);
        assert_eq!(flushed_moves.len(), 2);
        assert_eq!(laq.queue.len(), 0);
        assert!(flushed_moves[0].start_v > 0.0 || flushed_moves[0].cruise_t > 0.0);
        assert!(flushed_moves[1].start_v > 0.0 || flushed_moves[1].cruise_t > 0.0);
    }
}

[end of klipper_host_rust/src/toolhead.rs]

[end of klipper_host_rust/src/toolhead.rs]

[end of klipper_host_rust/src/toolhead.rs]

[end of klipper_host_rust/src/toolhead.rs]
