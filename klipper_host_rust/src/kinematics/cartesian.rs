// klipper_host_rust/src/kinematics/cartesian.rs

use std::collections::HashMap;
// Assuming Kinematics trait is in toolhead.rs or crate::core_traits
// Let's assume it's in core_traits for better separation for now
// use crate::core_traits::{Kinematics};
use crate::toolhead::{Kinematics, Move}; // Kinematics trait is in toolhead.rs for now


// Simplified Stepper representation for host-side kinematics
#[derive(Debug, Clone)]
pub struct Stepper {
    name: String,
    // Future fields: step_dist, units_in_radians, etc.
}

impl Stepper {
    pub fn new(name: String) -> Self {
        Stepper { name }
    }

    #[allow(dead_code)]
    pub fn get_name(&self) -> &str {
        &self.name
    }
}

// Simplified Rail representation for host-side kinematics
// A Rail is typically associated with one axis and can have multiple steppers.
#[derive(Debug, Clone)]
pub struct Rail {
    name: String, // e.g., "x", "y", "z"
    steppers: Vec<Stepper>, // For cartesian, usually one stepper per rail (X, Y, Z)
    pub position_min: f64,
    pub position_max: f64,
    pub position_endstop: f64, // Machine coordinate of the endstop for this rail
    // Future fields: homing_speed, homing_retract_dist, etc.
}

impl Rail {
    pub fn new(
        name: String,
        stepper_names: Vec<String>,
        position_min: f64,
        position_max: f64,
        position_endstop: f64,
    ) -> Self {
        let steppers = stepper_names.into_iter().map(Stepper::new).collect();
        Rail {
            name,
            steppers,
            position_min,
            position_max,
            position_endstop,
        }
    }

    pub fn get_range(&self) -> (f64, f64) {
        (self.position_min, self.position_max)
    }

    #[allow(dead_code)]
    pub fn get_steppers(&self) -> &Vec<Stepper> {
        &self.steppers
    }

    // In Klipper, Rail.set_position updates the logical position of its steppers.
    // For now, this is a placeholder as our Stepper struct is simple.
    #[allow(dead_code)]
    pub fn set_stepper_positions(&self, _gcode_coord: f64) {
        // TODO: When steppers store their own positions, update them here
        // based on gcode_coord and the rail's step_distance etc.
        // For cartesian, it's usually a direct mapping.
    }
}

// CartesianKinematics struct will be defined next.

pub struct CartesianKinematics {
    rails: [Rail; 3], // Index 0 for X, 1 for Y, 2 for Z
    max_z_velocity: f64,
    max_z_accel: f64,
    limits: [(f64, f64); 3], // (min, max) operational limits, updated after homing
}

impl CartesianKinematics {
    pub fn new(
        x_rail_config: (Vec<String>, f64, f64, f64), // stepper_names, min, max, endstop_pos
        y_rail_config: (Vec<String>, f64, f64, f64),
        z_rail_config: (Vec<String>, f64, f64, f64),
        max_z_velocity: f64,
        max_z_accel: f64,
    ) -> Self {
        let rail_x = Rail::new("x".to_string(), x_rail_config.0, x_rail_config.1, x_rail_config.2, x_rail_config.3);
        let rail_y = Rail::new("y".to_string(), y_rail_config.0, y_rail_config.1, y_rail_config.2, y_rail_config.3);
        let rail_z = Rail::new("z".to_string(), z_rail_config.0, z_rail_config.1, z_rail_config.2, z_rail_config.3);

        CartesianKinematics {
            rails: [rail_x, rail_y, rail_z],
            max_z_velocity,
            max_z_accel,
            // Initialize limits to "un-homed" state (min > max)
            // These will be updated by set_position after homing.
            limits: [(1.0, -1.0), (1.0, -1.0), (1.0, -1.0)],
        }
    }

    // Methods for Kinematics trait will be added next
}

impl Kinematics for CartesianKinematics {
    fn check_move(&self, move_params: &mut Move) -> Result<(), String> {
        let x_target = move_params.end_pos[0];
        let y_target = move_params.end_pos[1];
        let z_target = move_params.end_pos[2];

        // Check X and Y limits first
        if x_target < self.limits[0].0 || x_target > self.limits[0].1 {
            if self.limits[0].0 > self.limits[0].1 { // Unhomed state
                return Err(move_params.move_error("X axis must be homed first"));
            }
            return Err(move_params.move_error(&format!(
                "Move out of range: X {:.3} (min: {:.3}, max: {:.3})",
                x_target, self.limits[0].0, self.limits[0].1
            )));
        }
        if y_target < self.limits[1].0 || y_target > self.limits[1].1 {
            if self.limits[1].0 > self.limits[1].1 { // Unhomed state
                return Err(move_params.move_error("Y axis must be homed first"));
            }
            return Err(move_params.move_error(&format!(
                "Move out of range: Y {:.3} (min: {:.3}, max: {:.3})",
                y_target, self.limits[1].0, self.limits[1].1
            )));
        }

        // If there's Z movement, check Z limits and apply Z-specific speed/accel
        if move_params.axes_d[2].abs() > f64::EPSILON { // axes_d[2] is dz
            if z_target < self.limits[2].0 || z_target > self.limits[2].1 {
                if self.limits[2].0 > self.limits[2].1 { // Unhomed state
                    return Err(move_params.move_error("Z axis must be homed first"));
                }
                return Err(move_params.move_error(&format!(
                    "Move out of range: Z {:.3} (min: {:.3}, max: {:.3})",
                    z_target, self.limits[2].0, self.limits[2].1
                )));
            }

            // Apply Z-specific speed/accel limits
            // Klipper: z_ratio = move.move_d / abs(move.axes_d[2])
            // move.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
            if move_params.move_d > f64::EPSILON && move_params.axes_d[2].abs() > f64::EPSILON {
                let z_ratio = move_params.move_d / move_params.axes_d[2].abs();
                move_params.limit_speed(
                    self.max_z_velocity * z_ratio,
                    self.max_z_accel * z_ratio,
                );
            }
        }
        Ok(())
    }

    fn set_kinematics_position(&mut self, new_pos_gcode: &[f64; 3], homed_axes_mask: [bool; 3]) {
        // new_pos_gcode contains the G-CODE coordinates for X, Y, Z that the machine is now at
        // after a G92 or homing operation for the axes specified in homed_axes_mask.

        for axis_idx in 0..3 {
            if homed_axes_mask[axis_idx] {
                // Mark axis as homed by setting its limits to the rail's full range
                self.limits[axis_idx] = self.rails[axis_idx].get_range();

                // Klipper's rail.set_position(newpos) updates the rail's steppers' internal positions.
                // This is important for `calc_position` if it were to be used.
                // For now, our Rail.set_stepper_positions is a placeholder.
                self.rails[axis_idx].set_stepper_positions(new_pos_gcode[axis_idx]);

                println!(
                    "CartesianKinematics: Axis {} homed. Limits set to: ({:.3}, {:.3}). G-code pos: {:.3}",
                    ['X', 'Y', 'Z'][axis_idx],
                    self.limits[axis_idx].0,
                    self.limits[axis_idx].1,
                    new_pos_gcode[axis_idx]
                );
            }
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rail_creation() {
        let rail_x = Rail::new("x_rail".to_string(), vec!["stepper_x".to_string()], 0.0, 200.0, 0.0);
        assert_eq!(rail_x.name, "x_rail");
        assert_eq!(rail_x.position_min, 0.0);
        assert_eq!(rail_x.position_max, 200.0);
        assert_eq!(rail_x.position_endstop, 0.0);
        assert_eq!(rail_x.steppers.len(), 1);
        assert_eq!(rail_x.steppers[0].get_name(), "stepper_x");
    }

    fn create_test_kinematics() -> CartesianKinematics {
        CartesianKinematics::new(
            (vec!["stepper_x".to_string()], 0.0, 200.0, 0.0), // X rail
            (vec!["stepper_y".to_string()], 0.0, 200.0, 0.0), // Y rail
            (vec!["stepper_z".to_string()], 0.0, 180.0, 0.0), // Z rail
            25.0, // max_z_velocity
            500.0, // max_z_accel
        )
    }

    // Helper to create a basic Move struct for testing check_move
    fn create_test_move(start_pos: [f64;4], end_pos: [f64;4], speed: f64) -> Move {
        Move::new(3000.0, 0.01, 300.0, 1500.0, start_pos, end_pos, speed)
    }


    #[test]
    fn test_check_move_valid() {
        let mut kin = create_test_kinematics();
        // Mark all axes as homed by setting valid limits
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);

        let mut move_valid = create_test_move([0.0,0.0,0.0,0.0], [10.0, 10.0, 10.0, 0.0], 100.0);
        assert!(kin.check_move(&mut move_valid).is_ok());
    }

    #[test]
    fn test_check_move_out_of_bounds_x_max() {
        let mut kin = create_test_kinematics();
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);
        let mut move_invalid = create_test_move([0.0,0.0,0.0,0.0], [210.0, 10.0, 10.0, 0.0], 100.0);
        let result = kin.check_move(&mut move_invalid);
        assert!(result.is_err());
        assert!(result.unwrap_err().message.contains("Move out of range: X 210.000"));
    }

    #[test]
    fn test_check_move_out_of_bounds_y_min() {
        let mut kin = create_test_kinematics();
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);
        let mut move_invalid = create_test_move([0.0,0.0,0.0,0.0], [10.0, -10.0, 10.0, 0.0], 100.0);
        let result = kin.check_move(&mut move_invalid);
        assert!(result.is_err());
        assert!(result.unwrap_err().message.contains("Move out of range: Y -10.000"));
    }

    #[test]
    fn test_check_move_out_of_bounds_z_max() {
        let mut kin = create_test_kinematics();
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);
        let mut move_invalid = create_test_move([0.0,0.0,0.0,0.0], [10.0, 10.0, 190.0, 0.0], 100.0);
        let result = kin.check_move(&mut move_invalid);
        assert!(result.is_err());
        assert!(result.unwrap_err().message.contains("Move out of range: Z 190.000"));
    }

    #[test]
    fn test_check_move_unhomed_axis() {
        let kin = create_test_kinematics(); // Limits are (1.0, -1.0) by default
        let mut move_unhomed = create_test_move([0.0,0.0,0.0,0.0], [10.0, 10.0, 10.0, 0.0], 100.0);
        let result = kin.check_move(&mut move_unhomed);
        assert!(result.is_err());
        assert!(result.unwrap_err().message.contains("X axis must be homed first"));
    }

    #[test]
    fn test_check_move_z_limits_speed_accel() {
        let mut kin = create_test_kinematics();
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);

        let original_max_cruise_v2 = (100.0_f64).powi(2);
        let original_accel = 3000.0;

        let mut move_z = create_test_move([0.0,0.0,0.0,0.0], [0.0, 0.0, 10.0, 0.0], 100.0);
        // Ensure initial move values are what we expect before limit_speed is called
        assert_eq!(move_z.max_cruise_v2, original_max_cruise_v2);
        assert_eq!(move_z.accel, original_accel);

        kin.check_move(&mut move_z).unwrap();

        // move_d is 10.0 for Z only move. axes_d[2] is 10.0. z_ratio = 10.0 / 10.0 = 1.0.
        // max_z_velocity = 25.0, max_z_accel = 500.0
        // Expected limited speed = 25.0 * 1.0 = 25.0
        // Expected limited accel = 500.0 * 1.0 = 500.0
        let expected_limited_speed = kin.max_z_velocity; // * z_ratio (which is 1.0)
        let expected_limited_accel = kin.max_z_accel;    // * z_ratio (which is 1.0)

        assert_eq!(move_z.max_cruise_v2, expected_limited_speed.powi(2));
        assert_eq!(move_z.accel, expected_limited_accel);
    }

     #[test]
    fn test_set_kinematics_position_updates_limits() {
        let mut kin = create_test_kinematics();
        assert_eq!(kin.limits[0], (1.0, -1.0)); // Unhomed

        kin.set_kinematics_position(&[0.0, 0.0, 0.0], [true, false, false]); // Home X
        assert_eq!(kin.limits[0], (0.0, 200.0)); // X homed
        assert_eq!(kin.limits[1], (1.0, -1.0));  // Y still unhomed
        assert_eq!(kin.limits[2], (1.0, -1.0));  // Z still unhomed

        kin.set_kinematics_position(&[0.0, 0.0, 0.0], [false, true, true]); // Home Y and Z
        assert_eq!(kin.limits[0], (0.0, 200.0));  // X still homed
        assert_eq!(kin.limits[1], (0.0, 200.0));  // Y homed
        assert_eq!(kin.limits[2], (0.0, 180.0));  // Z homed
    }
}
