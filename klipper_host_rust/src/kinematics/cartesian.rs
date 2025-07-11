// klipper_host_rust/src/kinematics/cartesian.rs

use std::collections::HashMap;
// Assuming Kinematics trait is in toolhead.rs or crate::core_traits
// Let's assume it's in core_traits for better separation for now
// use crate::core_traits::{Kinematics};
use crate::toolhead::{Kinematics, Move}; // Kinematics trait is in toolhead.rs for now

// Default step distance, e.g., for a 200 steps/rev motor, 16 microsteps, 2mm pitch leadscrew / GT2 belt with 20T pulley
// rotation_distance = 40 (for 20T pulley and GT2 belt)
// full_steps_per_rotation = 200
// microsteps = 16
// step_distance = 40 / (200 * 16) = 40 / 3200 = 0.0125 mm/step
pub const DEFAULT_STEP_DISTANCE: f64 = 0.0125; // Made pub for ToolHead defaults if needed
pub const DEFAULT_ROTATION_DISTANCE: f64 = 40.0;
pub const DEFAULT_FULL_STEPS_PER_ROTATION: u32 = 200;
pub const DEFAULT_MICROSTEPS: u32 = 16;


// Simplified Stepper representation for host-side kinematics
#[derive(Debug, Clone)]
pub struct Stepper {
    name: String,
    pub rotation_distance: f64,
    pub full_steps_per_rotation: u32,
    pub microsteps: u32,
    pub gear_ratio: f64,
    pub steps_per_rotation: u32, // Effective total steps for one motor rotation (full * micro * gear_inv)
    pub step_distance: f64,      // Calculated: rotation_distance / steps_per_rotation
    position_steps: i64,      // Current position in microsteps from mcu_position_offset origin
    mcu_position_offset: f64, // Offset in mm to align step counter with G-code zero
}

impl Stepper {
    pub fn new(
        name: String,
        rotation_distance: f64,
        full_steps_per_rotation: u32,
        microsteps: u32,
        gear_ratio: f64, // Simplified: product of gear ratios. Klipper: list of num:den
    ) -> Self {
        let effective_gearing = if gear_ratio == 0.0 { 1.0 } else { gear_ratio }; // Avoid div by zero if gear_ratio is misconfigured as 0
        let steps_per_rotation_calc = (full_steps_per_rotation * microsteps) as f64 * effective_gearing;

        if steps_per_rotation_calc < 1.0 { // Must be at least 1 effective step
             eprintln!("Warning: Calculated steps_per_rotation is less than 1 for stepper {}. Clamping to 1.", name);
        }
        // Ensure steps_per_rotation is at least 1 to avoid division by zero or nonsensical step_distance
        let steps_per_rotation = (steps_per_rotation_calc.max(1.0)) as u32;


        let step_distance = rotation_distance / (steps_per_rotation as f64);

        Stepper {
            name,
            rotation_distance,
            full_steps_per_rotation,
            microsteps,
            gear_ratio: effective_gearing, // Store the used gear_ratio
            steps_per_rotation, // Store calculated effective steps
            step_distance,
            position_steps: 0,
            mcu_position_offset: 0.0,
        }
    }

    #[allow(dead_code)]
    pub fn get_name(&self) -> &str {
        &self.name
    }

    /// Sets the G-code coordinate system origin for this stepper.
    /// After this, get_commanded_position_mm() will report gcode_coord_mm
    /// if position_steps is unchanged.
    pub fn set_commanded_position_mm(&mut self, gcode_coord_mm: f64) {
        self.mcu_position_offset = (self.position_steps as f64 * self.step_distance) - gcode_coord_mm;
        // println!("Stepper {}: set_commanded_pos_mm: gcode_coord={}, steps={}, new_offset={}",
        //    self.name, gcode_coord_mm, self.position_steps, self.mcu_position_offset);
    }

    /// Gets the current G-code position of this stepper in mm.
    pub fn get_commanded_position_mm(&self) -> f64 {
        (self.position_steps as f64 * self.step_distance) - self.mcu_position_offset
    }

    /// Directly sets the (simulated) hardware step counter for this stepper.
    /// Used to simulate the result of a move.
    pub fn set_position_steps(&mut self, steps: i64) {
        // println!("Stepper {}: set_position_steps from {} to {}", self.name, self.position_steps, steps);
        self.position_steps = steps;
    }

    #[allow(dead_code)]
    pub fn get_position_steps(&self) -> i64 {
        self.position_steps
    }

    pub fn get_step_distance(&self) -> f64 {
        self.step_distance
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
}

impl Rail {
    pub fn new(
        name: String,
        // Vec of (stepper_name, rotation_dist, full_steps, microsteps, gear_ratio)
        stepper_configs: Vec<(String, f64, u32, u32, f64)>,
        position_min: f64,
        position_max: f64,
        position_endstop: f64,
    ) -> Self {
        let steppers = stepper_configs
            .into_iter()
            .map(|(s_name, rot_dist, f_steps, m_steps, gr)| Stepper::new(s_name, rot_dist, f_steps, m_steps, gr))
            .collect();
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

    // For Cartesian, assume the first stepper defines the rail's position characteristics
    fn primary_stepper(&self) -> Result<&Stepper, String> {
        self.steppers.get(0).ok_or_else(|| format!("Rail {} has no steppers", self.name))
    }

    fn primary_stepper_mut(&mut self) -> Result<&mut Stepper, String> {
        self.steppers.get_mut(0).ok_or_else(|| format!("Rail {} has no steppers", self.name))
    }

    /// Sets the G-code position for this rail, updating its primary stepper's offset.
    pub fn set_rail_position_gcode(&mut self, gcode_coord: f64) -> Result<(), String> {
        self.primary_stepper_mut()?.set_commanded_position_mm(gcode_coord);
        Ok(())
    }

    /// Gets the current G-code position of this rail based on its primary stepper.
    pub fn get_rail_position_gcode(&self) -> Result<f64, String> {
        Ok(self.primary_stepper()?.get_commanded_position_mm())
    }

    /// Simulates moving the rail's steppers to a new G-code coordinate.
    /// This directly updates the stepper's internal step count.
    pub fn move_steppers_to_gcode_coord(&mut self, gcode_coord: f64) -> Result<(), String> {
        let stepper = self.primary_stepper_mut()?;
        let target_steps = ((gcode_coord + stepper.mcu_position_offset) / stepper.step_distance).round() as i64;
        stepper.set_position_steps(target_steps);
        Ok(())
    }
}


pub struct CartesianKinematics {
    rails: [Rail; 3], // Index 0 for X, 1 for Y, 2 for Z
    max_z_velocity: f64,
    max_z_accel: f64,
    limits: [(f64, f64); 3], // (min, max) operational limits, updated after homing
}

impl CartesianKinematics {
    pub fn new(
        // Each tuple: (Vec<(stepper_name, rot_dist, full_steps, microsteps, gear_ratio)>, min_pos, max_pos, endstop_pos)
        x_rail_config: (Vec<(String, f64, u32, u32, f64)>, f64, f64, f64),
        y_rail_config: (Vec<(String, f64, u32, u32, f64)>, f64, f64, f64),
        z_rail_config: (Vec<(String, f64, u32, u32, f64)>, f64, f64, f64),
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
            limits: [(1.0, -1.0), (1.0, -1.0), (1.0, -1.0)], // Unhomed
        }
    }

    /// Calculates current G-code position [X,Y,Z] from the rail/stepper states.
    /// This method assumes that the `position_steps` in each stepper accurately reflects
    /// its current physical state, and `mcu_position_offset` correctly maps this
    /// to the G-code coordinate system.
    pub fn calc_position_from_steppers(&self) -> Result<[f64; 3], String> {
        Ok([
            self.rails[0].get_rail_position_gcode()?,
            self.rails[1].get_rail_position_gcode()?,
            self.rails[2].get_rail_position_gcode()?,
        ])
    }

    /// Simulates moving steppers to a G-code target.
    /// This is for host-side simulation/state tracking, not actual motion planning.
    #[allow(dead_code)] // May be used by ToolHead later
    pub fn set_steppers_to_gcode_target(&mut self, target_gcode_pos: [f64;3]) -> Result<(), String> {
        self.rails[0].move_steppers_to_gcode_coord(target_gcode_pos[0])?;
        self.rails[1].move_steppers_to_gcode_coord(target_gcode_pos[1])?;
        self.rails[2].move_steppers_to_gcode_coord(target_gcode_pos[2])?;
        Ok(())
    }
}

impl Kinematics for CartesianKinematics {
    fn check_move(&self, move_params: &mut Move) -> Result<(), String> {
        let x_target = move_params.end_pos[0];
        let y_target = move_params.end_pos[1];
        let z_target = move_params.end_pos[2];

        if x_target < self.limits[0].0 || x_target > self.limits[0].1 {
            if self.limits[0].0 > self.limits[0].1 {
                return Err(move_params.move_error("X axis must be homed first"));
            }
            return Err(move_params.move_error(&format!(
                "Move out of range: X {:.3} (min: {:.3}, max: {:.3})",
                x_target, self.limits[0].0, self.limits[0].1
            )));
        }
        if y_target < self.limits[1].0 || y_target > self.limits[1].1 {
            if self.limits[1].0 > self.limits[1].1 {
                return Err(move_params.move_error("Y axis must be homed first"));
            }
            return Err(move_params.move_error(&format!(
                "Move out of range: Y {:.3} (min: {:.3}, max: {:.3})",
                y_target, self.limits[1].0, self.limits[1].1
            )));
        }

        if move_params.axes_d[2].abs() > f64::EPSILON {
            if z_target < self.limits[2].0 || z_target > self.limits[2].1 {
                if self.limits[2].0 > self.limits[2].1 {
                    return Err(move_params.move_error("Z axis must be homed first"));
                }
                return Err(move_params.move_error(&format!(
                    "Move out of range: Z {:.3} (min: {:.3}, max: {:.3})",
                    z_target, self.limits[2].0, self.limits[2].1
                )));
            }
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
        for axis_idx in 0..3 {
            if homed_axes_mask[axis_idx] {
                self.limits[axis_idx] = self.rails[axis_idx].get_range();
                // Update the rail's (and its stepper's) understanding of the new G-code coordinate
                if let Err(e) = self.rails[axis_idx].set_rail_position_gcode(new_pos_gcode[axis_idx]) {
                    // This should ideally not fail if rail has steppers. Log or handle error.
                    eprintln!("Error setting rail position for axis {}: {}", ['X','Y','Z'][axis_idx], e);
                }

                println!(
                    "CartesianKinematics: Axis {} homed/set. Limits: ({:.3}, {:.3}). G-code pos: {:.3}. Stepper G-code pos: {:.3}",
                    ['X', 'Y', 'Z'][axis_idx],
                    self.limits[axis_idx].0,
                    self.limits[axis_idx].1,
                    new_pos_gcode[axis_idx],
                    self.rails[axis_idx].get_rail_position_gcode().unwrap_or(f64::NAN)
                );
            }
        }
    }

    #[cfg(test)]
    fn get_axis_limits_for_test(&self) -> [(f64, f64); 3] {
        self.limits
    }

    fn set_steppers_to_gcode_target(&mut self, target_gcode_pos: [f64;3]) -> Result<(), String> {
        self.rails[0].move_steppers_to_gcode_coord(target_gcode_pos[0])?;
        self.rails[1].move_steppers_to_gcode_coord(target_gcode_pos[1])?;
        self.rails[2].move_steppers_to_gcode_coord(target_gcode_pos[2])?;
        Ok(())
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::toolhead::Move;


    #[test]
    fn rail_creation_and_stepper_init() {
        let stepper_configs = vec![(
            "stepper_x".to_string(),
            DEFAULT_ROTATION_DISTANCE,
            DEFAULT_FULL_STEPS_PER_ROTATION,
            DEFAULT_MICROSTEPS,
            1.0 // gear_ratio
        )];
        let rail_x = Rail::new(
            "x_rail".to_string(),
            stepper_configs,
            0.0, 200.0, 0.0
        );
        assert_eq!(rail_x.name, "x_rail");
        assert_eq!(rail_x.position_min, 0.0);
        assert_eq!(rail_x.position_max, 200.0);
        assert_eq!(rail_x.position_endstop, 0.0);
        assert_eq!(rail_x.steppers.len(), 1);
        assert_eq!(rail_x.steppers[0].get_name(), "stepper_x");
        assert_eq!(rail_x.steppers[0].rotation_distance, DEFAULT_ROTATION_DISTANCE);
        assert_eq!(rail_x.steppers[0].full_steps_per_rotation, DEFAULT_FULL_STEPS_PER_ROTATION);
        assert_eq!(rail_x.steppers[0].microsteps, DEFAULT_MICROSTEPS);
        assert_eq!(rail_x.steppers[0].gear_ratio, 1.0);
        let expected_steps_per_rotation = DEFAULT_FULL_STEPS_PER_ROTATION * DEFAULT_MICROSTEPS;
        assert_eq!(rail_x.steppers[0].steps_per_rotation, expected_steps_per_rotation);
        assert!((rail_x.steppers[0].step_distance - (DEFAULT_ROTATION_DISTANCE / expected_steps_per_rotation as f64)).abs() < 1e-9);
        assert_eq!(rail_x.steppers[0].position_steps, 0);
        assert_eq!(rail_x.steppers[0].mcu_position_offset, 0.0);
    }

    #[test]
    fn stepper_creation_calculates_step_distance() {
        let stepper1 = Stepper::new("s1".to_string(), 40.0, 200, 16, 1.0); // rot_dist, full_steps, microsteps, gear_ratio
        assert_eq!(stepper1.steps_per_rotation, 3200); // 200 * 16 * 1.0
        assert!((stepper1.step_distance - (40.0 / 3200.0)).abs() < 1e-9); // 0.0125

        let stepper2 = Stepper::new("s2".to_string(), 40.0, 200, 16, 2.0); // Double gear ratio
        assert_eq!(stepper2.steps_per_rotation, 6400); // 200 * 16 * 2.0
        assert!((stepper2.step_distance - (40.0 / 6400.0)).abs() < 1e-9); // 0.00625

        let stepper3 = Stepper::new("s3".to_string(), 8.0, 400, 32, 0.5); // 0.9deg motor, different gear
        assert_eq!(stepper3.steps_per_rotation, (400*32) as u32 / 2); // 400 * 32 * 0.5 = 6400
        assert!((stepper3.step_distance - (8.0 / ((400*32) as f64 * 0.5))).abs() < 1e-9);

        // Test clamping of steps_per_rotation to 1 if calculation is less
        let stepper4 = Stepper::new("s4_low_steps".to_string(), 40.0, 1, 1, 0.5); // effective 0.5 steps, clamped to 1
        assert_eq!(stepper4.steps_per_rotation, 1);
        assert_eq!(stepper4.step_distance, 40.0);
    }


    #[test]
    fn stepper_position_methods() {
        let mut stepper = Stepper::new("test_stepper".to_string(), DEFAULT_ROTATION_DISTANCE, DEFAULT_FULL_STEPS_PER_ROTATION, DEFAULT_MICROSTEPS, 1.0);
        assert!((stepper.step_distance - DEFAULT_STEP_DISTANCE).abs() < 1e-9);

        // Initial state
        assert_eq!(stepper.get_commanded_position_mm(), 0.0); // 0 * step_dist - 0 = 0

        // Simulate MCU reports stepper is at 1000 steps
        stepper.set_position_steps(1000);
        assert_eq!(stepper.get_commanded_position_mm(), 10.0); // 1000 * 0.01 - 0 = 10.0

        // Set G-code position to 5.0mm (G92 X5)
        // Current machine pos in mm is 10.0 (from steps). New G-code pos is 5.0.
        // Offset = 10.0 - 5.0 = 5.0
        stepper.set_commanded_position_mm(5.0);
        assert_eq!(stepper.mcu_position_offset, 5.0);
        assert_eq!(stepper.get_commanded_position_mm(), 5.0); // (1000 * 0.01) - 5.0 = 5.0

        // Simulate a move: G-code target 15.0mm
        // Target steps = (G-code_target + offset) / step_dist = (15.0 + 5.0) / 0.01 = 20.0 / 0.01 = 2000 steps
        let target_gcode_pos = 15.0;
        let target_steps = ((target_gcode_pos + stepper.mcu_position_offset) / stepper.step_distance).round() as i64;
        stepper.set_position_steps(target_steps);
        assert_eq!(stepper.position_steps, 2000);
        assert_eq!(stepper.get_commanded_position_mm(), 15.0); // (2000 * 0.01) - 5.0 = 15.0
    }

    #[test]
    fn rail_position_methods() {
        let mut rail = Rail::new(
            "x_rail".to_string(),
            vec![("stepper_x".to_string(), DEFAULT_STEP_DISTANCE)],
            0.0, 200.0, 0.0
        );
        // Initial G-code position should be 0
        assert_eq!(rail.get_rail_position_gcode().unwrap(), 0.0);

        // Simulate G92 X10
        rail.set_rail_position_gcode(10.0).unwrap();
        assert_eq!(rail.get_rail_position_gcode().unwrap(), 10.0);
        // Check internal stepper state
        let stepper = rail.primary_stepper().unwrap();
        assert_eq!(stepper.position_steps, 0); // Steps haven't changed yet
        assert_eq!(stepper.mcu_position_offset, -10.0); // Offset = (0 * step_dist) - 10.0

        // Simulate moving the rail to G-code position 50.0
        rail.move_steppers_to_gcode_coord(50.0).unwrap();
        assert_eq!(rail.get_rail_position_gcode().unwrap(), 50.0);
        let expected_steps = ((50.0 + stepper.mcu_position_offset) / stepper.step_distance).round() as i64;
        assert_eq!(rail.primary_stepper().unwrap().position_steps, expected_steps);
    }


    fn create_test_kinematics_with_step_dist() -> CartesianKinematics {
        let stepper_config = || vec![(
            "stepper".to_string(),
            DEFAULT_ROTATION_DISTANCE,
            DEFAULT_FULL_STEPS_PER_ROTATION,
            DEFAULT_MICROSTEPS,
            1.0 // gear_ratio
        )];
        CartesianKinematics::new(
            (stepper_config(), 0.0, 200.0, 0.0), // X rail
            (stepper_config(), 0.0, 200.0, 0.0), // Y rail
            (stepper_config(), 0.0, 180.0, 0.0), // Z rail
            25.0, // max_z_velocity
            500.0, // max_z_accel
        )
    }

    #[test]
    fn test_cartesian_kinematics_calc_position() {
        let mut kin = create_test_kinematics_with_step_dist();
        // Home all axes to G-code 0,0,0. This sets their stepper offsets.
        // Assume steppers are at 0 steps initially.
        // For X: offset = (0 * step_dist) - 0 = 0.
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);
        assert_eq!(kin.calc_position_from_steppers().unwrap(), [0.0, 0.0, 0.0]);

        // Simulate steppers moving
        // X moves to 10mm / step_dist steps
        let x_steps = (10.0 / DEFAULT_STEP_DISTANCE).round() as i64;
        kin.rails[0].primary_stepper_mut().unwrap().set_position_steps(x_steps);

        // Y moves to 20mm / step_dist steps
        let y_steps = (20.0 / DEFAULT_STEP_DISTANCE).round() as i64;
        kin.rails[1].primary_stepper_mut().unwrap().set_position_steps(y_steps);

        // Z moves to 5mm / step_dist steps
        let z_steps = (5.0 / DEFAULT_STEP_DISTANCE).round() as i64;
        kin.rails[2].primary_stepper_mut().unwrap().set_position_steps(z_steps);

        let calculated_pos = kin.calc_position_from_steppers().unwrap();
        assert!((calculated_pos[0] - 10.0).abs() < 1e-9);
        assert!((calculated_pos[1] - 20.0).abs() < 1e-9);
        assert!((calculated_pos[2] - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_set_kinematics_position_updates_stepper_offsets_and_limits() {
        let mut kin = create_test_kinematics_with_step_dist();

        // Simulate X stepper is physically at 1000 steps.
        // 1000 steps * 0.0125 mm/step = 12.5 mm (machine position)
        kin.rails[0].primary_stepper_mut().unwrap().set_position_steps(1000);

        // Home X, setting current G-code position to 0.0
        kin.set_kinematics_position(&[0.0, 0.0, 0.0], [true, false, false]);

        // Check X rail/stepper state
        let x_rail = &kin.rails[0];
        let x_stepper = x_rail.primary_stepper().unwrap();
        assert_eq!(x_stepper.position_steps, 1000); // Physical steps unchanged by G92/homing logic
        // Offset = (steps * step_dist) - gcode_pos = (1000 * 0.0125) - 0.0 = 12.5
        assert!((x_stepper.mcu_position_offset - 12.5).abs() < 1e-9);
        assert!((x_rail.get_rail_position_gcode().unwrap() - 0.0).abs() < 1e-9); // G-code pos is 0
        assert_eq!(kin.limits[0], (0.0, 200.0)); // X limits updated

        // Y and Z should still be unhomed
        assert_eq!(kin.limits[1], (1.0, -1.0));
        assert_eq!(kin.limits[2], (1.0, -1.0));
        assert_eq!(kin.rails[1].primary_stepper().unwrap().mcu_position_offset, 0.0); // Y offset still 0
    }


    fn create_test_kinematics() -> CartesianKinematics { // Old helper, ensure it's not used or update it
        let stepper_config = || vec![("stepper".to_string(), DEFAULT_STEP_DISTANCE)];
        CartesianKinematics::new(
            (stepper_config(), 0.0, 200.0, 0.0),
            (stepper_config(), 0.0, 200.0, 0.0),
            (stepper_config(), 0.0, 180.0, 0.0),
            25.0,
            500.0,
        )
    }


    #[test]
    fn test_check_move_valid() {
        let mut kin = create_test_kinematics_with_step_dist();
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);

        let mut move_valid = create_test_move([0.0,0.0,0.0,0.0], [10.0, 10.0, 10.0, 0.0], 100.0);
        assert!(kin.check_move(&mut move_valid).is_ok());
    }

    #[test]
    fn test_check_move_out_of_bounds_x_max() {
        let mut kin = create_test_kinematics_with_step_dist();
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);
        let mut move_invalid = create_test_move([0.0,0.0,0.0,0.0], [200.1, 10.0, 10.0, 0.0], 100.0);
        let result = kin.check_move(&mut move_invalid);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Move out of range: X 200.100"));
    }

    #[test]
    fn test_check_move_out_of_bounds_y_min() {
        let mut kin = create_test_kinematics_with_step_dist();
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);
        let mut move_invalid = create_test_move([0.0,0.0,0.0,0.0], [10.0, -0.1, 10.0, 0.0], 100.0);
        let result = kin.check_move(&mut move_invalid);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Move out of range: Y -0.100"));
    }

    #[test]
    fn test_check_move_out_of_bounds_z_max() {
        let mut kin = create_test_kinematics_with_step_dist();
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);
        let mut move_invalid = create_test_move([0.0,0.0,0.0,0.0], [10.0, 10.0, 180.1, 0.0], 100.0);
        let result = kin.check_move(&mut move_invalid);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Move out of range: Z 180.100"));
    }

    #[test]
    fn test_check_move_unhomed_axis() {
        let kin = create_test_kinematics_with_step_dist();
        let mut move_unhomed = create_test_move([0.0,0.0,0.0,0.0], [10.0, 10.0, 10.0, 0.0], 100.0);
        let result = kin.check_move(&mut move_unhomed);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("X axis must be homed first"));
    }

    #[test]
    fn test_check_move_z_limits_speed_accel() {
        let mut kin = create_test_kinematics_with_step_dist();
        kin.set_kinematics_position(&[0.0,0.0,0.0], [true,true,true]);

        let mut move_z = create_test_move([0.0,0.0,0.0,0.0], [0.0, 0.0, 10.0, 0.0], 100.0);
        let original_max_cruise_v2 = move_z.max_cruise_v2;
        let original_accel = move_z.accel;

        kin.check_move(&mut move_z).unwrap();

        let expected_limited_speed = kin.max_z_velocity;
        let expected_limited_accel = kin.max_z_accel;

        assert_ne!(move_z.max_cruise_v2, original_max_cruise_v2, "max_cruise_v2 should have been limited for Z move");
        assert_ne!(move_z.accel, original_accel, "accel should have been limited for Z move");
        assert_eq!(move_z.max_cruise_v2, expected_limited_speed.powi(2));
        assert_eq!(move_z.accel, expected_limited_accel);
    }

     #[test]
    fn test_set_kinematics_position_updates_limits_and_stepper_offsets() { // Renamed for clarity
        let mut kin = create_test_kinematics_with_step_dist();

        // Simulate X stepper is physically at 1000 steps.
        // 1000 steps * 0.0125 mm/step = 12.5 mm (machine position)
        kin.rails[0].primary_stepper_mut().unwrap().set_position_steps(1000);

        // Home X, setting current G-code position to 0.0
        kin.set_kinematics_position(&[0.0, 0.0, 0.0], [true, false, false]);

        // Check X rail/stepper state
        assert_eq!(kin.get_axis_limits_for_test()[0], (0.0, 200.0)); // X limits updated
        let x_stepper = kin.rails[0].primary_stepper().unwrap();
        assert_eq!(x_stepper.position_steps, 1000); // Physical steps unchanged
        // Offset = (steps * step_dist) - gcode_pos = (1000 * 0.0125) - 0.0 = 12.5
        assert!((x_stepper.mcu_position_offset - 12.5).abs() < 1e-9);
        assert!((kin.rails[0].get_rail_position_gcode().unwrap() - 0.0).abs() < 1e-9); // G-code pos is 0

        // Y and Z should still be unhomed
        assert_eq!(kin.get_axis_limits_for_test()[1], (1.0, -1.0));
        assert_eq!(kin.get_axis_limits_for_test()[2], (1.0, -1.0));
    }
}
