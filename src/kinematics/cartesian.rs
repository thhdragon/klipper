// src/kinematics/cartesian.rs
#![cfg_attr(not(test), no_std)]

use defmt::Format;

/// Represents the kinematics for a standard Cartesian robot.
#[derive(Debug, Format, Copy, Clone)]
pub struct CartesianKinematics {
    steps_per_mm_x: f32,
    steps_per_mm_y: f32,
    // steps_per_mm_z: f32, // For later
}

impl CartesianKinematics {
    /// Creates a new CartesianKinematics instance with the given configuration.
    pub fn new(steps_per_mm_x: f32, steps_per_mm_y: f32) -> Self {
        Self {
            steps_per_mm_x,
            steps_per_mm_y,
        }
    }

    /// Calculates the number of steps required for each axis to move to a target position.
    ///
    /// # Arguments
    /// * `target_pos`: A tuple `(x_mm, y_mm)` representing the target position.
    /// * `current_pos`: A tuple `(x_mm, y_mm)` representing the current position.
    ///
    /// # Returns
    /// A tuple `(x_steps, y_steps)` representing the absolute number of steps for each axis.
    /// The direction of movement is not handled here; it must be determined by comparing
    /// target_pos and current_pos.
    pub fn move_to(&self, target_pos: (f32, f32), current_pos: (f32, f32)) -> (u32, u32) {
        let delta_x_mm = (target_pos.0 - current_pos.0).abs();
        let delta_y_mm = (target_pos.1 - current_pos.1).abs();

        let x_steps = (delta_x_mm * self.steps_per_mm_x).round() as u32;
        let y_steps = (delta_y_mm * self.steps_per_mm_y).round() as u32;

        (x_steps, y_steps)
    }
}
