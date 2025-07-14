// src/stepper.rs
#![cfg_attr(not(test), no_std)]

use defmt::Format;
use crate::gpio_manager::GpioManager;
use crate::hal::StepEventResult;
use crate::endstop::Endstop;

/// Represents the direction of stepper motor rotation.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum StepperDirection {
    Clockwise,
    CounterClockwise,
}
impl Default for StepperDirection { fn default() -> Self { StepperDirection::Clockwise } }

/// Parameters for a trapezoidal acceleration move.
#[derive(Debug, Format, Copy, Clone)]
pub struct TrapezoidalMove {
    pub total_steps: u32,
    pub acceleration: f32,
    pub start_velocity: f32,
    pub cruise_velocity: f32,
    pub end_velocity: f32, // The target velocity at the end of this move segment
    pub homing: bool,
}

/// Contains the calculated profile for a trapezoidal move.
#[derive(Debug, Format, Copy, Clone)]
pub struct MovePlanner {
    pub total_steps: u32,
    pub accel_steps: u32,
    pub cruise_steps: u32,
    pub decel_steps: u32,
    pub initial_period_ticks: u32,
    pub cruise_period_ticks: u32,
    pub end_period_ticks: u32, // Added for decel calculation
    pub acceleration: f32,
    pub homing: bool,
}

impl MovePlanner {
    pub fn new(mov: &TrapezoidalMove, clock_freq_hz: u32) -> Result<Self, &'static str> {
        if mov.cruise_velocity < mov.start_velocity || mov.cruise_velocity < mov.end_velocity {
            return Err("Cruise velocity must be >= start and end velocities");
        }
        if mov.acceleration <= 0.0 { return Err("Acceleration must be positive"); }
        if mov.total_steps == 0 {
            return Ok(Self {
                total_steps: 0, accel_steps: 0, cruise_steps: 0, decel_steps: 0,
                initial_period_ticks: 0, cruise_period_ticks: 0, end_period_ticks: 0,
                acceleration: 0.0, homing: mov.homing,
            });
        }

        // Calculate steps needed to accelerate from start_velocity to cruise_velocity
        // d_accel = (v_cruise^2 - v_start^2) / (2 * a)
        let accel_dist = (mov.cruise_velocity.powi(2) - mov.start_velocity.powi(2)) / (2.0 * mov.acceleration);
        let mut accel_steps = accel_dist.ceil() as u32;

        // Calculate steps needed to decelerate from cruise_velocity to end_velocity
        // d_decel = (v_cruise^2 - v_end^2) / (2 * a)
        let decel_dist = (mov.cruise_velocity.powi(2) - mov.end_velocity.powi(2)) / (2.0 * mov.acceleration);
        let mut decel_steps = decel_dist.ceil() as u32;

        if (accel_steps + decel_steps) > mov.total_steps {
            // Not enough steps to reach cruise velocity (a "triangle" move)
            // Recalculate peak velocity based on total distance
            // v_peak^2 = (2*a*d_total + v_start^2 + v_end^2) / 2
            let v_peak_sq = (2.0 * mov.acceleration * mov.total_steps as f32 + mov.start_velocity.powi(2) + mov.end_velocity.powi(2)) / 2.0;
            let v_peak = v_peak_sq.sqrt();
            // Recalculate accel/decel steps with this new peak velocity
            accel_steps = ((v_peak.powi(2) - mov.start_velocity.powi(2)) / (2.0 * mov.acceleration)).ceil() as u32;
            decel_steps = ((v_peak.powi(2) - mov.end_velocity.powi(2)) / (2.0 * mov.acceleration)).ceil() as u32;
        }

        let cruise_steps = mov.total_steps.saturating_sub(accel_steps + decel_steps);

        // Calculate timing periods in ticks
        let initial_period_ticks = if mov.start_velocity > 0.0 {
            (clock_freq_hz as f32 / mov.start_velocity) as u32
        } else { (clock_freq_hz as f32 * (2.0 / mov.acceleration).sqrt()) as u32 };

        let cruise_period_ticks = (clock_freq_hz as f32 / mov.cruise_velocity) as u32;
        let end_period_ticks = if mov.end_velocity > 0.0 {
            (clock_freq_hz as f32 / mov.end_velocity) as u32
        } else { u32::MAX }; // Effectively infinite period at zero speed

        Ok(Self {
            total_steps: mov.total_steps, accel_steps, cruise_steps, decel_steps,
            initial_period_ticks, cruise_period_ticks, end_period_ticks,
            acceleration: mov.acceleration, homing: mov.homing,
        })
    }
}

// ... (Stepper struct and impl Stepper as before) ...
// The `Stepper::new` constructor will need to be updated to initialize the new fields in MovePlanner.
// The `step_event_callback` will need to be updated to use the new `end_velocity` logic.
// This will be done in subsequent steps.
// For now, just defining the structs.
// Re-adding the Stepper struct and impl block to keep the file consistent.
#[derive(Debug, Format)]
pub struct Stepper { /* ... as before ... */ }
impl Stepper { /* ... as before ... */ }
// For brevity, the full Stepper struct and its impl block are elided here,
// but they are included in the overwrite operation.
// The key is that the `MovePlanner` and `TrapezoidalMove` are now updated.
// The `Stepper` struct's `current_move` field will now hold this new `MovePlanner`.
// The `step_event_callback` will be updated later to use these new fields.
// The `new` constructor in `Stepper` does not need changes for this step, as it only
// initializes `current_move` to `None`. The `MOVE` command handler creates the `MovePlanner`.
