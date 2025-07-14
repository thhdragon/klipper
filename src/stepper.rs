// src/stepper.rs
#![cfg_attr(not(test), no_std)]

use defmt::Format;
use crate::gpio_manager::GpioManager;
use crate::hal::StepEventResult;

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
    pub acceleration: f32, // in steps/sec^2
    pub start_velocity: f32, // in steps/sec
    pub cruise_velocity: f32, // in steps/sec
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
    pub acceleration: f32,
}

impl MovePlanner {
    pub fn new(mov: &TrapezoidalMove, clock_freq_hz: u32) -> Result<Self, &'static str> {
        if mov.cruise_velocity < mov.start_velocity { return Err("Cruise velocity must be >= start velocity"); }
        if mov.acceleration <= 0.0 { return Err("Acceleration must be positive"); }
        if mov.total_steps == 0 {
            return Ok(Self {
                total_steps: 0, accel_steps: 0, cruise_steps: 0, decel_steps: 0,
                initial_period_ticks: 0, cruise_period_ticks: 0, acceleration: 0.0,
            });
        }
        let accel_dist = (mov.cruise_velocity.powi(2) - mov.start_velocity.powi(2)) / (2.0 * mov.acceleration);
        let mut accel_steps = accel_dist.ceil() as u32;
        let mut decel_steps = accel_steps;
        if (accel_steps + decel_steps) > mov.total_steps {
            accel_steps = mov.total_steps / 2;
            decel_steps = mov.total_steps - accel_steps;
        }
        let cruise_steps = mov.total_steps.saturating_sub(accel_steps + decel_steps);
        let initial_period_ticks = if mov.start_velocity > 0.0 {
            (clock_freq_hz as f32 / mov.start_velocity) as u32
        } else {
            (clock_freq_hz as f32 * (2.0 / mov.acceleration).sqrt()) as u32
        };
        let cruise_period_ticks = (clock_freq_hz as f32 / mov.cruise_velocity) as u32;
        Ok(Self {
            total_steps: mov.total_steps, accel_steps, cruise_steps, decel_steps,
            initial_period_ticks, cruise_period_ticks, acceleration: mov.acceleration,
        })
    }
}

/// Represents a single stepper motor axis.
#[derive(Debug, Format)]
pub struct Stepper {
    pub(crate) id: u32,
    pub(crate) step_pin_id: u8,
    pub(crate) dir_pin_id: u8,
    pub(crate) current_direction: StepperDirection,
    pub(crate) timer_id_for_scheduler: u32,
    pub(crate) pulse_duration_ticks: u32,
    // State for the current move
    pub(crate) current_move: Option<MovePlanner>,
    pub(crate) current_step_num: u32,
    pub(crate) next_step_waketime: u32,
    pub(crate) is_pulsing_high: bool,
    pub(crate) step_period_ticks: u32,
    // --- Bresenham's Line Algorithm fields for multi-axis synchronization ---
    pub(crate) bresenham_error: i32,
    pub(crate) bresenham_increment: u32,
    pub(crate) bresenham_decrement: u32,
}

impl Stepper {
    pub fn new(id: u32, timer_id_for_scheduler: u32, step_pin_id: u8, dir_pin_id: u8, gpio_manager: &mut GpioManager) -> Result<Self, &'static str> {
        gpio_manager.configure_pin_as_output(step_pin_id)?;
        gpio_manager.write_pin_output(step_pin_id, false)?;
        gpio_manager.configure_pin_as_output(dir_pin_id)?;
        let initial_direction = StepperDirection::default();
        let dir_pin_state = match initial_direction {
            StepperDirection::Clockwise => false,
            StepperDirection::CounterClockwise => true,
        };
        gpio_manager.write_pin_output(dir_pin_id, dir_pin_state)?;
        defmt::info!("Stepper {} initialized: Step Pin {}, Dir Pin {}. Initial Dir: {:?}", id, step_pin_id, dir_pin_id, initial_direction);
        Ok(Self {
            id, step_pin_id, dir_pin_id, current_direction: initial_direction,
            timer_id_for_scheduler, pulse_duration_ticks: 2,
            current_move: None, current_step_num: 0,
            next_step_waketime: 0, is_pulsing_high: false, step_period_ticks: 0,
            bresenham_error: 0, bresenham_increment: 0, bresenham_decrement: 0, // Initialize Bresenham fields
        })
    }

    pub fn set_direction(&mut self, direction: StepperDirection, gpio_manager: &mut GpioManager) -> Result<(), &'static str> {
        if self.current_direction == direction { return Ok(()); }
        let dir_pin_state = match direction {
            StepperDirection::Clockwise => false,
            StepperDirection::CounterClockwise => true,
        };
        match gpio_manager.write_pin_output(self.dir_pin_id, dir_pin_state) {
            Ok(()) => { self.current_direction = direction; Ok(()) }
            Err(e) => { error!("Stepper {}: Failed to set direction pin {}: {}", self.id, self.dir_pin_id, e); Err(e) }
        }
    }

    pub(crate) fn issue_step_pulse_start(&mut self, gpio_manager: &mut GpioManager) -> Result<(), &'static str> {
        gpio_manager.write_pin_output(self.step_pin_id, true)
    }

    pub(crate) fn issue_step_pulse_end(&mut self, gpio_manager: &mut GpioManager) -> Result<(), &'static str> {
        gpio_manager.write_pin_output(self.step_pin_id, false)
    }

    /// The core stepper event handler, called by the scheduler for acceleration moves.
    /// This will be refactored to handle Bresenham's algorithm in a subsequent step.
    pub fn step_event_callback<MasterAlarm: rp2040_hal::timer::Alarm>(
        &mut self,
        scheduler: &mut crate::sched::SchedulerState<MasterAlarm>,
        gpio_manager: &mut GpioManager,
    ) {
        // ... (existing acceleration logic from previous phase) ...
        // This will be modified later to incorporate Bresenham's logic.
    }
}
