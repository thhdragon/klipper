// src/stepper.rs
#![cfg_attr(not(test), no_std)]

use defmt::Format;

/// Represents the direction of stepper motor rotation.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum StepperDirection {
    Clockwise,
    CounterClockwise,
}

impl Default for StepperDirection {
    fn default() -> Self {
        StepperDirection::Clockwise
    }
}

/// Represents a single stepper motor axis.
#[derive(Debug, Format)]
pub struct Stepper {
    /// A unique identifier for this stepper instance.
    pub(crate) id: u32,

    /// The GPIO pin number for the STEP signal.
    pub(crate) step_pin_id: u8,

    /// The GPIO pin number for the DIR signal.
    pub(crate) dir_pin_id: u8,

    /// The current configured direction of the motor.
    pub(crate) current_direction: StepperDirection,

    /// The ID used to register this stepper's tasks with the scheduler.
    /// This should be unique among all scheduler tasks. It can be derived from `id`.
    pub(crate) timer_id_for_scheduler: u32,

    /// Number of steps remaining to be executed in the current move.
    /// When this reaches 0, the current move sequence is complete.
    pub(crate) steps_to_move: u32,

    /// Duration of the high part of the step pulse, in raw timer ticks.
    /// (e.g., if timer is 1MHz, 2 ticks = 2 microseconds).
    pub(crate) pulse_duration_ticks: u32,

    /// The absolute time (in raw timer ticks) for the next event
    /// (either step pulse rising edge or falling edge).
    pub(crate) next_step_waketime: u32,

    /// Tracks the state of the step pulse generation.
    /// `true` if the pulse is currently in its HIGH phase (waiting to go LOW).
    /// `false` if the pulse is currently in its LOW phase (waiting for the next pulse to start, or move is done).
    pub(crate) is_pulsing_high: bool,

    /// Period between the start of one step pulse and the start of the next, in timer ticks.
    /// This determines the step rate (frequency).
    pub(crate) step_period_ticks: u32,
}

use crate::gpio_manager::GpioManager; // For GpioManager interaction
use crate::hal::StepEventResult; // For callback return types, though not used by these methods directly yet

impl Stepper {
    /// Creates a new Stepper instance.
    ///
    /// `id`: A unique ID for this stepper.
    /// `timer_id_for_scheduler`: The ID this stepper will use for its scheduler tasks.
    /// `step_pin_id`, `dir_pin_id`: GPIO pin numbers.
    /// `gpio_manager`: A mutable reference to configure the pins.
    ///
    /// Note: This constructor configures pins as outputs. It might be better
    /// if pin configuration is handled more explicitly by the caller or a setup command.
    pub fn new(
        id: u32,
        timer_id_for_scheduler: u32,
        step_pin_id: u8,
        dir_pin_id: u8,
        gpio_manager: &mut GpioManager,
    ) -> Result<Self, &'static str> {
        // Configure step and dir pins as outputs
        gpio_manager.configure_pin_as_output(step_pin_id)?;
        gpio_manager.write_pin_output(step_pin_id, false)?; // Start with step pin low

        gpio_manager.configure_pin_as_output(dir_pin_id)?;
        // Default direction will be set by an explicit call to set_direction or a command.
        // For now, let's set a default and write it.
        let initial_direction = StepperDirection::default();
        let dir_pin_state = match initial_direction {
            StepperDirection::Clockwise => false, // Assuming LOW for Clockwise, this is arbitrary
            StepperDirection::CounterClockwise => true,
        };
        gpio_manager.write_pin_output(dir_pin_id, dir_pin_state)?;

        defmt::info!(
            "Stepper {} initialized: Step Pin {}, Dir Pin {}. Initial Dir: {:?}",
            id, step_pin_id, dir_pin_id, initial_direction
        );

        Ok(Self {
            id,
            step_pin_id,
            dir_pin_id,
            current_direction: initial_direction,
            timer_id_for_scheduler,
            steps_to_move: 0,
            pulse_duration_ticks: 2, // Default 2 ticks (e.g., 2us at 1MHz)
            next_step_waketime: 0,
            is_pulsing_high: false,
            step_period_ticks: 2000, // Default to 2000 ticks period (e.g., 500 Hz at 1MHz)
        })
    }

    /// Sets the direction of the stepper motor.
    pub fn set_direction(
        &mut self,
        direction: StepperDirection,
        gpio_manager: &mut GpioManager,
    ) -> Result<(), &'static str> {
        if self.current_direction == direction && !self.is_pulsing_high { // Optimization: only change if different and not mid-pulse
             // If mid-pulse, changing direction could be problematic. Klipper usually sets dir before stepping.
            // For now, allow changing anytime but it's better to set dir when not actively pulsing.
            // return Ok(());
        }

        let dir_pin_state = match direction {
            StepperDirection::Clockwise => false, // Assuming LOW for Clockwise
            StepperDirection::CounterClockwise => true,
        };

        match gpio_manager.write_pin_output(self.dir_pin_id, dir_pin_state) {
            Ok(()) => {
                self.current_direction = direction;
                // defmt::trace!("Stepper {}: Direction set to {:?}", self.id, direction);
                Ok(())
            }
            Err(e) => {
                defmt::error!("Stepper {}: Failed to set direction pin {}: {}", self.id, self.dir_pin_id, e);
                Err(e)
            }
        }
    }

    /// Starts the high phase of a step pulse.
    pub(crate) fn issue_step_pulse_start(
        &mut self,
        gpio_manager: &mut GpioManager,
    ) -> Result<(), &'static str> {
        // defmt::trace!("Stepper {}: Pulse START on pin {}", self.id, self.step_pin_id);
        gpio_manager.write_pin_output(self.step_pin_id, true)
    }

    /// Ends the step pulse by setting the step pin low.
    pub(crate) fn issue_step_pulse_end(
        &mut self,
        gpio_manager: &mut GpioManager,
    ) -> Result<(), &'static str> {
        // defmt::trace!("Stepper {}: Pulse END on pin {}", self.id, self.step_pin_id);
        gpio_manager.write_pin_output(self.step_pin_id, false)
    }

    /// The core stepper event handler, called by the scheduler.
    /// This generates one step pulse (rising and falling edge) over two scheduler events.
    pub fn step_event_callback<MasterAlarm: rp2040_hal::timer::Alarm>(
        &mut self,
        scheduler: &mut crate::sched::SchedulerState<MasterAlarm>,
        gpio_manager: &mut GpioManager,
    ) {
        if self.is_pulsing_high {
            // --- End of pulse ---
            // Set step pin low
            if self.issue_step_pulse_end(gpio_manager).is_err() {
                defmt::error!("Stepper {}: Failed to end step pulse!", self.id);
                // Abort move on error
                self.steps_to_move = 0;
                return;
            }
            self.is_pulsing_high = false;
            self.steps_to_move -= 1;

            if self.steps_to_move > 0 {
                // Schedule the start of the next pulse
                let time_between_pulses = self.step_period_ticks.saturating_sub(self.pulse_duration_ticks);
                self.next_step_waketime = self.next_step_waketime.wrapping_add(time_between_pulses);
                scheduler.schedule_task(self.timer_id_for_scheduler, self.next_step_waketime);
            } else {
                // Move complete
                defmt::info!("Stepper {}: Move complete.", self.id);
            }
        } else {
            // --- Start of pulse ---
            if self.steps_to_move > 0 {
                // Set step pin high
                if self.issue_step_pulse_start(gpio_manager).is_err() {
                    defmt::error!("Stepper {}: Failed to start step pulse!", self.id);
                    self.steps_to_move = 0; // Abort
                    return;
                }
                self.is_pulsing_high = true;

                // Schedule the end of this pulse
                self.next_step_waketime = self.next_step_waketime.wrapping_add(self.pulse_duration_ticks);
                scheduler.schedule_task(self.timer_id_for_scheduler, self.next_step_waketime);
            }
        }
    }
}
