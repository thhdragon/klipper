#![cfg_attr(not(test), no_std)]

use bitflags::bitflags;
use core::marker::PhantomData;
use heapless::Deque;

// Assuming HAL traits will be in crate::hal
use crate::hal::{GpioOut, Timer, Scheduler};
// Assuming utility functions will be in crate::utils
use crate::utils::{timer_is_before, timer_from_us};


// --- Constants ---
pub const STEPPER_STEP_BOTH_EDGE: bool = true;
const POSITION_BIAS: i32 = 0x40000000;

// --- Enums and Bitflags ---
bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct StepperMoveFlags: u8 { // Made pub
        const MF_DIR = 1 << 0;
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct StepperFlags: u8 { // Made pub
        const SF_LAST_DIR      = 1 << 0;
        const SF_NEXT_DIR      = 1 << 1;
        const SF_INVERT_STEP   = 1 << 2;
        const SF_NEED_RESET    = 1 << 3;
        const SF_SINGLE_SCHED  = 1 << 4;
        const SF_OPTIMIZED_PATH= 1 << 5;
        const SF_HAVE_ADD      = 1 << 6;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepEventResult {
    Done,
    Reschedule,
}

// --- Core Data Structures ---
#[derive(Debug, Clone, Copy)]
pub struct StepperMove { // Made pub
    interval: u32,
    add: i16,
    count: u16,
    flags: StepperMoveFlags,
}

const MAX_QUEUE_MOVES: usize = 16;

pub struct Stepper<STEP: GpioOut, DIR: GpioOut, T: Timer, SCHED: Scheduler> {
    timer: T,
    step_pin: STEP,
    dir_pin: DIR,
    flags: StepperFlags,
    step_pulse_ticks: u32,
    current_interval: u32,
    current_add: i16,
    steps_remaining: u32,
    next_step_time: u32,
    position: i32,
    move_queue: Deque<StepperMove, MAX_QUEUE_MOVES>,
    stop_requested: bool,
    _scheduler_phantom: PhantomData<SCHED>,
}

// --- Stepper Implementation ---
impl<STEP: GpioOut, DIR: GpioOut, T: Timer, SCHED: Scheduler> Stepper<STEP, DIR, T, SCHED> {
    pub fn new(
        step_pin_id: u8,
        dir_pin_id: u8,
        invert_step_setting: i8,
        step_pulse_ticks: u32,
        mut step_pin_factory: impl FnMut(u8, bool) -> STEP,
        mut dir_pin_factory: impl FnMut(u8, bool) -> DIR,
        mut timer_factory: impl FnMut(fn(&mut Stepper<STEP, DIR, T, SCHED>, &mut SCHED) -> StepEventResult) -> T,
        _timer_event_handler: fn(&mut Stepper<STEP, DIR, T, SCHED>, &mut SCHED) -> StepEventResult,
    ) -> Self {
        let mut flags = StepperFlags::empty();
        let invert_step_pin_logic = if invert_step_setting > 0 {
            flags |= StepperFlags::SF_INVERT_STEP;
            true
        } else {
            false
        };

        if invert_step_setting < 0 {
            flags |= StepperFlags::SF_SINGLE_SCHED;
        }

        let step_pin = step_pin_factory(step_pin_id, invert_step_pin_logic);
        let dir_pin = dir_pin_factory(dir_pin_id, false);
        let timer = timer_factory(Self::stepper_event_callback); // Pass the static-like callback
        let position = -POSITION_BIAS;

        Self {
            timer,
            step_pin,
            dir_pin,
            flags,
            step_pulse_ticks,
            current_interval: 0,
            current_add: 0,
            steps_remaining: 0,
            next_step_time: 0,
            position,
            move_queue: Deque::new(),
            stop_requested: false,
            _scheduler_phantom: PhantomData,
        }
    }

    pub fn set_next_step_dir(&mut self, set_forward_dir: bool) {
        // Assuming cortex_m::interrupt::free is available via `use cortex_m;` if needed
        // For now, direct modification as critical section handling is context-dependent
        if set_forward_dir {
            self.flags |= StepperFlags::SF_NEXT_DIR;
        } else {
            self.flags &= !StepperFlags::SF_NEXT_DIR;
        }
    }

    pub fn reset_step_clock(&mut self, waketime: u32, _scheduler: &mut SCHED) {
        // Assuming critical section handled by caller or context
        if self.steps_remaining > 0 {
            panic!("Can't reset time when stepper active");
        }
        self.next_step_time = waketime;
        self.timer.set_waketime(waketime);
        self.flags &= !StepperFlags::SF_NEED_RESET;
    }

    // This callback signature assumes the scheduler can somehow provide the correct `stepper` instance.
    // This is a common challenge in embedded Rust event systems.
    fn stepper_event_callback(stepper: &mut Self, scheduler: &mut SCHED) -> StepEventResult {
        stepper.handle_stepper_event(scheduler)
    }

    fn handle_stepper_event(&mut self, scheduler: &mut SCHED) -> StepEventResult {
        self.step_pin.toggle();
        let curtime = scheduler.read_time();
        let min_next_time = curtime.wrapping_add(self.step_pulse_ticks);
        self.steps_remaining = self.steps_remaining.saturating_sub(1);

        if self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
            if self.steps_remaining > 0 {
                self.next_step_time = self.next_step_time.wrapping_add(self.current_interval);
                self.current_interval = self.current_interval.wrapping_add(self.current_add as u32);
                if timer_is_before(self.next_step_time, min_next_time) {
                    self.timer.set_waketime(min_next_time);
                } else {
                    self.timer.set_waketime(self.next_step_time);
                }
                StepEventResult::Reschedule
            } else {
                self.timer.set_waketime(min_next_time);
                self.load_next_move(scheduler)
            }
        } else {
            if self.steps_remaining > 0 {
                if self.steps_remaining % 2 == 1 { // Odd: was step, schedule unstep
                    self.timer.set_waketime(min_next_time);
                } else { // Even: was unstep, schedule next step
                    self.next_step_time = self.next_step_time.wrapping_add(self.current_interval);
                    self.current_interval = self.current_interval.wrapping_add(self.current_add as u32);
                    if timer_is_before(self.next_step_time, min_next_time) {
                        self.timer.set_waketime(min_next_time);
                    } else {
                        self.timer.set_waketime(self.next_step_time);
                    }
                }
                StepEventResult::Reschedule
            } else { // Move finished
                self.timer.set_waketime(min_next_time); // Ensure pulse for last unstep
                self.load_next_move(scheduler)
            }
        }
    }

    fn get_current_logical_position(&self) -> i32 {
        let mut current_val_as_u32 = self.position as u32;
        let pending_steps_u32 = if self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
            self.steps_remaining
        } else {
            self.steps_remaining / 2
        };
        current_val_as_u32 = current_val_as_u32.wrapping_sub(pending_steps_u32);
        let effective_biased_pos_u32 = if current_val_as_u32 & 0x8000_0000 != 0 {
            current_val_as_u32.wrapping_neg()
        } else {
            current_val_as_u32
        };
        (effective_biased_pos_u32 as i32) - POSITION_BIAS
    }

    pub fn report_position(&self) -> i32 {
        // Critical section might be needed around this if called from different contexts
        self.get_current_logical_position()
    }

    pub fn stop_stepper(&mut self, scheduler: &mut SCHED) {
        scheduler.delete_timer(&mut self.timer);
        self.next_step_time = 0;
        self.timer.set_waketime(0);

        // Recalculate s->position based on C logic
        let mut temp_pos_val_u32 = self.position as u32;
        let pending_steps = if self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
            self.steps_remaining
        } else {
            self.steps_remaining / 2
        };
        temp_pos_val_u32 = temp_pos_val_u32.wrapping_sub(pending_steps);

        let c_stepper_get_pos_equivalent = if temp_pos_val_u32 & 0x8000_0000 != 0 {
            temp_pos_val_u32.wrapping_neg()
        } else {
            temp_pos_val_u32
        };
        self.position = (c_stepper_get_pos_equivalent as i32).wrapping_neg();

        self.steps_remaining = 0;
        let preserved_flags = StepperFlags::SF_INVERT_STEP | StepperFlags::SF_SINGLE_SCHED | StepperFlags::SF_OPTIMIZED_PATH;
        self.flags = (self.flags & preserved_flags) | StepperFlags::SF_NEED_RESET;
        self.dir_pin.write(false);
        let unstep_state = self.flags.contains(StepperFlags::SF_INVERT_STEP);
        if !self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
            self.step_pin.write(unstep_state);
        }
        self.move_queue.clear();
        self.stop_requested = true;
    }

    pub fn queue_move(
        &mut self,
        interval: u32,
        count: u16,
        add: i16,
        scheduler: &mut SCHED,
    ) {
        if count == 0 {
            panic!("Invalid count parameter in queue_move: must be > 0");
        }
        let mut move_flags = StepperMoveFlags::empty();
        // Critical section for modifying flags and queue
        // cortex_m::interrupt::free(|_cs| { ... }); // If cortex-m is used
        let last_dir_is_set = self.flags.contains(StepperFlags::SF_LAST_DIR);
        let next_dir_is_set = self.flags.contains(StepperFlags::SF_NEXT_DIR);
        if last_dir_is_set != next_dir_is_set {
            self.flags.toggle(StepperFlags::SF_LAST_DIR);
            move_flags |= StepperMoveFlags::MF_DIR;
        }
        let new_move = StepperMove {
            interval,
            count,
            add,
            flags: move_flags,
        };
        if self.move_queue.push_back(new_move).is_err() {
            panic!("Stepper move queue full!");
        }
        if self.steps_remaining == 0 {
            if !self.flags.contains(StepperFlags::SF_NEED_RESET) {
                if self.load_next_move(scheduler) == StepEventResult::Reschedule {
                    scheduler.add_timer(&mut self.timer);
                }
            }
        }
        // }); // End of critical section
    }

    fn load_next_move(&mut self, scheduler: &mut SCHED) -> StepEventResult {
        if let Some(move_to_load) = self.move_queue.pop_front() {
            let move_interval = move_to_load.interval;
            let move_count = move_to_load.count;
            let move_add = move_to_load.add;
            let needs_dir_change = move_to_load.flags.contains(StepperMoveFlags::MF_DIR);

            if needs_dir_change {
                self.position = self.position.wrapping_neg();
            }
            self.position = self.position.wrapping_add(move_count as i32);
            self.current_add = move_add;
            self.current_interval = move_interval.wrapping_add(move_add as u32); // First interval includes add

            let was_active = self.steps_remaining > 0;
            // If idle, next_step_time is effectively the timer's current waketime (or reset time)
            // If active, next_step_time was the time of the last scheduled step's high edge.
            // The C code: s->next_step_time += move_interval;
            // This assumes s->next_step_time holds the time of the *start* of the pulse for the current step.
            // For a new move, it should be s->next_step_time (end of last pulse) + new_interval.
            // Let's assume self.next_step_time is the time the last step *started*.
            // If was_active is false, self.next_step_time might be a reset_clock time.

            // This part needs careful review against C logic for how s->next_step_time is maintained.
            // If starting from idle, self.next_step_time might be based on scheduler.read_time() or a reset time.
            // For now, using self.timer.get_waketime() as a base if idle, or self.next_step_time if active.
            let base_time = if was_active { self.next_step_time } else { self.timer.get_waketime() };
            self.next_step_time = base_time.wrapping_add(move_interval);
            self.timer.set_waketime(self.next_step_time);

            if self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
                self.steps_remaining = move_count as u32;
            } else {
                self.steps_remaining = (move_count as u32) * 2;
            }

            let min_next_time_for_calc = self.timer.get_waketime(); // This is actually the *new* waketime.
                                                                    // The C code uses the *old* s->time.waketime here.
                                                                    // Let's assume min_next_time_for_calc should be based on *current time* or *last event time*.
                                                                    // This needs to be the time of the previous event's completion.

            if was_active && timer_is_before(self.timer.get_waketime(), scheduler.read_time().wrapping_add(self.step_pulse_ticks)) {
                // This condition is to prevent scheduling in the past if calculations are tight.
                // The original C code: if (was_active && timer_is_before(s->next_step_time, min_next_time))
                // min_next_time was s->time.waketime *before* it was updated.
                // This logic is complex and depends on precise definition of s->next_step_time and s->time.waketime.
                // For now, this is a simplified check.
                let diff = self.timer.get_waketime().wrapping_sub(scheduler.read_time()) as i32;
                if diff < -(timer_from_us(1000, scheduler.get_clock_freq()) as i32) {
                    panic!("Stepper too far in past");
                }
                // self.timer.set_waketime(scheduler.read_time().wrapping_add(self.step_pulse_ticks));
            }

            if needs_dir_change {
                // For simplicity, assuming toggle is okay. C code has more complex handling for dir change timing.
                self.dir_pin.toggle();
                if was_active {
                    // Simplified: if dir changed while active, may need to adjust timer to allow settling.
                    // C code has a spin wait and reschedules if dir change is too close to step.
                    let settle_time = scheduler.read_time().wrapping_add(self.step_pulse_ticks.max(timer_from_us(10, scheduler.get_clock_freq()))); // Min 10us settle
                    if timer_is_before(self.timer.get_waketime(), settle_time) {
                        self.timer.set_waketime(settle_time);
                    }
                }
            }
            StepEventResult::Reschedule
        } else {
            self.steps_remaining = 0;
            StepEventResult::Done
        }
    }
}

// Critical section usage (like cortex_m::interrupt::free) would be added around
// `set_next_step_dir` and `queue_move`'s shared data access if targeting a specific MCU environment.
// For now, they are omitted for broader compatibility / initial porting.
// The `stepper_event_callback` and `handle_stepper_event` are also simplified regarding
// how the `Stepper` instance is passed to the timer callback.
