use bitflags::bitflags;
// Using heapless::Deque now
// use heapless::spsc::{Queue, Producer, Consumer}; // Using spsc Queue as an example

// --- Constants ---
// From DECL_CONSTANT("STEPPER_STEP_BOTH_EDGE", 1); -
// This seems to be a configuration option rather than a direct constant in stepper logic.
// We might handle this differently in Rust, perhaps as a feature flag or a runtime config.
pub const STEPPER_STEP_BOTH_EDGE: bool = true; // Example, true if feature enabled

// From enum { POSITION_BIAS=0x40000000 };
const POSITION_BIAS: i32 = 0x40000000; // Using i32 for position as in sendf

// --- Enums and Bitflags ---

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct StepperMoveFlags: u8 {
        const MF_DIR = 1 << 0; // Indicates if this move requires a direction change
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct StepperFlags: u8 {
        const SF_LAST_DIR      = 1 << 0; // Last direction of movement
        const SF_NEXT_DIR      = 1 << 1; // Next direction of movement (for pending queue)
        const SF_INVERT_STEP   = 1 << 2; // Invert step pin logic
        const SF_NEED_RESET    = 1 << 3; // Stepper needs a clock reset before next move
        const SF_SINGLE_SCHED  = 1 << 4; // Stepper uses single event scheduling (vs. step/unstep)
        const SF_OPTIMIZED_PATH= 1 << 5; // Stepper is using an optimized event path
        const SF_HAVE_ADD      = 1 << 6; // Current move has an 'add' component (for AVR optimization)
    }
}

// Return codes for stepper_event functions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepEventResult {
    Done,        // No more moves, or current move finished and queue empty
    Reschedule,  // Timer needs to be rescheduled for the next event
    // SF_NO_TIMER_ADD not directly translated, scheduler handles timer adds
}

// --- Placeholder for Hardware Abstractions ---
// These would be replaced by types from a specific HAL crate

pub trait GpioOut {
    fn setup(pin_id: u8, inverted: bool) -> Self; // Simplified, HALs have more complex setup
    fn write(&mut self, high: bool);
    fn toggle(&mut self);
    // gpio_out_toggle_noirq in C implies direct register access,
    // HALs might provide this or it might require unsafe code.
    // For now, a simple toggle is provided.
}

pub trait Timer {
    // Represents a timer that can be scheduled by a scheduler
    fn new(callback: fn(&mut Self) -> StepEventResult) -> Self; // Simplified
    fn get_waketime(&self) -> u32;
    fn set_waketime(&mut self, waketime: u32);
    // func pointer is handled by the callback in new()
}

pub trait Scheduler {
    // Simplified scheduler interface
    fn add_timer(&mut self, timer: &mut impl Timer);
    fn delete_timer(&mut self, timer: &mut impl Timer);
    fn read_time(&self) -> u32; // Equivalent to timer_read_time()
    fn get_clock_freq(&self) -> u32; // To be used by timer_from_us
    // timer_is_before can be implemented using direct comparison of u32 times
}

use core::marker::PhantomData;

// --- Core Data Structures ---

#[derive(Debug, Clone, Copy)]
pub struct StepperMove {
    // node: struct move_node; // Queue implementation will handle node aspects
    interval: u32,   // Interval to next step event (or first step of this move)
    add: i16,        // Value to add to interval after each step (for acceleration)
    count: u16,      // Number of steps in this move
    flags: StepperMoveFlags,
}

const MAX_QUEUE_MOVES: usize = 16; // Example size, configure as needed

pub struct Stepper<STEP: GpioOut, DIR: GpioOut, T: Timer, SCHED: Scheduler> {
    timer: T, // In C, struct timer time; (embedded directly)
              // In Rust, it's common to hold the timer instance.
              // Waketime is managed within the Timer trait / its impl.

    // Stepper configuration and state
    step_pin: STEP,
    dir_pin: DIR,
    flags: StepperFlags,
    step_pulse_ticks: u32, // Minimum duration for a step pulse

    // Current move dynamics
    current_interval: u32, // s->interval in C; renamed for clarity
    current_add: i16,      // s->add in C
    steps_remaining: u32,  // s->count in C; represents remaining steps or half-steps

    // Timing for the "full" scheduler path
    next_step_time: u32,   // Time for the next full step (high edge)

    position: i32,         // Current absolute position of the stepper (biased)

    // Move queue
    // In C: struct move_queue_head mq;
    // Using heapless SPSC queue. Producer is owned by Stepper (or main code),
    // Consumer is used within Stepper logic.
    // For simplicity here, we'll assume Stepper owns both for now,
    // or methods take/return moves. Let's simplify to a VecDeque for now
    // as SPSC might be too complex for initial translation.
    move_queue: heapless::Deque<StepperMove, MAX_QUEUE_MOVES>,


    // trsync_signal stop_signal; // This needs a more complex system for event sync
    // For now, we'll omit direct translation of trsync and handle stop differently.
    stop_requested: bool, // Simplified stop mechanism

    // Reference to the scheduler (could be a &'a mut SCHED or a shared global)
    // For now, let's assume it's passed in where needed or owned if only one stepper.
    // If multiple steppers share a scheduler, a reference is better.
    // scheduler: SCHED, // This might be better passed into methods needing it.

    // Placeholder for target-specific optimizations
    // HAVE_EDGE_OPTIMIZATION, HAVE_AVR_OPTIMIZATION will be handled by
    // different implementations or conditional compilation later.
    _scheduler_phantom: PhantomData<SCHED>,
}

// --- Helper Functions (Conceptual for now) ---

// timer_is_before(t1, t2) -> t1 < t2 (considering wraparound for u32 timers)
fn timer_is_before(time1: u32, time2: u32) -> bool {
    // Handles timer wraparound for u32 timers.
    // Assumes a difference of up to 2^31 ticks in either direction.
    (time1 as i32).wrapping_sub(time2 as i32) < 0
}

// timer_from_us(us) -> ticks
// This depends on CONFIG_CLOCK_FREQ, which is not available here.
// We'll assume it's provided or calculated elsewhere.
fn timer_from_us(us: u32, clock_freq: u32) -> u32 {
    (us as u64 * clock_freq as u64 / 1_000_000) as u32
}


// oid_alloc, move_alloc, move_free:
// Rust's ownership system replaces manual memory management.
// StepperMoves can be created on the stack or from a pool if using an allocator.
// OID lookup would be part of a higher-level stepper management system.

// --- Stepper Implementation ---

impl<STEP: GpioOut, DIR: GpioOut, T: Timer, SCHED: Scheduler> Stepper<STEP, DIR, T, SCHED> {
    /// Creates a new Stepper instance.
    /// Corresponds to `command_config_stepper`.
    ///
    /// # Arguments
    /// * `step_pin_id`, `dir_pin_id`: Identifiers for the GPIO pins.
    /// * `invert_step_setting`: Controls step pin inversion and scheduling mode.
    ///   - `> 0`: Invert step pin.
    ///   - `< 0`: Use single schedule mode.
    ///   - `0`: Normal operation.
    /// * `step_pulse_ticks`: Minimum duration for a step pulse in timer ticks.
    /// * `step_pin_factory`, `dir_pin_factory`: Functions/closures to create GpioOut instances.
    /// * `timer_factory`: Function/closure to create a Timer instance.
    ///
    /// Note: The factories are a way to abstract HAL-specific pin/timer creation.
    /// In a real scenario, these might be passed in directly if already initialized.
    pub fn new(
        step_pin_id: u8, // Assuming u8 is sufficient for pin identification
        dir_pin_id: u8,
        invert_step_setting: i8,
        step_pulse_ticks: u32,
        mut step_pin_factory: impl FnMut(u8, bool) -> STEP,
        mut dir_pin_factory: impl FnMut(u8, bool) -> DIR,
        mut timer_factory: impl FnMut(fn(&mut Stepper<STEP, DIR, T, SCHED>, &mut SCHED) -> StepEventResult) -> T, // Timer needs a way to call back
        // The callback for the timer is tricky. It needs context (self) and scheduler.
        // For now, let's assume the timer callback is set up to call a static-like method
        // or the `Timer` trait itself handles the dispatch to the correct `Stepper` instance's method.
        // This is a common challenge in Rust embedded event systems.
        // A simpler timer might just take `fn(&mut Self)` where Self is the Stepper.
        // Let's simplify the timer factory for now and assume the event loop handles routing.
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
        let dir_pin = dir_pin_factory(dir_pin_id, false); // Dir pin usually not inverted by default

        // Initialize timer - this is complex due to callback ownership.
        // One common pattern is that the scheduler knows which timer belongs to which stepper.
        // Or, the Timer holds a reference or ID to the Stepper.
        // For now, we'll assume the timer is created and the Stepper will manage its scheduling.
        // The actual callback mechanism needs careful design based on the specific async/event model.
        // Let's assume the timer_factory gives us a timer, and we'll later associate its handler.
        // For now, the `timer_event_handler` is passed, but not directly attached in `new`.
        // It will be used when scheduling the timer.

        let timer = timer_factory(Self::stepper_event_callback); // Simplified: timer takes a static-like callback


        // C code: s->position = -POSITION_BIAS;
        // In Rust, using i32, so direct assignment.
        let position = -POSITION_BIAS;

        // C code: move_queue_setup(&s->mq, sizeof(struct stepper_move));
        // `heapless::VecDeque` is already initialized by `new` or `default`.

        // Optimizations (HAVE_EDGE_OPTIMIZATION, HAVE_AVR_OPTIMIZATION)
        // These depend on CONFIG flags and specific conditions.
        // For now, we'll assume they are off or handled by feature flags in Rust.
        // if (HAVE_EDGE_OPTIMIZATION) {
        //     if (invert_step < 0 && s->step_pulse_ticks <= EDGE_STEP_TICKS)
        //         s->flags |= SF_OPTIMIZED_PATH;
        //     else
        //         s->time.func = stepper_event_full; // This implies different event handlers
        // } // ... and similar for AVR.
        // We'll default to a "full" event path and can add optimized paths later.

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
            move_queue: heapless::Deque::new(),
            stop_requested: false,
            _scheduler_phantom: PhantomData,
        }
    }

    /// Sets the direction of the next queued step.
    /// Corresponds to `command_set_next_step_dir`.
    pub fn set_next_step_dir(&mut self, set_forward_dir: bool) {
        // irq_disable(); // In Rust, this would be a critical section
        cortex_m::interrupt::free(|_cs| { // Example critical section, replace with actual mechanism
            if set_forward_dir {
                self.flags |= StepperFlags::SF_NEXT_DIR;
            } else {
                self.flags &= !StepperFlags::SF_NEXT_DIR;
            }
        });
        // irq_enable();
    }

    /// Resets the step clock to a specific time.
    /// Corresponds to `command_reset_step_clock`.
    ///
    /// # Panics
    /// Panics if called when the stepper is active (has steps remaining).
    /// This matches the C code's `shutdown("Can't reset time when stepper active")`.
    pub fn reset_step_clock(&mut self, waketime: u32, _scheduler: &mut SCHED) {
        // irq_disable();
        cortex_m::interrupt::free(|_cs| { // Example critical section
            if self.steps_remaining > 0 {
                // In C, this is shutdown. In Rust, panic is the typical equivalent for unrecoverable errors.
                // Consider returning a Result if this can be a recoverable error.
                panic!("Can't reset time when stepper active");
            }
            self.next_step_time = waketime;
            self.timer.set_waketime(waketime); // s->time.waketime = waketime;
            self.flags &= !StepperFlags::SF_NEED_RESET;
        });
        // irq_enable();
    }

    // This is a placeholder for the actual timer event handler.
    // The challenge is how the scheduler calls this method on the correct Stepper instance.
    // This often involves some form of registration or context passing.
    // One way is for the Timer trait to be implemented by Stepper itself,
    // or for the Timer object to store a pointer/ID back to its Stepper.
    // For now, a static-like dispatcher that would need to look up the stepper.
    // This part is highly dependent on the specific RTOS/scheduler.
    // Let's assume for now that the scheduler can somehow pass the correct `Stepper` instance.
    fn stepper_event_callback(stepper: &mut Self /* how to get this? */, scheduler: &mut SCHED) -> StepEventResult {
        // This would dispatch to stepper_event_full, _edge, or _avr based on flags.
        // For now, let's assume it calls a unified event handler on the stepper instance.
        stepper.handle_stepper_event(scheduler)
    }

    // Unified event handler - primarily translates stepper_event_full() for now.
    fn handle_stepper_event(&mut self, scheduler: &mut SCHED) -> StepEventResult {
        // Corresponds to stepper_event_full(struct timer *t)

        // 1. Toggle step pin
        self.step_pin.toggle(); // gpio_out_toggle_noirq(s->step_pin);

        let curtime = scheduler.read_time();
        let min_next_time = curtime.wrapping_add(self.step_pulse_ticks);

        // 2. Decrement step count
        self.steps_remaining = self.steps_remaining.saturating_sub(1);

        // 3. Check if current phase (step or unstep) or move is complete
        if self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
            // Single schedule mode: one event per step
            if self.steps_remaining > 0 {
                // More steps in this move
                self.next_step_time = self.next_step_time.wrapping_add(self.current_interval);
                self.current_interval = self.current_interval.wrapping_add(self.current_add as u32);

                if timer_is_before(self.next_step_time, min_next_time) {
                    // Next step event is too close, push it back
                    self.timer.set_waketime(min_next_time);
                } else {
                    self.timer.set_waketime(self.next_step_time);
                }
                StepEventResult::Reschedule
            } else {
                // Move finished, load next
                self.timer.set_waketime(min_next_time); // Ensure pulse width for last step
                self.load_next_move(scheduler) // This will return Done or Reschedule
            }
        } else {
            // Dual schedule mode: two events per step (step and unstep)
            // s->count was initialized to move_count * 2
            if self.steps_remaining > 0 {
                if self.steps_remaining % 2 == 1 { // Odd count means this was a step, schedule unstep
                    // Schedule unstep event (reschedule_min in C)
                    self.timer.set_waketime(min_next_time);
                } else { // Even count means this was an unstep, schedule next step
                    self.next_step_time = self.next_step_time.wrapping_add(self.current_interval);
                    self.current_interval = self.current_interval.wrapping_add(self.current_add as u32);

                    if timer_is_before(self.next_step_time, min_next_time) {
                        // Next step event is too close, push it back
                        self.timer.set_waketime(min_next_time);
                    } else {
                        self.timer.set_waketime(self.next_step_time);
                    }
                }
                StepEventResult::Reschedule
            } else {
                // Move finished (after an unstep), load next
                self.timer.set_waketime(min_next_time); // Ensure pulse width for last unstep
                self.load_next_move(scheduler) // This will return Done or Reschedule
            }
        }
    }

    /// Calculates the current logical position of the stepper, adjusted for bias.
    /// This is intended to be called internally or within a critical section.
    fn get_current_logical_position(&self) -> i32 {
        // This translates the logic from C's `stepper_get_position()` and the final bias subtraction.
        // It aims to replicate the C code's uint32_t behavior for s->position.

        // Start with self.position (i32), treat its bit pattern as if it were the C uint32_t s->position.
        let mut current_val_as_u32 = self.position as u32;

        // Subtract steps not yet taken in the current move (as u32, mirroring C)
        let pending_steps_u32 = if self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
            self.steps_remaining // steps_remaining is u32
        } else {
            self.steps_remaining / 2 // steps_remaining is u32
        };
        current_val_as_u32 = current_val_as_u32.wrapping_sub(pending_steps_u32);

        // Apply C's sign logic from `stepper_get_position`:
        // `if (position & 0x80000000) return -position; else return position;`
        // This effectively returns a non-negative version of the biased count.
        let effective_biased_pos_u32 = if current_val_as_u32 & 0x8000_0000 != 0 {
            current_val_as_u32.wrapping_neg() // e.g., 0xC0000000u32.wrapping_neg() is 0x40000000u32
        } else {
            current_val_as_u32
        };

        // Final subtraction of bias, result is i32
        // (int32_t)X_uint - POSITION_BIAS
        (effective_biased_pos_u32 as i32) - POSITION_BIAS
    }


    /// Reports the current logical position of the stepper.
    /// Corresponds to `command_stepper_get_position`.
    /// This function should be called from a context where interrupts are disabled
    /// if called concurrently with stepper operations. (e.g. via `cortex_m::interrupt::free`)
    pub fn report_position(&self) -> i32 {
        // In C, irq_disable() / irq_enable() surrounds the call to stepper_get_position().
        // The caller of this Rust function should ensure similar protection if needed.
        // For simplicity, this method itself doesn't manage critical sections, assuming
        // the caller does if necessary (e.g. if `&self` could be read while `&mut self` methods run).
        // If this is part of a larger system, `&self` reads are often safe if state updates are atomic or CS protected.
        // Given the complexity, let's assume it's called when state is consistent.
        self.get_current_logical_position()
    }

    /// Stops the stepper immediately, clears its move queue, and resets its state.
    /// Corresponds to `stepper_stop()` in C.
    ///
    /// # Arguments
    /// * `scheduler`: A mutable reference to the scheduler for deleting the timer.
    ///
    /// Note: This function should ideally be called within a critical section
    /// if concurrent access is possible.
    pub fn stop_stepper(&mut self, scheduler: &mut SCHED) {
        // 1. Delete timer from scheduler
        scheduler.delete_timer(&mut self.timer);

        // 2. Reset next_step_time and waketime
        self.next_step_time = 0;
        self.timer.set_waketime(0);

        // 3. Recalculate s->position
        // C: s->position = -stepper_get_position(s);
        // stepper_get_position() in C returns an unsigned value which is `abs(internal_pos - pending_steps)`.
        // Then it's negated. So, `s->position = -(abs(internal_pos - pending_steps))`.
        // Our get_current_logical_position() returns `abs_val - BIAS`.
        // We need the equivalent of C's `stepper_get_position(s)` which is `abs_biased_pos`.
        // Let's get `current_pos_val` adjusted for pending steps first.
        let mut temp_pos_val = self.position;
        if self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
            temp_pos_val = temp_pos_val.wrapping_sub(self.steps_remaining as i32);
        } else {
            temp_pos_val = temp_pos_val.wrapping_sub((self.steps_remaining / 2) as i32);
        }
        // Now `temp_pos_val` is the C's `position` before `if (position & 0x80000000)`.
        // C's `stepper_get_position` returns `temp_pos_val.abs()`.
        // So, `self.position` should become `-(temp_pos_val.abs())`.
        self.position = -(temp_pos_val.abs());


        // 4. Reset count
        self.steps_remaining = 0;

        // 5. Update flags
        let preserved_flags = StepperFlags::SF_INVERT_STEP | StepperFlags::SF_SINGLE_SCHED | StepperFlags::SF_OPTIMIZED_PATH;
        self.flags = (self.flags & preserved_flags) | StepperFlags::SF_NEED_RESET;
        // SF_LAST_DIR and SF_NEXT_DIR are cleared by not preserving them.

        // 6. Set dir pin to default state (0 = typically forward for many setups, or a known state)
        // C: gpio_out_write(s->dir_pin, 0);
        // Assuming our GpioOut::write(false) corresponds to logic level 0.
        self.dir_pin.write(false);


        // 7. If not single schedule or AVR optimized, set step pin to "unstep" state
        // C: if (!(s->flags & SF_SINGLE_SCHED) || (HAVE_AVR_OPTIMIZATION && s->flags & SF_OPTIMIZED_PATH))
        //       gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
        // We don't have HAVE_AVR_OPTIMIZATION flag yet, so simplify.
        // The "unstep" state depends on whether the step pin is inverted.
        let unstep_state = self.flags.contains(StepperFlags::SF_INVERT_STEP);
        if !self.flags.contains(StepperFlags::SF_SINGLE_SCHED) { // Simplified condition
            self.step_pin.write(unstep_state);
        }
        // If SF_SINGLE_SCHED is true, the pin is left as is from the last toggle.
        // If it's dual schedule, the last event was an unstep, so toggling it would make it a step.
        // This logic ensures it's actively set to the un-energized state.

        // 8. Clear the move queue
        // C: while (!move_queue_empty(&s->mq)) { struct move_node *mn = move_queue_pop(&s->mq); move_free(m); }
        self.move_queue.clear(); // heapless::VecDeque::clear() handles this.

        // Note: The trsync mechanism for `stepper_stop_on_trigger` is not implemented here.
        // That would require a broader event/trigger system.
        self.stop_requested = true; // Mark that a stop was explicitly requested.
    }

    /// Queues a new move for the stepper.
    /// Corresponds to `command_queue_step`.
    ///
    /// # Arguments
    /// * `interval`: Interval for the first step of this move.
    /// * `count`: Number of steps in this move. Must be > 0.
    /// * `add`: Value to add to interval after each step.
    /// * `scheduler`: A mutable reference to the scheduler for adding timers.
    ///
    /// # Panics
    /// * If `count` is 0.
    /// * If the move queue is full.
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

        // Critical section to access and modify shared stepper state (flags, queue)
        cortex_m::interrupt::free(|_cs| {
            // Determine if direction needs to change for this move
            // C: if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR))
            let last_dir_is_set = self.flags.contains(StepperFlags::SF_LAST_DIR);
            let next_dir_is_set = self.flags.contains(StepperFlags::SF_NEXT_DIR);

            if last_dir_is_set != next_dir_is_set {
                self.flags.toggle(StepperFlags::SF_LAST_DIR); // Flip SF_LAST_DIR
                move_flags |= StepperMoveFlags::MF_DIR;
            }

            let new_move = StepperMove {
                interval,
                count,
                add,
                flags: move_flags,
            };

            if self.move_queue.push_back(new_move).is_err() {
                panic!("Stepper move queue full!"); // Or return Result::Err
            }

            // If stepper is not currently active, load and start this move.
            // C: if (s->count) { ... } else if (flags & SF_NEED_RESET) { ... } else { ... }
            if self.steps_remaining == 0 { // s->count == 0 means stepper is idle
                if self.flags.contains(StepperFlags::SF_NEED_RESET) {
                    // Move was queued, but stepper needs reset, so don't start it.
                    // The C code frees the move here; in Rust, if we panic above on full queue,
                    // the new_move might be dropped. If we stored it and then decided not to use,
                    // it would be dropped when this scope ends if not consumed by load_next_move.
                    // Here, it's in the queue, but load_next_move won't be called.
                } else {
                    // Load the new move and schedule the timer.
                    // This mirrors the C logic of calling stepper_load_next and sched_add_timer.
                    if self.load_next_move(scheduler) == StepEventResult::Reschedule {
                        scheduler.add_timer(&mut self.timer);
                    }
                }
            }
            // If s->count is non-zero, the move is simply queued.
            // The current move's completion will trigger loading the next one.
        });
    }

    /// Loads the next move from the queue into the stepper's active state.
    /// Corresponds to `stepper_load_next` from C.
    /// This function is typically called when the previous move completes or when
    /// a move is queued to an idle stepper.
    ///
    /// Returns `StepEventResult::Reschedule` if a move was loaded and needs scheduling,
    /// or `StepEventResult::Done` if the queue was empty.
    fn load_next_move(&mut self, scheduler: &mut SCHED) -> StepEventResult {
        if let Some(move_to_load) = self.move_queue.pop_front() {
            // Read next 'struct stepper_move' (already done by pop_front)
            let move_interval = move_to_load.interval;
            let move_count = move_to_load.count;
            let move_add = move_to_load.add;
            let needs_dir_change = move_to_load.flags.contains(StepperMoveFlags::MF_DIR);

            // C: s->position = (need_dir_change ? -s->position : s->position) + move_count;
            // The position logic in C is a bit complex due to the top bit flag.
            // Our `self.position` is a standard i32.
            // The C code `s->position` stores `actual_pos + POSITION_BIAS` or `-actual_pos - POSITION_BIAS`
            // Let's clarify the position update:
            // If dir changes, the sign of accumulated position flips.
            // stepper_get_position() untangles this.
            // For now, let's simplify: if dir changes, the effective direction of future steps flips.
            // The `s->position` in C seems to accumulate raw step counts, and its sign indicates overall direction.
            // Let's assume `self.position` stores the "biased" position.
            // The C code's `s->position = (need_dir_change ? -s->position : s->position) + move_count;`
            // seems to imply that `s->position` itself might flip sign.
            // This needs careful handling to match `stepper_get_position`.
            // For now, we'll update a separate "absolute logical position" and defer reconciling with C's `s->position` representation.
            // Or, more directly:
            if needs_dir_change {
                // This implies the *meaning* of position increments changes.
                // The C code: s->position = -s->position; then s->position += move_count;
                // This is tricky because s->position is already biased.
                // Let's assume stepper_get_position correctly interprets the current position with its bias.
                // If dir changes, then the current position effectively flips sign *relative to the bias point*.
                // Example: current logical is 10 (stored as POSITION_BIAS + 10). If dir flips, it becomes -10 (stored as POSITION_BIAS - 10).
                // No, the C code is simpler: `s->position = (dir_change ? -current_raw_pos : current_raw_pos) + count`.
                // And `current_raw_pos` is `s->position`. So if `s->position` is `X`, it becomes `-X + count`.
                // This suggests `s->position` might not always be `logical_pos + BIAS` but `-logical_pos + BIAS` too.
                // This interpretation makes `stepper_get_position` crucial.
                // Let current_logical_pos = self.get_logical_position(); // before this move
                // if needs_dir_change { self.position_is_negative_relative_to_bias = !self.position_is_negative_relative_to_bias }
                // self.position += move_count (adjusting for direction)
                // This part is the trickiest to port directly without fully replicating `stepper_get_position` first.
                // For now, let's assume `self.position` always stores steps in the "current" direction.
                // And `SF_LAST_DIR` reflects that current direction.
                // `stepper_get_position` will need to use `SF_LAST_DIR` to interpret `self.position`.

                // Simpler: `s->position` in C is not `logical_pos + BIAS` but rather `(direction_flag ? -logical_pos : logical_pos) + BIAS`
                // where `direction_flag` is effectively stored in the top bit of `s->position` itself.
                // Our `self.position` is `i32`, so we can't use the top bit trick directly in the same way unless we manage it manually.
                // Let's stick to the C code's operation on s->position for now and ensure get_position matches.
                if needs_dir_change {
                    self.position = self.position.wrapping_neg(); // This negates the biased value.
                }
            }
            self.position = self.position.wrapping_add(move_count as i32);


            // Load next move into 'struct stepper'
            self.current_add = move_add;
            self.current_interval = move_interval.wrapping_add(move_add as u32); // First interval includes add

            // C code has optimized paths (HAVE_EDGE_OPTIMIZATION, HAVE_AVR_OPTIMIZATION)
            // We are targeting the "fully scheduled" path first.
            // s->time.waketime is current time for an idle stepper.
            // s->next_step_time += move_interval;
            // s->time.waketime = s->next_step_time;
            // s->count = (s->flags & SF_SINGLE_SCHED ? move_count : (uint32_t)move_count * 2);

            let was_active = self.steps_remaining > 0; // Equivalent to C's `!!s->count`
            let min_next_time_for_calc = self.timer.get_waketime(); // Current timer expiry, effectively "now" if stepper was idle

            self.next_step_time = self.timer.get_waketime().wrapping_add(move_interval); // If idle, timer waketime is "current" or last event time.
                                                                              // If it was already stepping, next_step_time should be based on s->next_step_time
                                                                              // C code: s->next_step_time += move_interval (this is for full path)
                                                                              // Let's trace: if stepper was idle, s->next_step_time was 0 or reset by reset_step_clock.
                                                                              // command_reset_step_clock sets s->next_step_time = s->time.waketime = waketime.
                                                                              // So, if idle, self.next_step_time is the time of the *end* of the last step (or reset time).
                                                                              // Correct C logic for full path:
                                                                              // s->next_step_time += move_interval (this is from previous step's next_step_time)
                                                                              // s->time.waketime = s->next_step_time (for the high part of the step)
                                                                              // This implies self.next_step_time should persist.
                                                                              // If starting from idle, self.next_step_time was set by reset_step_clock.
            self.timer.set_waketime(self.next_step_time);


            if self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
                self.steps_remaining = move_count as u32;
            } else {
                self.steps_remaining = (move_count as u32) * 2; // Each step takes two events (high and low)
            }

            if was_active && timer_is_before(self.next_step_time, min_next_time_for_calc) {
                // Actively stepping and next step event close to the last unstep
                // (This logic seems to handle cases where calculations might put next step in past)
                let diff = self.next_step_time.wrapping_sub(min_next_time_for_calc) as i32;
                if diff < -(timer_from_us(1000, scheduler.get_clock_freq()) as i32) { // Assuming scheduler can give clock_freq
                    panic!("Stepper too far in past"); // Or return Result::Err
                }
                self.timer.set_waketime(min_next_time_for_calc);
            }

            // Set new direction (if needed)
            // C code: if (need_dir_change) gpio_out_toggle_noirq(s->dir_pin);
            // This happens *after* was_active checks in C.
            // The C code has a more complex block for `was_active && need_dir_change`
            // ensuring minimum time between step change and dir change.
            if was_active && needs_dir_change {
                // Must ensure minimum time between step change and dir change
                if self.flags.contains(StepperFlags::SF_SINGLE_SCHED) {
                    // C: while (timer_is_before(timer_read_time(), min_next_time_for_calc)) ;
                    // This is a busy wait, generally bad. In Rust, this might involve yielding
                    // or a specific delay mechanism if truly required.
                    // For now, we'll assume such short delays are either handled by GPIO timing
                    // or this specific spin wait is acceptable in the context.
                    while timer_is_before(scheduler.read_time(), min_next_time_for_calc) {
                        // Spin or yield if possible
                        core::hint::spin_loop();
                    }
                }
                self.dir_pin.toggle(); // Assumes toggle flips state correctly.
                                      // Klipper's gpio_out_toggle_noirq implies it knows the current state or just XORs.
                                      // Our GpioOut trait might need a `set_state` or ensure toggle is well-defined.
                                      // For now, assume toggle is sufficient.

                let curtime = scheduler.read_time();
                let min_dir_change_settle_time = curtime.wrapping_add(self.step_pulse_ticks); // Using step_pulse_ticks as proxy for settle time
                if timer_is_before(self.timer.get_waketime(), min_dir_change_settle_time) {
                    self.timer.set_waketime(min_dir_change_settle_time);
                }
                return StepEventResult::Reschedule; // Return immediately to reschedule with new dir timing
            } else if needs_dir_change {
                self.dir_pin.toggle();
            }

            StepEventResult::Reschedule
        } else {
            // Queue is empty
            self.steps_remaining = 0; // s->count = 0 in C
            StepEventResult::Done
        }
    }
}

#[cfg(test)]
mod tests {
    // use super::*; // No longer needed for this minimal test

    #[test]
    fn basic_test_works() {
        assert_eq!(2 + 2, 4);
    }
}
