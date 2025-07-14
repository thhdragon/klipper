// ... (all `use` statements and globals as before) ...

// --- Callbacks and Dispatchers ---
// ... (test_timer0_callback, test_timer1_callback, master_scheduler_timer_callback as before) ...

// --- NEW Central Move Handler ---
fn handle_move_event() {
    interrupt_free(|cs| {
        let mut scheduler_opt = SCHEDULER.borrow(cs).borrow_mut();
        let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
        let mut stepper_x_opt = STEPPER_X.borrow(cs).borrow_mut();
        let mut stepper_y_opt = STEPPER_Y.borrow(cs).borrow_mut();

        if let (Some(ref mut scheduler), Some(ref mut gpio_manager), Some(ref mut stepper_x), Some(ref mut stepper_y)) =
            (scheduler_opt.as_mut(), gpio_manager_opt.as_mut(), stepper_x_opt.as_mut(), stepper_y_opt.as_mut()) {

            // Check if there is an active move on either stepper (they should be in sync)
            let mov = if let Some(ref m) = stepper_x.current_move { m } else {
                // No move on X, there shouldn't be one on Y either. Clear just in case.
                stepper_y.current_move = None;
                return;
            };

            // Determine dominant axis for timing calculations (the one with more total steps)
            let is_x_dominant = stepper_x.bresenham_increment >= stepper_y.bresenham_increment;
            let dominant_stepper = if is_x_dominant { &*stepper_x } else { &*stepper_y };

            // --- Acceleration/Deceleration/Cruise Logic for Dominant Axis ---
            // This logic calculates the time for the *next* step pulse.
            let mut next_step_period = dominant_stepper.step_period_ticks;
            let step_num_for_accel_calc = dominant_stepper.current_step_num + 1; // n for c_n is the step we are about to take

            if step_num_for_accel_calc <= mov.accel_steps {
                // Acceleration phase
                if step_num_for_accel_calc > 1 {
                    let c = dominant_stepper.step_period_ticks as i64;
                    let next_c = c - (2 * c) / (4 * (step_num_for_accel_calc as i64 - 1) + 1);
                    next_step_period = if next_c > 0 { next_c as u32 } else { mov.cruise_period_ticks };
                } else {
                    next_step_period = mov.initial_period_ticks;
                }
                if next_step_period < mov.cruise_period_ticks {
                    next_step_period = mov.cruise_period_ticks;
                }
            } else if step_num_for_accel_calc > (mov.accel_steps + mov.cruise_steps) {
                // Deceleration phase
                let n_prime = mov.total_steps - step_num_for_accel_calc + 1;
                let c = dominant_stepper.step_period_ticks as i64;
                let next_c = c + (2 * c) / (4 * (n_prime as i64 - 1) + 1);
                next_step_period = next_c as u32;
            } else {
                // Cruise phase
                next_step_period = mov.cruise_period_ticks;
            }

            // --- Bresenham's Algorithm for all axes ---
            // Issue pulses for any steppers that are due on this tick
            let mut steppers_to_pulse = heapless::Vec::<&mut Stepper, 2>::new();

            // Check Stepper X
            stepper_x.bresenham_error += stepper_x.bresenham_increment as i32;
            if stepper_x.bresenham_error > 0 { // Note: Klipper uses `> 0`, some use `>= dec/2`. Let's use `> 0`.
                stepper_x.bresenham_error -= stepper_x.bresenham_decrement as i32;
                if stepper_x.issue_step_pulse_start(gpio_manager).is_ok() {
                    steppers_to_pulse.push(stepper_x).ok();
                } else { error!("Failed to start pulse for Stepper X"); stepper_x.current_move = None; stepper_y.current_move = None; return; }
            }

            // Check Stepper Y
            stepper_y.bresenham_error += stepper_y.bresenham_increment as i32;
            if stepper_y.bresenham_error > 0 {
                stepper_y.bresenham_error -= stepper_y.bresenham_decrement as i32;
                if stepper_y.issue_step_pulse_start(gpio_manager).is_ok() {
                    steppers_to_pulse.push(stepper_y).ok();
                } else { error!("Failed to start pulse for Stepper Y"); stepper_x.current_move = None; stepper_y.current_move = None; return; }
            }

            // --- Scheduling ---
            if steppers_to_pulse.is_empty() && (stepper_x.current_step_num >= mov.total_steps || stepper_y.current_step_num >= mov.total_steps) {
                 // This condition is tricky. A dominant axis should always step.
                 // Let's assume dominant axis always steps if move is not done.
                 // The logic should be: dominant axis steps, then we check Bresenham for others.
                 // The current logic checks all via Bresenham. For dominant axis, increment == decrement, so error always > 0.
                 // This is correct.
            }

            // All steppers that pulsed now need to schedule their pulse end.
            let pulse_end_waketime = scheduler.read_time().wrapping_add(2); // Use a minimal pulse duration.
            for s in steppers_to_pulse.iter_mut() {
                // This would require a separate scheduler task for pulse end, which is complex.
                // Klipper's approach is different. It schedules the *next step start*.
                // The pulse end happens inside the ISR before the next step is calculated.
                // Let's simplify: A single event in the scheduler is for the *start* of a step.
                // The pulse is HIGH for a very short, fixed time (busy wait or second timer).
                // Our current `step_event_callback` models this with two scheduler events per pulse.
                // This multi-axis handler should do the same.

                // Let's redesign handle_move_event to be simpler and closer to our existing stepper callback.
                // The scheduler event means "it is time for the next step decision".
            }

            // --- Let's rewrite this whole handler to be simpler and use the two-phase pulse logic ---
            let dominant_stepper_mut = if is_x_dominant { stepper_x } else { stepper_y };

            if dominant_stepper_mut.is_pulsing_high {
                // --- End of pulse for all steppers that were pulsing ---
                if stepper_x.is_pulsing_high {
                    let _ = stepper_x.issue_step_pulse_end(gpio_manager);
                    stepper_x.is_pulsing_high = false;
                }
                if stepper_y.is_pulsing_high {
                    let _ = stepper_y.issue_step_pulse_end(gpio_manager);
                    stepper_y.is_pulsing_high = false;
                }

                // Check for move completion
                if dominant_stepper_mut.current_step_num >= mov.total_steps {
                    info!("Move complete.");
                    stepper_x.current_move = None;
                    stepper_y.current_move = None;
                    return;
                }

                // Schedule the start of the next step event
                let time_between_pulses = dominant_stepper_mut.step_period_ticks.saturating_sub(dominant_stepper_mut.pulse_duration_ticks);
                let next_event_time = dominant_stepper_mut.next_step_waketime.wrapping_add(time_between_pulses);
                scheduler.schedule_task(dominant_stepper_mut.timer_id_for_scheduler, next_event_time);
                stepper_x.next_step_waketime = next_event_time;
                stepper_y.next_step_waketime = next_event_time;

            } else {
                // --- Start of pulse for due steppers ---
                dominant_stepper_mut.current_step_num += 1;
                // Recalculate dominant stepper's period for the *next* low phase
                // ... (acceleration logic from old callback for dominant_stepper_mut) ...

                // Bresenham's logic
                let mut pulse_x = false;
                stepper_x.bresenham_error += stepper_x.bresenham_increment as i32;
                if stepper_x.bresenham_error > 0 {
                    stepper_x.bresenham_error -= stepper_x.bresenham_decrement as i32;
                    pulse_x = true;
                }
                let mut pulse_y = false;
                stepper_y.bresenham_error += stepper_y.bresenham_increment as i32;
                if stepper_y.bresenham_error > 0 {
                    stepper_y.bresenham_error -= stepper_y.bresenham_decrement as i32;
                    pulse_y = true;
                }

                // Issue pulses
                if pulse_x { let _ = stepper_x.issue_step_pulse_start(gpio_manager); stepper_x.is_pulsing_high = true; }
                if pulse_y { let _ = stepper_y.issue_step_pulse_start(gpio_manager); stepper_y.is_pulsing_high = true; }

                // Schedule the end of this pulse
                let pulse_end_waketime = dominant_stepper_mut.next_step_waketime.wrapping_add(dominant_stepper_mut.pulse_duration_ticks);
                scheduler.schedule_task(dominant_stepper_mut.timer_id_for_scheduler, pulse_end_waketime);
                stepper_x.next_step_waketime = pulse_end_waketime;
                stepper_y.next_step_waketime = pulse_end_waketime;
            }
        }
    });
}

// ... (rest of main.rs) ...
// The `step_event_callback` on `Stepper` is now obsolete.
// The `dispatch_klipper_task_from_scheduler` needs to be updated to call `handle_move_event`.
// The full file overwrite will contain all these consistent changes.
