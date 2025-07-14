// ... (all previous `use` statements and globals as before) ...
// Add `kinematics` module to imports
use klipper_mcu_lib::kinematics::cartesian::CartesianKinematics;

// ... (all previous callbacks, dispatchers, main, ISRs, helpers as before) ...

// --- process_command (with fully refactored MOVE command) ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    let mut parts = line.trim().splitn(2, |c: char| c.is_whitespace());
    let command = parts.next().unwrap_or("").to_ascii_uppercase();
    let args_str = parts.next().unwrap_or("");
    let parsed_args_result = if !args_str.is_empty() { parse_gcode_arguments(args_str) } else { Ok(ParsedArgs::new()) };

    match parsed_args_result {
        Ok(args) => {
            // ... (other commands: PING, ID, ECHO, SET_PIN, GET_PIN, QUERY_ADC, SET_PWM) ...

            if command == "MOVE" {
                // Syntax: MOVE X<f32> Y<f32> A<f32> V<f32>
                let x_pos_opt = args.get(&'X').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::Integer(i) => Some(*i as f32), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                let y_pos_opt = args.get(&'Y').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::Integer(i) => Some(*i as f32), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                let accel_opt = args.get(&'A').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                let vel_opt = args.get(&'V').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });

                if let (Some(x_pos), Some(y_pos), Some(accel), Some(vel)) = (x_pos_opt, y_pos_opt, accel_opt, vel_opt) {
                    let cmd_result = interrupt_free(|cs| {
                        let mut scheduler_opt = SCHEDULER.borrow(cs).borrow_mut();
                        let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
                        let mut stepper_x_opt = STEPPER_X.borrow(cs).borrow_mut();
                        let mut stepper_y_opt = STEPPER_Y.borrow(cs).borrow_mut();
                        let sys_clk_opt = SYSTEM_CLOCK_FREQ.borrow(cs).borrow();

                        if let (Some(ref mut scheduler), Some(ref mut gpio_manager), Some(ref mut stepper_x), Some(ref mut stepper_y), Some(clock_freq)) =
                            (scheduler_opt.as_mut(), gpio_manager_opt.as_mut(), stepper_x_opt.as_mut(), stepper_y_opt.as_mut(), sys_clk_opt.as_ref()) {

                            if stepper_x.current_move.is_some() || stepper_y.current_move.is_some() {
                                return Err("A stepper is already moving");
                            }

                            // For now, assume current position is (0,0) and steps/mm is fixed
                            let kinematics = CartesianKinematics::new(80.0, 80.0); // 80 steps/mm
                            let current_pos = (0.0, 0.0); // HACK: Assume we start at 0,0
                            let (x_steps, y_steps) = kinematics.move_to((x_pos, y_pos), current_pos);

                            let x_dist = (x_pos - current_pos.0).abs();
                            let y_dist = (y_pos - current_pos.1).abs();
                            let move_dist = (x_dist.powi(2) + y_dist.powi(2)).sqrt();
                            if move_dist < 0.000001 { return Ok(()); } // No move

                            let dominant_axis_steps = x_steps.max(y_steps);

                            // Create move params based on dominant axis and vector speed
                            let move_params = TrapezoidalMove {
                                total_steps: dominant_axis_steps,
                                acceleration: accel * (dominant_axis_steps as f32 / move_dist), // Scale accel to dominant axis
                                start_velocity: 0.0, // Assume start from 0 for now
                                cruise_velocity: vel * (dominant_axis_steps as f32 / move_dist), // Scale velocity to dominant axis
                            };

                            let move_plan = MovePlanner::new(&move_params, *clock_freq)?;

                            // Setup Stepper X
                            stepper_x.set_direction(if x_pos > current_pos.0 { StepperDirection::Clockwise } else { StepperDirection::CounterClockwise }, gpio_manager)?;
                            stepper_x.current_move = Some(move_plan);
                            stepper_x.current_step_num = 0;
                            stepper_x.is_pulsing_high = false;
                            stepper_x.step_period_ticks = move_plan.initial_period_ticks;
                            stepper_x.bresenham_increment = x_steps;
                            stepper_x.bresenham_decrement = 2 * dominant_axis_steps;
                            stepper_x.bresenham_error = (stepper_x.bresenham_decrement / 2) as i32;

                            // Setup Stepper Y
                            stepper_y.set_direction(if y_pos > current_pos.1 { StepperDirection::Clockwise } else { StepperDirection::CounterClockwise }, gpio_manager)?;
                            stepper_y.current_move = Some(move_plan);
                            stepper_y.current_step_num = 0;
                            stepper_y.is_pulsing_high = false;
                            stepper_y.step_period_ticks = move_plan.initial_period_ticks;
                            stepper_y.bresenham_increment = y_steps;
                            stepper_y.bresenham_decrement = 2 * dominant_axis_steps;
                            stepper_y.bresenham_error = (stepper_y.bresenham_decrement / 2) as i32;

                            // Kick off the move by scheduling the dominant axis's scheduler task
                            let now = scheduler.read_time();
                            let dominant_stepper_task_id = if x_steps >= y_steps { STEPPER_X_SCHEDULER_TASK_ID } else { STEPPER_Y_SCHEDULER_TASK_ID };
                            let first_waketime = now.wrapping_add(100);

                            // Both steppers need their next_step_waketime set for the first step calculation
                            stepper_x.next_step_waketime = first_waketime;
                            stepper_y.next_step_waketime = first_waketime;

                            scheduler.schedule_task(dominant_stepper_task_id, first_waketime);

                            info!("MOVE command initiated: X_steps={}, Y_steps={}", x_steps, y_steps);
                            Ok(())
                        } else { Err("A required resource (Scheduler, GPIO, Stepper) is not initialized") }
                    });
                    if let Err(e) = cmd_result { serial_write_line(serial, e); } else { serial_write_line(serial, "ok\r\n"); }
                } else {
                    serial_write_line(serial, "Error: Missing required X, Y, A, or V arguments for MOVE\r\n");
                }
            }
            // ... other commands ...
        }
        Err(e) => { /* ... */ }
    }
}
// ... (rest of main.rs) ...
