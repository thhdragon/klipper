// ... (all `use` statements and globals as before) ...

// --- Callbacks and Dispatchers ---
// ... (as before) ...

// --- Entry Point & Main Loop ---
#[entry]
fn main() -> ! { /* ... as before ... */ }

// --- Interrupt Handlers ---
// ... (as before) ...

// --- Helper Functions ---
// ... (as before) ...

// --- process_command (with HOMING_MOVE command) ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    // ... (logic to split command and parse args as before) ...
    let mut parts = line.trim().splitn(2, |c: char| c.is_whitespace());
    let command = parts.next().unwrap_or("").to_ascii_uppercase();
    let args_str = parts.next().unwrap_or("");
    let parsed_args_result = if !args_str.is_empty() { parse_gcode_arguments(args_str) } else { Ok(ParsedArgs::new()) };

    match parsed_args_result {
        Ok(args) => {
            let cmd_result : Result<(), &'static str> = if command == "PING" { /* ... */ Ok(()) }
            else if command == "ID" { /* ... */ Ok(()) }
            else if command == "ECHO" { /* ... */ Ok(()) }
            else if command == "SET_PIN" { /* ... */ Ok(()) }
            else if command == "GET_PIN" { /* ... */ Ok(()) }
            else if command == "QUERY_ADC" { /* ... */ Ok(()) }
            else if command == "SET_PWM" { /* ... */ Ok(()) }
            else if command == "MOVE" { /* ... */ Ok(()) }
            else if command == "HOMING_MOVE" {
                let x_vel_opt = args.get(&'X').and_then(|v| match v { CommandArgValue::Float(f) => Some(f.abs()), CommandArgValue::Integer(i) => Some(i.abs() as f32), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                let y_vel_opt = args.get(&'Y').and_then(|v| match v { CommandArgValue::Float(f) => Some(f.abs()), CommandArgValue::Integer(i) => Some(i.abs() as f32), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                let accel_opt = args.get(&'A').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });

                if (x_vel_opt.is_some() && y_vel_opt.is_some()) || (x_vel_opt.is_none() && y_vel_opt.is_none()) {
                    return Err("HOMING_MOVE requires exactly one axis (X or Y)");
                }
                if accel_opt.is_none() { return Err("HOMING_MOVE requires A (acceleration)"); }

                let accel = accel_opt.unwrap();
                let kinematics = CartesianKinematics::new(80.0, 80.0); // 80 steps/mm

                interrupt_free(|cs| {
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

                        // Make up a very long move to ensure we hit the endstop
                        const HOMING_STEPS: u32 = 400 * 80; // 400mm worth of steps

                        if let Some(x_vel) = x_vel_opt {
                            // Home X axis
                            if stepper_x.endstop.is_none() { return Err("No endstop configured for X axis"); }
                            stepper_x.set_direction(StepperDirection::CounterClockwise, gpio_manager)?; // Assume homing towards negative

                            let move_params = TrapezoidalMove {
                                total_steps: HOMING_STEPS,
                                acceleration: kinematics.steps_per_mm_x * accel,
                                start_velocity: 0.0,
                                cruise_velocity: kinematics.steps_per_mm_x * x_vel,
                                homing: true, // This is a homing move
                            };
                            let move_plan = MovePlanner::new(&move_params, *clock_freq)?;

                            stepper_x.current_move = Some(move_plan);
                            stepper_x.current_step_num = 0;
                            stepper_x.is_pulsing_high = false;
                            stepper_x.step_period_ticks = move_plan.initial_period_ticks;
                            stepper_x.bresenham_increment = HOMING_STEPS;
                            stepper_x.bresenham_decrement = 2 * HOMING_STEPS;
                            stepper_x.bresenham_error = (stepper_x.bresenham_decrement / 2) as i32;

                            let now = scheduler.read_time();
                            stepper_x.next_step_waketime = now.wrapping_add(100);
                            scheduler.schedule_task(stepper_x.timer_id_for_scheduler, stepper_x.next_step_waketime);

                            info!("Homing X axis...");
                            Ok(())
                        } else if let Some(y_vel) = y_vel_opt {
                            // Home Y axis
                            if stepper_y.endstop.is_none() { return Err("No endstop configured for Y axis"); }
                            stepper_y.set_direction(StepperDirection::CounterClockwise, gpio_manager)?;

                            let move_params = TrapezoidalMove {
                                total_steps: HOMING_STEPS,
                                acceleration: kinematics.steps_per_mm_y * accel,
                                start_velocity: 0.0,
                                cruise_velocity: kinematics.steps_per_mm_y * y_vel,
                                homing: true,
                            };
                            let move_plan = MovePlanner::new(&move_params, *clock_freq)?;

                            stepper_y.current_move = Some(move_plan);
                            stepper_y.current_step_num = 0;
                            stepper_y.is_pulsing_high = false;
                            stepper_y.step_period_ticks = move_plan.initial_period_ticks;
                            stepper_y.bresenham_increment = HOMING_STEPS;
                            stepper_y.bresenham_decrement = 2 * HOMING_STEPS;
                            stepper_y.bresenham_error = (stepper_y.bresenham_decrement / 2) as i32;

                            let now = scheduler.read_time();
                            stepper_y.next_step_waketime = now.wrapping_add(100);
                            scheduler.schedule_task(stepper_y.timer_id_for_scheduler, stepper_y.next_step_waketime);

                            info!("Homing Y axis...");
                            Ok(())
                        } else { unreachable!(); }
                    } else { Err("A required resource is not initialized") }
                })
            }
            else if command.is_empty() { Ok(()) }
            else { Err("Unknown command") }
        }
        Err(e) => { Err("Argument parsing failed") }
    };
    // ... (handle cmd_result and send response) ...
}
