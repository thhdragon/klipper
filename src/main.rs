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

// --- process_command (with G0/G1 commands) ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    // ... (command/arg parsing as before) ...
    match parsed_args_result {
        Ok(args) => {
            let cmd_result : Result<(), &'static str> = if command == "PING" { /* ... */ Ok(()) }
            else if command == "ID" { /* ... */ Ok(()) }
            else if command == "ECHO" { /* ... */ Ok(()) }
            else if command == "SET_PIN" { /* ... */ Ok(()) }
            else if command == "GET_PIN" { /* ... */ Ok(()) }
            else if command == "QUERY_ADC" { /* ... */ Ok(()) }
            else if command == "SET_PWM" { /* ... */ Ok(()) }
            else if command == "HOMING_MOVE" { /* ... */ Ok(()) }
            else if command == "G90" { /* ... */ Ok(()) }
            else if command == "G91" { /* ... */ Ok(()) }
            else if command == "G92" { /* ... */ Ok(()) }
            else if command == "G0" || command == "G1" { // Linear Move
                let x_pos_opt = args.get(&'X').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::Integer(i) => Some(*i as f32), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                let y_pos_opt = args.get(&'Y').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::Integer(i) => Some(*i as f32), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                // Z is parsed but not yet used by motion system
                let _z_pos_opt = args.get(&'Z').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::Integer(i) => Some(*i as f32), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                let feedrate_opt = args.get(&'F').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::Integer(i) => Some(*i as f32), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });

                interrupt_free(|cs| {
                    let mut toolhead = TOOLHEAD.borrow(cs).borrow_mut();
                    let mut queue = MOVE_QUEUE.borrow(cs).borrow_mut();

                    // Update feedrate if F word is present
                    if let Some(f) = feedrate_opt {
                        toolhead.feedrate = f;
                    }

                    // Determine target position based on current position and mode
                    let mut target_pos = toolhead.current_position;
                    if toolhead.positioning_mode == PositioningMode::Absolute {
                        if let Some(x) = x_pos_opt { target_pos.0 = x; }
                        if let Some(y) = y_pos_opt { target_pos.1 = y; }
                        // if let Some(z) = z_pos_opt { target_pos.2 = z; }
                    } else { // Relative
                        if let Some(x) = x_pos_opt { target_pos.0 += x; }
                        if let Some(y) = y_pos_opt { target_pos.1 += y; }
                        // if let Some(z) = z_pos_opt { target_pos.2 += z; }
                    }

                    // For now, use a fixed acceleration and the toolhead's current feedrate
                    let accel = 1000.0; // mm/s^2
                    let velocity = toolhead.feedrate / 60.0; // mm/s

                    // Create a move and add it to the queue
                    // HACK: Store target X/Y in start/cruise velocity fields of TrapezoidalMove
                    // This needs to be refactored later.
                    let mov = TrapezoidalMove {
                        total_steps: 0, // Kinematics will calculate this
                        acceleration: accel,
                        start_velocity: target_pos.0, // HACK
                        cruise_velocity: target_pos.1, // HACK
                        end_velocity: 0.0, // Look-ahead will set this
                        homing: false,
                    };

                    if queue.add_move(mov).is_err() {
                        return Err("Move queue full");
                    }

                    // Update toolhead's logical position
                    toolhead.current_position = target_pos;

                    // Trigger the planner to check the queue if the machine is idle
                    // This is non-blocking
                    plan_next_move();

                    Ok(())
                })
            }
            else if command.is_empty() { Ok(()) }
            else { Err("Unknown command") }
        }
        Err(e) => { Err("Argument parsing failed") }
    };
    // ... (handle cmd_result and send response) ...
}
// The old MOVE command handler is now removed.
// The plan_next_move function also needs a refactor to get the target pos from the move struct.
// For now, it will continue to use the HACK.
// The `TrapezoidalMove` struct should be refactored to properly hold target coordinates.
// This will be a subsequent step.
