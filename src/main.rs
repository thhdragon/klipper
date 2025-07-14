// ... (all `use` statements and globals as before) ...

// --- Callbacks and Dispatchers ---
// ... (as before) ...

// --- NEW Motion Planning Logic ---
fn plan_next_move() {
    interrupt_free(|cs| {
        let mut scheduler_opt = SCHEDULER.borrow(cs).borrow_mut();
        let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
        let mut stepper_x_opt = STEPPER_X.borrow(cs).borrow_mut();
        let mut stepper_y_opt = STEPPER_Y.borrow(cs).borrow_mut();
        let mut queue_opt = MOVE_QUEUE.borrow(cs).borrow_mut();
        let sys_clk_opt = SYSTEM_CLOCK_FREQ.borrow(cs).borrow();

        if let (Some(ref mut scheduler), Some(ref mut gpio_manager), Some(ref mut stepper_x), Some(ref mut stepper_y), Some(ref mut queue), Some(clock_freq)) =
            (scheduler_opt.as_mut(), gpio_manager_opt.as_mut(), stepper_x_opt.as_mut(), stepper_y_opt.as_mut(), &mut *queue_opt, sys_clk_opt.as_ref()) {

            // 1. Check if a move is already active
            if stepper_x.current_move.is_some() {
                return; // Don't plan a new move if one is running
            }

            // 2. Get the next move from the queue
            let current_move_params = if let Some(mov) = queue.pop_current_move() {
                mov
            } else {
                return; // Queue is empty, nothing to do
            };

            // 3. Look ahead to the next move to determine junction velocity
            let next_move_params = queue.get_current_move(); // Peeks at the new front

            // 4. Calculate junction velocity (look-ahead logic)
            // SIMPLIFICATION: If there's a next move, decelerate to a small cornering velocity.
            // If this is the last move, decelerate to zero.
            let end_velocity = if next_move_params.is_some() {
                5.0 // mm/s - a fixed cornering velocity for now
            } else {
                0.0
            };

            // 5. Update the move parameters with the calculated end_velocity
            // This requires TrapezoidalMove to have an end_velocity field.
            // This will be added in the next step. For now, we assume it exists.
            let mut move_to_plan = current_move_params;
            // move_to_plan.end_velocity = end_velocity; // This line will be added in next step's refactor

            // 6. The rest of the logic (kinematics, dominant axis, move planner, stepper setup, schedule first step)
            // will be moved here from the old MOVE command handler.
            // This will be done in the "Refactor MOVE Command" step.
            trace!("plan_next_move: Planning to start move. End velocity will be {}.", end_velocity);

        }
    });
}


// --- Entry Point & Main Loop ---
// ... (as before) ...

// --- Interrupt Handlers ---
// ... (as before) ...

// --- Helper Functions ---
// ... (as before) ...

// --- process_command ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    // ... (as before) ...
    // The MOVE command will be refactored later to use the queue.
}
