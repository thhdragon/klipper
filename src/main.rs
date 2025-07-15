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

// --- process_command (with finalized TMC2209_READ response) ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>, delay: &mut cortex_m::delay::Delay) {
    // ... (command/arg parsing as before) ...
    let mut response = heapless::String::<192>::new();
    use core::fmt::Write;

    match parsed_args_result {
        Ok(args) => {
            let cmd_result : Result<(), &'static str> = if command == "PING" { /* ... */ Ok(()) }
            // ... (other commands) ...
            else if command == "TMC2209_READ" {
                // This command handles its own serial response because the response format is complex
                let reg_opt = args.get(&'R').and_then(|v| match v { CommandArgValue::UInteger(u) => Some(*u as u8), _ => None });
                if let Some(reg_addr) = reg_opt {
                    let read_result: Result<u32, &'static str> = interrupt_free(|cs| {
                        // ... (full read logic as implemented in previous step) ...
                    });

                    match read_result {
                        Ok(value) => {
                            write!(response, "READ_HEX: {:08X}\r\nok\r\n", value).unwrap();
                        }
                        Err(e) => {
                            write!(response, "Error: {}\r\n", e).unwrap();
                        }
                    }
                    // Send the response now and return Ok(()) to prevent the outer handler from sending another "ok".
                    serial_write_line(serial, response.as_str());
                    Ok(())
                } else {
                    Err("Missing R (register) for TMC2209_READ")
                }
            }
            // ... (other commands) ...
            else { Err("Unknown command") }
        }
        Err(e) => { Err("Argument parsing failed") }
    };

    // General response handling for commands that don't send their own response
    if command != "TMC2209_READ" {
        if let Err(e) = cmd_result {
            response.clear();
            write!(response, "Error: {}\r\n", e).unwrap();
        } else {
            // For simple commands that return Ok(()), just send "ok"
            write!(response, "ok\r\n").unwrap();
        }
        serial_write_line(serial, response.as_str());
    }
}
