// ... (all `use` statements and globals as before, including tmc2209) ...
use klipper_mcu_lib::tmc2209::TMC2209;

// --- Globals ---
// ...
static TMC2209_DRIVER: TMC2209 = TMC2209::new(); // ZST constructor

// --- process_command ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    // ... (logic to split command and parse args as before) ...
    match parsed_args_result {
        Ok(args) => {
            // ... (other commands) ...
            else if command == "TMC2209_WRITE" {
                let reg_opt = args.get(&'R').and_then(|v| match v { CommandArgValue::UInteger(u) => Some(*u as u8), _ => None });
                let val_opt = args.get(&'V').and_then(|v| match v { CommandArgValue::UInteger(u) => Some(*u), _ => None });
                // Slave address 'A' can be added later, default to 0 for now.

                if let (Some(reg_addr), Some(value)) = (reg_opt, val_opt) {
                    let write_result = interrupt_free(|cs| {
                        let mut uart_opt = UART0_PERIPHERAL.borrow(cs).borrow_mut();
                        if let Some(ref mut uart) = uart_opt.as_mut() {
                            TMC2209_DRIVER.write_register(uart, 0, reg_addr, value)
                                .map_err(|e| { error!("TMC Write Error: {:?}", e); "TMC Write Error" })
                        } else { Err("UART0 not initialized") }
                    });
                    if write_result.is_ok() { write!(response, "ok\r\n").unwrap(); }
                    else { write!(response, "Error: {}\r\n", write_result.unwrap_err()).unwrap(); }
                } else {
                    write!(response, "Error: Missing R (register) or V (value) for TMC2209_WRITE\r\n").unwrap();
                }
            }
            else if command == "TMC2209_READ" {
                 let reg_opt = args.get(&'R').and_then(|v| match v { CommandArgValue::UInteger(u) => Some(*u as u8), _ => None });
                 if let Some(reg_addr) = reg_opt {
                    // This is the complex single-wire read operation.
                    // 1. Send read request
                    let send_res = interrupt_free(|cs| {
                        let mut uart_opt = UART0_PERIPHERAL.borrow(cs).borrow_mut();
                        if let Some(ref mut uart) = uart_opt.as_mut() {
                            TMC2209_DRIVER.send_read_request(uart, 0, reg_addr)
                                .map_err(|e| { error!("TMC Read Request Error: {:?}", e); "TMC Read Request Error" })
                        } else { Err("UART0 not initialized") }
                    });

                    if send_res.is_err() {
                        write!(response, "Error: {}\r\n", send_res.unwrap_err()).unwrap();
                    } else {
                        // 2. Reconfigure pin and read response.
                        // This part is highly complex and requires deconstructing/reconstructing the UART peripheral
                        // to gain temporary GPIO access to the TX pin for reading.
                        // This is a placeholder for that logic.
                        warn!("TMC2209_READ: Sent read request for reg {}. Manual pin-swapping and read logic is not yet implemented.", reg_addr);
                        write!(response, "ok (read request sent, response reading not implemented)\r\n").unwrap();
                    }
                 } else {
                    write!(response, "Error: Missing R (register) for TMC2209_READ\r\n").unwrap();
                 }
            }
            // ... (other commands) ...
        }
        // ... (error handling) ...
    }
    serial_write_line(serial, response.as_str());
}
// ... (rest of file) ...
