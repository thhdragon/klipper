// ... (all `use` statements and globals as before, including `use klipper_mcu_lib::hal::{KlipperSpi, SpiError};`) ...

// --- Callbacks and Dispatchers ---
// ... (as before) ...

// --- Entry Point & Main Loop ---
#[entry]
fn main() -> ! {
    // ... (setup as before) ...
    loop { /* Main loop body as before */ }
}

// --- Interrupt Handlers ---
// ... (as before) ...

// --- Helper Functions ---
fn hex_str_to_bytes(hex_str: &str) -> Result<heapless::Vec<u8, 64>, &'static str> {
    let mut bytes = heapless::Vec::new();
    if hex_str.len() % 2 != 0 { return Err("Hex string must have an even number of characters"); }
    for i in (0..hex_str.len()).step_by(2) {
        let byte_str = &hex_str[i..i+2];
        let byte = u8::from_str_radix(byte_str, 16).map_err(|_| "Invalid hex character in WRITE data")?;
        if bytes.push(byte).is_err() { return Err("WRITE data exceeds maximum length (64 bytes)"); }
    }
    Ok(bytes)
}
// ... (other helpers: get_pwm_slice_channel_for_pin, calculate_pwm_settings, serial_write_line) ...


// --- process_command (with DEBUG_SPI command) ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    let mut parts = line.trim().splitn(2, |c: char| c.is_whitespace());
    let command = parts.next().unwrap_or("").to_ascii_uppercase();
    let args_str = parts.next().unwrap_or("");
    let parsed_args_result = if !args_str.is_empty() { parse_gcode_arguments(args_str) } else { Ok(ParsedArgs::new()) };

    let mut response = heapless::String::<192>::new(); // Buffer for responses
    use core::fmt::Write;

    match parsed_args_result {
        Ok(args) => {
            let cmd_result : Result<(), &'static str> = if command == "PING" { /* ... */ Ok(()) }
            else if command == "ID" { write!(response, "KlipperRustRP2040_SPI_v1\r\n").unwrap(); Ok(()) }
            // ... (other commands: ECHO, SET_PIN, GET_PIN, QUERY_ADC, SET_PWM, MOVE) ...
            else if command == "DEBUG_SPI" {
                let write_hex_opt = args.get(&'W').and_then(|val| match val { CommandArgValue::String(s) => Some(s.clone()), _ => None });
                let read_len_opt = args.get(&'R').and_then(|val| match val { CommandArgValue::UInteger(u) => Some(*u as usize), _ => None });
                let cs_pin_opt = args.get(&'C').and_then(|val| match val { CommandArgValue::UInteger(u) => Some(*u as u8), _ => None });

                if let Some(write_hex_str) = write_hex_opt {
                    interrupt_free(|cs| {
                        let mut spi_opt = SPI0_PERIPHERAL.borrow(cs).borrow_mut();
                        let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
                        if let (Some(ref mut spi), Some(ref mut manager)) = (spi_opt.as_mut(), gpio_manager_opt.as_mut()) {

                            // 1. Acquire and assert CS pin if provided
                            let mut cs_pin_typed : Option<Pin<_, PushPullOutput>> = None;
                            if let Some(cs_pin_num) = cs_pin_opt {
                                // Match on cs_pin_num to take and configure the correct pin
                                // This is verbose but safe. Example for GPIO17.
                                if cs_pin_num == 17 {
                                    if let Some(pin) = manager.Gpio17.take() {
                                        let mut pin_out = pin.into_push_pull_output();
                                        let _ = pin_out.set_low(); // Assert CS
                                        cs_pin_typed = Some(pin_out);
                                    } else { return Err("CS Pin 17 is busy"); }
                                } else { return Err("Unsupported CS Pin"); }
                            }

                            // 2. Parse hex data
                            let mut data_buf = match hex_str_to_bytes(&write_hex_str) {
                                Ok(b) => b,
                                Err(e) => { // Release CS pin on parse error before returning
                                    if let Some(mut cs) = cs_pin_typed {
                                        let _ = cs.set_high();
                                        manager.Gpio17.replace(cs.into_mode());
                                    }
                                    return Err(e);
                                },
                            };

                            // 3. Perform SPI transaction
                            let spi_result = if let Some(read_len) = read_len_opt {
                                if data_buf.len() < read_len {
                                    if data_buf.resize_default(read_len).is_err() {
                                        return Err("Read length exceeds buffer capacity");
                                    }
                                }
                                spi.transfer(&mut data_buf[..read_len])
                                    .map(|read_slice| {
                                        write!(response, "READ_HEX: ").unwrap();
                                        for byte in read_slice {
                                            write!(response, "{:02X}", byte).unwrap();
                                        }
                                        Ok(())
                                    })
                                    .map_err(|_| "SPI Transfer Error")
                            } else {
                                spi.write(&data_buf).map_err(|_| "SPI Write Error")
                            };

                            // 4. De-assert and release CS pin
                            if let Some(mut cs) = cs_pin_typed {
                                let _ = cs.set_high();
                                manager.Gpio17.replace(cs.into_mode());
                            }

                            if spi_result.is_ok() {
                                write!(response, "ok\r\n").unwrap();
                                Ok(())
                            } else {
                                spi_result
                            }
                        } else { Err("SPI or GpioManager not initialized") }
                    })
                } else { Err("Missing required W (WRITE) argument") }
            }
            else if command.is_empty() { Ok(()) }
            else { Err("Unknown command") }
        }
        Err(e) => { Err("Argument parsing failed") }
    };

    if let Err(e) = cmd_result {
        response.clear();
        write!(response, "Error: {}\r\n", e).unwrap();
    }
    serial_write_line(serial, response.as_str());
}
