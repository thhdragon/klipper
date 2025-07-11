// klipper_host_rust/src/gcode.rs
// Corresponds to klippy/gcode.py - G-code processing.

use std::collections::HashMap;
use crate::toolhead::ToolHead; // Assuming ToolHead is in the crate root or accessible

// Represents the current position of the toolhead
// In Klipper, this is often managed within the GCodeMove class or similar state holders.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coord {
    pub x: Option<f64>,
    pub y: Option<f64>,
    pub z: Option<f64>,
    pub e: Option<f64>,
}

impl Coord {
    pub fn new(x: Option<f64>, y: Option<f64>, z: Option<f64>, e: Option<f64>) -> Self {
        Coord { x, y, z, e }
    }

    // Helper to get a value or default to 0.0 if None.
    // Useful for calculations where an unspecified coordinate means no change (relative)
    // or a specific value (absolute, though usually taken from last_position).
    fn get_or_default(&self, axis_val: Option<f64>) -> f64 {
        axis_val.unwrap_or(0.0)
    }
}

// State that GCode commands modify or use
#[derive(Debug, Clone, PartialEq)]
pub struct GCodeState {
    pub absolute_coord: bool,          // G90/G91
    pub absolute_extrude: bool,        // M82/M83
    pub base_position: Coord,          // Last commanded position in G92-based coordinates (raw values from G1 with G92 offsets)
    pub last_position: Coord,          // Actual machine position (base_position - gcode_offsets)
    // home_position: Coord,           // Position after homing - Klipper seems to set gcode_offset instead of this directly for G28
    pub speed: f64,                    // Current speed / feedrate F parameter (mm/min)
    pub speed_factor: f64,             // M220 speed factor override (S)
    pub extrude_factor: f64,           // M221 extrude factor override (S)
    // current_z: f64,                 // current_z in Klipper is last_position.z - gcode_offset.z
    // pub virtual_sdcard: Option<VirtualSD>, // TODO: If/when SD card support is added
    // pub move_with_transform: bool, // TODO: If/when bed_mesh or other transforms are added
    // Offsets applied by G92. When G92 X10 is called and current X is 50, offset becomes 40.
    // New position = G92_value - offset. So, Machine_pos = GCode_pos - G92_offset
    pub gcode_offset: Coord,
}

impl GCodeState {
    pub fn new() -> Self {
        GCodeState {
            absolute_coord: true,
            absolute_extrude: true, // Klipper's default is true via M82 in [printer] usually
            base_position: Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)),
            last_position: Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)),
            // home_position: Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)),
            speed: 25.0 * 60.0, // Default speed from Klipper gcode.py (25mm/s * 60 = 1500mm/min)
            speed_factor: 1.0,  // Factor of 1.0 means 100%
            extrude_factor: 1.0, // Factor of 1.0 means 100%
            // current_z: 0.0, // Removed, use last_position.z
            gcode_offset: Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)),
        }
    }
}

// Represents a parsed G-code command
#[derive(Debug, PartialEq)]
pub struct GCodeCommand<'a> {
    pub command_letter: char, // 'G', 'M', 'T', etc.
    pub command_number: f64,  // Numerical part of the command, e.g., 1.0, 28.0, 104.0
    pub raw_line: &'a str,
    pub params: HashMap<char, f64>,
    // TODO: Add any other fields needed from Klipper's GCodeCommand if necessary
}


// The main G-Code processor
pub struct GCode<'a> {
    pub state: GCodeState,
    printer_name: String, // Just an example, Klipper has a printer object
    // Required for ToolHead methods, even if ToolHead itself is passed as arg to cmd_g0_g1
    _toolhead_lifetime_marker: std::marker::PhantomData<&'a mut ()>,
}

impl<'a> GCode<'a> {
    pub fn new(printer_name: String) -> Self {
        GCode {
            state: GCodeState::new(),
            printer_name,
            _toolhead_lifetime_marker: std::marker::PhantomData,
        }
    }

    // Parses a G-code line into a GCodeCommand struct
    pub fn parse_line<'line>(&self, line: &'line str) -> Result<GCodeCommand<'line>, CommandError> {
        let clean_line = line.split(';').next().unwrap_or("").trim();
        if clean_line.is_empty() {
            return Err(CommandError("Empty G-code line".to_string()));
        }

        let mut parts = clean_line.split_whitespace();
        let command_part = parts.next().ok_or_else(|| CommandError(format!("Missing command in line: {}", line)))?.to_uppercase();

        if command_part.len() < 1 || !command_part.is_ascii() { // Command can be just "G" or "M" if number is 0 and omitted
            return Err(CommandError(format!("Invalid command format: {}", command_part)));
        }

        let command_letter = command_part.chars().next().unwrap();
        if !command_letter.is_alphabetic() {
            return Err(CommandError(format!("Command must start with a letter: {}", command_part)));
        }

        let command_number_str = if command_part.len() > 1 { &command_part[1..] } else { "0" }; // Default to 0 if no number
        let command_number = command_number_str.parse::<f64>().map_err(|_| {
            CommandError(format!("Invalid command number: {} in {}", command_number_str, command_part))
        })?;


        let mut params = HashMap::new();
        for part in parts {
            if part.is_empty() {
                continue;
            }
            let param_char = part.chars().next().ok_or_else(|| CommandError(format!("Invalid parameter format: {}", part)))?.to_ascii_uppercase();
            if !param_char.is_alphabetic() {
                 return Err(CommandError(format!("Parameter must start with a letter: {}", part)));
            }

            if part.len() > 1 {
                let value_str = &part[1..];
                // Special handling for M109 S / M109 R without a value - Klipper specific
                if command_letter == 'M' && command_number == 109.0 && (param_char == 'S' || param_char == 'R') && value_str.is_empty() {
                    // Klipper's M109 S (no value) means "wait for temp previously set by M104"
                    // Klipper's M109 R (no value) means "wait for temp previously set by M104 R"
                    // We'll insert NaN to signify this special case, to be handled by M109 logic.
                    params.insert(param_char, f64::NAN);
                    continue;
                }

                let value = value_str.parse::<f64>().map_err(|_| {
                    CommandError(format!("Invalid parameter value for {}: {}", param_char, value_str))
                })?;
                params.insert(param_char, value);
            } else {
                // Flag parameter like G28 X (no value for X, Y, Z means home that axis)
                // Insert NaN to signify a flag.
                params.insert(param_char, f64::NAN);
            }
        }

        Ok(GCodeCommand {
            command_letter,
            command_number,
            raw_line: line,
            params,
        })
    }

    // Helper to get a parameter value, returning None if not present or not a valid float
    fn get_float_param_opt(&self, params: &HashMap<char, f64>, key: char) -> Option<f64> {
        params.get(&key).copied().filter(|&v| !v.is_nan())
    }

    // Helper to check if a parameter flag is present (value is NaN)
    #[allow(dead_code)] // Will be used by G28 etc.
    fn get_flag_param(&self, params: &HashMap<char, f64>, key: char) -> bool {
        params.get(&key).map_or(false, |v| v.is_nan())
    }


    // Generic command dispatcher
    pub fn process_command(&mut self, cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        match cmd.command_letter {
            'G' => match cmd.command_number as i32 {
                0 | 1 => self.cmd_g0_g1(cmd, toolhead),
                4 => self.cmd_g4(cmd, toolhead),
                28 => self.cmd_g28(cmd, toolhead),
                90 => self.cmd_g90(cmd),
                91 => self.cmd_g91(cmd),
                92 => self.cmd_g92(cmd),
                _ => Err(CommandError(format!("Unknown G-code: G{}", cmd.command_number))),
            },
            'M' => match cmd.command_number as i32 {
                82 => self.cmd_m82(cmd),
                83 => self.cmd_m83(cmd),
                104 => self.cmd_m104(cmd, toolhead),
                106 => self.cmd_m106(cmd, toolhead), // Single entry
                107 => self.cmd_m107(cmd, toolhead), // Single entry
                106 => self.cmd_m106(cmd, toolhead),
                107 => self.cmd_m107(cmd, toolhead),
                109 => self.cmd_m109(cmd, toolhead),
                114 => self.cmd_m114(cmd, toolhead),
                140 => self.cmd_m140(cmd, toolhead),
                190 => self.cmd_m190(cmd, toolhead),
                // TODO: Add other M-codes
                _ => Err(CommandError(format!("Unknown M-code: M{}", cmd.command_number))),
            },
            _ => Err(CommandError(format!("Unsupported command type: {}{}", cmd.command_letter, cmd.command_number))),
        }
    }

    // G0/G1: Move
    fn cmd_g0_g1(&mut self, cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        if let Some(f_val) = self.get_float_param_opt(&cmd.params, 'F') {
            self.state.speed = f_val; // F value is typically in mm/min
        }

        let mut target_pos = self.state.last_position; // Start with current known position
        let mut new_base_pos = self.state.base_position; // For G92-relative calculations

        let cur_x = self.state.last_position.x.unwrap_or(0.0);
        let cur_y = self.state.last_position.y.unwrap_or(0.0);
        let cur_z = self.state.last_position.z.unwrap_or(0.0);
        let cur_e = self.state.last_position.e.unwrap_or(0.0);

        let gcode_off_x = self.state.gcode_offset.x.unwrap_or(0.0);
        let gcode_off_y = self.state.gcode_offset.y.unwrap_or(0.0);
        let gcode_off_z = self.state.gcode_offset.z.unwrap_or(0.0);
        let gcode_off_e = self.state.gcode_offset.e.unwrap_or(0.0);

        if self.state.absolute_coord {
            target_pos.x = self.get_float_param_opt(&cmd.params, 'X').map(|v| v - gcode_off_x).or(target_pos.x);
            target_pos.y = self.get_float_param_opt(&cmd.params, 'Y').map(|v| v - gcode_off_y).or(target_pos.y);
            target_pos.z = self.get_float_param_opt(&cmd.params, 'Z').map(|v| v - gcode_off_z).or(target_pos.z);

            new_base_pos.x = self.get_float_param_opt(&cmd.params, 'X').or(self.state.base_position.x);
            new_base_pos.y = self.get_float_param_opt(&cmd.params, 'Y').or(self.state.base_position.y);
            new_base_pos.z = self.get_float_param_opt(&cmd.params, 'Z').or(self.state.base_position.z);
        } else { // Relative coordinates
            target_pos.x = Some(cur_x + self.get_float_param_opt(&cmd.params, 'X').unwrap_or(0.0));
            target_pos.y = Some(cur_y + self.get_float_param_opt(&cmd.params, 'Y').unwrap_or(0.0));
            target_pos.z = Some(cur_z + self.get_float_param_opt(&cmd.params, 'Z').unwrap_or(0.0));

            new_base_pos.x = Some(self.state.base_position.x.unwrap_or(0.0) + self.get_float_param_opt(&cmd.params, 'X').unwrap_or(0.0));
            new_base_pos.y = Some(self.state.base_position.y.unwrap_or(0.0) + self.get_float_param_opt(&cmd.params, 'Y').unwrap_or(0.0));
            new_base_pos.z = Some(self.state.base_position.z.unwrap_or(0.0) + self.get_float_param_opt(&cmd.params, 'Z').unwrap_or(0.0));
        }

        if self.state.absolute_extrude {
            target_pos.e = self.get_float_param_opt(&cmd.params, 'E').map(|v| v - gcode_off_e).or(target_pos.e);
            new_base_pos.e = self.get_float_param_opt(&cmd.params, 'E').or(self.state.base_position.e);
        } else { // Relative extrusion
            target_pos.e = Some(cur_e + self.get_float_param_opt(&cmd.params, 'E').unwrap_or(0.0));
            new_base_pos.e = Some(self.state.base_position.e.unwrap_or(0.0) + self.get_float_param_opt(&cmd.params, 'E').unwrap_or(0.0));
        }

        // Klipper's toolhead.move takes [x, y, z, e] and speed (mm/sec)
        // Our speed is in mm/min, so convert.
        let speed_mm_s = self.state.speed / 60.0 * self.state.speed_factor;

        // Ensure all target coordinates are Some, defaulting to current if not specified
        let final_x = target_pos.x.unwrap_or(cur_x);
        let final_y = target_pos.y.unwrap_or(cur_y);
        let final_z = target_pos.z.unwrap_or(cur_z);
        let final_e = target_pos.e.unwrap_or(cur_e);

        toolhead.move_to([final_x, final_y, final_z, final_e], speed_mm_s)
            .map_err(|e| CommandError(format!("Toolhead move error: {}", e)))?;

        self.state.last_position = Coord::new(Some(final_x), Some(final_y), Some(final_z), Some(final_e));
        self.state.base_position = new_base_pos;

        Ok(())
    }

    // G90: Absolute Coordinates
    fn cmd_g90(&mut self, _cmd: &GCodeCommand) -> Result<(), CommandError> {
        self.state.absolute_coord = true;
        Ok(())
    }

    // G91: Relative Coordinates
    fn cmd_g91(&mut self, _cmd: &GCodeCommand) -> Result<(), CommandError> {
        self.state.absolute_coord = false;
        Ok(())
    }

    // G92: Set Position
    // G92 X10 Y20 ; set current X to 10, Y to 20
    // G92 ; reset all axis offsets to zero (equivalent to G92 X0 Y0 Z0 E0 if current pos is 0,0,0,0)
    // G92 E0 ; reset only extruder offset
    fn cmd_g92(&mut self, cmd: &GCodeCommand) -> Result<(), CommandError> {
        let mut new_offsets = self.state.gcode_offset;
        let mut new_base_pos = self.state.base_position;

        let current_mpos_x = self.state.last_position.x.unwrap_or(0.0);
        let current_mpos_y = self.state.last_position.y.unwrap_or(0.0);
        let current_mpos_z = self.state.last_position.z.unwrap_or(0.0);
        let current_mpos_e = self.state.last_position.e.unwrap_or(0.0);

        if cmd.params.is_empty() { // G92 without params resets all offsets
            new_offsets = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));
            new_base_pos = self.state.last_position; // base_pos becomes current machine pos
        } else {
            if let Some(x_val) = self.get_float_param_opt(&cmd.params, 'X') {
                new_offsets.x = Some(current_mpos_x - x_val);
                new_base_pos.x = Some(x_val);
            }
            if let Some(y_val) = self.get_float_param_opt(&cmd.params, 'Y') {
                new_offsets.y = Some(current_mpos_y - y_val);
                new_base_pos.y = Some(y_val);
            }
            if let Some(z_val) = self.get_float_param_opt(&cmd.params, 'Z') {
                new_offsets.z = Some(current_mpos_z - z_val);
                new_base_pos.z = Some(z_val);
            }
            if let Some(e_val) = self.get_float_param_opt(&cmd.params, 'E') {
                new_offsets.e = Some(current_mpos_e - e_val);
                new_base_pos.e = Some(e_val);
            }
        }
        self.state.gcode_offset = new_offsets;
        self.state.base_position = new_base_pos;

        // Determine which axes were explicitly mentioned in G92 for updating kinematics limits
        let mut homed_axes_mask = [false; 3];
        let mut g92_had_xyz_params = false;
        if cmd.params.contains_key(&'X') { homed_axes_mask[0] = true; g92_had_xyz_params = true; }
        if cmd.params.contains_key(&'Y') { homed_axes_mask[1] = true; g92_had_xyz_params = true; }
        if cmd.params.contains_key(&'Z') { homed_axes_mask[2] = true; g92_had_xyz_params = true; }

        // If G92 had no X, Y, or Z params, it resets all offsets.
        // In this case, all axes are effectively having their coordinate system defined
        // relative to the current machine position. So, mark all as "homed" for kinematics limits.
        if !g92_had_xyz_params && cmd.params.is_empty() { // Check cmd.params.is_empty() to ensure it was a plain G92
            homed_axes_mask = [true, true, true];
        }
        // If G92 E0 was issued, g92_had_xyz_params would be false, but cmd.params wouldn't be empty.
        // In that case, we don't want to mark X,Y,Z as homed. So the mask remains [false,false,false].
        // This seems correct: G92 E0 should not affect XYZ homed status.

        // Prepare the full new G-code position for ToolHead
        let th_new_pos = [
            self.state.base_position.x.unwrap_or(0.0), // Default to 0 if not set, though G92 usually sets them
            self.state.base_position.y.unwrap_or(0.0),
            self.state.base_position.z.unwrap_or(0.0),
            self.state.base_position.e.unwrap_or(0.0),
        ];

        // Inform ToolHead and Kinematics about the new G-code coordinate system definition
        toolhead.set_position(th_new_pos, Some(homed_axes_mask));

        Ok(())
    }

    // G28: Home Axes
    fn cmd_g28(&mut self, cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        // In Klipper, G28 without params homes X, then Y, then Z by default,
        // or specific axes if provided (G28 X Z).
        // We'll use the constants defined in ToolHead for G-code positions after homing for now.
        // These would typically come from printer config ([stepper_x] position_endstop etc.)

        let home_x = self.get_flag_param(&cmd.params, 'X');
        let home_y = self.get_flag_param(&cmd.params, 'Y');
        let home_z = self.get_flag_param(&cmd.params, 'Z');

        let home_all = !home_x && !home_y && !home_z;

        let axes_to_home: Vec<(usize, f64)> = if home_all {
            vec![
                (0, toolhead::X_GCODE_POSITION_AFTER_HOMING), // Axis index, G-code position to set
                (1, toolhead::Y_GCODE_POSITION_AFTER_HOMING),
                (2, toolhead::Z_GCODE_POSITION_AFTER_HOMING),
            ]
        } else {
            let mut axes = Vec::new();
            if home_x { axes.push((0, toolhead::X_GCODE_POSITION_AFTER_HOMING)); }
            if home_y { axes.push((1, toolhead::Y_GCODE_POSITION_AFTER_HOMING)); }
            if home_z { axes.push((2, toolhead::Z_GCODE_POSITION_AFTER_HOMING)); }
            axes
        };

        if axes_to_home.is_empty() && !home_all {
            // G28 with parameters, but none are X, Y, or Z (e.g., G28 F100 - invalid by most standards)
            // Klipper's G28 with no X/Y/Z params means home all configured default axes.
            // If specific non-X/Y/Z params are given, it's usually an error or ignored for homing.
            // For simplicity, if specific axes are requested but none are X,Y,Z, we do nothing or error.
            // Let's do nothing for now, as Klipper might just ignore unknown flags for G28.
            return Ok(());
        }

        // Ensure moves are flushed before homing
        // toolhead.flush_moves(); // In Klipper, this is more complex. wait_moves() in perform_homing_move is enough for mock.

        for (axis_index, gcode_pos_to_set) in axes_to_home {
            let machine_pos_at_trigger = toolhead.perform_homing_move(axis_index)?;

            // Update GCodeState based on homing result (similar to G92)
            match axis_index {
                0 => {
                    self.state.gcode_offset.x = Some(machine_pos_at_trigger - gcode_pos_to_set);
                    self.state.base_position.x = Some(gcode_pos_to_set);
                    self.state.last_position.x = Some(machine_pos_at_trigger);
                }
                1 => {
                    self.state.gcode_offset.y = Some(machine_pos_at_trigger - gcode_pos_to_set);
                    self.state.base_position.y = Some(gcode_pos_to_set);
                    self.state.last_position.y = Some(machine_pos_at_trigger);
                }
                2 => {
                    self.state.gcode_offset.z = Some(machine_pos_at_trigger - gcode_pos_to_set);
                    self.state.base_position.z = Some(gcode_pos_to_set);
                    self.state.last_position.z = Some(machine_pos_at_trigger);
                }
                _ => unreachable!(),
            }

            // Update toolhead's internal commanded_pos to the new G-CODE coordinate
            let mut new_th_pos = toolhead.get_position();
            new_th_pos[axis_index] = gcode_pos_to_set;

            let mut homing_axes_flags = [false; 3];
            homing_axes_flags[axis_index] = true;
            toolhead.set_position(new_th_pos, Some(homing_axes_flags));
        }

        // Ensure moves are flushed after homing
        // toolhead.flush_moves();

        Ok(())
    }

    // M82: Use Absolute E distances
    fn cmd_m82(&mut self, _cmd: &GCodeCommand) -> Result<(), CommandError> {
        self.state.absolute_extrude = true;
        Ok(())
    }

    // M83: Use Relative E distances
    fn cmd_m83(&mut self, _cmd: &GCodeCommand) -> Result<(), CommandError> {
        self.state.absolute_extrude = false;
        Ok(())
    }

    // G4: Dwell
    // Klipper's G4 P specifies milliseconds.
    fn cmd_g4(&mut self, cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        let p_val_ms = self.get_float_param_opt(&cmd.params, 'P').unwrap_or(0.0);
        // Klipper's get_float with minval=0 ensures non-negative.
        // Our get_float_param_opt doesn't do that yet, so add a check.
        let delay_ms = if p_val_ms < 0.0 { 0.0 } else { p_val_ms };
        let delay_s = delay_ms / 1000.0;

        toolhead.dwell(delay_s).map_err(|e| CommandError(e))?; // Assuming toolhead.dwell might return Result in future
        Ok(())
    }

    // M104: Set Extruder Temperature (and continue)
    fn cmd_m104(&mut self, cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        let target_temp = self.get_float_param_opt(&cmd.params, 'S')
            .ok_or_else(|| CommandError("Missing S (temperature) parameter for M104".to_string()))?;

        // TODO: Handle T parameter for selecting extruder if multiple extruders are supported.
        // For now, assume the primary extruder.
        toolhead.extruder_heater.set_target_temp(target_temp);
        Ok(())
    }

    // M140: Set Bed Temperature (and continue)
    fn cmd_m140(&mut self, cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        let target_temp = self.get_float_param_opt(&cmd.params, 'S')
            .ok_or_else(|| CommandError("Missing S (temperature) parameter for M140".to_string()))?;

        toolhead.bed_heater.set_target_temp(target_temp);
        Ok(())
    }

    // Common logic for M109 and M190
    fn wait_for_temperature(
        &mut self,
        heater: &mut Heater, // Mutable reference to the specific heater
        toolhead_reactor: &mut dyn Reactor, // Reactor from ToolHead for pausing
        target_temp: f64,
        tolerance: f64, // e.g., PID_SETTLE_DELTA from Klipper
        wait_interval_s: f64, // How often to check, e.g., 1.0 second
    ) -> Result<(), CommandError> {
        // Ensure target temp is set on the heater object itself
        heater.set_target_temp(target_temp);

        if target_temp <= 0.0 { // Don't wait if turning off
            return Ok(());
        }

        println!(
            "Waiting for {} to reach {:.1}°C (current: {:.1}°C)...",
            heater.name, target_temp, heater.current_temp
        );

        loop {
            if heater.check_target_reached(tolerance) {
                println!("{} reached target temperature.", heater.name);
                break;
            }

            // Simulate reactor pause and time passing
            // In a real system, reactor.pause() would yield control.
            // For this mock, we'll use thread::sleep.
            // A real reactor.pause() would return the actual time paused.
            // Here, we assume it paused for exactly wait_interval_s.
            std::thread::sleep(std::time::Duration::from_millis((wait_interval_s * 1000.0) as u64));
            // TODO: Replace sleep with actual reactor.pause(eventtime + wait_interval_s)
            // and get eventtime from reactor to pass to update_current_temp.
            // For now, using fixed interval for simulation.

            heater.update_current_temp(wait_interval_s);

            // Optional: Print current temperature during wait, similar to M105 output
            // println!("{} current: {:.1}°C, target: {:.1}°C", heater.name, heater.current_temp, heater.target_temp);

            // TODO: Add check for printer shutdown state from reactor: toolhead_reactor.is_shutdown()
            // if toolhead_reactor.is_shutdown() {
            //     return Err(CommandError(format!("Printer shutdown while waiting for {}", heater.name)));
            // }
        }
        Ok(())
    }

    // M109: Wait for Extruder Temperature
    fn cmd_m109(&mut self, cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        // Klipper's M109:
        // - S<temp> sets target and waits.
        // - R<temp> sets target (for cooling) and waits.
        // - If no S or R, it waits for previously set M104 target.
        // - Klipper default tolerance for PID is PID_SETTLE_DELTA = 1.0 degree.
        let mut target_temp_opt: Option<f64> = None;

        if let Some(s_val) = self.get_float_param_opt(&cmd.params, 'S') {
            target_temp_opt = Some(s_val);
        }
        if let Some(r_val) = self.get_float_param_opt(&cmd.params, 'R') {
            // If S was also present, R might override or be for a different condition (cooling).
            // For simplicity, if S is set, we use S. If only R, use R.
            // Klipper's logic is: if S, target is S. If R, target is R. If both, it's more complex (usually S for heat, R for cool).
            // Let's assume if S is present, it's the primary target. If only R, R is target.
            if target_temp_opt.is_none() {
                target_temp_opt = Some(r_val);
            } else {
                // If S is already set, R might imply a different kind of wait (e.g. wait for temp to cool down to R *after* reaching S)
                // This is more complex than typically handled by simple M109 R.
                // Most firmwares: M109 Sxxx (heat and wait), M109 Rxxx (cool and wait).
                // For now, if S is given, it takes precedence. If only R, R is used.
                // If M109 T0 S200 R190 - Klipper waits for S200.
                // If M109 T0 R100 - Klipper waits for R100 (cooling).
                // Let's simplify: S param takes precedence. If no S, then R.
                // This is handled by the order of ifs above.
            }
        }

        let final_target_temp = match target_temp_opt {
            Some(t) => t,
            None => toolhead.extruder_heater.target_temp, // Wait for previously set M104 target
        };

        // TODO: Get tolerance from config, Klipper uses ~1-2 degrees for PID settle.
        let tolerance = 1.0;
        let wait_interval_s = 1.0; // Check every 1 second

        self.wait_for_temperature(
            &mut toolhead.extruder_heater,
            toolhead.get_reactor_mut(), // Need a way to get reactor mutably
            final_target_temp,
            tolerance,
            wait_interval_s,
        )
    }

    // M190: Wait for Bed Temperature
    fn cmd_m190(&mut self, cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        let mut target_temp_opt: Option<f64> = None;

        if let Some(s_val) = self.get_float_param_opt(&cmd.params, 'S') {
            target_temp_opt = Some(s_val);
        }
        if let Some(r_val) = self.get_float_param_opt(&cmd.params, 'R') {
            if target_temp_opt.is_none() {
                target_temp_opt = Some(r_val);
            }
        }

        let final_target_temp = match target_temp_opt {
            Some(t) => t,
            None => toolhead.bed_heater.target_temp, // Wait for previously set M140 target
        };

        let tolerance = 2.0; // Bed might have a slightly larger tolerance
        let wait_interval_s = 1.0;

         self.wait_for_temperature(
            &mut toolhead.bed_heater,
            toolhead.get_reactor_mut(),
            final_target_temp,
            tolerance,
            wait_interval_s,
        )
    }

    // M114: Get Current Position
    fn cmd_m114(&self, _cmd: &GCodeCommand, toolhead: &ToolHead<'a>) -> Result<(), CommandError> {
        let pos = toolhead.get_position(); // This is toolhead.commanded_pos, which are G-code coordinates

        // In Klipper, gcode.get_position() also returns gcode coordinates (base_position).
        // For M114, toolhead.get_position() is the primary source.
        // let gcode_coords = self.state.base_position;

        // Format similar to Klipper: "X:%.3f Y:%.3f Z:%.3f E:%.3f"
        // We don't have a direct gcmd.respond_raw yet, so we'll use println! for now.
        // This output will be visible in klippy_main.rs integration.
        println!(
            "M114: X:{:.3} Y:{:.3} Z:{:.3} E:{:.3}",
            pos[0], pos[1], pos[2], pos[3]
        );
        // Klipper also adds " Count X:%.3f Y:%.3f Z:%.3f" using gcode_coords.
        // Since toolhead.commanded_pos and gcode.state.base_position should be in sync
        // regarding G-code coordinates, we can omit the "Count" part for now or add it later
        // if we differentiate between toolhead's view and gcode_move's internal view.
        Ok(())
    }

    // M106: Set Fan Speed
    fn cmd_m106(&mut self, cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        // Klipper's M106 S<value> uses value from 0-255. Default is 255 if S is not specified.
        let s_value = self.get_float_param_opt(&cmd.params, 'S').unwrap_or(255.0);

        if s_value < 0.0 || s_value > 255.0 {
            return Err(CommandError(format!(
                "M106 S value {:.1} out of range (0-255)",
                s_value
            )));
        }

        let speed_ratio = s_value / 255.0;
        toolhead.part_cooling_fan.set_speed(speed_ratio as f32);
        Ok(())
    }

    // M107: Fan Off
    fn cmd_m107(&mut self, _cmd: &GCodeCommand, toolhead: &mut ToolHead<'a>) -> Result<(), CommandError> {
        toolhead.part_cooling_fan.turn_off();
        Ok(())
    }
}

// Basic error type for G-Code processing
#[derive(Debug, PartialEq)]
pub struct CommandError(String);

impl std::fmt::Display for CommandError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for CommandError {}

impl From<&str> for CommandError {
    fn from(s: &str) -> Self {
        CommandError(s.to_string())
    }
}
impl From<String> for CommandError {
    fn from(s: String) -> Self {
        CommandError(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reactor::Reactor; // Mock or simple reactor needed for ToolHead
    use crate::mcu::Mcu; // Mock Mcu
    use crate::configfile::Configfile; // Mock Configfile


    // Mock ToolHead for testing G-code commands that call it
    struct MockToolHead {
        last_move: Option<([f64; 4], f64)>,
    }
    impl MockToolHead {
        fn new() -> Self { MockToolHead { last_move: None } }
    }
    // Implement a dummy ToolHead for the lifetime 'a if needed, or adjust GCode struct not to need it directly
    // For now, let's assume ToolHead methods are simplified or don't require complex lifetime management for mocks
    impl<'a> ToolHead<'a> {
        // A simplified mock `move_to` for testing purposes.
        // This is not a full mock of the actual ToolHead struct but a stand-in.
        // To use this, you'd need to ensure the actual ToolHead struct can be created or mocked appropriately.
        // This is a bit of a hack for now. A proper mock setup would be more involved.
        #[allow(unused_variables)]
        pub fn mock_move_to(&mut self, pos: [f64; 4], speed: f64) -> Result<(), String> {
            // In a real mock, you might store pos and speed to assert later.
            // For now, just print or do nothing.
            // println!("MockToolHead: move_to {:?} at speed {}", pos, speed);
            // To use with the GCode tests that need a mutable ToolHead:
            // We can't easily swap out the real ToolHead::move_to with this one
            // unless we use traits and dependency injection for ToolHead in GCode.
            // The current GCode::cmd_g0_g1 directly calls toolhead.move_to.
            Ok(())
        }
    }
     // Minimal Reactor for ToolHead instantiation
    struct MockReactor;
    impl Reactor for MockReactor {
        fn monotonic(&self) -> f64 { 0.0 }
        fn register_timer(&mut self, _time: f64, _callback: Box<dyn FnMut(f64) -> Option<f64>>) -> usize { 0 }
        fn register_fd(&mut self, _fd: i32, _callback: Box<dyn FnMut(f64)>) -> usize {0}
        fn unregister_fd(&mut self, _handle_id: usize) {}
        fn unregister_timer(&mut self, _handle_id: usize) {}
        fn is_shutdown(&self) -> bool {false}
        fn run(&mut self) {}
        fn pause(&mut self, _waketime: f64) {}
        fn _check_timers(&mut self, _eventtime: f64, _idle: bool) {}
    }
     // Minimal Mcu for ToolHead instantiation
    struct MockMcu;
    impl Mcu for MockMcu {
        fn estimated_print_time(&self, _curtime: f64) -> f64 {0.0}
        // Other methods as needed by ToolHead, or ensure ToolHead mock doesn't call them
    }


    fn create_test_gcode() -> GCode<'static> { // Use 'static for simplicity in test setup
        GCode::new("TestPrinter".to_string())
    }

    // Helper to create a ToolHead instance for tests.
    // This is very basic and likely needs more setup for real ToolHead functionality.
    fn create_test_toolhead() -> ToolHead<'static> {
        let mut config = Configfile::new(None); // Assuming a basic constructor
        // Add minimal required config for ToolHead if any (e.g., max_velocity, max_accel)
        config.add_section("printer");
        config.set("printer", "max_velocity", "500");
        config.set("printer", "max_accel", "3000");
        // config.set("printer", "kinematics", "cartesian"); // This would trigger kinematic loading

        let reactor = Box::new(MockReactor); // This needs to be a stable reference
        let mcu = Box::new(MockMcu);

        // Hack: Use Box::leak to get a 'static reference. This is not ideal for general code.
        let static_reactor: &'static MockReactor = Box::leak(reactor);
        let static_mcu: &'static MockMcu = Box::leak(mcu);


        ToolHead::new(&config, static_reactor, vec![static_mcu]).unwrap()
    }


    #[test]
    fn gcode_state_initialization() {
        let state = GCodeState::new();
        assert_eq!(state.absolute_coord, true);
        assert_eq!(state.absolute_extrude, true);
        assert_eq!(state.speed, 1500.0);
        assert_eq!(state.base_position.x, Some(0.0));
        assert_eq!(state.gcode_offset.x, Some(0.0));
    }

    #[test]
    fn gcode_new_initialization() {
        let gcode_parser = create_test_gcode();
        assert_eq!(gcode_parser.printer_name, "TestPrinter");
        assert_eq!(gcode_parser.state.absolute_coord, true);
    }

    #[test]
    fn test_parse_line_simple_g1() {
        let gcode = create_test_gcode();
        let line = "G1 X10 Y20.5 Z0.2 F3000 E1.0";
        let cmd = gcode.parse_line(line).unwrap();
        assert_eq!(cmd.command_letter, 'G');
        assert_eq!(cmd.command_number, 1.0);
        assert_eq!(cmd.raw_line, line);
        assert_eq!(cmd.params.get(&'X'), Some(&10.0));
        assert_eq!(cmd.params.get(&'Y'), Some(&20.5));
        assert_eq!(cmd.params.get(&'Z'), Some(&0.2));
        assert_eq!(cmd.params.get(&'F'), Some(&3000.0));
        assert_eq!(cmd.params.get(&'E'), Some(&1.0));
    }

    #[test]
    fn test_parse_line_with_comment() {
        let gcode = create_test_gcode();
        let line = "G1 X10 Y20 ; this is a comment";
        let cmd = gcode.parse_line(line).unwrap();
        assert_eq!(cmd.command_letter, 'G');
        assert_eq!(cmd.command_number, 1.0);
        assert_eq!(cmd.params.get(&'X'), Some(&10.0));
        assert_eq!(cmd.params.get(&'Y'), Some(&20.0));
        assert!(cmd.params.get(&';').is_none());
    }

    #[test]
    fn test_parse_line_lowercase_command() {
        let gcode = create_test_gcode();
        let line = "g1 x10";
        let cmd = gcode.parse_line(line).unwrap();
        assert_eq!(cmd.command_letter, 'G');
        assert_eq!(cmd.command_number, 1.0);
        assert_eq!(cmd.params.get(&'X'), Some(&10.0));
    }

    #[test]
    fn test_parse_line_no_params() {
        let gcode = create_test_gcode();
        let line = "G28";
        let cmd = gcode.parse_line(line).unwrap();
        assert_eq!(cmd.command_letter, 'G');
        assert_eq!(cmd.command_number, 28.0); // Defaulted to 0 by parser logic for G (no num)
        assert!(cmd.params.is_empty());
    }

    #[test]
    fn test_parse_line_param_flag() { // e.g. G28 X Y
        let gcode = create_test_gcode();
        let line = "G28 X Y";
        let cmd = gcode.parse_line(line).unwrap();
        assert_eq!(cmd.command_letter, 'G');
        assert_eq!(cmd.command_number, 28.0);
        assert!(cmd.params.get(&'X').unwrap().is_nan());
        assert!(cmd.params.get(&'Y').unwrap().is_nan());
        assert_eq!(cmd.params.len(), 2);
    }


    #[test]
    fn test_parse_line_m_command() {
        let gcode = create_test_gcode();
        let line = "M104 S200";
        let cmd = gcode.parse_line(line).unwrap();
        assert_eq!(cmd.command_letter, 'M');
        assert_eq!(cmd.command_number, 104.0);
        assert_eq!(cmd.params.get(&'S'), Some(&200.0));
    }

    #[test]
    fn test_parse_line_empty_line() {
        let gcode = create_test_gcode();
        let line = "";
        assert_eq!(gcode.parse_line(line), Err(CommandError("Empty G-code line".to_string())));
    }

    #[test]
    fn test_parse_line_comment_only() {
        let gcode = create_test_gcode();
        let line = "; just a comment";
         assert_eq!(gcode.parse_line(line), Err(CommandError("Empty G-code line".to_string())));
    }

    #[test]
    fn test_parse_line_invalid_command() {
        let gcode = create_test_gcode();
        let line = "X10 Y20"; // No command
        assert!(gcode.parse_line(line).is_err());

        let line = "1G X10"; // Starts with number
        assert!(gcode.parse_line(line).is_err());
    }

    #[test]
    fn test_parse_line_invalid_param_value() {
        let gcode = create_test_gcode();
        let line = "G1 X10Y20"; // Missing space, Y20 becomes value for X
        // This should parse X as 10Y20 which is not a float.
        assert!(gcode.parse_line(line).is_err());


        let line = "G1 XABC";
        assert_eq!(gcode.parse_line(line), Err(CommandError("Invalid parameter value for X: ABC".to_string())));
    }
     #[test]
    fn test_parse_line_float_command_number() {
        let gcode = create_test_gcode();
        let line = "G1.0 X10"; // Marlin allows G1.0
        let cmd = gcode.parse_line(line).unwrap();
        assert_eq!(cmd.command_letter, 'G');
        assert_eq!(cmd.command_number, 1.0);
        assert_eq!(cmd.params.get(&'X'), Some(&10.0));

        let line = "M115.2"; // Some firmwares might have M codes with decimals
        let cmd = gcode.parse_line(line).unwrap();
        assert_eq!(cmd.command_letter, 'M');
        assert_eq!(cmd.command_number, 115.2);
    }

    #[test]
    fn test_parse_g_command_with_dot() {
        let gcode = create_test_gcode();
        let line = "G0.1 X10";
        let cmd = gcode.parse_line(line).unwrap();
        assert_eq!(cmd.command_letter, 'G');
        assert_eq!(cmd.command_number, 0.1);
        assert_eq!(cmd.params.get(&'X'), Some(&10.0));
    }

    // Tests for G90/G91
    #[test]
    fn test_cmd_g90() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        gcode.state.absolute_coord = false; // Start relative
        let cmd = gcode.parse_line("G90").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();
        assert!(gcode.state.absolute_coord);
    }

    #[test]
    fn test_cmd_g91() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        gcode.state.absolute_coord = true; // Start absolute
        let cmd = gcode.parse_line("G91").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();
        assert!(!gcode.state.absolute_coord);
    }

    // Tests for G0/G1
    #[test]
    fn test_cmd_g1_absolute() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead(); // Real toolhead for move_to
        gcode.state.absolute_coord = true;
        gcode.state.last_position = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));
        gcode.state.base_position = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));
        gcode.state.gcode_offset = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));


        let cmd = gcode.parse_line("G1 X10 Y20 Z5 E1 F1200").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();

        assert_eq!(gcode.state.speed, 1200.0);
        assert_eq!(gcode.state.last_position, Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(1.0)));
        assert_eq!(gcode.state.base_position, Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(1.0)));
        // toolhead.last_move should be Some(([10.0, 20.0, 5.0, 1.0], 1200.0 / 60.0))
    }

    #[test]
    fn test_cmd_g1_relative() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        gcode.state.absolute_coord = false; // Relative
        gcode.state.last_position = Coord::new(Some(5.0), Some(5.0), Some(1.0), Some(0.5));
        gcode.state.base_position = Coord::new(Some(5.0), Some(5.0), Some(1.0), Some(0.5));
        gcode.state.gcode_offset = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));


        let cmd = gcode.parse_line("G1 X2 Y3 Z1 E0.5 F600").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();

        assert_eq!(gcode.state.speed, 600.0);
        assert_eq!(gcode.state.last_position, Coord::new(Some(7.0), Some(8.0), Some(2.0), Some(1.0)));
        assert_eq!(gcode.state.base_position, Coord::new(Some(7.0), Some(8.0), Some(2.0), Some(1.0)));
    }

    #[test]
    fn test_cmd_g1_absolute_with_g92_offset() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        gcode.state.absolute_coord = true;
        // Machine is at X5, Y5. G92 X0 Y0 was issued.
        gcode.state.last_position = Coord::new(Some(5.0), Some(5.0), Some(0.0), Some(0.0)); // Actual machine pos
        gcode.state.base_position = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)); // GCode pos
        gcode.state.gcode_offset  = Coord::new(Some(5.0), Some(5.0), Some(0.0), Some(0.0)); // Offset = machine - gcode

        // Move to GCode X10 Y10. Machine should move to X15 Y15.
        let cmd = gcode.parse_line("G1 X10 Y10 F1000").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();

        assert_eq!(gcode.state.last_position, Coord::new(Some(15.0), Some(15.0), Some(0.0), Some(0.0))); // Machine pos
        assert_eq!(gcode.state.base_position, Coord::new(Some(10.0), Some(10.0), Some(0.0), Some(0.0))); // GCode pos
    }


    // Tests for G92
    #[test]
    fn test_cmd_g92_set_specific_axes() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        gcode.state.last_position = Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(2.0)); // Current machine pos
        gcode.state.base_position = Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(2.0)); // Assume no prev G92
        gcode.state.gcode_offset = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));

        // Set current X to 100, Y to 50. Z and E are untouched by this G92.
        // Machine X is 10, new GCode X is 100. Offset X = 10 - 100 = -90.
        // Machine Y is 20, new GCode Y is 50.  Offset Y = 20 - 50 = -30.
        let cmd = gcode.parse_line("G92 X100 Y50").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();

        assert_eq!(gcode.state.gcode_offset, Coord::new(Some(-90.0), Some(-30.0), Some(0.0), Some(0.0)));
        assert_eq!(gcode.state.base_position, Coord::new(Some(100.0), Some(50.0), Some(5.0), Some(2.0))); // Z, E from old base
        assert_eq!(gcode.state.last_position, Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(2.0))); // Unchanged by G92
    }

    #[test]
    fn test_cmd_g92_reset_all_axes() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        gcode.state.last_position = Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(2.0));
        gcode.state.base_position = Coord::new(Some(100.0), Some(200.0), Some(50.0), Some(20.0)); // Previous G92 state
        gcode.state.gcode_offset = Coord::new(Some(-90.0), Some(-180.0), Some(-45.0), Some(-18.0));

        let cmd = gcode.parse_line("G92").unwrap(); // Reset all offsets
        gcode.process_command(&cmd, &mut toolhead).unwrap();

        assert_eq!(gcode.state.gcode_offset, Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)));
        // base_position should now reflect the actual machine coordinates
        assert_eq!(gcode.state.base_position, Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(2.0)));
        assert_eq!(gcode.state.last_position, Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(2.0))); // Unchanged
    }

    #[test]
    fn test_cmd_g92_e_only() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        gcode.state.last_position = Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(200.0));
        gcode.state.base_position = Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(200.0));
        gcode.state.gcode_offset = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));

        let cmd = gcode.parse_line("G92 E0").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();

        // Machine E is 200. G92 E0. Offset E = 200 - 0 = 200.
        assert_eq!(gcode.state.gcode_offset, Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(200.0)));
        assert_eq!(gcode.state.base_position, Coord::new(Some(10.0), Some(20.0), Some(5.0), Some(0.0)));
    }

    // Tests for G28
    #[test]
    fn test_cmd_g28_home_all() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        // Prime toolhead with a known G-code position that is NOT 0,0,0 to see G92 effects
        toolhead.set_position([10.0, 10.0, 10.0, 0.0], None);
        gcode.state.last_position = Coord::new(Some(10.0), Some(10.0), Some(10.0), Some(0.0)); // Machine pos
        gcode.state.base_position = Coord::new(Some(10.0), Some(10.0), Some(10.0), Some(0.0)); // GCode pos
        gcode.state.gcode_offset = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));   // No initial offset


        let cmd = gcode.parse_line("G28").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();

        // Assuming *_MACHINE_POSITION_AT_ENDSTOP are all 0.0
        // And *_GCODE_POSITION_AFTER_HOMING are all 0.0
        // Offset = machine_triggered_pos (0.0) - gcode_to_set (0.0) = 0.0
        // Base position (gcode) becomes 0.0
        // Last position (machine) becomes 0.0 (where it triggered)
        assert_eq!(gcode.state.gcode_offset, Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)));
        assert_eq!(gcode.state.base_position, Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)));
        assert_eq!(gcode.state.last_position, Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)));

        // Check toolhead's commanded position reflects the new G-code coordinates
        let th_pos = toolhead.get_position();
        assert_eq!(th_pos[0], toolhead::X_GCODE_POSITION_AFTER_HOMING);
        assert_eq!(th_pos[1], toolhead::Y_GCODE_POSITION_AFTER_HOMING);
        assert_eq!(th_pos[2], toolhead::Z_GCODE_POSITION_AFTER_HOMING);

        let kin_limits = toolhead.kin.get_axis_limits_for_test();
        assert_eq!(kin_limits[0], (0.0, 200.0), "X axis limits after G28 all");
        assert_eq!(kin_limits[1], (0.0, 200.0), "Y axis limits after G28 all");
        assert_eq!(kin_limits[2], (0.0, 180.0), "Z axis limits after G28 all");
    }

    #[test]
    fn test_cmd_g28_home_x_z() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        toolhead.set_position([10.0, 20.0, 30.0, 0.0], None);
        gcode.state.last_position = Coord::new(Some(10.0), Some(20.0), Some(30.0), Some(0.0));
        gcode.state.base_position = Coord::new(Some(10.0), Some(20.0), Some(30.0), Some(0.0));
        gcode.state.gcode_offset = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));

        let cmd = gcode.parse_line("G28 X Z").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();

        // X and Z should be homed (machine pos 0, gcode pos 0, offset 0)
        // Y should be untouched
        assert_eq!(gcode.state.gcode_offset, Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)));
        assert_eq!(gcode.state.base_position, Coord::new(Some(0.0), Some(20.0), Some(0.0), Some(0.0))); // Y base is original
        assert_eq!(gcode.state.last_position, Coord::new(Some(0.0), Some(20.0), Some(0.0), Some(0.0))); // Y last_pos is original

        let th_pos = toolhead.get_position();
        assert_eq!(th_pos[0], toolhead::X_GCODE_POSITION_AFTER_HOMING);
        assert_eq!(th_pos[1], 20.0); // Y unchanged
        assert_eq!(th_pos[2], toolhead::Z_GCODE_POSITION_AFTER_HOMING);

        let kin_limits = toolhead.kin.get_axis_limits_for_test();
        assert_eq!(kin_limits[0], (0.0, 200.0), "X axis limits after G28 X Z");
        assert_eq!(kin_limits[1], (1.0, -1.0), "Y axis limits should be unhomed after G28 X Z");
        assert_eq!(kin_limits[2], (0.0, 180.0), "Z axis limits after G28 X Z");
    }

    #[test]
    fn test_cmd_g28_home_y_with_initial_offset() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        // Machine Y is at 100. G92 Y50 was issued.
        // So, gcode Y is 50, machine Y is 100. Offset Y = 100 - 50 = 50.
        gcode.state.last_position = Coord::new(Some(10.0), Some(100.0), Some(5.0), Some(0.0)); // Machine pos
        gcode.state.base_position = Coord::new(Some(10.0), Some(50.0), Some(5.0), Some(0.0));  // GCode pos
        gcode.state.gcode_offset  = Coord::new(Some(0.0), Some(50.0), Some(0.0), Some(0.0));    // Offset for Y

        toolhead.set_position(gcode.state.base_position.x.unwrap_or_default(), /* ... */); // Ensure toolhead starts at GCODE 50 for Y
         let mut initial_th_pos = [0.0;4];
        initial_th_pos[0] = gcode.state.base_position.x.unwrap_or_default();
        initial_th_pos[1] = gcode.state.base_position.y.unwrap_or_default();
        initial_th_pos[2] = gcode.state.base_position.z.unwrap_or_default();
        initial_th_pos[3] = gcode.state.base_position.e.unwrap_or_default();
        toolhead.set_position(initial_th_pos, None);


        let cmd = gcode.parse_line("G28 Y").unwrap();
        gcode.process_command(&cmd, &mut toolhead).unwrap();

        // Y is homed. Assume Y_MACHINE_POSITION_AT_ENDSTOP = 0 and Y_GCODE_POSITION_AFTER_HOMING = 0.
        // New Y offset = machine_triggered_pos (0.0) - gcode_to_set (0.0) = 0.0.
        // New Y base_position (gcode) = 0.0.
        // New Y last_position (machine) = 0.0.
        // X, Z, E offsets and positions should be unchanged.
        assert_eq!(gcode.state.gcode_offset, Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0)));
        assert_eq!(gcode.state.base_position, Coord::new(Some(10.0), Some(0.0), Some(5.0), Some(0.0)));
        assert_eq!(gcode.state.last_position, Coord::new(Some(10.0), Some(0.0), Some(5.0), Some(0.0)));

        let th_pos = toolhead.get_position();
        assert_eq!(th_pos[0], 10.0); // X unchanged
        assert_eq!(th_pos[1], toolhead::Y_GCODE_POSITION_AFTER_HOMING);
        assert_eq!(th_pos[2], 5.0);  // Z unchanged
    }

    #[test]
    fn test_cmd_g92_updates_kinematics_limits() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();

        // Initial state: machine and gcode pos are 10,10,10. No offsets. Kinematics unhomed.
        gcode.state.last_position = Coord::new(Some(10.0), Some(10.0), Some(10.0), Some(0.0));
        gcode.state.base_position = Coord::new(Some(10.0), Some(10.0), Some(10.0), Some(0.0));
        gcode.state.gcode_offset = Coord::new(Some(0.0), Some(0.0), Some(0.0), Some(0.0));

        // Verify initial kinematics limits are unhomed
        // Accessing kin directly for test inspection. This is a bit of a hack.
        // In a real scenario, we might have a get_kinematics_status method.
        let initial_kin_limits = {
            let kin_ref = toolhead.get_kinematics_ref_for_test(); // Needs a temporary getter
            [kin_ref.limits[0], kin_ref.limits[1], kin_ref.limits[2]]
        };
        assert_eq!(initial_kin_limits[0], (1.0, -1.0));


        // G92 X0 Y0: current machine pos (10,10) is now g-code (0,0)
        // X offset = 10 - 0 = 10. Y offset = 10 - 0 = 10. Z offset unchanged.
        // X and Y kinematics limits should be set to full range. Z should remain unhomed.
        let cmd_g92_xy = gcode.parse_line("G92 X0 Y0").unwrap();
        gcode.process_command(&cmd_g92_xy, &mut toolhead).unwrap();

        assert_eq!(gcode.state.gcode_offset.x, Some(10.0));
        assert_eq!(gcode.state.gcode_offset.y, Some(10.0));
        assert_eq!(gcode.state.gcode_offset.z, Some(0.0)); // Z offset unchanged
        assert_eq!(gcode.state.base_position.x, Some(0.0));
        assert_eq!(gcode.state.base_position.y, Some(0.0));
        assert_eq!(gcode.state.base_position.z, Some(10.0)); // Z base_pos unchanged

        let kin_limits_after_g92xy = {
            let kin_ref = toolhead.get_kinematics_ref_for_test();
            [kin_ref.limits[0], kin_ref.limits[1], kin_ref.limits[2]]
        };
        assert_eq!(kin_limits_after_g92xy[0], (0.0, 200.0)); // X homed/defined
        assert_eq!(kin_limits_after_g92xy[1], (0.0, 200.0)); // Y homed/defined
        assert_eq!(kin_limits_after_g92xy[2], (1.0, -1.0));  // Z still unhomed


        // G92 (no params): Reset all offsets. Current machine pos (10,10,10) becomes g-code (10,10,10).
        // All axes (X,Y,Z) should now have their kinematics limits set to full range.
        let cmd_g92_none = gcode.parse_line("G92").unwrap();
        gcode.process_command(&cmd_g92_none, &mut toolhead).unwrap();

        assert_eq!(gcode.state.gcode_offset, Coord::new(Some(0.0),Some(0.0),Some(0.0),Some(0.0))); // All offsets 0
        assert_eq!(gcode.state.base_position, gcode.state.last_position); // Base = machine pos

        let kin_limits_after_g92_none = {
            let kin_ref = toolhead.get_kinematics_ref_for_test();
            [kin_ref.limits[0], kin_ref.limits[1], kin_ref.limits[2]]
        };
        assert_eq!(kin_limits_after_g92_none[0], (0.0, 200.0));
        assert_eq!(kin_limits_after_g92_none[1], (0.0, 200.0));
        assert_eq!(kin_limits_after_g92_none[2], (0.0, 180.0)); // Z now also defined


        // G92 E0: Only E offset should change. X,Y,Z kin limits should remain as they were.
        gcode.state.last_position.e = Some(50.0); // Simulate machine E pos
        gcode.state.base_position.e = Some(50.0);  // Gcode E pos
        gcode.state.gcode_offset.e = Some(0.0);   // E offset

        let cmd_g92_e0 = gcode.parse_line("G92 E0").unwrap();
        gcode.process_command(&cmd_g92_e0, &mut toolhead).unwrap();

        assert_eq!(gcode.state.gcode_offset.e, Some(50.0)); // E offset = 50 - 0 = 50
        assert_eq!(gcode.state.base_position.e, Some(0.0));

        let kin_limits_after_g92_e0 = {
            let kin_ref = toolhead.get_kinematics_ref_for_test();
            [kin_ref.limits[0], kin_ref.limits[1], kin_ref.limits[2]]
        };
        // Kinematics limits for X,Y,Z should be unchanged by G92 E0
        assert_eq!(kin_limits_after_g92_e0[0], (0.0, 200.0));
        assert_eq!(kin_limits_after_g92_e0[1], (0.0, 200.0));
        assert_eq!(kin_limits_after_g92_e0[2], (0.0, 180.0));
    }


    // Tests for M82/M83
    #[test]
    fn test_cmd_m82_sets_absolute_extrude() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead(); // Not strictly needed for M82/M83 but good practice for dispatcher
        gcode.state.absolute_extrude = false; // Start relative

        let cmd_m82 = gcode.parse_line("M82").unwrap();
        gcode.process_command(&cmd_m82, &mut toolhead).unwrap();
        assert!(gcode.state.absolute_extrude, "M82 should set absolute_extrude to true");
    }

    #[test]
    fn test_cmd_m83_sets_relative_extrude() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        gcode.state.absolute_extrude = true; // Start absolute

        let cmd_m83 = gcode.parse_line("M83").unwrap();
        gcode.process_command(&cmd_m83, &mut toolhead).unwrap();
        assert!(!gcode.state.absolute_extrude, "M83 should set absolute_extrude to false");
    }

    #[test]
    fn test_g1_extrusion_absolute_mode_m82() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();

        // Initial state: E=0, absolute extrusion mode
        gcode.state.last_position.e = Some(0.0);
        gcode.state.base_position.e = Some(0.0);
        gcode.state.gcode_offset.e = Some(0.0);
        let cmd_m82 = gcode.parse_line("M82").unwrap(); // Ensure absolute extrusion
        gcode.process_command(&cmd_m82, &mut toolhead).unwrap();

        // Move E to 10.0
        let cmd_g1_e10 = gcode.parse_line("G1 E10.0").unwrap();
        gcode.process_command(&cmd_g1_e10, &mut toolhead).unwrap();
        assert_eq!(gcode.state.last_position.e, Some(10.0));
        assert_eq!(gcode.state.base_position.e, Some(10.0));

        // Move E to 5.0 (retract)
        let cmd_g1_e5 = gcode.parse_line("G1 E5.0").unwrap();
        gcode.process_command(&cmd_g1_e5, &mut toolhead).unwrap();
        assert_eq!(gcode.state.last_position.e, Some(5.0));
        assert_eq!(gcode.state.base_position.e, Some(5.0));
    }

    #[test]
    fn test_g1_extrusion_relative_mode_m83() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();

        // Initial state: E=0, relative extrusion mode
        gcode.state.last_position.e = Some(0.0);
        gcode.state.base_position.e = Some(0.0);
        gcode.state.gcode_offset.e = Some(0.0); // G92 E offset should not affect relative E moves
        let cmd_m83 = gcode.parse_line("M83").unwrap(); // Ensure relative extrusion
        gcode.process_command(&cmd_m83, &mut toolhead).unwrap();

        // Extrude 10.0mm
        let cmd_g1_e10 = gcode.parse_line("G1 E10.0").unwrap();
        gcode.process_command(&cmd_g1_e10, &mut toolhead).unwrap();
        assert_eq!(gcode.state.last_position.e, Some(10.0)); // 0 + 10 = 10
        assert_eq!(gcode.state.base_position.e, Some(10.0));

        // Retract 2.0mm
        let cmd_g1_e_neg2 = gcode.parse_line("G1 E-2.0").unwrap();
        gcode.process_command(&cmd_g1_e_neg2, &mut toolhead).unwrap();
        assert_eq!(gcode.state.last_position.e, Some(8.0)); // 10 - 2 = 8
        assert_eq!(gcode.state.base_position.e, Some(8.0));
    }

    #[test]
    fn test_g1_extrusion_absolute_mode_with_g92_e_offset() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();

        // Initial state: Machine E is at 100. G92 E50 was issued. So G-code E is 50.
        gcode.state.last_position.e = Some(100.0); // Machine E
        gcode.state.base_position.e = Some(50.0);  // G-code E
        gcode.state.gcode_offset.e = Some(50.0); // Offset = Machine - G-code = 100 - 50 = 50

        let cmd_m82 = gcode.parse_line("M82").unwrap(); // Absolute extrusion
        gcode.process_command(&cmd_m82, &mut toolhead).unwrap();

        // Command G1 E60 (absolute G-code value)
        // Target machine E should be G-code E + offset = 60 + 50 = 110
        let cmd_g1_e60 = gcode.parse_line("G1 E60.0").unwrap();
        gcode.process_command(&cmd_g1_e60, &mut toolhead).unwrap();

        assert_eq!(gcode.state.last_position.e, Some(110.0), "Machine E position check"); // 100 (start) + 10 (gcode diff 60-50) = 110
        assert_eq!(gcode.state.base_position.e, Some(60.0), "G-code E base position check");
        assert_eq!(gcode.state.gcode_offset.e, Some(50.0), "G92 E offset should remain unchanged");
    }

    // Tests for G4
    #[test]
    fn test_cmd_g4_with_p_param() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        let initial_print_time = toolhead.get_last_move_time();

        let cmd_g4_p100 = gcode.parse_line("G4 P100").unwrap(); // 100 ms
        gcode.process_command(&cmd_g4_p100, &mut toolhead).unwrap();

        // dwell should advance print_time by 0.1 seconds
        let expected_print_time = initial_print_time + 0.1;
        assert!((toolhead.get_last_move_time() - expected_print_time).abs() < 1e-9,
                "Toolhead print_time should advance by 0.1s for G4 P100. Expected {}, got {}",
                expected_print_time, toolhead.get_last_move_time());
    }

    #[test]
    fn test_cmd_g4_no_param() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        let initial_print_time = toolhead.get_last_move_time();

        let cmd_g4 = gcode.parse_line("G4").unwrap();
        gcode.process_command(&cmd_g4, &mut toolhead).unwrap();

        // Default P value is 0, so no change in print_time
        assert!((toolhead.get_last_move_time() - initial_print_time).abs() < 1e-9,
                "Toolhead print_time should not change for G4 with no params. Expected {}, got {}",
                initial_print_time, toolhead.get_last_move_time());
    }

    #[test]
    fn test_cmd_g4_negative_p_param() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        let initial_print_time = toolhead.get_last_move_time();

        let cmd_g4_neg_p = gcode.parse_line("G4 P-100").unwrap();
        gcode.process_command(&cmd_g4_neg_p, &mut toolhead).unwrap();

        // Negative P should result in 0s dwell
        assert!((toolhead.get_last_move_time() - initial_print_time).abs() < 1e-9,
                "Toolhead print_time should not change for G4 with negative P. Expected {}, got {}",
                initial_print_time, toolhead.get_last_move_time());
    }

    #[test]
    fn test_cmd_g4_s_param_is_ignored() { // Klipper toolhead.cmd_G4 only uses P
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        let initial_print_time = toolhead.get_last_move_time();

        let cmd_g4_s = gcode.parse_line("G4 S2").unwrap(); // Should be ignored by cmd_G4
        gcode.process_command(&cmd_g4_s, &mut toolhead).unwrap();

        // S param is ignored, P defaults to 0, so no change in print_time
        assert!((toolhead.get_last_move_time() - initial_print_time).abs() < 1e-9,
                "Toolhead print_time should not change for G4 S2 as only P is considered. Expected {}, got {}",
                initial_print_time, toolhead.get_last_move_time());

        let cmd_g4_p50_s2 = gcode.parse_line("G4 P50 S2").unwrap();
        gcode.process_command(&cmd_g4_p50_s2, &mut toolhead).unwrap();
        let expected_print_time = initial_print_time + 0.05; // From P50
         assert!((toolhead.get_last_move_time() - expected_print_time).abs() < 1e-9,
                "Toolhead print_time should advance by 0.05s for G4 P50 S2. Expected {}, got {}",
                expected_print_time, toolhead.get_last_move_time());
    }

    // Tests for M104 / M140
    #[test]
    fn test_cmd_m104_set_extruder_temp() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();

        let cmd_m104 = gcode.parse_line("M104 S210.5").unwrap();
        gcode.process_command(&cmd_m104, &mut toolhead).unwrap();
        assert_eq!(toolhead.extruder_heater.target_temp, 210.5);

        // Test setting to 0
        let cmd_m104_off = gcode.parse_line("M104 S0").unwrap();
        gcode.process_command(&cmd_m104_off, &mut toolhead).unwrap();
        assert_eq!(toolhead.extruder_heater.target_temp, 0.0);
    }

    #[test]
    fn test_cmd_m104_missing_s_param() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        let cmd_m104_no_s = gcode.parse_line("M104").unwrap();
        let result = gcode.process_command(&cmd_m104_no_s, &mut toolhead);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), CommandError("Missing S (temperature) parameter for M104".to_string()));
    }

    #[test]
    fn test_cmd_m140_set_bed_temp() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();

        let cmd_m140 = gcode.parse_line("M140 S100.0").unwrap();
        gcode.process_command(&cmd_m140, &mut toolhead).unwrap();
        assert_eq!(toolhead.bed_heater.target_temp, 100.0);

        // Test setting to 0
        let cmd_m140_off = gcode.parse_line("M140 S0").unwrap();
        gcode.process_command(&cmd_m140_off, &mut toolhead).unwrap();
        assert_eq!(toolhead.bed_heater.target_temp, 0.0);
    }

    #[test]
    fn test_cmd_m140_missing_s_param() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        let cmd_m140_no_s = gcode.parse_line("M140").unwrap();
        let result = gcode.process_command(&cmd_m140_no_s, &mut toolhead);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), CommandError("Missing S (temperature) parameter for M140".to_string()));
    }

    // Tests for M109 / M190
    #[test]
    fn test_cmd_m109_wait_for_extruder_temp_s_param() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        toolhead.extruder_heater.current_temp = 25.0; // Start at ambient

        let cmd_m109 = gcode.parse_line("M109 S50").unwrap(); // Target 50C
        // Proportional heat rate, interval 0.5s.
        // Target diff starts at 25C.
        // Step 1: current=25, diff=25, change=25*0.2*0.5=2.5. new_curr=27.5
        // Step 2: current=27.5, diff=22.5, change=22.5*0.2*0.5=2.25. new_curr=29.75
        // ... This will take several steps.
        let start_time = std::time::Instant::now();
        gcode.process_command(&cmd_m109, &mut toolhead).unwrap();
        let duration = start_time.elapsed();

        assert_eq!(toolhead.extruder_heater.target_temp, 50.0);
        assert!(toolhead.extruder_heater.check_target_reached(1.0));
        // Verify it actually waited (took some time)
        assert!(duration.as_secs_f64() >= 0.5, "M109 S50 should take some time to wait. Duration: {:?}", duration);
    }

    #[test]
    fn test_cmd_m190_wait_for_bed_temp_r_param() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        toolhead.bed_heater.current_temp = 100.0;
        toolhead.bed_heater.target_temp = 100.0;

        let cmd_m190 = gcode.parse_line("M190 R60").unwrap();
        // Proportional cool rate, interval 0.5s.
        // Target diff starts at -40C.
        let start_time = std::time::Instant::now();
        gcode.process_command(&cmd_m190, &mut toolhead).unwrap();
        let duration = start_time.elapsed();

        assert_eq!(toolhead.bed_heater.target_temp, 60.0);
        assert!(toolhead.bed_heater.check_target_reached(2.0));
        assert!(duration.as_secs_f64() >= 0.5, "M190 R60 should take some time to wait. Duration: {:?}", duration);
    }

    #[test]
    fn test_cmd_m109_no_s_or_r_waits_for_m104_target() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        toolhead.extruder_heater.current_temp = 30.0;
        toolhead.extruder_heater.set_target_temp(35.0); // Target previously set by M104

        let cmd_m109 = gcode.parse_line("M109").unwrap(); // No S or R
        // Heat rate proportional, interval 0.5s. Diff = 5. Change = 5*0.2*0.5 = 0.5.
        // 30->30.5 ... will take a few steps.
        let start_time = std::time::Instant::now();
        gcode.process_command(&cmd_m109, &mut toolhead).unwrap();
        let duration = start_time.elapsed();

        assert_eq!(toolhead.extruder_heater.target_temp, 35.0);
        assert!(toolhead.extruder_heater.check_target_reached(1.0));
        assert!(duration.as_secs_f64() >= 0.5, "M109 (no params) wait duration out of expected range: {:?}", duration);
    }

    #[test]
    fn test_cmd_m109_target_zero_no_wait() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();
        toolhead.extruder_heater.current_temp = 200.0;
        toolhead.extruder_heater.set_target_temp(200.0);

        let cmd_m109_s0 = gcode.parse_line("M109 S0").unwrap();
        let start_time = std::time::Instant::now();
        gcode.process_command(&cmd_m109_s0, &mut toolhead).unwrap();
        let duration = start_time.elapsed();

        assert_eq!(toolhead.extruder_heater.target_temp, 0.0);
        assert!(duration.as_secs_f64() < 0.1, "M109 S0 should not wait. Duration: {:?}", duration);
    }

    // Test for M114
    #[test]
    fn test_cmd_m114_reports_current_position() {
        let mut gcode = create_test_gcode();
        let mut toolhead = create_test_toolhead();

        // Simulate some state
        let expected_pos = [10.1234, 20.5678, 5.8912, 100.1111];
        toolhead.commanded_pos = expected_pos; // Directly set for testing M114's source
        gcode.state.base_position = Coord { // Keep GCodeState somewhat consistent for conceptual accuracy
            x: Some(expected_pos[0]),
            y: Some(expected_pos[1]),
            z: Some(expected_pos[2]),
            e: Some(expected_pos[3]),
        };

        let cmd_m114 = gcode.parse_line("M114").unwrap();

        // We can't easily capture println! output in a standard unit test without external crates or complex setup.
        // So, we'll verify the command executes and conceptually it would print the right thing.
        // A more robust test would involve a mock "responder" passed to gcode commands.
        match gcode.process_command(&cmd_m114, &mut toolhead) {
            Ok(()) => {
                // Command executed, visual check of println! output during test run if enabled.
                // For automated check, we trust toolhead.get_position() is correct
                // and formatting is as shown in cmd_m114.
                let reported_pos = toolhead.get_position();
                assert_eq!(reported_pos[0], expected_pos[0]);
                assert_eq!(reported_pos[1], expected_pos[1]);
                assert_eq!(reported_pos[2], expected_pos[2]);
                assert_eq!(reported_pos[3], expected_pos[3]);
                // println! will format to 3 decimal places, e.g. X:10.123 Y:20.568 Z:5.891 E:100.111
            }
            Err(e) => panic!("cmd_M114 failed: {:?}", e),
        }
    }
}
