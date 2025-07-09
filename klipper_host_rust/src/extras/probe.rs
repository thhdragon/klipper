// klipper_host_rust/src/extras/probe.rs

use crate::configfile::{Configfile, ConfigError, ParseError};
use crate::klippy_main::Printer;
use crate::gcode::{CommandError, GCodeCommand, GCode}; // Added GCode
use crate::toolhead::{Toolhead}; // Added Toolhead
use crate::mcu::{Mcu, EndstopWrapper}; // Added Mcu and EndstopWrapper for pin interaction
use crate::pins::{PrinterPins, PinArgs, PinMapValue}; // Added for pin setup

use std::collections::HashMap;
use std::sync::{Arc, Weak}; // For Toolhead and other shared objects, Weak for PrinterProbe in GCode closure
use parking_lot::Mutex; // For Toolhead and other shared objects

// Define error types for the probe module
#[derive(Debug, Clone)]
pub enum ProbeError {
    ConfigError(String),
    CommandError(String),
    ProbeFailure(String),
}

impl From<ConfigError> for ProbeError {
    fn from(e: ConfigError) -> Self {
        ProbeError::ConfigError(e.to_string())
    }
}

impl From<ParseError> for ProbeError {
    fn from(e: ParseError) -> Self {
        ProbeError::ConfigError(format!("Parse error: {}", e))
    }
}

impl From<CommandError> for ProbeError {
    fn from(e: CommandError) -> Self {
        ProbeError::CommandError(e.to_string())
    }
}


#[derive(Debug, Clone)]
pub struct ProbeParams {
    pub pin_name: String, // Assuming pin name is crucial, like "probe:z_virtual_endstop" or a real pin
    pub x_offset: f64,
    pub y_offset: f64,
    pub z_offset: f64,
    pub speed: f64,
    pub lift_speed: f64,
    pub samples: i32,
    pub sample_retract_dist: f64,
    pub samples_result: String, // "average" or "median"
    pub samples_tolerance: f64,
    pub samples_tolerance_retries: i32,
    pub activate_gcode: Option<String>, // Using String for now, GCodeTemplate processing later
    pub deactivate_gcode: Option<String>, // Using String for now
    pub stow_on_each_sample: bool,
    // Basic pin parameters, more might be needed depending on pin handling
    pub invert_pin: bool,
    pub pullup_pin: bool,
}

impl ProbeParams {
    pub fn new(config: &mut Configfile) -> Result<Self, ProbeError> {
        let section = "probe";
        if !config.has_section(section) {
            return Err(ProbeError::ConfigError(format!("Config section '[{}]' not found", section)));
        }

        let pin_name = config.get(section, "pin")?;

        // Pin parsing logic might be more complex, e.g. handling `^!` prefixes
        // For now, assume it's just the name.
        // Real pin parsing would involve ppins.setup_pin
        let (final_pin_name, invert_pin, pullup_pin) = Self::parse_pin_name(&pin_name)?;


        let x_offset = config.getfloat(section, "x_offset", Some(0.0))?;
        let y_offset = config.getfloat(section, "y_offset", Some(0.0))?;
        let z_offset = config.getfloat(section, "z_offset", None)?; // z_offset is mandatory

        let default_speed = 5.0;
        let speed = config.getfloat(section, "speed", Some(default_speed))?;
        if speed <= 0.0 {
            return Err(ProbeError::ConfigError("'speed' in section [probe] must be positive".to_string()));
        }

        let lift_speed = config.getfloat(section, "lift_speed", Some(speed))?;
        if lift_speed <= 0.0 {
            return Err(ProbeError::ConfigError("'lift_speed' in section [probe] must be positive".to_string()));
        }

        let samples = config.getint(section, "samples", Some(1))?;
        if samples < 1 {
            return Err(ProbeError::ConfigError("'samples' in section [probe] must be at least 1".to_string()));
        }

        let sample_retract_dist = config.getfloat(section, "sample_retract_dist", Some(2.0))?;
        if sample_retract_dist <= 0.0 {
            // Klipper's code has 'above=0.' for this, implying strictly positive.
             return Err(ProbeError::ConfigError("'sample_retract_dist' in section [probe] must be positive".to_string()));
        }

        let samples_result = config.getchoice(section, "samples_result", &["average", "median"], Some("average"))?;
        let samples_tolerance = config.getfloat(section, "samples_tolerance", Some(0.100))?;
        if samples_tolerance < 0.0 {
            return Err(ProbeError::ConfigError("'samples_tolerance' in section [probe] must be non-negative".to_string()));
        }

        let samples_tolerance_retries = config.getint(section, "samples_tolerance_retries", Some(0))?;
        if samples_tolerance_retries < 0 {
             return Err(ProbeError::ConfigError("'samples_tolerance_retries' in section [probe] must be non-negative".to_string()));
        }

        // For activate_gcode and deactivate_gcode, we'll just get them as strings for now.
        // Actual GCodeTemplate parsing and execution is a larger task.
        let activate_gcode = config.get_template_str(section, "activate_gcode", Some("")).ok();
        let deactivate_gcode = config.get_template_str(section, "deactivate_gcode", Some("")).ok();

        let stow_on_each_sample = config.getboolean(section, "deactivate_on_each_sample", Some(true))?;


        Ok(Self {
            pin_name: final_pin_name,
            x_offset,
            y_offset,
            z_offset,
            speed,
            lift_speed,
            samples,
            sample_retract_dist,
            samples_result,
            samples_tolerance,
            samples_tolerance_retries,
            activate_gcode,
            deactivate_gcode,
            stow_on_each_sample,
            invert_pin,
            pullup_pin,
        })
    }

    fn parse_pin_name(pin_str: &str) -> Result<(String, bool, bool), ProbeError> {
        let mut name = pin_str.to_string();
        let mut invert = false;
        let mut pullup = false;

        if name.starts_with('!') {
            invert = true;
            name.remove(0);
        }
        if name.starts_with('^') {
            pullup = true;
            name.remove(0);
        }
        // Klipper allows repeated prefixes for error or specific hardware meaning,
        // but for now, we'll just check once.
        if name.starts_with('!') || name.starts_with('^') {
            // This check might be too simplistic. Klipper's C code might allow this for specific chip pins.
            // However, for virtual pins or general use, it's usually an error.
            // For probe:z_virtual_endstop, Klipper explicitly errors on pullup/invert.
            // Let's assume for now that `probe:z_virtual_endstop` should not have these.
            if name == "z_virtual_endstop" || name.starts_with("probe:") && (invert || pullup) {
                 return Err(ProbeError::ConfigError(
                    format!("Cannot pullup/invert probe virtual endstop or special pin: {}", pin_str)
                ));
            }
        }

        // Specific check for z_virtual_endstop as per original Python code for HomingViaProbeHelper
        if name == "z_virtual_endstop" && (invert || pullup) {
            return Err(ProbeError::ConfigError(
                "Cannot pullup/invert probe virtual endstop".to_string()
            ));
        }


        Ok((name, invert, pullup))
    }
}

pub struct PrinterProbe {
    printer: Printer,
    pub params: ProbeParams,
    toolhead: Arc<Mutex<Toolhead>>,
    mcu_probe: Option<Arc<Mutex<dyn EndstopWrapper>>>, // Actual endstop pin
    last_z_result: f64,
    last_query_state: bool,
    z_min_position: f64, // To store printer's minimum Z position
}

impl PrinterProbe {
    pub fn new(
        printer: Printer,
        params: ProbeParams,
        toolhead: Arc<Mutex<Toolhead>>,
        mcu_probe: Option<Arc<Mutex<dyn EndstopWrapper>>>,
        z_min_position: f64
    ) -> Self {
        Self {
            printer,
            params,
            toolhead,
            mcu_probe,
            last_z_result: 0.0,
            last_query_state: false,
            z_min_position,
        }
    }

    // This is the entry point Klippy uses
    pub fn load_config(printer: Printer, config: &mut Configfile) -> Result<Self, ProbeError> {
        let params = ProbeParams::new(config)?;
        let toolhead = printer.lookup_object::<Toolhead>("toolhead")
            .map_err(|e| ProbeError::ConfigError(format!("Failed to lookup toolhead: {}",e)))?;

        let mut mcu_probe_pin: Option<Arc<Mutex<dyn EndstopWrapper>>> = None;
        let pins_obj = printer.lookup_object::<PrinterPins>("pins")
             .map_err(|e| ProbeError::ConfigError(format!("Failed to lookup pins: {}",e)))?;

        // Simplified pin setup. Real setup is more involved.
        // Does not yet handle "probe:z_virtual_endstop" specifically which needs HomingViaProbeHelper logic.
        // This part is a placeholder and will need significant expansion for full pin functionality.
        if params.pin_name != "z_virtual_endstop" && !params.pin_name.starts_with("probe:") {
            // Attempt to set up a normal endstop pin
            // The actual `setup_pin` for endstops returns Arc<Mutex<dyn EndstopPin>>,
            // which needs to be compatible with EndstopWrapper or be wrapped.
            // This is a simplification.
            let pin_args = PinArgs {
                pin: params.pin_name.clone(),
                invert: params.invert_pin,
                pullup: params.pullup_pin,
                // Other PinArgs fields might be needed depending on setup_pin implementation
                ..Default::default()
            };
            match pins_obj.lock().setup_pin("endstop", pin_args) {
                Ok(PinMapValue::Endstop(pin_wrapper)) => {
                     mcu_probe_pin = Some(pin_wrapper);
                }
                Ok(_) => return Err(ProbeError::ConfigError(format!("Pin '{}' is not an endstop pin", params.pin_name))),
                Err(e) => return Err(ProbeError::ConfigError(format!("Failed to setup pin {}: {}", params.pin_name, e))),
            }
        } else if params.pin_name == "z_virtual_endstop" {
            // TODO: This is where HomingViaProbeHelper logic would be initialized.
            // For now, we'll leave mcu_probe_pin as None, and run_single_probe will have to simulate.
            // This means QUERY_PROBE would not work for z_virtual_endstop yet.
            println!("Warning: 'probe:z_virtual_endstop' selected. Full functionality requires HomingViaProbeHelper port.");
        }


        // Lookup minimum Z position (simplified from Klipper's lookup_minimum_z)
        // This would ideally use manual_probe::lookup_z_endstop_config or similar.
        let z_min_pos = config.get_section("stepper_z")
            .and_then(|s| s.get("position_min").ok().and_then(|v| v.parse::<f64>().ok()))
            .or_else(|| config.get_section("printer").and_then(|s| s.get("minimum_z_position").ok().and_then(|v| v.parse::<f64>().ok())))
            .unwrap_or(0.0);


        Ok(PrinterProbe::new(printer, params, toolhead, mcu_probe_pin, z_min_pos))
    }

    /// Perform a single Z probe action.
    /// Moves the Z axis downwards until the probe triggers or a minimum Z is reached.
    pub fn run_single_probe(&mut self, gcmd: &GCodeCommand) -> Result<[f64; 3], ProbeError> {
        let toolhead_locked = self.toolhead.lock();

        // Check if homed (simplified check)
        let th_status = toolhead_locked.get_status(self.printer.get_reactor().monotonic());
        if !th_status.homed_axes.contains("z") { // Klipper checks 'z' in homed_axes
            return Err(ProbeError::CommandError("Must home Z axis before probe".to_string()));
        }

        // Get current XY position
        let mut current_pos = toolhead_locked.get_position();
        let probe_xy = [current_pos[0], current_pos[1]];

        // Determine target Z for probing
        // In Klipper, this is more complex, involving homing moves for z_virtual_endstop.
        // For a simplified version, we'll move towards z_min_position.
        let target_z = self.z_min_position;
        current_pos[2] = target_z; // This is the position to probe towards

        // Get probe speed from GCode or defaults
        let probe_speed = gcmd.get_float("PROBE_SPEED", Some(self.params.speed), Some(0.0..)).map_err(ProbeError::from)?;


        // Simplified: Simulate move and trigger.
        // A real implementation would use toolhead.move_z_for_probe() or similar,
        // which would interact with MCU endstops.
        // For now, assume it triggers at some point or reaches min_z.

        // TODO: Implement activate_gcode if defined (deferred)
        // if let Some(gcode_str) = &self.params.activate_gcode { ... }

        // Simulate the probing move
        // In a real scenario, this would be a call to something like:
        // let triggered_pos = toolhead_locked.perform_homing_move(
        //     &self.mcu_probe.as_ref().unwrap(), // This requires mcu_probe to be Some and correctly typed
        //     current_pos, // target position [x,y,z_min_position]
        //     probe_speed,
        //     // ... other homing parameters
        // );
        // For now, we'll just assume it triggers at z_offset above the bed if it's a positive offset,
        // or slightly above z_min_position. This is highly simplified.

        let triggered_z: f64;
        if self.mcu_probe.is_some() {
            // If we had a real mcu_probe, we'd use it here with phoming.probing_move()
            // This part is placeholder for actual probing hardware interaction
            gcmd.respond_info("Simulating probe with mcu_probe (actual trigger logic is placeholder)");
            // This is a placeholder. Real logic would call into homing/mcu.
            // For now, let's assume it triggers at half of z_offset if positive, or z_min_position + small_amount
            triggered_z = if self.params.z_offset > 0.0 {
                self.z_min_position + self.params.z_offset / 2.0
            } else {
                self.z_min_position + 0.1
            };
            // Ensure toolhead position is updated after simulated probe
            let mut final_pos_arr = probe_xy.to_vec();
            final_pos_arr.push(triggered_z);
            toolhead_locked.set_position(final_pos_arr.try_into().unwrap(), Some(0.0), None, None);


        } else if self.params.pin_name == "z_virtual_endstop" {
            // Simplified logic for z_virtual_endstop (without full HomingViaProbeHelper)
            // This would normally involve a homing move.
            gcmd.respond_info("Simulating z_virtual_endstop probe (highly simplified)");
             // Let's assume it triggers at params.z_offset above the target_z if positive, or slightly above target_z
            triggered_z = target_z + self.params.z_offset.max(0.1); // Ensure it's slightly above min if z_offset is small/negative

            // Update toolhead position
            let mut final_pos_arr = probe_xy.to_vec();
            final_pos_arr.push(triggered_z);
            toolhead_locked.set_position(final_pos_arr.try_into().unwrap(), Some(0.0), None, None);
        }
        else {
            return Err(ProbeError::ProbeFailure("No mcu_probe configured and not z_virtual_endstop. Cannot probe.".to_string()));
        }


        // TODO: Implement deactivate_gcode if defined (deferred)
        // if let Some(gcode_str) = &self.params.deactivate_gcode { ... }


        let result_pos = [probe_xy[0], probe_xy[1], triggered_z];
        self.last_z_result = triggered_z;

        gcmd.respond_info(&format!("probe at {:.3},{:.3} is z={:.6}", result_pos[0], result_pos[1], result_pos[2]));

        Ok(result_pos)
    }

    fn cmd_PROBE(&mut self, gcmd: &GCodeCommand) -> Result<(), ProbeError> {
        // Note: Klipper's cmd_PROBE in probe.py takes gcmd, calls run_single_probe,
        // then formats the result. run_single_probe already calls respond_info.
        // So we just need to call it.
        match self.run_single_probe(gcmd) {
            Ok(pos) => {
                // run_single_probe already calls respond_info with the format "probe at X,Y is z=Z"
                // Klipper's original cmd_PROBE then does another respond_info: "Result is z=%.6f"
                // We can replicate that here.
                gcmd.respond_info(&format!("Result is z={:.6}", pos[2]));
                self.last_z_result = pos[2]; // Ensure last_z_result is updated from the command context
                Ok(())
            }
            Err(e) => {
                // Convert ProbeError to CommandError for GCode handler
                Err(e)
            }
        }
    }

    // Called by klippy_main during Printer object construction
    pub fn register_commands(printer: &Printer, probe_obj: Weak<Mutex<PrinterProbe>>) -> Result<(), ConfigError> {
        let gcode = printer.lookup_object::<GCode>("gcode")?;
        let mut gcode_locked = gcode.lock();

        let probe_clone_for_gcode = probe_obj.clone();
        gcode_locked.register_command_raw("PROBE",
            // TODO: Add help string similar to "Probe Z-height at current XY position"
            Box::new(move |gcmd_ref| {
                if let Some(probe_arc) = probe_clone_for_gcode.upgrade() {
                    let mut probe_locked = probe_arc.lock();
                    match probe_locked.cmd_PROBE(gcmd_ref) {
                        Ok(()) => Ok(()),
                        Err(ProbeError::CommandError(s)) => Err(CommandError::GCodeError(s)),
                        Err(ProbeError::ConfigError(s)) => Err(CommandError::ConfigError(s)),
                        Err(ProbeError::ProbeFailure(s)) => Err(CommandError::GCodeError(format!("Probe failed: {}", s))),
                    }
                } else {
                    Err(CommandError::Shutdown("Probe object dropped".to_string()))
                }
            })
        );
        Ok(())
    }


    pub fn get_status(&self, _eventtime: f64) -> HashMap<String, serde_json::Value> {
        let mut status = HashMap::new();
        // status.insert("name".to_string(), serde_json::Value::String(self.params.pin_name.clone())); // Or section name
        status.insert("last_query".to_string(), serde_json::Value::Bool(self.last_query_state));
        status.insert("last_z_result".to_string(), serde_json::Value::Number(serde_json::Number::from_f64(self.last_z_result).unwrap()));
        status
    }
}

// Trampoline functions might be needed if GCode command handlers need to capture `self`
// Example:
// use std::sync::{Arc, Mutex};
// fn cmd_PROBE_trampoline(probe_instance: Arc<Mutex<PrinterProbe>>) -> Box<dyn Fn(&GCodeCommand) -> Result<(), CommandError>> {
//     Box::new(move |gcmd| {
//         let mut probe = probe_instance.lock().unwrap();
//         probe.cmd_PROBE(gcmd)
//     })
// }

// Required for printer.register_event_handler and lookup_object
impl crate::core_traits::PrintKObject for PrinterProbe {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
}

// Basic test for ProbeParams parsing
#[cfg(test)]
mod tests {
    use super::*;
    use crate::configfile::Configfile;
    use crate::klippy_main::Printer; // Required for mock Printer
    use crate::toolhead::Toolhead;
    use crate::pins::PrinterPins;
    use crate::gcode::GCode;
    use crate::reactor::Reactor;


    use std::collections::HashMap;
    use std::sync::{Arc, Weak};
    use parking_lot::Mutex;

    // Helper to create a basic Configfile with a [probe] section
    fn create_probe_configfile(options: HashMap<&str, &str>) -> Configfile {
        let mut cf = Configfile::new(None).expect("Failed to create test configfile");
        let mut probe_options = HashMap::new();
        for (k,v) in options {
            probe_options.insert(k.to_string(), v.to_string());
        }
        cf.add_section("probe".to_string(), probe_options);
        cf
    }

    // Mock Printer and related components for testing
    // This is a simplified mock setup. A more robust solution might involve a dedicated test harness.
    fn create_mock_printer(config: &mut Configfile) -> (Printer, Arc<Mutex<Toolhead>>, Arc<Mutex<PrinterPins>>, Arc<Mutex<GCode>>) {
        let reactor = Reactor::new(); // Dummy reactor
        let printer = Printer::new(reactor.clone()); // Create a basic printer instance

        // Add a dummy [printer] section if not present, as Toolhead might need it
        if !config.has_section("printer") {
            config.add_section("printer".to_string(), HashMap::new());
        }
        // Add dummy kinematic info if toolhead needs it
        if !config.has_section("cartesian") { // Assuming cartesian might be a default or simple kinematic
            let mut cart_opts = HashMap::new();
            cart_opts.insert("max_velocity".to_string(), "300".to_string());
             cart_opts.insert("max_accel".to_string(), "3000".to_string());
            config.add_section("cartesian".to_string(), cart_opts);
        }


        let toolhead = Toolhead::new(printer.clone(), config).expect("Failed to create mock Toolhead");
        let pins_obj = PrinterPins::new(printer.clone(), config).expect("Failed to create mock PrinterPins");
        let gcode = GCode::new(printer.clone(), false, None, None).expect("Failed to create mock GCode");

        let toolhead_arc = Arc::new(Mutex::new(toolhead));
        let pins_arc = Arc::new(Mutex::new(pins_obj));
        let gcode_arc = Arc::new(Mutex::new(gcode));

        printer.add_object("toolhead".to_string(), toolhead_arc.clone() as Arc<Mutex<dyn crate::core_traits::PrintKObject>>);
        printer.add_object("pins".to_string(), pins_arc.clone() as Arc<Mutex<dyn crate::core_traits::PrintKObject>>);
        printer.add_object("gcode".to_string(), gcode_arc.clone() as Arc<Mutex<dyn crate::core_traits::PrintKObject>>);

        // Add a dummy Z stepper section for z_min_position lookup
        if !config.has_section("stepper_z") {
            let mut stepper_z_opts = HashMap::new();
            stepper_z_opts.insert("position_min".to_string(), "0".to_string());
            stepper_z_opts.insert("position_endstop".to_string(), "0".to_string());
            // Add other mandatory fields for stepper if any
             stepper_z_opts.insert("step_pin".to_string(), "ar0".to_string()); // Dummy pin
            config.add_section("stepper_z".to_string(), stepper_z_opts);
        }


        (printer, toolhead_arc, pins_arc, gcode_arc)
    }


    #[test]
    fn test_probe_params_parsing_basic() {
        let mut options = HashMap::new();
        options.insert("pin", "P1.23");
        options.insert("z_offset", "5.0");

        let mut cf = create_probe_configfile(options); // Use new helper
        let params = ProbeParams::new(&mut cf).unwrap();

        assert_eq!(params.pin_name, "P1.23");
        assert!(!params.invert_pin);
        assert!(!params.pullup_pin);
        assert_eq!(params.z_offset, 5.0);
        assert_eq!(params.speed, 5.0); // Default
        assert_eq!(params.lift_speed, 5.0); // Default (same as speed)
        assert_eq!(params.samples, 1); // Default
        assert_eq!(params.sample_retract_dist, 2.0); // Default
    }

    #[test]
    fn test_probe_params_parsing_full() {
        let mut options = HashMap::new();
        options.insert("pin", "^!P0.10");
        options.insert("x_offset", "10.5");
        options.insert("y_offset", "-5.2");
        options.insert("z_offset", "1.25");
        options.insert("speed", "10.0");
        options.insert("lift_speed", "15.0");
        options.insert("samples", "3");
        options.insert("sample_retract_dist", "1.0");
        options.insert("samples_result", "median");
        options.insert("samples_tolerance", "0.05");
        options.insert("samples_tolerance_retries", "2");
        options.insert("activate_gcode", "SET_PIN PIN=probe_enable VALUE=1");
        options.insert("deactivate_gcode", "SET_PIN PIN=probe_enable VALUE=0");
        options.insert("deactivate_on_each_sample", "false");

        let mut cf = create_probe_configfile(options); // Use new helper
        let params = ProbeParams::new(&mut cf).unwrap();

        assert_eq!(params.pin_name, "P0.10");
        assert!(params.invert_pin);
        assert!(params.pullup_pin);
        assert_eq!(params.x_offset, 10.5);
        assert_eq!(params.y_offset, -5.2);
        assert_eq!(params.z_offset, 1.25);
        assert_eq!(params.speed, 10.0);
        assert_eq!(params.lift_speed, 15.0);
        assert_eq!(params.samples, 3);
        assert_eq!(params.sample_retract_dist, 1.0);
        assert_eq!(params.samples_result, "median");
        assert_eq!(params.samples_tolerance, 0.05);
        assert_eq!(params.samples_tolerance_retries, 2);
        assert_eq!(params.activate_gcode.unwrap(), "SET_PIN PIN=probe_enable VALUE=1");
        assert_eq!(params.deactivate_gcode.unwrap(), "SET_PIN PIN=probe_enable VALUE=0");
        assert!(!params.stow_on_each_sample);
    }

    #[test]
    fn test_probe_params_z_offset_mandatory() {
        let mut options = HashMap::new();
        options.insert("pin", "ar0"); // Just needs a pin
        // z_offset is missing
        let mut cf = create_probe_configfile(options); // Use new helper
        let result = ProbeParams::new(&mut cf);
        assert!(result.is_err());
        if let Err(ProbeError::ConfigError(msg)) = result {
            assert!(msg.contains("Option 'z_offset' is not specified"));
        } else {
            panic!("Expected ConfigError for missing z_offset");
        }
    }

    #[test]
    fn test_parse_pin_name_virtual_endstop_error() {
        // Virtual endstop should not have pullup or invert
        let result_invert = ProbeParams::parse_pin_name("!z_virtual_endstop");
        assert!(result_invert.is_err());
         if let Err(ProbeError::ConfigError(msg)) = result_invert {
            assert!(msg.contains("Cannot pullup/invert probe virtual endstop"));
        } else {
            panic!("Expected ConfigError for invalid virtual endstop pin !z_virtual_endstop");
        }

        let result_pullup = ProbeParams::parse_pin_name("^z_virtual_endstop");
        assert!(result_pullup.is_err());
        if let Err(ProbeError::ConfigError(msg)) = result_pullup {
            assert!(msg.contains("Cannot pullup/invert probe virtual endstop"));
        } else {
            panic!("Expected ConfigError for invalid virtual endstop pin ^z_virtual_endstop");
        }

        let result_ok = ProbeParams::parse_pin_name("z_virtual_endstop");
        assert!(result_ok.is_ok());
        assert_eq!(result_ok.unwrap(), ("z_virtual_endstop".to_string(), false, false));

        let result_probe_prefix_ok = ProbeParams::parse_pin_name("probe:z_virtual_endstop");
         assert!(result_probe_prefix_ok.is_ok());
        assert_eq!(result_probe_prefix_ok.unwrap(), ("probe:z_virtual_endstop".to_string(), false, false));

        // This test might need adjustment if "probe:" prefix is special
        let result_probe_prefix_err_invert = ProbeParams::parse_pin_name("!probe:some_pin");
         assert!(result_probe_prefix_err_invert.is_err()); // This should error based on current parse_pin_name
         if let Err(ProbeError::ConfigError(msg)) = result_probe_prefix_err_invert {
            assert!(msg.contains("Cannot pullup/invert probe virtual endstop or special pin"));
        } else {
            panic!("Expected ConfigError for invalid probe pin !probe:some_pin");
        }
    }

    #[test]
    fn test_invalid_speed_values() {
        let mut options_zero_speed = HashMap::new();
        options_zero_speed.insert("pin", "P1.0");
        options_zero_speed.insert("z_offset", "1.0");
        options_zero_speed.insert("speed", "0.0");
        let mut cf_zero = create_probe_configfile(options_zero_speed); // Use new helper
        let res_zero = ProbeParams::new(&mut cf_zero);
        assert!(res_zero.is_err(), "Expected error for zero speed");

        let mut options_neg_speed = HashMap::new();
        options_neg_speed.insert("pin", "P1.0");
        options_neg_speed.insert("z_offset", "1.0");
        options_neg_speed.insert("speed", "-1.0");
        let mut cf_neg = create_probe_configfile(options_neg_speed); // Use new helper
        let res_neg = ProbeParams::new(&mut cf_neg);
        assert!(res_neg.is_err(), "Expected error for negative speed");

        let mut options_neg_lift = HashMap::new();
        options_neg_lift.insert("pin", "P1.0");
        options_neg_lift.insert("z_offset", "1.0");
        options_neg_lift.insert("lift_speed", "-1.0");
        let mut cf_neg_lift = create_probe_configfile(options_neg_lift); // Use new helper
        let res_neg_lift = ProbeParams::new(&mut cf_neg_lift);
        assert!(res_neg_lift.is_err(), "Expected error for negative lift_speed");
    }

    #[test]
    fn test_printer_probe_load_config_ok() {
        let mut options = HashMap::new();
        options.insert("pin", "z_virtual_endstop"); // Using virtual endstop to avoid real pin setup issues in mock
        options.insert("z_offset", "5.0");
        // Add other necessary options for PrinterProbe::new or its dependencies if any emerge
        // For now, Toolhead might need [printer] and kinematics (e.g. [cartesian])
        // PrinterPins might need some basic valid config too.

        let mut cf = create_probe_configfile(options);
        // Add dummy printer section for toolhead
        cf.add_section("printer".to_string(), {
            let mut po = HashMap::new();
            po.insert("max_velocity".to_string(), "500".to_string());
            po.insert("max_accel".to_string(), "5000".to_string());
            po
        });
        // Add dummy kinematics for toolhead
        cf.add_section("cartesian".to_string(), {
            let mut ko = HashMap::new();
            ko.insert("max_z_velocity".to_string(), "10".to_string());
            ko.insert("max_z_accel".to_string(), "100".to_string());
            ko
        });


        let (printer, _toolhead, _pins, _gcode) = create_mock_printer(&mut cf);

        let probe_result = PrinterProbe::load_config(printer, &mut cf);
        assert!(probe_result.is_ok(), "load_config failed: {:?}", probe_result.err());
        let probe = probe_result.unwrap();
        assert_eq!(probe.params.z_offset, 5.0);
        assert_eq!(probe.params.pin_name, "z_virtual_endstop");
    }

    #[test]
    fn test_printer_probe_run_single_probe_virtual_endstop() {
        let mut options = HashMap::new();
        options.insert("pin", "z_virtual_endstop");
        options.insert("z_offset", "2.0"); // Probe triggers 2mm above nominal zero
        options.insert("speed", "5.0");

        let mut cf = create_probe_configfile(options);
        cf.add_section("printer".to_string(), {
            let mut po = HashMap::new();
            po.insert("minimum_z_position".to_string(), "-2.0".to_string()); // Allow probing below zero
            po.insert("max_velocity".to_string(), "500".to_string());
            po.insert("max_accel".to_string(), "5000".to_string());
            po
        });
         cf.add_section("cartesian".to_string(), {
            let mut ko = HashMap::new();
            ko.insert("max_z_velocity".to_string(), "10".to_string());
            ko.insert("max_z_accel".to_string(), "100".to_string());
            ko
        });


        let (printer, toolhead_arc, _pins_arc, _gcode_arc) = create_mock_printer(&mut cf);
        let mut probe = PrinterProbe::load_config(printer.clone(), &mut cf).unwrap();

        // Manually set toolhead position and homed axes for test
        {
            let mut th = toolhead_arc.lock();
            th.set_position([10.0, 10.0, 30.0].into(), None, None, None); // Start high
            let mut homed_axes = HashMap::new(); // Using HashMap like in toolhead status
            homed_axes.insert("x".to_string(), true);
            homed_axes.insert("y".to_string(), true);
            homed_axes.insert("z".to_string(), true);
            // This part of setting homed_axes is tricky without direct access or a method
            // For now, we assume the check inside run_single_probe works based on Toolhead's internal state
            // which should be set by gcode_move's SetPosition.
            // We can simulate homing by setting GCodeMove status if necessary, or rely on Toolhead's internal state.
            // For this test, the `toolhead.set_position` above implies it's homed for simplicity of the mock.
            // A real test might need to call G28 or directly manipulate GCodeMove state.
            // The `get_status` in toolhead.rs looks at `last_kin_move_time > 0.0` for axes.
            // `set_position` updates `last_kin_move_time`.
        }

        let gcmd_params = HashMap::new(); // No extra params for PROBE command in this test
        let gcode_cmd = GCodeCommand::new("PROBE", gcmd_params, printer.get_gcode_arc().unwrap());


        let result = probe.run_single_probe(&gcode_cmd);
        assert!(result.is_ok(), "run_single_probe failed: {:?}", result.err());
        let pos = result.unwrap();

        assert_eq!(pos[0], 10.0); // X should not change
        assert_eq!(pos[1], 10.0); // Y should not change

        // With z_virtual_endstop, simplified logic is target_z + z_offset.max(0.1)
        // target_z is printer.minimum_z_position = -2.0
        // z_offset = 2.0
        // expected_z = -2.0 + 2.0.max(0.1) = -2.0 + 2.0 = 0.0
        assert!((pos[2] - 0.0).abs() < 0.0001, "Expected Z near 0.0, got {}", pos[2]);
        assert!((probe.last_z_result - 0.0).abs() < 0.0001);

        // Test with negative z_offset
        probe.params.z_offset = -1.0;
        // expected_z = -2.0 + (-1.0).max(0.1) = -2.0 + 0.1 = -1.9
        let result2 = probe.run_single_probe(&gcode_cmd);
         assert!(result2.is_ok(), "run_single_probe failed: {:?}", result2.err());
        let pos2 = result2.unwrap();
        assert!((pos2[2] - (-1.9)).abs() < 0.0001, "Expected Z near -1.9, got {}", pos2[2]);
    }

    #[test]
    fn test_run_single_probe_not_homed() {
        let mut options = HashMap::new();
        options.insert("pin", "z_virtual_endstop");
        options.insert("z_offset", "1.0");

        let mut cf = create_probe_configfile(options);
        cf.add_section("printer".to_string(), HashMap::new()); // Basic printer section

        let (printer, toolhead_arc, _pins, _gcode) = create_mock_printer(&mut cf);
        let mut probe = PrinterProbe::load_config(printer.clone(), &mut cf).unwrap();

        // Ensure toolhead is NOT homed (or at least Z is not)
        // Default Toolhead state might be unhomed. If not, we'd need a way to set it.
        // For now, we rely on the default state of a new Toolhead or clear its position.
        {
            let mut th = toolhead_arc.lock();
            // A bit of a hack: set last_kin_move_time to 0.0 for Z to simulate not homed
            // This depends on Toolhead's get_status implementation.
            // A proper mock or toolhead API would be better.
            // Based on current toolhead.rs, status() checks last_kin_move_time[axis_idx] > 0.0
            // Toolhead::new sets these to 0.0, so a new toolhead is not homed.
        }

        let gcmd_params = HashMap::new();
        let gcode_cmd = GCodeCommand::new("PROBE", gcmd_params, printer.get_gcode_arc().unwrap());

        let result = probe.run_single_probe(&gcode_cmd);
        assert!(result.is_err());
        if let Err(ProbeError::CommandError(msg)) = result {
            assert!(msg.contains("Must home Z axis before probe"));
        } else {
            panic!("Expected CommandError for probing unhomed Z");
        }
    }
}
