// klipper_host_rust/src/klippy_main.rs
// Corresponds to klippy/klippy.py - Main Klipper host program logic.

// Import necessary modules. Adjust paths as necessary based on your project structure.
use klipper_host_rust::{
    configfile::{self, Configfile}, // Ensure ConfigError is accessible if needed, or handle errors as String
    gcode::{GCode, CommandError},
    toolhead::{ToolHead, PrinterUtility, DEFAULT_HOMING_SPEED, X_MACHINE_POSITION_AT_ENDSTOP, Y_MACHINE_POSITION_AT_ENDSTOP, Z_MACHINE_POSITION_AT_ENDSTOP, X_GCODE_POSITION_AFTER_HOMING, Y_GCODE_POSITION_AFTER_HOMING, Z_GCODE_POSITION_AFTER_HOMING},
    reactor::Reactor,
    mcu::Mcu,
    heaters::Heater,
    extras::fan::Fan,
    kinematics::cartesian::{CartesianKinematics, DEFAULT_ROTATION_DISTANCE, DEFAULT_FULL_STEPS_PER_ROTATION, DEFAULT_MICROSTEPS, DEFAULT_STEP_DISTANCE},
    trapq::TrapQ,
};


// Dummy Reactor for main function
struct MainReactor;
impl Reactor for MainReactor {
    fn monotonic(&self) -> f64 { std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64() }
    fn register_timer(&mut self, _time: f64, _callback: Box<dyn FnMut(f64) -> Option<f64>>) -> usize { 0 }
    fn register_fd(&mut self, _fd: i32, _callback: Box<dyn FnMut(f64)>) -> usize {0}
    fn unregister_fd(&mut self, _handle_id: usize) {}
    fn unregister_timer(&mut self, _handle_id: usize) {}
    fn is_shutdown(&self) -> bool {false} // Simulate not shutdown for tests
    fn run(&mut self) { println!("MainReactor: run called"); }
    fn pause(&mut self, _waketime: f64) {
        // In a real reactor, this would yield or schedule. For sync simulation:
        // std::thread::sleep(std::time::Duration::from_secs_f64(duration_to_sleep));
        // However, gcode command handlers are using std::thread::sleep directly for now.
    }
    fn _check_timers(&mut self, _eventtime: f64, _idle: bool) {}
}

// Dummy MCU for main function
struct MainMcu;
impl Mcu for MainMcu {
    fn estimated_print_time(&self, _curtime: f64) -> f64 { _curtime } // Simplistic: mcu time = host time
}

// Dummy PrinterUtility
struct MainPrinterUtility;
impl PrinterUtility for MainPrinterUtility {
    fn send_event(&self, event: &str, params_str: String) {
        println!("PrinterEvent: {} - {}", event, params_str);
    }
}


pub fn klippy_main() {
    println!("Klipper Host (Rust) Integration Test Starting...");

    let sample_config_content = r#"
[printer]
max_velocity: 300
max_accel: 2000
square_corner_velocity: 5.0
max_z_velocity: 25
max_z_accel: 300

[stepper_x]
position_min: 0
position_max: 210
position_endstop: 0
rotation_distance: 40
full_steps_per_rotation: 200
microsteps: 16
homing_speed: 50
homing_retract_dist: 5

[stepper_y]
position_min: 0
position_max: 220
position_endstop: 0
rotation_distance: 40
full_steps_per_rotation: 200
microsteps: 16
gear_ratio: 1.0
homing_speed: 50
homing_retract_dist: 5

[stepper_z]
position_min: 0
position_max: 190
position_endstop: 0
rotation_distance: 8
full_steps_per_rotation: 200
microsteps: 16
homing_speed: 10
homing_retract_dist: 2

[extruder]
heater_name: my_extruder_heater
min_temp: 10
max_temp: 275

[heater_bed]
heater_name: my_bed_heater
min_temp: 10
max_temp: 110

[fan]
name: my_part_fan
max_power: 0.8
off_below: 0.1
kick_start_time: 0.2
"#;

    let mut config = Configfile::new(Some("memory_config.cfg".to_string()));
    if let Err(e) = config.parse(sample_config_content) {
        eprintln!("Failed to parse sample config: {}", e);
        return;
    }
    println!("Successfully parsed sample configuration.");

    let reactor = Box::new(MainReactor);
    let printer_util = Box::new(MainPrinterUtility);
    let mcu = Box::new(MainMcu);

    let static_reactor: &'static MainReactor = Box::leak(reactor);
    let static_printer_util: &'static MainPrinterUtility = Box::leak(printer_util);
    let static_mcu: &'static MainMcu = Box::leak(mcu);
    let mcus_list_static: Vec<&'static dyn Mcu> = vec![static_mcu];


    let mut toolhead = match ToolHead::new(&config, static_reactor, static_printer_util, mcus_list_static) {
        Ok(th) => th,
        Err(e) => {
            eprintln!("Failed to initialize ToolHead: {}", e);
            return;
        }
    };

    let mut gcode_processor = GCode::new("MyRustKlipper".to_string());

    println!("Initial GCode State: {:?}", gcode_processor.state);
    println!("Initial ToolHead Position: {:?}", toolhead.get_position());
    println!("Initial Extruder Target: {:.1}, Bed Target: {:.1}", toolhead.extruder_heater.target_temp, toolhead.bed_heater.target_temp);
    println!("Initial Fan Speed: {:.2}", toolhead.part_cooling_fan.speed);


    let gcode_lines = vec![
        "G1 X1 Y1 F3000",           // Attempt move before homing (should fail)
        "G28",                      // Home all axes
        "M114",                     // Initial position after homing
        "M104 S100",                // Set extruder temp to 100 (don't wait yet)
        "M140 S50",                 // Set bed temp to 50 (don't wait yet)
        "M190 S50",                 // Wait for bed to reach 50C
        "M114",                     // Position after bed wait
        "M109 S100",                // Wait for extruder to reach 100C
        "M114",                     // Position after extruder wait
        "G90",                      // Absolute positioning
        "G1 X10 Y20 Z5 F3000",      // Move to X10 Y20 Z5
        "M82",                      // Absolute extrusion mode
        "G1 E10",                   // Extrude to E=10
        "G1 E12",                   // Extrude to E=12 (2mm more)
        "M83",                      // Relative extrusion mode
        "G1 E2",                    // Extrude 2mm more (E becomes 14)
        "G1 E-1 F1800",             // Retract 1mm (E becomes 13)
        "G91",                      // Relative positioning for XYZ
        "G1 X5 Y-5",                // Move relatively X5 Y-5 (E should remain 13)
        "G92 X0 Y0 E0",             // Set current X, Y, and E to 0 (creates offsets)
        "G90",                      // Absolute positioning for XYZ
        "M82",                      // Absolute extrusion
        "G1 X1 Z1 E5",              // Move to G-code X1 Z1, G-code E5
        "G4 P500",                  // Dwell for 500 milliseconds
        "G28 Y",                    // Re-home Y
        "G1 Y15",                   // Move Y to 15 (should be G-code 15)
        "G4 P100",                  // Dwell for 100 milliseconds
        "M104 S205",                // Set extruder temp to 205C
        "M140 S60",                 // Set bed temp to 60C
        "M114",                     // Check position
        "M106 S128",                // Set fan to 50% (0.5 * 0.8_max_power = 0.4 actual)
        "G1 X20 Y20 Z20 F1000",     // Another move after temps are stable
        "M114",                     // Check position after move
        "M106 S255",                // Fan full (0.8 due to max_power)
        "G1 X22 Y22 Z22 F1000",     // Move with fan on
        "M106 S20",                 // Fan low (S20/255 = ~0.078. If off_below=0.1, should be off)
        "G1 X23 Y23 Z23 F1000",     // Move with fan (likely) off
        "M106 S30",                 // Fan just above off_below (S30/255 = ~0.117. speed=~0.117*0.8)
        "G1 X24 Y24 Z24 F1000",     // Move with fan on low
        "M107",                     // Turn fan off
        "M104 S300",                // Attempt to set extruder temp above its max_temp (275) - SHOULD ERROR
        "M104 S5",                  // Attempt to set extruder temp below its min_temp (10) - SHOULD ERROR
        "M140 S150",                // Attempt to set bed temp above its max_temp (110) - SHOULD ERROR
        "M140 S5",                  // Attempt to set bed temp below its min_temp (10) - SHOULD ERROR
        "M104 S0",                  // Turn off extruder (should be OK)
        "M140 S0",                  // Turn off bed (should be OK)
        "G1 X250 F3000",            // Attempt to move X out of bounds (max 210 from config) - SHOULD ERROR
        "G1 Y-10 F3000",            // Attempt to move Y out of bounds (min 0 from config) - SHOULD ERROR
        "G1 Z191 F1000",            // Attempt to move Z out of bounds (max 190 from config) - SHOULD ERROR
        "M114",                     // Check position after failed moves
        "INVALID GCODE",            // Test error handling
    ];

    for line_str in gcode_lines { // Changed from gcode_lines_with_unhomed_test
        println!("\nProcessing G-code: '{}'", line_str);
        match gcode_processor.parse_line(line_str) {
            Ok(gcode_command) => {
                println!("Parsed Command: {:?}", gcode_command);
                match gcode_processor.process_command(&gcode_command, &mut toolhead) {
                    Ok(()) => {
                        println!("Command executed successfully.");
                        println!("Current GCode State: {:?}", gcode_processor.state);
                        let th_pos = toolhead.get_position();
                        println!("ToolHead Commanded Position (M114 source): X:{:.3} Y:{:.3} Z:{:.3} E:{:.3}", th_pos[0], th_pos[1], th_pos[2], th_pos[3]);

                        if gcode_command.command_letter == 'G' && (gcode_command.command_number == 0.0 || gcode_command.command_number == 1.0 || gcode_command.command_number == 28.0 ) ||
                           (gcode_command.command_letter == 'G' && gcode_command.command_number == 92.0) {
                            if let Err(sim_err) = toolhead.move_kinematics_steppers_to_gcode_target([th_pos[0], th_pos[1], th_pos[2]]) {
                                eprintln!("Error simulating stepper move: {}", sim_err);
                            }
                        }

                        match toolhead.get_kinematics_calculated_position() {
                            Ok(calc_pos) => println!("Kinematics Calculated G-code Position: X:{:.3} Y:{:.3} Z:{:.3}", calc_pos[0], calc_pos[1], calc_pos[2]),
                            Err(e) => eprintln!("Error calculating kinematics position: {}", e),
                        }

                        println!("Extruder Target Temp: {:.1}°C, Bed Target Temp: {:.1}°C",
                                 toolhead.extruder_heater.target_temp,
                                 toolhead.bed_heater.target_temp);
                        println!("Part Cooling Fan Speed: {:.2}", toolhead.part_cooling_fan.speed);
                    }
                    Err(e) => {
                        eprintln!("Error executing command '{}': {}", line_str, e);
                    }
                }
            }
            Err(e) => {
                eprintln!("Error parsing G-code line '{}': {}", line_str, e);
            }
        }
    }

    println!("\nKlipper Host (Rust) Integration Test Finished.");
}

// fn main() {
//     klippy_main();
// }
