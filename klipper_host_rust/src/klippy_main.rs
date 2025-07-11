// klipper_host_rust/src/klippy_main.rs
// Corresponds to klippy/klippy.py - Main Klipper host program logic.

// Import necessary modules. Adjust paths as necessary based on your project structure.
use klipper_host_rust::{
    configfile::Configfile,
    gcode::{GCode, CommandError},
    toolhead::ToolHead,
    reactor::Reactor, // Assuming a Reactor trait/struct exists
    mcu::Mcu,         // Assuming an Mcu trait/struct exists
};


// Dummy Reactor for main function
struct MainReactor;
impl Reactor for MainReactor {
    fn monotonic(&self) -> f64 { 0.0 }
    fn register_timer(&mut self, _time: f64, _callback: Box<dyn FnMut(f64) -> Option<f64>>) -> usize { 0 }
    fn register_fd(&mut self, _fd: i32, _callback: Box<dyn FnMut(f64)>) -> usize {0}
    fn unregister_fd(&mut self, _handle_id: usize) {}
    fn unregister_timer(&mut self, _handle_id: usize) {}
    fn is_shutdown(&self) -> bool {false}
    fn run(&mut self) { println!("MainReactor: run called"); }
    fn pause(&mut self, _waketime: f64) {}
    fn _check_timers(&mut self, _eventtime: f64, _idle: bool) {}
}

// Dummy MCU for main function
struct MainMcu;
impl Mcu for MainMcu {
    fn estimated_print_time(&self, _curtime: f64) -> f64 {0.0}
    // Add other Mcu methods if ToolHead::new or other parts require them with specific signatures
}


pub fn klippy_main() {
    println!("Klipper Host (Rust) Integration Test Starting...");

    // 1. Define and parse sample configuration
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

[stepper_y]
position_min: 0
position_max: 220
position_endstop: 0

[stepper_z]
position_min: 0
position_max: 190
position_endstop: 0

[extruder]
# placeholder for extruder specific settings like nozzle_diameter, sensor_type, etc.

[heater_bed]
# placeholder for bed specific settings like sensor_type, etc.
"#;

    let mut config = Configfile::new(Some("memory_config.cfg".to_string()));
    if let Err(e) = config.parse(sample_config_content) {
        eprintln!("Failed to parse sample config: {}", e);
        return;
    }
    println!("Successfully parsed sample configuration.");

    // 2. Initialize Reactor and MCU (mocked)
    // These need to be 'static for the current ToolHead constructor if not using more advanced lifetime management.
    // Using Box::leak is a common way to achieve this in test/example code.
    let reactor: &'static mut MainReactor = Box::leak(Box::new(MainReactor));
    let mcu: &'static MainMcu = Box::leak(Box::new(MainMcu));
    let mcus: Vec<&'static dyn Mcu> = vec![mcu];


    // 3. Initialize ToolHead
    let mut toolhead = match ToolHead::new(&config, reactor, mcus) {
        Ok(th) => th,
        Err(e) => {
            eprintln!("Failed to initialize ToolHead: {}", e);
            return;
        }
    };

    // 4. Initialize GCode processor
    let mut gcode_processor = GCode::new("MyRustKlipper".to_string());

    println!("Initial GCode State: {:?}", gcode_processor.state);
    println!("Initial ToolHead Position: {:?}", toolhead.get_position());

    // 5. Simulate processing some G-code lines
    let gcode_lines = vec![
        "G28",                      // Home all axes
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
                                    // (actual machine E will be E5 + E_offset from G92 E0)
        "G4 P500",                  // Dwell for 500 milliseconds
        "G28 Y",                    // Re-home Y
        "G1 Y15",                   // Move Y to 15 (should be machine 15, gcode 15)
        "G4 P100",                  // Dwell for 100 milliseconds
        "M104 S205",                // Set extruder temp to 205C
        "M140 S60",                 // Set bed temp to 60C
        "G1 X10 Y10 Z10 F1200",     // Another move
        "M104 S0",                  // Turn off extruder
        "M140 S0",                  // Turn off bed
        "M104 S200",                // Set extruder to 200 (don't wait)
        "M190 S70",                 // Set bed to 70 AND WAIT
        "M109 S200",                // Wait for extruder to reach 200 (should be quick if M104 was effective)
        "M114",                     // Check position before next move
        "G1 X20 Y20 Z20 F1000",     // Another move after temps are stable
        "M114",                     // Check position after move
        "G1 X250 F3000",            // Attempt to move X out of bounds (max 200) - AFTER HOMING
        "G1 Y-10 F3000",            // Attempt to move Y out of bounds (min 0) - AFTER HOMING
        "G1 Z181 F1000",            // Attempt to move Z out of bounds (max 180) - AFTER HOMING
        "M114",                     // Check position after failed moves (should be same as last good G1)
        "INVALID GCODE",            // Test error handling
        // "G1 X10000",             // This was more of a placeholder, specific out-of-bounds are better.
    ];

    // Insert an unhomed move attempt before G28
    let mut gcode_lines_with_unhomed_test = vec!["G1 X1 Y1 F3000"]; // Attempt move before homing
    gcode_lines_with_unhomed_test.extend(gcode_lines);


    for line_str in gcode_lines_with_unhomed_test {
        println!("\nProcessing G-code: '{}'", line_str);
        match gcode_processor.parse_line(line_str) {
            Ok(gcode_command) => {
                println!("Parsed Command: {:?}", gcode_command);
                match gcode_processor.process_command(&gcode_command, &mut toolhead) {
                    Ok(()) => {
                        println!("Command executed successfully.");
                        println!("Current GCode State: {:?}", gcode_processor.state);
                        println!("ToolHead Position: {:?}", toolhead.get_position());
                        println!("Extruder Target Temp: {:.1}°C, Bed Target Temp: {:.1}°C",
                                 toolhead.extruder_heater.target_temp,
                                 toolhead.bed_heater.target_temp);
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

// If this is intended to be a binary, you might have a main function like this:
// fn main() {
//     klippy_main();
// }
