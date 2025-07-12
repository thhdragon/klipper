// klipper_host_rust/src/klippy_main.rs

use crate::{
    configfile::{self, Configfile},
    gcode::{GCode, CommandError},
    toolhead::{ToolHead, PrinterUtility, DEFAULT_HOMING_SPEED,
               X_MACHINE_POSITION_AT_ENDSTOP, Y_MACHINE_POSITION_AT_ENDSTOP, Z_MACHINE_POSITION_AT_ENDSTOP,
               X_GCODE_POSITION_AFTER_HOMING, Y_GCODE_POSITION_AFTER_HOMING, Z_GCODE_POSITION_AFTER_HOMING,
               DEFAULT_HOMING_RETRACT_DIST, DEFAULT_SECOND_HOMING_SPEED_RATIO},
    reactor::{Reactor, TimerHandle}, // Added TimerHandle
    core_traits::Mcu, // Corrected path from previous step, assuming Mcu trait is in core_traits
    heaters::Heater,
    extras::fan::Fan,
    kinematics::cartesian::{CartesianKinematics, DEFAULT_ROTATION_DISTANCE, DEFAULT_FULL_STEPS_PER_ROTATION, DEFAULT_MICROSTEPS},
    trapq::TrapQ,
};

struct MainReactor;
impl Reactor for MainReactor {
    fn monotonic(&self) -> f64 { std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64() }
    fn register_timer(&mut self, _eventtime: f64, _callback: Box<dyn FnMut(f64) -> Option<f64>>) -> TimerHandle { TimerHandle(0) } // Changed return
    fn unregister_timer(&mut self, _handle: TimerHandle) {} // Changed param type
    fn update_timer(&mut self, _handle: TimerHandle, _eventtime: f64) {} // Added

    fn register_fd(&mut self, _fd: i32, _callback: Box<dyn FnMut(f64)>) -> usize {0}
    fn unregister_fd(&mut self, _handle_id: usize) {}

    fn is_shutdown(&self) -> bool {false}
    fn run(&mut self) { println!("MainReactor: run called"); }
    fn pause(&self, _waketime: f64) {} // Changed to &self
    fn _check_timers(&mut self, _eventtime: f64, _idle: bool) {}
}

struct MainMcu;
impl Mcu for MainMcu {
    fn get_name(&self) -> String { "mcu".to_string() }
    fn register_config_callback(&self, _callback: Box<dyn Fn() + Send + Sync>) {}
    fn create_oid(&self) -> u8 { 0 }
    fn add_config_cmd(&self, _cmd: &str, _is_init: bool) {}
    fn estimated_print_time(&self, curtime: f64) -> f64 { curtime }
}

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
max_z_velocity: 15 # Lowered for more distinct Z homing speed
max_z_accel: 150   # Lowered for more distinct Z homing speed

[stepper_x]
position_min: 0
position_max: 210
position_endstop: 0.0 # Explicit float
rotation_distance: 40
full_steps_per_rotation: 200
microsteps: 16
# gear_ratio: 1.0 # Default
homing_speed: 40.0
second_homing_speed: 15.0
homing_retract_dist: 3

[stepper_y]
position_min: 0
position_max: 220
position_endstop: 0.0 # Explicit float
rotation_distance: 40
full_steps_per_rotation: 200
microsteps: 16
gear_ratio: 1.0
homing_speed: 40.0
second_homing_speed: 15.0
homing_retract_dist: 3

[stepper_z]
position_min: -2
position_max: 190
position_endstop: -0.5 # Example for Z endstop slightly below bed
rotation_distance: 8
full_steps_per_rotation: 200
microsteps: 16
homing_speed: 5.0
second_homing_speed: 2.0
homing_retract_dist: 1.0

[extruder]
heater_name: main_extruder
min_temp: 10
max_temp: 275

[heater_bed]
heater_name: main_bed
min_temp: 10
max_temp: 110

[fan]
name: print_cooling_fan
max_power: 0.9
off_below: 0.05
kick_start_time: 0.15
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
    // println!("X Endstop Homing Speed: {}", toolhead.x_endstop_internal_for_test().homing_speed); // Temporary access for verification - REMOVED

    let gcode_lines = vec![
        "G1 X1 Y1 F3000",           // Attempt move before homing (should fail)
        "G28",                      // Home all axes
        "M114",                     // Initial position after homing
        "M104 S210",                // Set extruder temp
        "M140 S60",                 // Set bed temp
        "M190 S60",                 // Wait for bed
        "M109 S210",                // Wait for extruder
        "G1 X10 Y10 Z10 F3000",     // Move
        "M106 S128",                // Fan 50% (of max_power)
        "M114",
        "M107",                     // Fan off
        "M104 S300",                // Error: extruder temp > max_temp (275)
        "M140 S5",                  // Error: bed temp < min_temp (10)
        "G1 X250",                  // Error: X > max (210)
        "G1 Y-1",                   // Error: Y < min (0)
        "G1 Z195",                  // Error: Z > max (190)
    ];

    for line_str in gcode_lines {
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
