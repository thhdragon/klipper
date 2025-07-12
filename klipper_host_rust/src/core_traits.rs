// klipper_host_rust/src/core_traits.rs

use std::sync::Arc;
use std::any::Any;

// --- Reactor Trait ---
/// A trait for event loop and timer management.
pub trait Reactor: Send + Sync {
    /// Registers a callback to be called once after a delay.
    /// The callback takes the event time at which it's executed.
    fn register_callback_once(&self, delay_s: f64, callback: Box<dyn FnOnce(f64) + Send>);

    // TODO: Add methods for repeated timers, I/O event handling, etc.
    // fn register_timer(&self, delay_s: f64, callback: Box<dyn FnMut(f64) -> bool + Send>) -> TimerId;
    // fn unregister_timer(&self, id: TimerId);
}

// --- ConfigSection Trait ---
/// Represents a section of the Klipper configuration file.
pub trait ConfigSection: Send + Sync {
    fn get_name(&self) -> String; // e.g., "fan_generic my_fan" -> "my_fan" or full name based on usage
    fn get_str(&self, name: &str, default: Option<&str>) -> Result<String, String>; // String for simplicity
    fn get_float(&self, name: &str, default: Option<f64>, minval: Option<f64>, maxval: Option<f64>) -> Result<f64, String>;
    fn get_int(&self, name: &str, default: Option<i32>, minval: Option<i32>, maxval: Option<i32>) -> Result<i32, String>;
    fn get_bool(&self, name: &str, default: Option<bool>) -> Result<bool, String>;
    // TODO: getlist, etc.
}

// --- PinController Trait and PinParams Struct ---
/// Parameters describing a pin.
#[derive(Debug, Clone)]
pub struct PinParams {
    pub name: String,       // The original pin name string (e.g., "ar100:PA5")
    pub chip_name: String,  // MCU name (e.g., "mcu", "ar100")
    pub chip_pin: String,   // Pin name on the chip (e.g., "PA5")
    pub invert: bool,
    pub pullup: i32, // 0 for none, 1 for pullup, 2 for pulldown (consistent with Klipper's setup_pin)
    // TODO: Add other relevant fields like `is_adc`, etc.
}

/// Trait for managing and looking up pins.
pub trait PinController: Send + Sync {
    /// Looks up pin details. `can_invert` and `can_pullup` indicate if the pin syntax supports `!` and `^`.
    fn lookup_pin(&self, pin_name: &str, can_invert: bool, can_pullup: bool) -> Result<PinParams, String>;

    // TODO: Methods like setup_pin_digital_out, setup_pin_adc, etc.
    // fn setup_pin_digital_out(&self, params: PinParams) -> Result<Arc<dyn DigitalOutputPin>, String>;
    // fn setup_pin_adc(&self, params: PinParams) -> Result<Arc<dyn AdcPin>, String>;
}

// --- GCodeDispatcher Trait ---
/// For registering G-code command handlers.
pub trait GCodeDispatcher: Send + Sync {
    type GCodeHandler: FnMut(Arc<GCodeCommand>) -> Result<(), String> + Send + Sync + 'static;

    fn register_command(
        &self,
        command: &str, // e.g., "SET_FAN_SPEED"
        handler: Self::GCodeHandler,
        description: Option<&str>, // For help
    );

    /// For commands like M106/M107 that might be aliased or have prefixes.
    fn register_mux_command(
        &self,
        base_command: &str, // e.g., "SET_FAN_SPEED"
        param_name: &str,   // e.g., "FAN"
        param_value: &str,  // e.g., "my_fan_name"
        handler: Self::GCodeHandler,
        description: Option<&str>,
    );
}


// --- Mcu Trait ---
/// Represents a connection to a microcontroller.
pub trait Mcu: Send + Sync {
    fn get_name(&self) -> String;
    fn register_config_callback(&self, callback: Box<dyn Fn() + Send + Sync>); // Simplified
    fn create_oid(&self) -> u8; // Simplified
    fn add_config_cmd(&self, cmd: &str, is_init: bool);
    fn estimated_print_time(&self, curtime: f64) -> f64; // Added
    // alloc_command_queue, lookup_command, get_query_slot, seconds_to_clock, register_response etc.
    // These are more complex and will be detailed when mcu.rs is properly ported/designed.
    // For now, methods needed by buttons.py (if any direct ones beyond config) would go here.
    // `MCU_buttons` in python directly calls `self.mcu.lookup_command`, `self.mcu.alloc_command_queue` etc.
    // These would require more fleshed out command queue and response handling traits/structs.
}

// --- Printer Trait ---
/// The main application object, providing access to other components.
pub trait Printer: Send + Sync {
    fn get_reactor(&self) -> Arc<dyn Reactor>;
    fn lookup_object(&self, name: &str) -> Result<Arc<dyn Any + Send + Sync>, String>; // Use Any for now
    fn get_config(&self) -> Arc<dyn ConfigSection>; // Get the root config or a specific one

    // Helper to get a typed object
    // fn lookup_typed<T: 'static + Send + Sync>(&self, name: &str) -> Result<Arc<T>, String> {
    //     self.lookup_object(name)?
    //         .downcast_arc::<T>()
    //         .map_err(|_| format!("Object '{}' is not of the expected type", name))
    // }
}

// Placeholder for a way to downcast Arc<dyn Any + Send + Sync>
// pub fn downcast_arc<T: Any + Send + Sync>(arc_any: Arc<dyn Any + Send + Sync>) -> Result<Arc<T>, Arc<dyn Any + Send + Sync>> {
//     if arc_any.is::<T>() {
//         unsafe {
//             let ptr = Arc::into_raw(arc_any) as *const T;
//             Ok(Arc::from_raw(ptr))
//         }
//     } else {
//         Err(arc_any)
//     }
// }

// --- PrintKObject Trait ---
/// Trait for Klipper objects that can provide status information.
/// Corresponds to Klipper's PrintKObject class.
pub trait PrintKObject: Send + Sync {
    // In Klipper, get_status(eventtime) returns a dict.
    // Using serde_json::Value for flexibility, or a custom Status struct.
    fn get_status(&self, eventtime: f64) -> Result<serde_json::Value, String>;
    // Add other common methods from PrintKObject if needed, e.g., for logging.
}

// --- ButtonReactor Trait ---
/// Trait for reactor functionality specific to button handling (debouncing, callbacks).
/// This might be merged into the main Reactor trait or kept separate.
pub trait ButtonReactor: Send + Sync {
    // TODO: Define methods needed by extras/buttons.rs, e.g.,
    // fn register_button_callback(&mut self, ...) -> ButtonTimerHandle;
    // fn update_button_timer(&mut self, handle: ButtonTimerHandle, eventtime: f64);
    // fn unregister_button_timer(&mut self, handle: ButtonTimerHandle);
    // For now, it can be an empty marker trait if buttons.rs only needs it as a bound.
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[test]
    fn gcode_command_get_float_ok() {
        let mut params = HashMap::new();
        params.insert('X', "10.5".to_string());
        let gcmd = GCodeCommand { command: "G1".to_string(), command_letter: 'G', command_number: 1, params };
        assert_eq!(gcmd.get_float('X', None), Ok(10.5));
    }

    #[test]
    fn gcode_command_get_float_default() {
        let params = HashMap::new();
        let gcmd = GCodeCommand { command: "G1".to_string(), command_letter: 'G', command_number: 1, params };
        assert_eq!(gcmd.get_float('X', Some(5.0)), Ok(5.0));
    }

    #[test]
    fn gcode_command_get_float_missing() {
        let params = HashMap::new();
        let gcmd = GCodeCommand { command: "G1".to_string(), command_letter: 'G', command_number: 1, params };
        assert!(gcmd.get_float('X', None).is_err());
    }

    #[test]
    fn gcode_command_get_float_parse_error() {
        let mut params = HashMap::new();
        params.insert('X', "abc".to_string());
        let gcmd = GCodeCommand { command: "G1".to_string(), command_letter: 'G', command_number: 1, params };
        assert!(gcmd.get_float('X', None).is_err());
    }
}
