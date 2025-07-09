// klipper_host_rust/src/extras/fan.rs

// --- Configuration Parameters ---

pub struct FanParams {
    pub max_power: f64,
    pub kick_start_time: f64,
    pub off_below: f64,
    pub cycle_time: f64,
    pub hardware_pwm: bool,
    pub shutdown_speed: f64,
    // Pins are represented as Strings for now, actual pin objects would be resolved
    // by a pin resolver in a complete system.
    pub pin: String,
    pub enable_pin: Option<String>,
    pub tachometer_pin: Option<String>,
    pub tachometer_ppr: i32,
    pub tachometer_poll_interval: f64,
}

// --- Hardware Abstraction Traits ---

pub trait PwmPin {
    fn setup_max_duration(&mut self, max_duration: f64);
    fn setup_cycle_time(&mut self, cycle_time: f64, hardware_pwm: bool);
    fn setup_start_value(&mut self, start_value: f64, shutdown_value: f64);
    fn set_pwm(&mut self, print_time: f64, value: f64);
    // fn get_mcu(&self) -> Arc<Mcu>; // Assuming Mcu type and Arc for shared ownership
}

pub trait DigitalOutPin {
    fn setup_max_duration(&mut self, max_duration: f64);
    fn set_digital(&mut self, print_time: f64, value: u8);
}

pub trait FrequencyCounter {
    fn get_frequency(&self) -> f64;
    // fn new(printer: Printer, pin_name: String, sample_time: f64, poll_time: f64) -> Self;
}

// --- Main Fan Logic ---

pub struct Fan {
    // printer: Printer, // Reference to the global printer object
    params: FanParams,
    mcu_fan: Box<dyn PwmPin>, // Using Box<dyn Trait> for trait objects
    enable_pin: Option<Box<dyn DigitalOutPin>>,
    tachometer: FanTachometer,
    last_fan_value: f64,
    last_req_value: f64,
    // gcrq: GCodeRequestQueue, // To be decided how to handle this
}

pub struct FanTachometer {
    // printer: Printer,
    freq_counter: Option<Box<dyn FrequencyCounter>>,
    ppr: i32,
}

// --- G-Code Command Handler ---

pub struct PrinterFan {
    fan: Fan,
    // gcode: GCode, // Reference to GCode dispatcher
}

// --- Placeholder for types that would come from other modules ---
// These would be properly imported or defined elsewhere in a full port.

// pub struct Printer; // Placeholder
// pub struct GCode; // Placeholder
// pub struct Mcu; // Placeholder
// pub struct GCodeRequestQueue; // Placeholder

// --- Configuration Helper Trait (Simulation) ---
// In a real system, this would likely come from a central config module.
pub trait Config {
    fn get(&self, section: &str, key: &str) -> Option<String>; // Simplified: returns string
    fn get_float(&self, section: &str, key: &str, default: f64, minval: Option<f64>, maxval: Option<f64>) -> f64;
    fn get_int(&self, section: &str, key: &str, default: i32, minval: Option<i32>) -> i32;
    fn get_boolean(&self, section: &str, key: &str, default: bool) -> bool;
    fn get_name(&self) -> String; // To get the section name like "fan", "heater_fan my_custom_fan"
}


// --- Basic Implementations (stubs for now) ---

impl FanParams {
    // Constructor that simulates loading from a Klipper-like config object
    pub fn new_from_config(config: &impl Config, section_name: &str, default_shutdown_speed: f64) -> Self {
        // In Klipper, pin names are often prefixed by the MCU name, we'd need a pin resolver.
        // For now, we assume the config provides the resolved pin name string directly.
        let pin = config.get(section_name, "pin")
            .expect(&format!("'pin' is a required option for section '{}'", section_name));

        FanParams {
            pin,
            max_power: config.get_float(section_name, "max_power", 1.0, Some(0.0), Some(1.0)),
            kick_start_time: config.get_float(section_name, "kick_start_time", 0.1, Some(0.0), None),
            off_below: config.get_float(section_name, "off_below", 0.0, Some(0.0), Some(1.0)),
            cycle_time: config.get_float(section_name, "cycle_time", 0.010, Some(0.000000001), None), // ensure above 0
            hardware_pwm: config.get_boolean(section_name, "hardware_pwm", false),
            shutdown_speed: config.get_float(section_name, "shutdown_speed", default_shutdown_speed, Some(0.0), Some(1.0)),
            enable_pin: config.get(section_name, "enable_pin"),
            tachometer_pin: config.get(section_name, "tachometer_pin"),
            tachometer_ppr: config.get_int(section_name, "tachometer_ppr", 2, Some(1)),
            tachometer_poll_interval: config.get_float(section_name, "tachometer_poll_interval", 0.0015, Some(0.000000001), None), // ensure above 0
        }
    }
}

impl FanTachometer {
    pub fn new(
        // printer: Printer,
        params: &FanParams,
        freq_counter_resolver: impl FnOnce(String, f64, f64) -> Option<Box<dyn FrequencyCounter>>,
    ) -> Self {
        let freq_counter = params
            .tachometer_pin
            .as_ref()
            .and_then(|pin_name| {
                freq_counter_resolver(
                    pin_name.clone(),
                    1.0, // sample_time, hardcoded for now from python version
                    params.tachometer_poll_interval,
                )
            });

        Self {
            // printer,
            freq_counter,
            ppr: params.tachometer_ppr,
        }
    }

    pub fn get_status(&self, _eventtime: f64) -> FanTachometerStatus {
        // _eventtime is not used in the Python version's FanTachometer.get_status
        // but is present in the Fan's get_status, so kept for consistency if needed later.
        let rpm = self.freq_counter.as_ref().map(|fc| {
            fc.get_frequency() * 60.0 / self.ppr as f64 // Changed from 30.0 to 60.0 as per standard RPM calculation: freq (Hz) * 60 / PPR
        });
        FanTachometerStatus { rpm }
    }
}

// --- Status Structures ---
// It's good practice in Rust to define specific structs for status messages.

#[derive(Debug, PartialEq)]
pub struct FanTachometerStatus {
    pub rpm: Option<f64>,
}

#[derive(Debug, PartialEq)]
pub struct FanStatus {
    pub speed: f64,
    pub rpm: Option<f64>,
}

// Enum to manage kick-start state
enum KickStartPhase {
    Idle,
    Kicking(f64, f64), // target_value, kick_start_until_time
}


impl Fan {
    pub fn new(
        // printer: Printer,
        params: FanParams,
        pwm_pin_resolver: impl FnOnce(String) -> Box<dyn PwmPin>,
        digital_out_pin_resolver: impl FnOnce(String) -> Option<Box<dyn DigitalOutPin>>,
        freq_counter_resolver: impl FnOnce(String, f64, f64) -> Option<Box<dyn FrequencyCounter>>,
    ) -> Self {
        let mut mcu_fan = pwm_pin_resolver(params.pin.clone());
        mcu_fan.setup_max_duration(0.0);
        mcu_fan.setup_cycle_time(params.cycle_time, params.hardware_pwm);
        let shutdown_power = params.shutdown_speed.max(0.0).min(params.max_power);
        mcu_fan.setup_start_value(0.0, shutdown_power);

        let enable_pin = params
            .enable_pin
            .as_ref()
            .and_then(|pin_name| digital_out_pin_resolver(pin_name.clone()))
            .map(|mut pin| {
                pin.setup_max_duration(0.0);
                pin
            });

        let tachometer = FanTachometer::new(&params, freq_counter_resolver);

        Self {
            // printer,
            params,
            mcu_fan,
            enable_pin,
            tachometer,
            last_fan_value: 0.0, // Actual last value set to PWM
            last_req_value: 0.0, // Last requested speed (before off_below etc.)
            // gcrq: GCodeRequestQueue::new(config, mcu_fan.get_mcu(), self._apply_speed),
            // For now, we'll manage kick_start internally without a full GCodeRequestQueue
            // This state would ideally be managed by something akin to GCRQ or reactor events
            kick_start_phase: KickStartPhase::Idle,
        }
    }

    // Internal method, analogous to Python's _apply_speed
    // Returns true if a kickstart phase was initiated
    fn _apply_speed(&mut self, print_time: f64, requested_value: f64) -> bool {
        let mut internal_value = requested_value; // This is 0.0-1.0
        if internal_value < self.params.off_below {
            internal_value = 0.0;
        }

        // Calculate the actual PWM value to be applied to the pin
        // Python: value = max(0., min(self.max_power, value * self.max_power))
        // Here, 'value' in python's context is the requested_value (0.0-1.0)
        let pwm_value_to_set = (internal_value * self.params.max_power) // Scale input by max_power
                                .min(self.params.max_power)             // Clamp to max_power (ensures it's not > configured max_power)
                                .max(0.0);                              // Ensure non-negative

        if pwm_value_to_set == self.last_fan_value && matches!(self.kick_start_phase, KickStartPhase::Idle) {
            // If the PWM value hasn't changed and we are not in a kick-start phase that needs to revert, do nothing.
            // Also update last_req_value if it's different, so get_status is accurate.
            if self.last_req_value != requested_value {
                 self.last_req_value = requested_value;
            }
            return false;
        }

        self.last_req_value = requested_value; // Store the original requested value (0.0-1.0) for status

        if let Some(enable_pin_ref) = self.enable_pin.as_mut() {
            if pwm_value_to_set > 0.0 && self.last_fan_value == 0.0 {
                enable_pin_ref.set_digital(print_time, 1);
            } else if pwm_value_to_set == 0.0 && self.last_fan_value > 0.0 {
                enable_pin_ref.set_digital(print_time, 0);
            }
        }

        // Handle kick-start logic
        // Python: if (value and self.kick_start_time and (not self.last_fan_value or value - self.last_fan_value > .5)):
        // Here, 'value' in python's kick-start check is the final pwm_value_to_set
        let needs_kick_start = pwm_value_to_set > 0.0
            && self.params.kick_start_time > 0.0
            && (self.last_fan_value == 0.0 || (pwm_value_to_set - self.last_fan_value) > 0.5 * self.params.max_power);
            // The 0.5 in python is relative to max_power (1.0), so scale it if max_power is different.
            // A simpler check might be if `value > self.last_fan_value` when last_fan_value was not 0.
            // Let's stick to the python logic: "value - self.last_fan_value > .5" (assuming it implies 0.5 of full scale)


        if needs_kick_start {
            self.mcu_fan.set_pwm(print_time, self.params.max_power);
            self.last_fan_value = self.params.max_power; // PWM is now at max_power
            // Store the final intended pwm_value_to_set as the target for after kick-start
            self.kick_start_phase = KickStartPhase::Kicking(pwm_value_to_set, print_time + self.params.kick_start_time);
            return true; // Kickstart initiated
        } else {
            // If we were kicking and now need to set the target value
            if let KickStartPhase::Kicking(kick_target_pwm_value, kick_until) = self.kick_start_phase {
                if print_time >= kick_until {
                    // Kick start duration is over, set to actual target pwm value
                    self.mcu_fan.set_pwm(print_time, kick_target_pwm_value);
                    self.last_fan_value = kick_target_pwm_value;
                    self.kick_start_phase = KickStartPhase::Idle;
                } else {
                    // Still in kick phase, PWM is already at max_power.
                    // self.last_fan_value should correctly be self.params.max_power.
                    // The target for after kick (kick_target_pwm_value) is stored in KickStartPhase::Kicking.
                }
            } else { // Not kicking, or kick just finished by previous block
                self.mcu_fan.set_pwm(print_time, pwm_value_to_set);
                self.last_fan_value = pwm_value_to_set;
            }
            return false; // No new kickstart initiated
        }
    }

    /// Public method to set fan speed.
    /// `print_time` is the current time, crucial for kick-start timing.
    /// In a real system, this might queue a request. Here, it applies directly.
    pub fn set_speed(&mut self, print_time: f64, value: f64) {
        // If a kick start was initiated by _apply_speed, and a new set_speed comes
        // before the kick_start_time is over, we need to handle it.
        // The Python version relies on GCodeRequestQueue to delay the next command.

        let clamped_value = value.clamp(0.0, 1.0); // User requested speed 0.0-1.0

        // If currently kicking, and this new value is different from the target of the kick
        if let KickStartPhase::Kicking(kick_target_pwm, kick_until) = self.kick_start_phase {
            if print_time < kick_until {
                // Still in active kick phase. Update the target value for when kick ends.
                // The new target for KickStartPhase::Kicking should be the *PWM value*
                let new_target_pwm = (clamped_value * self.params.max_power)
                                        .min(self.params.max_power)
                                        .max(0.0);
                self.kick_start_phase = KickStartPhase::Kicking(new_target_pwm, kick_until);
                self.last_req_value = clamped_value; // Update the user's requested value (0.0-1.0)
                // PWM is already at max_power, do not call _apply_speed.
                // The actual change to new_target_pwm will happen via check_kick_start_completion.
                return;
            } else {
                // Kick phase should have completed, reset to Idle before applying new speed.
                self.kick_start_phase = KickStartPhase::Idle;
            }
        }

        self._apply_speed(print_time, clamped_value);
    }

    /// Call this periodically or before next command if not using a reactor system.
    /// This ensures that after a kick-start, the fan reverts to its target speed.
    pub fn check_kick_start_completion(&mut self, print_time: f64) {
        if let KickStartPhase::Kicking(target_pwm_value, kick_until) = self.kick_start_phase {
            if print_time >= kick_until {
                // Kick start duration is over.
                // We need to set the fan to target_pwm_value.
                // To do this, we call _apply_speed. _apply_speed expects a 0.0-1.0 `requested_value`.
                // We need to find a `requested_value` that would result in `target_pwm_value`.
                // Since target_pwm_value = (requested_value * max_power).min(max_power).max(0),
                // and assuming target_pwm_value is already valid (within 0 to max_power),
                // then requested_value = target_pwm_value / max_power (if max_power > 0).

                let requested_value_for_target_pwm = if self.params.max_power > 0.0 {
                    target_pwm_value / self.params.max_power
                } else {
                    0.0 // Avoid division by zero, fan should be off if max_power is 0
                };

                self.kick_start_phase = KickStartPhase::Idle; // Set to idle *before* calling _apply_speed
                                                              // to prevent re-kicking.
                // _apply_speed will then recalculate the pwm_value from requested_value_for_target_pwm,
                // which should result back in target_pwm_value. It also handles enable_pin logic.
                self._apply_speed(print_time, requested_value_for_target_pwm.clamp(0.0,1.0));
            }
        }
    }


    // To be called from G-Code handler
    pub fn set_speed_from_command(&mut self, print_time: f64, value: f64) {
        // In Python, this queues a G-code request.
        // For now, it's similar to set_speed.
        self.set_speed(print_time, value); // value is already 0.0-1.0
    }

    fn _handle_request_restart(&mut self, print_time: f64) {
        self.set_speed(print_time, 0.0);
    }

    pub fn get_status(&self, eventtime: f64) -> FanStatus {
        let tachometer_status = self.tachometer.get_status(eventtime);
        FanStatus {
            speed: self.last_req_value, // Report user's last requested speed (0.0-1.0)
            rpm: tachometer_status.rpm,
        }
    }

    // This would be called by a system event (e.g. printer.register_event_handler("gcode:request_restart",...))
    pub fn handle_request_restart(&mut self, print_time: f64) {
        self._handle_request_restart(print_time);
    }
}


impl PrinterFan {
    pub fn new(
        // printer: Printer, // Would be used to get GCode dispatcher, register events etc.
        // gcode_dispatcher: &mut GCode, // To register commands
        params: FanParams,
        pwm_pin_resolver: impl FnOnce(String) -> Box<dyn PwmPin>,
        digital_out_pin_resolver: impl FnOnce(String) -> Option<Box<dyn DigitalOutPin>>,
        freq_counter_resolver: impl FnOnce(String, f64, f64) -> Option<Box<dyn FrequencyCounter>>,
    ) -> Self {
        let fan = Fan::new(params, pwm_pin_resolver, digital_out_pin_resolver, freq_counter_resolver);

        // In a full system, command registration would happen here:
        // gcode_dispatcher.register_command("M106", Self::cmd_M106_wrapper);
        // gcode_dispatcher.register_command("M107", Self::cmd_M107_wrapper);
        // The wrappers would adapt the GCode command context to the method calls.

        Self {
            fan,
        }
    }

    /// Simulates M106 G-code command: Set Fan Speed
    /// S<speed>: Fan speed (0-255)
    /// print_time: Current time from the G-code processing context
    pub fn cmd_m106(&mut self, print_time: f64, s_param: Option<f64>) {
        // Ensure kickstart completion for any previous commands before applying new speed
        self.fan.check_kick_start_completion(print_time);

        let value = s_param.unwrap_or(255.0) / 255.0;
        self.fan.set_speed_from_command(print_time, value.clamp(0.0, 1.0));
    }

    /// Simulates M107 G-code command: Fan Off
    /// print_time: Current time from the G-code processing context
    pub fn cmd_m107(&mut self, print_time: f64) {
        // Ensure kickstart completion for any previous commands before applying new speed
        self.fan.check_kick_start_completion(print_time);

        self.fan.set_speed_from_command(print_time, 0.0);
    }

    /// Get status of the fan
    pub fn get_status(&self, eventtime: f64) -> FanStatus {
        self.fan.get_status(eventtime)
    }

    /// Handles printer restart requests by turning off the fan.
    /// This would be connected to an event system.
    pub fn handle_request_restart(&mut self, print_time: f64) {
        self.fan.handle_request_restart(print_time);
    }
}

// Minimal test to ensure the file compiles and basic structures are present.
#[cfg(test)]
mod tests {
    use super::*;

    use std::cell::RefCell;
    use std::rc::Rc;

    // Mock implementations for testing
    #[derive(Debug, Clone)]
    struct PwmCommand {
        print_time: f64,
        value: f64,
    }

    #[derive(Debug, Clone)]
    struct DigitalOutCommand {
        print_time: f64,
        value: u8,
    }

    #[derive(Default, Debug, Clone)]
    struct MockPwmPinState {
        max_duration: f64,
        cycle_time: f64,
        hardware_pwm: bool,
        start_value: f64,
        shutdown_value: f64,
        pwm_commands: Vec<PwmCommand>,
        current_pwm: f64,
    }

    // Using Rc<RefCell<...>> to allow multiple parts of the test setup to inspect the mock's state
    // This is common for more complex mocks in Rust.
    #[derive(Clone, Debug)]
    struct MockPwmPin {
        state: Rc<RefCell<MockPwmPinState>>,
    }
    impl MockPwmPin {
        fn new() -> Self {
            Self { state: Rc::new(RefCell::new(MockPwmPinState::default())) }
        }
        fn last_pwm(&self) -> Option<f64> {
            self.state.borrow().pwm_commands.last().map(|cmd| cmd.value)
        }
        fn current_pwm(&self) -> f64 {
            self.state.borrow().current_pwm
        }
    }
    impl PwmPin for MockPwmPin {
        fn setup_max_duration(&mut self, max_duration: f64) {
            self.state.borrow_mut().max_duration = max_duration;
        }
        fn setup_cycle_time(&mut self, cycle_time: f64, hardware_pwm: bool) {
            let mut state = self.state.borrow_mut();
            state.cycle_time = cycle_time;
            state.hardware_pwm = hardware_pwm;
        }
        fn setup_start_value(&mut self, start_value: f64, shutdown_value: f64) {
            let mut state = self.state.borrow_mut();
            state.start_value = start_value;
            state.shutdown_value = shutdown_value;
        }
        fn set_pwm(&mut self, print_time: f64, value: f64) {
            let mut state = self.state.borrow_mut();
            state.pwm_commands.push(PwmCommand { print_time, value });
            state.current_pwm = value;
        }
    }

    #[derive(Default, Debug, Clone)]
    struct MockDigitalOutPinState {
        max_duration: f64,
        commands: Vec<DigitalOutCommand>,
        current_value: u8,
    }

    #[derive(Clone, Debug)]
    struct MockDigitalOutPin {
         state: Rc<RefCell<MockDigitalOutPinState>>,
    }
    impl MockDigitalOutPin {
        fn new() -> Self {
            Self { state: Rc::new(RefCell::new(MockDigitalOutPinState::default())) }
        }
        fn last_value(&self) -> Option<u8> {
            self.state.borrow().commands.last().map(|cmd| cmd.value)
        }
        fn current_value(&self) -> u8 {
            self.state.borrow().current_value
        }
    }

    impl DigitalOutPin for MockDigitalOutPin {
        fn setup_max_duration(&mut self, max_duration: f64) {
            self.state.borrow_mut().max_duration = max_duration;
        }
        fn set_digital(&mut self, print_time: f64, value: u8) {
            let mut state = self.state.borrow_mut();
            state.commands.push(DigitalOutCommand { print_time, value });
            state.current_value = value;
        }
    }

    #[derive(Clone)]
    struct MockFrequencyCounter {
        freq: f64,
    }
    impl FrequencyCounter for MockFrequencyCounter {
        fn get_frequency(&self) -> f64 {
            self.freq
        }
    }

    // Helper to create a MockPwmPin and return it boxed, along with a way to access its state for assertions
    fn new_mock_pwm_pin_resolver(_pin_name: String) -> (Box<dyn PwmPin>, Rc<RefCell<MockPwmPinState>>) {
        let mock_pin = MockPwmPin::new();
        let state_access = mock_pin.state.clone();
        (Box::new(mock_pin), state_access)
    }

    // Helper for MockDigitalOutPin
    fn new_mock_digital_out_pin_resolver(_pin_name: String) -> (Option<Box<dyn DigitalOutPin>>, Option<Rc<RefCell<MockDigitalOutPinState>>>) {
        let mock_pin = MockDigitalOutPin::new();
        let state_access = mock_pin.state.clone();
        (Some(Box::new(mock_pin)), Some(state_access))
    }

    fn new_mock_freq_counter_resolver(_pin_name: String, freq: f64) -> Option<Box<dyn FrequencyCounter>> {
        Some(Box::new(MockFrequencyCounter { freq }))
    }


    use std::collections::HashMap;

    // Mock Config implementation for testing
    struct MockConfig {
        data: HashMap<(String, String), String>,
        name: String,
    }

    impl Config for MockConfig {
        fn get(&self, section: &str, key: &str) -> Option<String> {
            self.data.get(&(section.to_string(), key.to_string())).cloned()
        }

        fn get_float(&self, section: &str, key: &str, default: f64, minval: Option<f64>, maxval: Option<f64>) -> f64 {
            let val = self.get(section, key)
                .and_then(|s| s.parse::<f64>().ok())
                .unwrap_or(default);
            let val = minval.map_or(val, |min| val.max(min));
            maxval.map_or(val, |max| val.min(max))
        }

        fn get_int(&self, section: &str, key: &str, default: i32, minval: Option<i32>) -> i32 {
            let val = self.get(section, key)
                .and_then(|s| s.parse::<i32>().ok())
                .unwrap_or(default);
            minval.map_or(val, |min| val.max(min))
        }

        fn get_boolean(&self, section: &str, key: &str, default: bool) -> bool {
            self.get(section, key)
                .and_then(|s| match s.to_lowercase().as_str() {
                    "true" | "yes" | "1" => Some(true),
                    "false" | "no" | "0" => Some(false),
                    _ => None,
                })
                .unwrap_or(default)
        }
        fn get_name(&self) -> String {
            self.name.clone()
        }
    }


    #[test]
    fn can_create_fan_params_from_mock_config() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        config_data.insert(("fan".to_string(), "max_power".to_string()), "0.9".to_string());
        config_data.insert(("fan".to_string(), "kick_start_time".to_string()), "0.25".to_string());
        config_data.insert(("fan".to_string(), "enable_pin".to_string()), "PB1".to_string());

        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);

        assert_eq!(params.pin, "PA0");
        assert_eq!(params.max_power, 0.9);
        assert_eq!(params.kick_start_time, 0.25);
        assert_eq!(params.enable_pin, Some("PB1".to_string()));
        assert_eq!(params.off_below, 0.0); // Default
    }

    #[test]
    #[should_panic(expected = "'pin' is a required option for section 'fan'")]
    fn fan_params_panics_if_pin_is_missing() {
        let mock_config = MockConfig { data: HashMap::new(), name: "fan".to_string() };
        // This should panic because "pin" is missing
        FanParams::new_from_config(&mock_config, "fan", 0.0);
    }


    #[test]
    fn fan_tachometer_calculates_rpm_correctly() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan_generic my_fan".to_string(), "pin".to_string()), "PA0".to_string()); // Dummy pin for FanParams
        config_data.insert(("fan_generic my_fan".to_string(), "tachometer_pin".to_string()), "ar1".to_string());
        config_data.insert(("fan_generic my_fan".to_string(), "tachometer_ppr".to_string()), "4".to_string());
        let mock_config = MockConfig { data: config_data, name: "fan_generic my_fan".to_string() };

        let params = FanParams::new_from_config(&mock_config, "fan_generic my_fan", 0.0);

        assert_eq!(params.tachometer_pin, Some("ar1".to_string()));
        assert_eq!(params.tachometer_ppr, 4);

        let tach = FanTachometer::new(&params, |_, _, _| new_mock_freq_counter_resolver("ar1".to_string(), 100.0));

        let status = tach.get_status(0.0); // eventtime is unused for now
        // RPM = (100.0 Hz * 60) / 4 PPR = 6000 / 4 = 1500 RPM
        assert_eq!(status.rpm, Some(1500.0));
    }

    #[test]
    fn fan_tachometer_returns_none_rpm_if_no_tach_pin_configured() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string()); // Only main fan pin
        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };

        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);
        assert!(params.tachometer_pin.is_none());

        // Resolver for freq_counter should not be called if tachometer_pin is None.
        // The FanTachometer::new will handle the None tachometer_pin from params.
        let tach = FanTachometer::new(&params, |pn, _, _| panic!("Freq counter resolver should not be called for {}", pn) );
        let status = tach.get_status(0.0);
        assert_eq!(status.rpm, None);
    }

    // Helper to setup a Fan with mocks for testing
    fn setup_fan_with_mocks(
        params: FanParams,
    ) -> (
        Fan,
        Rc<RefCell<MockPwmPinState>>,
        Option<Rc<RefCell<MockDigitalOutPinState>>>,
        Option<Box<dyn FrequencyCounter>>, // Though FanTachometer owns this
    ) {
        let (pwm_pin, pwm_state) = new_mock_pwm_pin_resolver(params.pin.clone());
        let (enable_pin, enable_pin_state) = if params.enable_pin.is_some() {
            let (pin, state) = new_mock_digital_out_pin_resolver(params.enable_pin.clone().unwrap());
            (pin, Some(state.unwrap()))
        } else {
            (None, None)
        };

        let freq_counter = params.tachometer_pin.as_ref().and_then(|_| {
            // Defaulting to 0.0 freq for these tests unless specified
            new_mock_freq_counter_resolver(params.tachometer_pin.clone().unwrap(), 0.0)
        });

        // The Fan::new takes a resolver function, not the counter itself.
        // We need to pass a closure that captures our already created (or None) freq_counter.
        // This is a bit awkward because FanTachometer wants to create it.
        // For simplicity in Fan tests not focusing on tachometer, we can pass a resolver that returns None if not needed.
        let fan = Fan::new(params, |_| pwm_pin, |_| enable_pin, |_,_,_| None ); // Tachometer not prime focus here

        (fan, pwm_state, enable_pin_state, freq_counter)
    }


    #[test]
    fn fan_sets_basic_speed() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);

        let (mut fan, pwm_state, _, _) = setup_fan_with_mocks(params);

        fan.set_speed(1.0, 0.5); // time 1.0, speed 50%
        assert_eq!(pwm_state.borrow().current_pwm, 0.5 * fan.params.max_power);
        let status = fan.get_status(1.0);
        assert_eq!(status.speed, 0.5); // Normalized speed
    }

    #[test]
    fn fan_respects_max_power() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        config_data.insert(("fan".to_string(), "max_power".to_string()), "0.8".to_string());
        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);

        let (mut fan, pwm_state, _, _) = setup_fan_with_mocks(params);

        fan.set_speed(1.0, 1.0); // Request 100%
                                 // PWM should be max_power (0.8) * max_power_config (0.8) = 0.64
        assert_eq!(pwm_state.borrow().current_pwm, 0.8 * 0.8);
        let status = fan.get_status(1.0);
        assert_eq!(status.speed, 1.0); // Status reports requested speed relative to configured max_power
    }

    #[test]
    fn fan_handles_off_below() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        config_data.insert(("fan".to_string(), "off_below".to_string()), "0.1".to_string()); // Off if request is < 10%
        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);

        let (mut fan, pwm_state, _, _) = setup_fan_with_mocks(params);

        fan.set_speed(1.0, 0.05); // Request 5%
        assert_eq!(pwm_state.borrow().current_pwm, 0.0); // Should be off
        let status = fan.get_status(1.0);
        assert_eq!(status.speed, 0.05); // Status shows requested

        fan.set_speed(2.0, 0.15); // Request 15%
        assert_eq!(pwm_state.borrow().current_pwm, 0.15 * fan.params.max_power); // Should be on
    }

    #[test]
    fn fan_toggles_enable_pin() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        config_data.insert(("fan".to_string(), "enable_pin".to_string()), "PB0".to_string());
        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);

        let (mut fan, _, enable_pin_state_opt, _) = setup_fan_with_mocks(params);
        let enable_pin_state = enable_pin_state_opt.expect("Enable pin state should exist");

        assert_eq!(enable_pin_state.borrow().current_value, 0); // Starts off

        fan.set_speed(1.0, 0.5); // Turn on
        assert_eq!(enable_pin_state.borrow().current_value, 1);

        fan.set_speed(2.0, 0.0); // Turn off
        assert_eq!(enable_pin_state.borrow().current_value, 0);

        // Turn on again, then set to a value that's off_below if that was configured
        fan.set_speed(3.0, 0.5); // On
        assert_eq!(enable_pin_state.borrow().current_value, 1);

        // If off_below was say 0.1, and we set to 0.05, PWM would be 0, enable pin should go off
        let mut params_with_off_below = FanParams::new_from_config(&mock_config, "fan", 0.0);
        params_with_off_below.off_below = 0.1;
        let (mut fan_off_below, _, enable_pin_state_off_below_opt, _) = setup_fan_with_mocks(params_with_off_below);
        let enable_pin_state_off_below = enable_pin_state_off_below_opt.expect("Enable pin state should exist for off_below test");

        fan_off_below.set_speed(1.0, 0.5); // On
        assert_eq!(enable_pin_state_off_below.borrow().current_value, 1);
        fan_off_below.set_speed(2.0, 0.05); // Off due to off_below
        assert_eq!(enable_pin_state_off_below.borrow().current_value, 0);

    }

    #[test]
    fn fan_kick_start_logic() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        config_data.insert(("fan".to_string(), "kick_start_time".to_string()), "0.1".to_string()); // 100ms kick
        config_data.insert(("fan".to_string(), "max_power".to_string()), "1.0".to_string());
        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);
        let original_kick_time = params.kick_start_time;

        let (mut fan, pwm_state, _, _) = setup_fan_with_mocks(params);

        // 1. Initial set from 0 to 0.6 -> should kick
        fan.set_speed(1.0, 0.6);
        assert_eq!(pwm_state.borrow().current_pwm, 1.0); // Kicking at max_power (1.0)
        match fan.kick_start_phase {
            KickStartPhase::Kicking(target, until) => {
                assert_eq!(target, 0.6 * fan.params.max_power); // Target is 0.6 (scaled by max_power if not 1.0)
                assert_eq!(until, 1.0 + original_kick_time);
            }
            _ => panic!("Should be in kick start phase"),
        }

        // 2. Call set_speed again before kick_start_time is over with a NEW target speed
        fan.set_speed(1.05, 0.8); // Time is 1.05 (still within 1.0 + 0.1 kick duration)
        assert_eq!(pwm_state.borrow().current_pwm, 1.0); // Still kicking at max_power
        match fan.kick_start_phase {
            KickStartPhase::Kicking(target, until) => {
                assert_eq!(target, 0.8 * fan.params.max_power); // Target updated to 0.8
                assert_eq!(until, 1.0 + original_kick_time); // Kick end time remains the same
            }
            _ => panic!("Should still be in kick start phase with updated target"),
        }
        let status_during_kick = fan.get_status(1.05);
        assert_eq!(status_during_kick.speed, 0.8); // Status reflects the new target

        // 3. Call check_kick_start_completion after kick_start_time
        fan.check_kick_start_completion(1.0 + original_kick_time + 0.001); // Time is 1.101
        assert_eq!(pwm_state.borrow().current_pwm, 0.8 * fan.params.max_power); // Now set to the target 0.8
        assert!(matches!(fan.kick_start_phase, KickStartPhase::Idle));

        // 4. Set speed again, small increment, no kick if already running significantly
        fan.set_speed(2.0, 0.85); // from 0.8 to 0.85
        assert_eq!(pwm_state.borrow().current_pwm, 0.85 * fan.params.max_power);
        assert!(matches!(fan.kick_start_phase, KickStartPhase::Idle)); // No new kick

        // 5. Set speed to 0, then high again to trigger another kick
        fan.set_speed(3.0, 0.0);
        assert_eq!(pwm_state.borrow().current_pwm, 0.0);
        fan.set_speed(4.0, 0.7);
        assert_eq!(pwm_state.borrow().current_pwm, 1.0); // Kicking
         match fan.kick_start_phase {
            KickStartPhase::Kicking(target, until) => {
                assert_eq!(target, 0.7 * fan.params.max_power);
                assert_eq!(until, 4.0 + original_kick_time);
            }
            _ => panic!("Should be in kick start phase"),
        }
        fan.check_kick_start_completion(4.0 + original_kick_time + 0.001);
        assert_eq!(pwm_state.borrow().current_pwm, 0.7 * fan.params.max_power);
    }

    #[test]
    fn fan_handle_request_restart_turns_fan_off() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);

        let (mut fan, pwm_state, _, _) = setup_fan_with_mocks(params);

        fan.set_speed(1.0, 0.5);
        assert_ne!(pwm_state.borrow().current_pwm, 0.0);

        fan.handle_request_restart(2.0);
        assert_eq!(pwm_state.borrow().current_pwm, 0.0);
        let status = fan.get_status(2.0);
        assert_eq!(status.speed, 0.0);
    }

    #[test]
    fn fan_get_status_works_correctly() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        config_data.insert(("fan".to_string(), "max_power".to_string()), "0.8".to_string());
        // tachometer parts for Fan::new, though not primary for this Fan status test
        config_data.insert(("fan".to_string(), "tachometer_pin".to_string()), "PB1".to_string());
        config_data.insert(("fan".to_string(), "tachometer_ppr".to_string()), "2".to_string());

        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);

        // Need to create Fan with a tachometer that has a mock frequency counter
        let (pwm_pin_boxed, pwm_state_ref) = new_mock_pwm_pin_resolver(params.pin.clone());

        let tach_pin_name = params.tachometer_pin.clone().unwrap();
        let ppr = params.tachometer_ppr;
        let poll_interval = params.tachometer_poll_interval;

        let mut fan = Fan::new(
            params,
            |_| pwm_pin_boxed,
            |_| None, // No enable pin for this test
            move |pin_name, _sample_time, _poll_time| { // Closure for freq_counter_resolver
                if pin_name == tach_pin_name {
                    Some(Box::new(MockFrequencyCounter{ freq: 50.0 }) as Box<dyn FrequencyCounter>)
                } else { None }
            }
        );

        fan.set_speed(1.0, 0.75); // Request 75%
        // PWM will be 0.75 * max_power_config (0.8) = 0.6
        assert_eq!(pwm_state_ref.borrow().current_pwm, 0.75 * 0.8);

        let status = fan.get_status(1.0);
        assert_eq!(status.speed, 0.75); // Speed relative to max_power
        // RPM = (50Hz * 60) / 2PPR = 3000 / 2 = 1500
        assert_eq!(status.rpm, Some(1500.0));
    }


    // --- Update existing simple creation tests to use new mock resolvers ---
    #[test]
    fn can_create_fan_with_new_mocks() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);

        let _fan = Fan::new(
            params,
            |_| new_mock_pwm_pin_resolver("PA0".to_string()).0,
            |_| new_mock_digital_out_pin_resolver("PB0".to_string()).0, // Example if it had enable_pin
            |_,_,_| new_mock_freq_counter_resolver("PC0".to_string(), 0.0) // Example if it had tach_pin
        );
    }

    #[test]
    fn can_create_printer_fan_with_new_mocks() {
        let mut config_data = HashMap::new();
        config_data.insert(("fan".to_string(), "pin".to_string()), "PA0".to_string());
        let mock_config = MockConfig { data: config_data, name: "fan".to_string() };
        let params = FanParams::new_from_config(&mock_config, "fan", 0.0);

        let _printer_fan = PrinterFan::new(
            params,
            |_| new_mock_pwm_pin_resolver("PA0".to_string()).0,
            |_| new_mock_digital_out_pin_resolver("PB0".to_string()).0,
            |_,_,_| new_mock_freq_counter_resolver("PC0".to_string(), 0.0)
        );
    }
}
