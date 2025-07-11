// klipper_host_rust/src/heaters.rs

#[derive(Debug, Clone)]
pub struct Heater {
    pub name: String,
    pub target_temp: f64,
    pub current_temp: f64, // Will be mocked/updated manually for now
    // TODO: Add other necessary fields from Klipper's Heater class as needed:
    // min_temp, max_temp, sensor, control algo, pwm_pin, etc.
}

impl Heater {
    pub fn new(name: String) -> Self {
        Heater {
            name,
            target_temp: 0.0,
            current_temp: 25.0, // Ambient temperature
        }
    }

    pub fn set_target_temp(&mut self, temp: f64) {
        // TODO: Add min/max temp checks eventually, similar to Klipper:
        // if temp != 0.0 && (temp < self.min_temp || temp > self.max_temp) {
        //     // raise error
        // }
        self.target_temp = temp;
        println!("Heater '{}': Target temperature set to {:.2}°C", self.name, self.target_temp);
    }

    // For M105 or status reporting later
    #[allow(dead_code)]
    pub fn get_target_temp(&self) -> f64 {
        self.target_temp
    }

    #[allow(dead_code)]
    pub fn get_current_temp(&self) -> f64 {
        // In a real system, this would read from a sensor.
        self.current_temp
    }

    // Simulates temperature changing over time.
    // A more realistic simulation would involve PID logic and power output.
    // For now, a simple linear change.
    pub fn update_current_temp(&mut self, time_elapsed_s: f64) {
        if self.target_temp == self.current_temp {
            return;
        }

        // Simulate heating/cooling rate (e.g., degrees per second)
        // This is a very simplistic model.
        let change_rate_deg_s = if self.target_temp > self.current_temp {
            10.0 // Heating at 10 C/s (very fast for mock)
        } else {
            -5.0 // Cooling at 5 C/s
        };

        let max_change = change_rate_deg_s * time_elapsed_s;

        if self.target_temp > self.current_temp {
            self.current_temp += max_change;
            if self.current_temp > self.target_temp {
                self.current_temp = self.target_temp;
            }
        } else {
            self.current_temp += max_change; // max_change is negative for cooling
            if self.current_temp < self.target_temp {
                self.current_temp = self.target_temp;
            }
        }
        // For debugging the simulation:
        // println!("Heater '{}': Time {:.2}s, Current {:.2}°C, Target {:.2}°C", self.name, time_elapsed_s, self.current_temp, self.target_temp);
    }

    // Checks if the current temperature is within tolerance of the target temperature.
    pub fn check_target_reached(&self, tolerance: f64) -> bool {
        if self.target_temp <= 0.0 { // If target is off, consider it "reached" immediately for waiting purposes.
            return true;
        }
        (self.current_temp - self.target_temp).abs() <= tolerance
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const TEMP_UPDATE_INTERVAL: f64 = 0.1; // 100ms interval for simulation
    const TEMP_TOLERANCE: f64 = 1.0; // 1 degree C tolerance

    #[test]
    fn heater_creation_and_set_target() {
        let mut heater = Heater::new("extruder_test".to_string());
        assert_eq!(heater.name, "extruder_test");
        assert_eq!(heater.target_temp, 0.0);
        assert_eq!(heater.current_temp, 25.0);

        heater.set_target_temp(200.5);
        assert_eq!(heater.target_temp, 200.5);
    }

    #[test]
    fn test_update_current_temp_heating() {
        let mut heater = Heater::new("test_heater".to_string());
        heater.set_target_temp(100.0);
        heater.current_temp = 25.0;

        // Simulate 1 second of heating (10 C/s rate)
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 35.0);

        // Simulate reaching target
        heater.current_temp = 95.0;
        heater.update_current_temp(1.0); // Should reach 100.0 (95 + 10*1 = 105, capped at 100)
        assert_eq!(heater.current_temp, 100.0);

        // Simulate already at target
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 100.0);
    }

    #[test]
    fn test_update_current_temp_cooling() {
        let mut heater = Heater::new("test_heater".to_string());
        heater.set_target_temp(50.0);
        heater.current_temp = 100.0;

        // Simulate 1 second of cooling (-5 C/s rate)
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 95.0);

        // Simulate reaching target
        heater.current_temp = 53.0;
        heater.update_current_temp(1.0); // Should reach 50.0 (53 - 5*1 = 48, capped at 50)
        assert_eq!(heater.current_temp, 50.0);

        // Simulate already at target
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 50.0);
    }

    #[test]
    fn test_check_target_reached() {
        let mut heater = Heater::new("test_heater".to_string());
        heater.set_target_temp(200.0);

        heater.current_temp = 199.0;
        assert!(heater.check_target_reached(TEMP_TOLERANCE)); // Within 1.0 tolerance

        heater.current_temp = 201.0;
        assert!(heater.check_target_reached(TEMP_TOLERANCE));

        heater.current_temp = 198.9;
        assert!(!heater.check_target_reached(TEMP_TOLERANCE));

        heater.current_temp = 201.1;
        assert!(!heater.check_target_reached(TEMP_TOLERANCE));

        heater.current_temp = 200.0;
        assert!(heater.check_target_reached(TEMP_TOLERANCE));

        // Test with target_temp = 0 (off)
        heater.set_target_temp(0.0);
        heater.current_temp = 50.0; // Current temp doesn't matter if target is 0
        assert!(heater.check_target_reached(TEMP_TOLERANCE), "Target 0 should always be considered reached for wait purposes");
    }
}
