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
        // This is a simplistic model. A real system has thermal mass, power limits, etc.
        // We'll use a factor that determines how much of the difference is closed per second.
        let heating_factor = 0.2; // Closes 20% of the gap per second when heating
        let cooling_factor = 0.1; // Closes 10% of the gap per second when cooling (passive cooling is slower)

        let diff = self.target_temp - self.current_temp;

        let change = if diff > 0.0 { // Heating
            diff * heating_factor * time_elapsed_s
        } else { // Cooling or at target
            diff * cooling_factor * time_elapsed_s // diff is negative, so change will be negative
        };

        self.current_temp += change;

        // Clamp to target to prevent overshooting in this simple model if time_elapsed_s is large
        if (diff > 0.0 && self.current_temp > self.target_temp) || (diff < 0.0 && self.current_temp < self.target_temp) {
            self.current_temp = self.target_temp;
        }

        // Ensure current_temp doesn't go below a reasonable ambient if target is 0 or very low.
        // This is a simplification; real ambient tracking is more complex.
        let min_ambient_like_temp = 15.0; // Don't let it cool below this in simulation if target is low
        if self.target_temp <= min_ambient_like_temp && self.current_temp < min_ambient_like_temp {
            // If target is off or very low, don't let simulation drop current_temp unrealistically low quickly.
            // A better model would have it asymptotically approach ambient.
            // For now, if it was cooling towards a low target and went below min_ambient_like_temp,
            // and the target itself is low, just set to min_ambient_like_temp.
            // This prevents cooling to absolute zero if target is 0.
            if change < 0.0 { // Only apply if it was a cooling step
                 self.current_temp = self.current_temp.max(min_ambient_like_temp);
            }
        }


        // For debugging the simulation:
        // println!("Heater '{}': Time {:.2}s, Current {:.2}°C, Target {:.2}°C, Change {:.3}", self.name, time_elapsed_s, self.current_temp, self.target_temp, change);
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
        heater.current_temp = 25.0; // 75 diff

        // Simulate 1 second of heating. Expected change: 75 * 0.2 * 1.0 = 15.0
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 40.0); // 25 + 15

        // Simulate multiple small steps
        heater.current_temp = 90.0; // 10 diff
        heater.update_current_temp(0.5); // Expected change: 10 * 0.2 * 0.5 = 1.0. New temp = 91.0
        assert_eq!(heater.current_temp, 91.0);
        heater.update_current_temp(0.5); // Diff 9. Expected change: 9 * 0.2 * 0.5 = 0.9. New temp = 91.9
        assert!((heater.current_temp - 91.9).abs() < 1e-9);


        // Simulate reaching target over a few steps
        heater.current_temp = 98.0; // 2 diff
        heater.update_current_temp(1.0); // Expected change: 2 * 0.2 * 1.0 = 0.4. New temp = 98.4
        assert_eq!(heater.current_temp, 98.4);
        heater.update_current_temp(1.0); // Diff 1.6. Expected change: 1.6 * 0.2 * 1.0 = 0.32. New temp = 98.72
        assert!((heater.current_temp - 98.72).abs() < 1e-9);

        // Simulate very close to target, one large step should clamp
        heater.current_temp = 99.9;
        heater.update_current_temp(5.0); // Large time step
        assert_eq!(heater.current_temp, 100.0);


        // Simulate already at target
        heater.current_temp = 100.0;
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 100.0);
    }

    #[test]
    fn test_update_current_temp_cooling() {
        let mut heater = Heater::new("test_heater".to_string());
        heater.set_target_temp(50.0);
        heater.current_temp = 100.0; // -50 diff

        // Simulate 1 second of cooling. Expected change: -50 * 0.1 * 1.0 = -5.0
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 95.0); // 100 - 5

        // Simulate reaching target
        heater.current_temp = 52.0; // -2 diff
        heater.update_current_temp(1.0); // Expected change: -2 * 0.1 * 1.0 = -0.2. New temp = 51.8
        assert_eq!(heater.current_temp, 51.8);

        // Simulate very close to target, one large step should clamp
        heater.current_temp = 50.1;
        heater.update_current_temp(5.0); // Large time step
        assert_eq!(heater.current_temp, 50.0);

        // Simulate already at target
        heater.current_temp = 50.0;
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 50.0);
    }

    #[test]
    fn test_update_current_temp_cooling_to_low_target_with_ambient_floor() {
        let mut heater = Heater::new("test_heater".to_string());
        heater.set_target_temp(0.0); // Target is off
        heater.current_temp = 25.0; // Start at ambient

        // Simulate 1 second of cooling. Diff = -25. Change = -25 * 0.1 * 1 = -2.5. New = 22.5
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 22.5);

        // Simulate more cooling
        // Current = 22.5, Target = 0. Diff = -22.5. Change = -22.5 * 0.1 * 1 = -2.25. New = 20.25
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 20.25);

        // Current = 20.25, Target = 0. Diff = -20.25. Change = -20.25 * 0.1 * 1 = -2.025. New = 18.225
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 18.225);

        // Current = 18.225, Target = 0. Diff = -18.225. Change = -18.225 * 0.1 * 1 = -1.8225. New = 16.4025
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 16.4025);

        // Current = 16.4025, Target = 0. Diff = -16.4025. Change = -16.4025 * 0.1 * 1 = -1.64025. New = 14.76225
        // This step will make current_temp go below 15.0, so it should be floored at 15.0.
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 15.0);

        // Further cooling should keep it at 15.0 if target is still 0
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 15.0);

        // If target is set to something higher, it should be able to heat up from 15
        heater.set_target_temp(20.0);
        // Current = 15.0, Target = 20.0. Diff = 5. Change = 5 * 0.2 * 1.0 = 1.0. New = 16.0
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 16.0);
    }


    #[test]
    fn test_check_target_reached() {
        let mut heater = Heater::new("test_heater".to_string());
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
        heater.current_temp = 25.0; // 75 diff

        // Simulate 1 second of heating. Expected change: 75 * 0.2 * 1.0 = 15.0
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 40.0); // 25 + 15

        // Simulate multiple small steps
        heater.current_temp = 90.0; // 10 diff
        heater.update_current_temp(0.5); // Expected change: 10 * 0.2 * 0.5 = 1.0. New temp = 91.0
        assert_eq!(heater.current_temp, 91.0);
        heater.update_current_temp(0.5); // Diff 9. Expected change: 9 * 0.2 * 0.5 = 0.9. New temp = 91.9
        assert!((heater.current_temp - 91.9).abs() < 1e-9);


        // Simulate reaching target over a few steps
        heater.current_temp = 98.0; // 2 diff
        heater.update_current_temp(1.0); // Expected change: 2 * 0.2 * 1.0 = 0.4. New temp = 98.4
        assert_eq!(heater.current_temp, 98.4);
        heater.update_current_temp(1.0); // Diff 1.6. Expected change: 1.6 * 0.2 * 1.0 = 0.32. New temp = 98.72
        assert!((heater.current_temp - 98.72).abs() < 1e-9);

        // Simulate very close to target, one large step should clamp
        heater.current_temp = 99.9;
        heater.update_current_temp(5.0); // Large time step
        assert_eq!(heater.current_temp, 100.0);


        // Simulate already at target
        heater.current_temp = 100.0;
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 100.0);
    }

    #[test]
    fn test_update_current_temp_cooling() {
        let mut heater = Heater::new("test_heater".to_string());
        heater.set_target_temp(50.0);
        heater.current_temp = 100.0; // -50 diff

        // Simulate 1 second of cooling. Expected change: -50 * 0.1 * 1.0 = -5.0
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 95.0); // 100 - 5

        // Simulate reaching target
        heater.current_temp = 52.0; // -2 diff
        heater.update_current_temp(1.0); // Expected change: -2 * 0.1 * 1.0 = -0.2. New temp = 51.8
        assert_eq!(heater.current_temp, 51.8);

        // Simulate very close to target, one large step should clamp
        heater.current_temp = 50.1;
        heater.update_current_temp(5.0); // Large time step
        assert_eq!(heater.current_temp, 50.0);

        // Simulate already at target
        heater.current_temp = 50.0;
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 50.0);
    }

    #[test]
    fn test_update_current_temp_cooling_to_low_target_with_ambient_floor() {
        let mut heater = Heater::new("test_heater".to_string());
        heater.set_target_temp(0.0); // Target is off
        heater.current_temp = 25.0; // Start at ambient

        // Simulate 1 second of cooling. Diff = -25. Change = -25 * 0.1 * 1 = -2.5. New = 22.5
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 22.5);

        // Simulate more cooling
        // Current = 22.5, Target = 0. Diff = -22.5. Change = -22.5 * 0.1 * 1 = -2.25. New = 20.25
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 20.25);

        // Current = 20.25, Target = 0. Diff = -20.25. Change = -20.25 * 0.1 * 1 = -2.025. New = 18.225
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 18.225);

        // Current = 18.225, Target = 0. Diff = -18.225. Change = -18.225 * 0.1 * 1 = -1.8225. New = 16.4025
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 16.4025);

        // Current = 16.4025, Target = 0. Diff = -16.4025. Change = -16.4025 * 0.1 * 1 = -1.64025. New = 14.76225
        // This step will make current_temp go below 15.0, so it should be floored at 15.0.
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 15.0);

        // Further cooling should keep it at 15.0 if target is still 0
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 15.0);

        // If target is set to something higher, it should be able to heat up from 15
        heater.set_target_temp(20.0);
        // Current = 15.0, Target = 20.0. Diff = 5. Change = 5 * 0.2 * 1.0 = 1.0. New = 16.0
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 16.0);
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
