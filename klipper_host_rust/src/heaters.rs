// klipper_host_rust/src/heaters.rs

#[derive(Debug, Clone, PartialEq)]
pub struct Heater {
    pub name: String,
    pub target_temp: f64,
    pub current_temp: f64,
    pub min_temp: f64,
    pub max_temp: f64,
    // TODO: Add other necessary fields from Klipper's Heater class as needed:
    // sensor, control algo, pwm_pin, etc.
}

impl Heater {
    pub fn new(name: String, min_temp: f64, max_temp: f64) -> Self {
        Heater {
            name,
            target_temp: 0.0,
            current_temp: 25.0, // Ambient temperature
            min_temp,
            max_temp,
        }
    }

    pub fn set_target_temp(&mut self, temp: f64) -> Result<(), String> {
        // Allow setting to 0 (off) regardless of min_temp
        if temp != 0.0 && (temp < self.min_temp || temp > self.max_temp) {
            return Err(format!(
                "Requested temperature {:.1} for {} is out of range ({:.1} to {:.1})",
                temp, self.name, self.min_temp, self.max_temp
            ));
        }
        self.target_temp = temp;
        // println!("Heater '{}': Target temperature set to {:.2}°C", self.name, self.target_temp);
        Ok(())
    }

    #[allow(dead_code)]
    pub fn get_target_temp(&self) -> f64 {
        self.target_temp
    }

    #[allow(dead_code)]
    pub fn get_current_temp(&self) -> f64 {
        self.current_temp
    }

    pub fn update_current_temp(&mut self, time_elapsed_s: f64) {
        if self.target_temp == self.current_temp {
            return;
        }
        let heating_factor = 0.2;
        let cooling_factor = 0.1;

        let diff = self.target_temp - self.current_temp;

        let change = if diff > 0.0 {
            diff * heating_factor * time_elapsed_s
        } else {
            diff * cooling_factor * time_elapsed_s
        };
        self.current_temp += change;
        if (diff > 0.0 && self.current_temp > self.target_temp) || (diff < 0.0 && self.current_temp < self.target_temp) {
            self.current_temp = self.target_temp;
        }
        let min_ambient_like_temp = 15.0;
        if self.target_temp <= min_ambient_like_temp && self.current_temp < min_ambient_like_temp {
            if change < 0.0 {
                 self.current_temp = self.current_temp.max(min_ambient_like_temp);
            }
        }
    }

    pub fn check_target_reached(&self, tolerance: f64) -> bool {
        if self.target_temp <= 0.0 {
            return true;
        }
        (self.current_temp - self.target_temp).abs() <= tolerance
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const TEMP_TOLERANCE: f64 = 1.0;

    #[test]
    fn heater_creation_and_set_target() {
        let mut heater = Heater::new("extruder_test".to_string(), 0.0, 280.0);
        assert_eq!(heater.name, "extruder_test");
        assert_eq!(heater.target_temp, 0.0);
        assert_eq!(heater.current_temp, 25.0);
        assert_eq!(heater.min_temp, 0.0);
        assert_eq!(heater.max_temp, 280.0);

        assert!(heater.set_target_temp(200.5).is_ok());
        assert_eq!(heater.target_temp, 200.5);

        // Test out of bounds
        let res_too_high = heater.set_target_temp(300.0);
        assert!(res_too_high.is_err());
        assert_eq!(res_too_high.unwrap_err(), "Requested temperature 300.0 for extruder_test is out of range (0.0 to 280.0)");
        assert_eq!(heater.target_temp, 200.5); // Should not change on error

        let res_too_low = heater.set_target_temp(-10.0);
        assert!(res_too_low.is_err());
        assert_eq!(res_too_low.unwrap_err(), "Requested temperature -10.0 for extruder_test is out of range (0.0 to 280.0)");
        assert_eq!(heater.target_temp, 200.5);

        assert!(heater.set_target_temp(0.0).is_ok()); // Setting to 0 (off) is allowed
        assert_eq!(heater.target_temp, 0.0);
    }

    #[test]
    fn test_update_current_temp_heating() {
        let mut heater = Heater::new("test_heater".to_string(), 0.0, 200.0);
        heater.set_target_temp(100.0).unwrap();
        heater.current_temp = 25.0;

        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 40.0);

        heater.current_temp = 90.0;
        heater.update_current_temp(0.5);
        assert_eq!(heater.current_temp, 91.0);
        heater.update_current_temp(0.5);
        assert!((heater.current_temp - 91.9).abs() < 1e-9);

        heater.current_temp = 98.0;
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 98.4);
        heater.update_current_temp(1.0);
        assert!((heater.current_temp - 98.72).abs() < 1e-9);

        heater.current_temp = 99.9;
        heater.update_current_temp(5.0);
        assert_eq!(heater.current_temp, 100.0);

        heater.current_temp = 100.0;
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 100.0);
    }

    #[test]
    fn test_update_current_temp_cooling() {
        let mut heater = Heater::new("test_heater".to_string(), 0.0, 200.0);
        heater.set_target_temp(50.0).unwrap();
        heater.current_temp = 100.0;

        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 95.0);

        heater.current_temp = 52.0;
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 51.8);

        heater.current_temp = 50.1;
        heater.update_current_temp(5.0);
        assert_eq!(heater.current_temp, 50.0);

        heater.current_temp = 50.0;
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 50.0);
    }

    #[test]
    fn test_update_current_temp_cooling_to_low_target_with_ambient_floor() {
        let mut heater = Heater::new("test_heater".to_string(), 0.0, 200.0);
        heater.set_target_temp(0.0).unwrap();
        heater.current_temp = 25.0;

        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 22.5);

        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 20.25);

        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 18.225);

        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 16.4025);

        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 15.0);

        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 15.0);

        heater.set_target_temp(20.0).unwrap();
        heater.update_current_temp(1.0);
        assert_eq!(heater.current_temp, 16.0);
    }


    #[test]
    fn test_check_target_reached() {
        let mut heater = Heater::new("test_heater".to_string(), 0.0, 250.0);
        heater.set_target_temp(200.0).unwrap();

        heater.current_temp = 199.0;
        assert!(heater.check_target_reached(TEMP_TOLERANCE));

        heater.current_temp = 201.0;
        assert!(heater.check_target_reached(TEMP_TOLERANCE));

        heater.current_temp = 198.9;
        assert!(!heater.check_target_reached(TEMP_TOLERANCE));

        heater.current_temp = 201.1;
        assert!(!heater.check_target_reached(TEMP_TOLERANCE));

        heater.current_temp = 200.0;
        assert!(heater.check_target_reached(TEMP_TOLERANCE));

        heater.set_target_temp(0.0).unwrap();
        heater.current_temp = 50.0;
        assert!(heater.check_target_reached(TEMP_TOLERANCE), "Target 0 should always be considered reached for wait purposes");
    }
}
