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
        // For now, we can manually update it for testing M109/M190 later,
        // or just return a fixed value.
        self.current_temp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heater_creation_and_set_target() {
        let mut heater = Heater::new("extruder_test".to_string());
        assert_eq!(heater.name, "extruder_test");
        assert_eq!(heater.target_temp, 0.0);
        assert_eq!(heater.current_temp, 25.0);

        heater.set_target_temp(200.5);
        assert_eq!(heater.target_temp, 200.5);
    }
}
