// klipper_host_rust/src/extras/fan.rs

#[derive(Debug, Clone, PartialEq)] // Added PartialEq for testing
pub struct Fan {
    pub name: String,
    pub speed: f32, // Represents speed from 0.0 to 1.0
    // TODO: Add other fields from Klipper's Fan class as needed:
    // max_power, kick_start_time, off_below, mcu_pin, etc.
}

impl Fan {
    pub fn new(name: String) -> Self {
        Fan {
            name,
            speed: 0.0, // Fan is off by default
        }
    }

    /// Sets the fan speed.
    /// speed_value is a float between 0.0 (off) and 1.0 (full speed).
    pub fn set_speed(&mut self, speed_value: f32) {
        self.speed = speed_value.max(0.0).min(1.0); // Clamp between 0.0 and 1.0
        // println!(
        //     "Fan '{}': Speed set to {:.2}",
        //     self.name, self.speed
        // );
        // TODO: In a real implementation, this would translate to PWM output to an MCU pin.
        // For now, we just store the speed.
    }

    /// Turns the fan off (sets speed to 0.0).
    pub fn turn_off(&mut self) {
        self.set_speed(0.0);
    }

    #[allow(dead_code)]
    pub fn get_speed(&self) -> f32 {
        self.speed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fan_creation_and_set_speed() {
        let mut fan = Fan::new("test_fan".to_string());
        assert_eq!(fan.name, "test_fan");
        assert_eq!(fan.speed, 0.0);

        fan.set_speed(0.5);
        assert_eq!(fan.speed, 0.5);

        fan.set_speed(1.0);
        assert_eq!(fan.speed, 1.0);

        // Test clamping
        fan.set_speed(1.5);
        assert_eq!(fan.speed, 1.0);

        fan.set_speed(-0.5);
        assert_eq!(fan.speed, 0.0);

        fan.turn_off();
        assert_eq!(fan.speed, 0.0);
    }
}
