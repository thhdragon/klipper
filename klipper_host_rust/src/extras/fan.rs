// klipper_host_rust/src/extras/fan.rs

#[derive(Debug, Clone, PartialEq)]
pub struct Fan {
    pub name: String,
    pub speed: f32, // Represents actual PWM duty cycle (0.0 to max_power, effectively 0.0 to 1.0 after max_power scaling)
    pub max_power: f32,
    pub off_below: f32,
    pub kick_start_time: f32, // Field added, logic deferred
    // TODO: mcu_pin, etc.
}

impl Fan {
    pub fn new(name: String, max_power: f32, off_below: f32, kick_start_time: f32) -> Self {
        Fan {
            name,
            speed: 0.0, // Fan is off by default
            max_power: max_power.max(0.0).min(1.0), // Ensure max_power is 0.0-1.0
            off_below: off_below.max(0.0).min(1.0), // Ensure off_below is 0.0-1.0
            kick_start_time,
        }
    }

    /// Sets the fan speed.
    /// speed_request is a float between 0.0 (off) and 1.0 (full speed request from M106 after scaling S param).
    pub fn set_speed(&mut self, speed_request: f32) {
        let mut effective_speed = speed_request.max(0.0).min(1.0); // Clamp input request

        if effective_speed < self.off_below {
            effective_speed = 0.0;
        }

        // The actual PWM duty cycle to apply, considering max_power
        // Klipper: value = max(0., min(self.max_power, value * self.max_power))
        // This implies 'value' coming into _apply_speed is the 0-1 scaled S param from M106.
        // So, if M106 S255 (value=1.0), then final_pwm = 1.0 * self.max_power.
        // If M106 S128 (value=0.5), then final_pwm = 0.5 * self.max_power.
        let final_pwm_duty_cycle = effective_speed * self.max_power;

        // We store this final PWM duty cycle, which is already clamped by max_power by multiplication.
        // And also ensure it's not negative (though effective_speed is already >=0).
        self.speed = final_pwm_duty_cycle.max(0.0);


        // TODO: Implement kick_start_time logic if self.kick_start_time > 0.0
        // This would involve temporarily setting a higher speed.

        // println!(
        //     "Fan '{}': Requested {:.2}, Effective {:.2}, MaxPower {:.2}, OffBelow {:.2} => Final PWM Speed set to {:.2}",
        //     self.name, speed_request, effective_speed, self.max_power, self.off_below, self.speed
        // );
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
        // Test with default-like values passed from new, matching Klipper's Fan defaults
        let mut fan = Fan::new("test_fan".to_string(), 1.0, 0.0, 0.1);
        assert_eq!(fan.name, "test_fan");
        assert_eq!(fan.speed, 0.0);
        assert_eq!(fan.max_power, 1.0);
        assert_eq!(fan.off_below, 0.0);
        assert_eq!(fan.kick_start_time, 0.1);


        fan.set_speed(0.5);
        assert_eq!(fan.speed, 0.5);

        fan.set_speed(1.0);
        assert_eq!(fan.speed, 1.0);

        // Test clamping of input speed_request
        fan.set_speed(1.5); // Input clamped to 1.0, then 1.0 * max_power (1.0) = 1.0
        assert_eq!(fan.speed, 1.0);

        fan.set_speed(-0.5); // Input clamped to 0.0, then 0.0 * max_power (1.0) = 0.0
        assert_eq!(fan.speed, 0.0);

        fan.turn_off();
        assert_eq!(fan.speed, 0.0);
    }

    #[test]
    fn fan_set_speed_with_max_power() {
        let mut fan = Fan::new("test_fan_max_power".to_string(), 0.8, 0.0, 0.1);
        assert_eq!(fan.max_power, 0.8);

        fan.set_speed(1.0); // Request full speed
        assert!((fan.speed - 0.8).abs() < 1e-6, "Speed should be 1.0 * max_power (0.8)");

        fan.set_speed(0.5); // Request half speed
        assert!((fan.speed - (0.5 * 0.8)).abs() < 1e-6, "Speed should be 0.5 * max_power (0.4)");

        fan.set_speed(0.0);
        assert_eq!(fan.speed, 0.0);
    }

    #[test]
    fn fan_set_speed_with_off_below() {
        let mut fan = Fan::new("test_fan_off_below".to_string(), 1.0, 0.1, 0.1);
        assert_eq!(fan.off_below, 0.1);

        fan.set_speed(0.05); // Below off_below threshold
        assert_eq!(fan.speed, 0.0, "Speed should be 0 if request is below off_below");

        fan.set_speed(0.1); // At off_below threshold (Klipper's logic: if value < self.off_below, value = 0)
                            // So, 0.1 is NOT < 0.1, it should pass.
        assert_eq!(fan.speed, 0.1, "Speed should be 0.1 if request is at off_below");


        fan.set_speed(0.15); // Above off_below threshold
        assert_eq!(fan.speed, 0.15);
    }

    #[test]
    fn fan_set_speed_with_max_power_and_off_below() {
        let mut fan = Fan::new("test_fan_combo".to_string(), 0.75, 0.2, 0.1);

        // Request below off_below
        fan.set_speed(0.1);
        assert_eq!(fan.speed, 0.0);

        // Request at off_below (should not be turned off, then scaled by max_power)
        fan.set_speed(0.2);
        assert!((fan.speed - (0.2 * 0.75)).abs() < 1e-6, "Expected 0.2 * 0.75 = 0.15. Got {}", fan.speed);


        // Request above off_below, below 1.0
        fan.set_speed(0.5); // 0.5 * 0.75 = 0.375
        assert!((fan.speed - 0.375).abs() < 1e-6);

        // Request full speed (1.0)
        fan.set_speed(1.0); // 1.0 * 0.75 = 0.75
        assert!((fan.speed - 0.75).abs() < 1e-6);
    }

     #[test]
    fn fan_new_clamps_max_power_and_off_below() {
        let fan1 = Fan::new("clamp_test".to_string(), 1.5, -0.1, 0.1);
        assert_eq!(fan1.max_power, 1.0);
        assert_eq!(fan1.off_below, 0.0);

        let fan2 = Fan::new("clamp_test2".to_string(), 0.5, 1.2, 0.1);
        assert_eq!(fan2.max_power, 0.5);
        assert_eq!(fan2.off_below, 1.0);
    }
}
