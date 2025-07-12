// klipper_host_rust/src/test_utils.rs

#[cfg(test)] // Or pub if used by main code, but seems test-specific
use crate::toolhead::Move; // Assuming Move is pub from toolhead module

#[cfg(test)]
pub fn create_test_move(
    start_pos: [f64; 4],
    end_pos: [f64; 4],
    speed: f64,
) -> Move {
    // Default values suitable for many tests.
    // These can be adjusted or made parameters if more control is needed.
    let max_accel = 3000.0;
    let junction_deviation = 0.01; // A common Klipper default is around 0.01 to 0.02
    let max_velocity = 500.0;
    let max_accel_to_decel = max_accel / 2.0; // Default Klipper behavior if not specified

    Move::new(
        max_accel,
        junction_deviation,
        max_velocity,
        max_accel_to_decel,
        start_pos,
        end_pos,
        speed,
    )
}
