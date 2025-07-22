use crate::configfile::Config;
use crate::toolhead::{DripCompletion, Move, ToolHead};

#[test]
fn test_toolhead_drip_move() {
    let config = Config::new();
    let mut toolhead = ToolHead::new(&config);
    let drip_completion = DripCompletion;
    toolhead.drip_move(&[0.0, 0.0, 0.0, 0.0], 0.0, &drip_completion);
}

#[test]
fn test_move_new() {
    let config = Config::new();
    let toolhead = ToolHead::new(&config);
    let start_pos = [0.0, 0.0, 0.0, 0.0];
    let end_pos = [0.0, 0.0, 0.0, 0.0];
    let speed = 0.0;
    let _move = Move::new(
        toolhead.max_velocity,
        toolhead.max_accel,
        toolhead.junction_deviation,
        toolhead.max_accel_to_decel,
        &start_pos,
        &end_pos,
        speed,
    );
}
