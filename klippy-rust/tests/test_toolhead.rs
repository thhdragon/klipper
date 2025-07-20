use klippy_rust::toolhead::{ToolHead, DripCompletion};
use klippy_rust::configfile::Config;

use klippy_rust::toolhead::Move;

#[test]
fn test_toolhead_drip_move() {
    let config = Config {};
    let mut toolhead = ToolHead::new(&config);
    let drip_completion = DripCompletion {};
    toolhead.drip_move([0.0, 0.0, 0.0, 0.0], 0.0, &drip_completion);
}

use klippy_rust::toolhead::LookAheadQueue;

#[test]
fn test_toolhead_move() {
    let config = Config {};
    let mut toolhead = ToolHead::new(&config);
    let start_pos = [0.0, 0.0, 0.0, 0.0];
    let end_pos = [10.0, 10.0, 0.0, 0.0];
    let speed = 100.0;
    let _move = Move::new(&mut toolhead, start_pos, end_pos, speed);
}

#[test]
fn test_lookahead_queue_new() {
    let _laq = LookAheadQueue::new();
}
