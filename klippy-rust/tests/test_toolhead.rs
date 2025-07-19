use klippy_rust::toolhead::{ToolHead, DripCompletion};
use klippy_rust::configfile::Config;

#[test]
fn test_toolhead_drip_move() {
    let config = Config {};
    let mut toolhead = ToolHead::new(&config);
    let drip_completion = DripCompletion {};
    toolhead.drip_move([0.0, 0.0, 0.0, 0.0], 0.0, &drip_completion);
}
