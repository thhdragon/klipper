use klippy_rust::trapq::{TrapQ, Move, Coord};

#[test]
fn test_trapq_new() {
    let tq = TrapQ::new();
    assert_eq!(tq.moves.len(), 0);
    assert_eq!(tq.history.len(), 0);
}

#[test]
fn test_trapq_add_move() {
    let mut tq = TrapQ::new();
    let m = Move {
        print_time: 1.0,
        move_t: 2.0,
        start_v: 3.0,
        half_accel: 4.0,
        start_pos: Coord { x: 5.0, y: 6.0, z: 7.0 },
        axes_r: Coord { x: 8.0, y: 9.0, z: 10.0 },
    };
    tq.moves.push_back(m);
    assert_eq!(tq.moves.len(), 1);
    let pulled_move = tq.moves.pop_front().unwrap();
    assert_eq!(pulled_move.print_time, 1.0);
}
