use libc::{c_double, c_int};
use std::collections::VecDeque;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Coord {
    pub x: c_double,
    pub y: c_double,
    pub z: c_double,
}

#[repr(C)]
pub struct Move {
    pub print_time: c_double,
    pub move_t: c_double,
    pub start_v: c_double,
    pub half_accel: c_double,
    pub start_pos: Coord,
    pub axes_r: Coord,
}

pub struct TrapQ {
    pub moves: VecDeque<Move>,
    pub history: VecDeque<Move>,
}

#[repr(C)]
pub struct PullMove {
    pub print_time: c_double,
    pub move_t: c_double,
    pub start_v: c_double,
    pub accel: c_double,
    pub start_x: c_double,
    pub start_y: c_double,
    pub start_z: c_double,
    pub x_r: c_double,
    pub y_r: c_double,
    pub z_r: c_double,
}

impl Move {
    pub fn new() -> Self {
        Move {
            print_time: 0.0,
            move_t: 0.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: Coord { x: 0.0, y: 0.0, z: 0.0 },
            axes_r: Coord { x: 0.0, y: 0.0, z: 0.0 },
        }
    }

    pub fn get_distance(&self, move_time: f64) -> f64 {
        (self.start_v + self.half_accel * move_time) * move_time
    }

    pub fn get_coord(&self, move_time: f64) -> Coord {
        let move_dist = self.get_distance(move_time);
        Coord {
            x: self.start_pos.x + self.axes_r.x * move_dist,
            y: self.start_pos.y + self.axes_r.y * move_dist,
            z: self.start_pos.z + self.axes_r.z * move_dist,
        }
    }
}

const MAX_NULL_MOVE: f64 = 1.0;

impl TrapQ {
    pub fn new() -> Self {
        TrapQ {
            moves: VecDeque::new(),
            history: VecDeque::new(),
        }
    }

    fn add_move(&mut self, m: Move) {
        if let Some(prev) = self.moves.back() {
            if prev.print_time + prev.move_t < m.print_time {
                // Add a null move to fill time gap
                let mut null_move = Move::new();
                null_move.start_pos = m.start_pos;
                if prev.print_time == 0.0 && m.print_time > MAX_NULL_MOVE {
                    // Limit the first null move to improve numerical stability
                    null_move.print_time = m.print_time - MAX_NULL_MOVE;
                } else {
                    null_move.print_time = prev.print_time + prev.move_t;
                }
                null_move.move_t = m.print_time - null_move.print_time;
                self.moves.push_back(null_move);
            }
        }
        self.moves.push_back(m);
    }

    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        let mut i = 0;
        while i < self.moves.len() {
            let m = &self.moves[i];
            if m.print_time + m.move_t > print_time {
                break;
            }
            i += 1;
        }
        let expired_moves = self.moves.drain(..i);
        for m in expired_moves {
            if m.start_v != 0.0 || m.half_accel != 0.0 {
                self.history.push_front(m);
            }
        }

        if self.history.is_empty() {
            return;
        }

        let mut i = self.history.len();
        while i > 0 {
            let m = &self.history[i - 1];
            if m.print_time + m.move_t > clear_history_time {
                break;
            }
            i -= 1;
        }
        self.history.drain(i..);
    }

    pub fn set_position(&mut self, print_time: f64, pos_x: f64, pos_y: f64, pos_z: f64) {
        self.finalize_moves(std::f64::INFINITY, 0.0);

        while let Some(m) = self.history.front_mut() {
            if m.print_time < print_time {
                if m.print_time + m.move_t > print_time {
                    m.move_t = print_time - m.print_time;
                }
                break;
            }
            self.history.pop_front();
        }

        let mut m = Move::new();
        m.print_time = print_time;
        m.start_pos.x = pos_x;
        m.start_pos.y = pos_y;
        m.start_pos.z = pos_z;
        self.history.push_front(m);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn trapq_extract_old(
    tq: *mut TrapQ,
    p: *mut PullMove,
    max: c_int,
    start_time: c_double,
    end_time: c_double,
) -> c_int {
    let tq = unsafe { &mut *tq };
    let p_slice = unsafe { std::slice::from_raw_parts_mut(p, max as usize) };
    let mut res = 0;
    for m in &tq.history {
        if start_time >= m.print_time + m.move_t || res >= max {
            break;
        }
        if end_time <= m.print_time {
            continue;
        }
        let pull_move = &mut p_slice[res as usize];
        pull_move.print_time = m.print_time;
        pull_move.move_t = m.move_t;
        pull_move.start_v = m.start_v;
        pull_move.accel = 2.0 * m.half_accel;
        pull_move.start_x = m.start_pos.x;
        pull_move.start_y = m.start_pos.y;
        pull_move.start_z = m.start_pos.z;
        pull_move.x_r = m.axes_r.x;
        pull_move.y_r = m.axes_r.y;
        pull_move.z_r = m.axes_r.z;
        res += 1;
    }
    res
}

#[unsafe(no_mangle)]
pub extern "C" fn trapq_alloc() -> *mut TrapQ {
    Box::into_raw(Box::new(TrapQ::new()))
}

#[unsafe(no_mangle)]
pub extern "C" fn trapq_free(tq: *mut TrapQ) {
    if !tq.is_null() {
        unsafe {
            drop(Box::from_raw(tq));
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn trapq_append(
    tq: *mut TrapQ,
    mut print_time: c_double,
    accel_t: c_double,
    cruise_t: c_double,
    decel_t: c_double,
    start_pos_x: c_double,
    start_pos_y: c_double,
    start_pos_z: c_double,
    axes_r_x: c_double,
    axes_r_y: c_double,
    axes_r_z: c_double,
    start_v: c_double,
    cruise_v: c_double,
    accel: c_double,
) {
    let tq = unsafe { &mut *tq };
    let mut start_pos = Coord {
        x: start_pos_x,
        y: start_pos_y,
        z: start_pos_z,
    };
    let axes_r = Coord {
        x: axes_r_x,
        y: axes_r_y,
        z: axes_r_z,
    };

    if accel_t > 0.0 {
        let mut m = Move::new();
        m.print_time = print_time;
        m.move_t = accel_t;
        m.start_v = start_v;
        m.half_accel = 0.5 * accel;
        m.start_pos = start_pos;
        m.axes_r = axes_r;
        let end_pos = m.get_coord(accel_t);
        tq.add_move(m);

        print_time += accel_t;
        start_pos = end_pos;
    }
    if cruise_t > 0.0 {
        let mut m = Move::new();
        m.print_time = print_time;
        m.move_t = cruise_t;
        m.start_v = cruise_v;
        m.half_accel = 0.0;
        m.start_pos = start_pos;
        m.axes_r = axes_r;
        let end_pos = m.get_coord(cruise_t);
        tq.add_move(m);

        print_time += cruise_t;
        start_pos = end_pos;
    }
    if decel_t > 0.0 {
        let mut m = Move::new();
        m.print_time = print_time;
        m.move_t = decel_t;
        m.start_v = cruise_v;
        m.half_accel = -0.5 * accel;
        m.start_pos = start_pos;
        m.axes_r = axes_r;
        tq.add_move(m);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn trapq_finalize_moves(
    tq: *mut TrapQ,
    print_time: c_double,
    clear_history_time: c_double,
) {
    let tq = unsafe { &mut *tq };
    tq.finalize_moves(print_time, clear_history_time);
}

#[unsafe(no_mangle)]
pub extern "C" fn trapq_set_position(
    tq: *mut TrapQ,
    print_time: c_double,
    pos_x: c_double,
    pos_y: c_double,
    pos_z: c_double,
) {
    let tq = unsafe { &mut *tq };
    tq.set_position(print_time, pos_x, pos_y, pos_z);
}
