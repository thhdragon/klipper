use crate::stepcompress::{
    stepcompress_append, stepcompress_commit, stepcompress_get_step_dir, StepCompress,
};
use crate::trapq::{Move, TrapQ};
use libc::{c_char, c_double, c_int};

#[repr(C)]
pub enum ActiveFlags {
    X = 1 << 0,
    Y = 1 << 1,
    Z = 1 << 2,
}

pub type SkCalcCallback =
    extern "C" fn(sk: *mut StepperKinematics, m: *mut Move, move_time: c_double) -> c_double;
pub type SkPostCallback = extern "C" fn(sk: *mut StepperKinematics);

#[repr(C)]
pub struct StepperKinematics {
    pub step_dist: c_double,
    pub commanded_pos: c_double,
    pub sc: *mut StepCompress,
    pub last_flush_time: c_double,
    pub last_move_time: c_double,
    pub tq: *mut TrapQ,
    pub active_flags: c_int,
    pub gen_steps_pre_active: c_double,
    pub gen_steps_post_active: c_double,
    pub calc_position_cb: SkCalcCallback,
    pub post_cb: Option<SkPostCallback>,
}

#[derive(Clone, Copy)]
struct TimePos {
    time: f64,
    position: f64,
}

const SEEK_TIME_RESET: f64 = 0.000100;

fn itersolve_gen_steps_range(
    sk: &mut StepperKinematics,
    m: &Move,
    abs_start: f64,
    abs_end: f64,
) -> i32 {
    let calc_position_cb = sk.calc_position_cb;
    let half_step = 0.5 * sk.step_dist;
    let mut start = abs_start - m.print_time;
    let mut end = abs_end - m.print_time;
    if start < 0.0 {
        start = 0.0;
    }
    if end > m.move_t {
        end = m.move_t;
    }
    let mut old_guess = TimePos {
        time: start,
        position: sk.commanded_pos,
    };
    let mut guess = old_guess;
    let mut sdir = stepcompress_get_step_dir(sk.sc);
    let mut is_dir_change = false;
    let mut have_bracket = false;
    let mut check_oscillate = false;
    let mut target = sk.commanded_pos + if sdir != 0 { half_step } else { -half_step };
    let mut last_time = start;
    let mut low_time = start;
    let mut high_time = start + SEEK_TIME_RESET;
    if high_time > end {
        high_time = end;
    }
    loop {
        let guess_dist = guess.position - target;
        let og_dist = old_guess.position - target;
        let mut next_time =
            (old_guess.time * guess_dist - guess.time * og_dist) / (guess_dist - og_dist);
        if !(next_time > low_time && next_time < high_time) {
            if have_bracket {
                next_time = (low_time + high_time) * 0.5;
                check_oscillate = false;
            } else if guess.time >= end {
                break;
            } else {
                next_time = high_time;
                high_time = 2.0 * high_time - last_time;
                if high_time > end {
                    high_time = end;
                }
            }
        }
        old_guess = guess;
        guess.time = next_time;
        guess.position = (calc_position_cb)(sk, m as *const Move as *mut Move, next_time);
        let guess_dist = guess.position - target;
        if guess_dist.abs() > 0.000000001 {
            let rel_dist = if sdir != 0 { guess_dist } else { -guess_dist };
            if rel_dist > 0.0 {
                if have_bracket && old_guess.time <= low_time {
                    if check_oscillate {
                        old_guess = guess;
                    }
                    check_oscillate = true;
                }
                high_time = guess.time;
                have_bracket = true;
            } else if rel_dist < -(half_step + half_step + 0.000000010) {
                sdir = if sdir != 0 { 0 } else { 1 };
                target = if sdir != 0 {
                    target + half_step + half_step
                } else {
                    target - half_step - half_step
                };
                low_time = last_time;
                high_time = guess.time;
                is_dir_change = true;
                have_bracket = true;
                check_oscillate = false;
            } else {
                low_time = guess.time;
            }
            if !have_bracket || high_time - low_time > 0.000000001 {
                if !is_dir_change && rel_dist >= -half_step {
                    stepcompress_commit(sk.sc);
                }
                continue;
            }
        }
        let ret = stepcompress_append(sk.sc, sdir, m.print_time, guess.time);
        if ret != 0 {
            return ret;
        }
        target = if sdir != 0 {
            target + half_step + half_step
        } else {
            target - half_step - half_step
        };
        let mut seek_time_delta = 1.5 * (guess.time - last_time);
        if seek_time_delta < 0.000000001 {
            seek_time_delta = 0.000000001;
        }
        if is_dir_change && seek_time_delta > SEEK_TIME_RESET {
            seek_time_delta = SEEK_TIME_RESET;
        }
        last_time = low_time;
        guess.time = low_time;
        high_time = guess.time + seek_time_delta;
        if high_time > end {
            high_time = end;
        }
        is_dir_change = false;
        have_bracket = false;
        check_oscillate = false;
    }
    sk.commanded_pos = target - if sdir != 0 { half_step } else { -half_step };
    if let Some(post_cb) = sk.post_cb {
        post_cb(sk);
    }
    0
}

fn check_active(sk: &StepperKinematics, m: &Move) -> bool {
    let af = sk.active_flags;
    (af & ActiveFlags::X as i32 != 0 && m.axes_r.x != 0.0)
        || (af & ActiveFlags::Y as i32 != 0 && m.axes_r.y != 0.0)
        || (af & ActiveFlags::Z as i32 != 0 && m.axes_r.z != 0.0)
}

#[unsafe(no_mangle)]
pub extern "C" fn itersolve_generate_steps(sk: *mut StepperKinematics, flush_time: c_double) -> i32 {
    let sk = unsafe { &mut *sk };
    let last_flush_time = sk.last_flush_time;
    sk.last_flush_time = flush_time;
    if sk.tq.is_null() {
        return 0;
    }
    // trapq_check_sentinels(sk.tq);
    let m = unsafe { (*sk.tq).moves.front().unwrap() };
    while last_flush_time >= m.print_time + m.move_t {
        // m = list_next_entry(m, node);
    }
    let mut force_steps_time = sk.last_move_time + sk.gen_steps_post_active;
    let mut skip_count = 0;
    loop {
        let move_start = m.print_time;
        let move_end = move_start + m.move_t;
        if check_active(sk, m) {
            if skip_count > 0 && sk.gen_steps_pre_active > 0.0 {
                let mut abs_start = move_start - sk.gen_steps_pre_active;
                if abs_start < last_flush_time {
                    abs_start = last_flush_time;
                }
                if abs_start < force_steps_time {
                    // abs_start = force_steps_time;
                }
                // let mut pm = list_prev_entry(m, node);
                // while {
                //     skip_count -= 1;
                //     skip_count != 0 && pm.print_time > abs_start
                // } {
                //     pm = list_prev_entry(pm, node);
                // }
                // loop {
                //     let ret = itersolve_gen_steps_range(sk, pm, abs_start, flush_time);
                //     if ret != 0 {
                //         return ret;
                //     }
                //     pm = list_next_entry(pm, node);
                //     if pm == m {
                //         break;
                //     }
                // }
            }
            let ret = itersolve_gen_steps_range(sk, m, last_flush_time, flush_time);
            if ret != 0 {
                return ret;
            }
            if move_end >= flush_time {
                sk.last_move_time = flush_time;
                return 0;
            }
            skip_count = 0;
            sk.last_move_time = move_end;
            force_steps_time = sk.last_move_time + sk.gen_steps_post_active;
        } else {
            if move_start < force_steps_time {
                let mut abs_end = force_steps_time;
                if abs_end > flush_time {
                    abs_end = flush_time;
                }
                let ret = itersolve_gen_steps_range(sk, m, last_flush_time, abs_end);
                if ret != 0 {
                    return ret;
                }
                skip_count = 1;
            } else {
                skip_count += 1;
            }
            if flush_time + sk.gen_steps_pre_active <= move_end {
                return 0;
            }
        }
        // m = list_next_entry(m, node);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn itersolve_check_active(
    _sk: *mut StepperKinematics,
    _flush_time: c_double,
) -> c_double {
    // This function will also require careful porting.
    // I will start by outlining the structure and then fill in the details.
    // For now, I will just return 0.0.
    0.0
}

#[unsafe(no_mangle)]
pub extern "C" fn itersolve_is_active_axis(_sk: *mut StepperKinematics, _axis: c_char) -> i32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn itersolve_set_trapq(_sk: *mut StepperKinematics, _tq: *mut TrapQ) {}

#[unsafe(no_mangle)]
pub extern "C" fn itersolve_set_stepcompress(
    _sk: *mut StepperKinematics,
    _sc: *mut StepCompress,
    _step_dist: c_double,
) {
}

#[unsafe(no_mangle)]
pub extern "C" fn itersolve_calc_position_from_coord(
    _sk: *mut StepperKinematics,
    _x: c_double,
    _y: c_double,
    _z: c_double,
) -> c_double {
    0.0
}

#[unsafe(no_mangle)]
pub extern "C" fn itersolve_set_position(
    _sk: *mut StepperKinematics,
    _x: c_double,
    _y: c_double,
    _z: c_double,
) {
}

#[unsafe(no_mangle)]
pub extern "C" fn itersolve_get_commanded_pos(_sk: *mut StepperKinematics) -> c_double {
    0.0
}
