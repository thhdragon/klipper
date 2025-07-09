// Minimal stub for trapq.rs based on shaper.rs needs

use core::ptr::null_mut;

#[derive(Default, Copy, Clone, Debug, PartialEq)]
pub struct Vec2D {
    pub x: f64,
    pub y: f64,
    pub z: f64, // Klipper's Vec2D often includes Z, even if not always used by XY kinematics
}

#[derive(Copy, Clone)]
pub struct Move {
    pub move_t: f64,      // Total time for this move
    pub start_v: f64,     // Starting velocity
    pub end_v: f64,       // Ending velocity
    pub start_pos: Vec2D, // Starting position (x,y,z)
    pub axes_r: Vec2D,    // Ratio of movement along each axis (unit vector for direction * total_distance_on_this_move / move_t ? No, it's simpler)
                          // axes_r is direction unit vector components: e.g. {x: cos(angle), y: sin(angle), z: 0}
                          // such that total displacement is start_pos + axes_r * total_move_distance
    pub accel: f64,       // Acceleration for this move

    // Timing fields for trapezoidal acceleration/deceleration phases
    pub first_accel_t: f64, // Duration of initial acceleration phase
    pub cruise_t: f64,      // Duration of constant speed (cruise) phase
    pub last_accel_t: f64, // Duration of final acceleration/deceleration phase (this name can be confusing, it's time point)
                           // Klipper's trapq:
                           // move_t = accel_t + cruise_t + decel_t
                           // first_accel_t is duration of accel phase
                           // cruise_t is duration of cruise phase
                           // last_accel_t is time at which decel phase *ends* (relative to move start)
                           // So, duration of decel_phase = move_t - (first_accel_t + cruise_t)
                           // Let's adjust field names for clarity if this is a new Rust struct,
                           // but for porting, we keep original C names/meanings.
                           // C struct: first_accel_t=accel_t, cruise_t, last_accel_t=accel_t+cruise_t
                           // No, this is incorrect. `last_accel_t` in C is `s->accel_t + s->cruise_t`.
                           // It's the time *until the start* of the final deceleration phase.
                           // So, accel phase: 0 to first_accel_t
                           // cruise phase: first_accel_t to last_accel_t
                           // decel phase: last_accel_t to move_t

    // Junction speeds (squared) - not directly used by shaper's get_position, but part of Move
    pub max_start_v2: f64,
    pub max_cruise_v2: f64,
    pub max_smooth_v2: f64,

    // Linked list pointers for traversing moves.
    // These are problematic in safe Rust. For a direct port where shaper expects
    // to walk these, they might remain raw pointers.
    // `get_axis_position_across_moves` in shaper.c uses these.
    pub prev_move: *mut Move,
    pub next_move: *mut Move,
}

impl Default for Move {
    fn default() -> Self {
        Self {
            move_t: 0.0,
            start_v: 0.0,
            end_v: 0.0,
            start_pos: Vec2D::default(),
            axes_r: Vec2D::default(), // Represents normalized direction vector
            accel: 0.0,
            first_accel_t: 0.0, // Duration of acceleration phase
            cruise_t: 0.0,      // Duration of cruise phase
            last_accel_t: 0.0,  // Time point when deceleration phase begins (first_accel_t + cruise_t)
            max_start_v2: 0.0,
            max_cruise_v2: 0.0,
            max_smooth_v2: 0.0,
            prev_move: null_mut(),
            next_move: null_mut(),
        }
    }
}

impl Move {
    /// Calculate the distance covered at a given time `t` into this move.
    /// This is a simplified port of `move_get_distance` from `trapq.c`.
    /// Note: Klipper's `move_get_distance` considers `req_move_t` as total requested time
    /// and `move_t` as actual time for the move if shortened. Here, `move_time` is `t_val`
    /// from `calc_move_time` which is time into the current segment.
    pub fn move_get_distance(&self, time_in_move: f64) -> f64 {
        if time_in_move <= 0.0 {
            return 0.0;
        }
        // Ensure time_in_move does not exceed the move's total time
        let t = time_in_move.min(self.move_t);

        // Deceleration for the final phase (end_v vs cruise_v)
        // accel_t is self.first_accel_t
        // cruise_vel is self.start_v + self.accel * self.first_accel_t
        // decel_t is self.move_t - self.last_accel_t (where last_accel_t is start of decel phase)
        let cruise_v = self.start_v + self.accel * self.first_accel_t;
        let decel_phase_duration = self.move_t - self.last_accel_t;
        let decel = if decel_phase_duration > 1e-9 { // Avoid division by zero if no decel phase
            (cruise_v - self.end_v) / decel_phase_duration
        } else {
            0.0
        };


        if t <= self.first_accel_t {
            // Acceleration phase
            self.start_v * t + 0.5 * self.accel * t * t
        } else if t <= self.last_accel_t {
            // Cruise phase
            // Distance covered during acceleration phase
            let accel_dist = self.start_v * self.first_accel_t + 0.5 * self.accel * self.first_accel_t * self.first_accel_t;
            // Time spent in cruise phase so far
            let time_in_cruise = t - self.first_accel_t;
            accel_dist + cruise_v * time_in_cruise
        } else {
            // Deceleration phase
            // Distance covered during acceleration phase
            let accel_dist = self.start_v * self.first_accel_t + 0.5 * self.accel * self.first_accel_t * self.first_accel_t;
            // Distance covered during cruise phase
            let cruise_dist = cruise_v * self.cruise_t; // self.cruise_t is duration of cruise phase
            // Time spent in deceleration phase so far
            let time_in_decel = t - self.last_accel_t;
            accel_dist + cruise_dist + cruise_v * time_in_decel - 0.5 * decel * time_in_decel * time_in_decel
        }
    }

    /// Calculate the coordinate (X,Y,Z) at a given time `t` into this move.
    /// Port of `move_get_coord` from `trapq.c`.
    pub fn move_get_coord(&self, time_in_move: f64) -> Vec2D {
        let dist = self.move_get_distance(time_in_move);
        Vec2D {
            x: self.start_pos.x + self.axes_r.x * dist,
            y: self.start_pos.y + self.axes_r.y * dist,
            z: self.start_pos.z + self.axes_r.z * dist, // Assuming Z is handled, even if often 0 for XY
        }
    }

    // Helper for tests: creates a simple constant velocity move for a specific axis
    #[cfg(test)]
    pub fn new_test_move(axis: char, start_pos_val: f64, velocity: f64, duration: f64) -> Self {
        let mut m = Move::default();
        m.move_t = duration;
        m.start_v = velocity;
        m.end_v = velocity;
        m.accel = 0.0; // Constant velocity, so no acceleration phase
        m.first_accel_t = 0.0; // No accel phase
        m.cruise_t = duration; // Entire move is cruise phase
        m.last_accel_t = duration; // Decel phase starts at the end (effectively no decel phase)

        match axis {
            'x' => {
                m.start_pos.x = start_pos_val;
                m.axes_r.x = 1.0; // Moving along +X
            }
            'y' => {
                m.start_pos.y = start_pos_val;
                m.axes_r.y = 1.0; // Moving along +Y
            }
            'z' => {
                m.start_pos.z = start_pos_val;
                m.axes_r.z = 1.0; // Moving along +Z
            }
            _ => panic!("Invalid axis for test move"),
        }
        m
    }
}
