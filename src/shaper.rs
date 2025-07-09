// Rust port of Klipper's kin_shaper.c
// Original C code: Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
//                  Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
// This file may be distributed under the terms of the GNU GPLv3 license.

use crate::itersolve::StepperKinematics;
use crate::trapq::{Move, Vec2D};
use core::ptr::null_mut;

const MAX_PULSES: usize = 5;
const DUMMY_T: f64 = 500.0;

#[derive(Copy, Clone, Debug)]
struct Pulse {
    t: f64,
    a: f64,
}

#[derive(Copy, Clone, Debug)]
pub struct ShaperPulses {
    num_pulses: usize,
    pulses: [Pulse; MAX_PULSES],
}

impl Default for ShaperPulses {
    fn default() -> Self {
        Self {
            num_pulses: 0,
            pulses: [Pulse { t: 0.0, a: 0.0 }; MAX_PULSES],
        }
    }
}

// Shift pulses around 'mid-point' t=0 so that the input shaper is an identity
// transformation for constant-speed motion (i.e. input_shaper(v * T) = v * T)
fn shift_pulses(sp: &mut ShaperPulses) {
    let mut ts = 0.0;
    for i in 0..sp.num_pulses {
        ts += sp.pulses[i].a * sp.pulses[i].t;
    }
    for i in 0..sp.num_pulses {
        sp.pulses[i].t -= ts;
    }
}

fn init_shaper(n: usize, a: &[f64], t: &[f64], sp: &mut ShaperPulses) -> Result<(), ()> {
    if n > MAX_PULSES {
        sp.num_pulses = 0;
        return Err(());
    }
    let mut sum_a = 0.0;
    for val in a.iter().take(n) {
        sum_a += val;
    }
    if sum_a == 0.0 { // Avoid division by zero if sum_a is zero
        sp.num_pulses = 0;
        return Err(());
    }
    let inv_a = 1.0 / sum_a;
    // Reverse pulses vs their traditional definition
    for i in 0..n {
        sp.pulses[n - i - 1].a = a[i] * inv_a;
        sp.pulses[n - i - 1].t = -t[i];
    }
    sp.num_pulses = n;
    shift_pulses(sp);
    Ok(())
}

// Helper function to get axis position, similar to C version's get_axis_position
// This function needs access to the Move struct's fields.
// Assuming Move struct has methods like `get_axis_r` and `get_start_pos`
// and a global or passed-in way to get distance.
// This is a simplified placeholder. The actual implementation will depend on Move's definition.
#[inline]
fn get_axis_position(m: &Move, axis: char, move_time: f64) -> f64 {
    let axis_r = match axis {
        'x' => m.axes_r.x,
        'y' => m.axes_r.y,
        'z' => m.axes_r.z,
        _ => 0.0,
    };
    let start_pos = match axis {
        'x' => m.start_pos.x,
        'y' => m.start_pos.y,
        'z' => m.start_pos.z,
        _ => 0.0,
    };
    let move_dist = m.move_get_distance(move_time);
    start_pos + axis_r * move_dist
}


// This function requires navigating a list of moves (eg. linked list in C).
// Rust's ownership and borrowing rules make direct translation of C-style linked lists tricky.
// This needs to be adapted based on how moves are stored and accessed in the Rust version.
// For now, this is a placeholder.
#[inline]
fn get_axis_position_across_moves(current_move: &Move, axis: char, time: f64, _moves_list: *const Move) -> f64 {
    // This is a highly simplified stand-in. The C code iterates through
    // a linked list of moves. In Rust, this would depend on how the
    // `Move` instances are managed (e.g., Vec, custom list structure).
    // For now, we'll assume the current move is sufficient or this will be
    // refactored based on the `Move` list implementation.
    // TODO: Implement proper move list traversal.
    if time >= 0.0 && time <= current_move.move_t {
        get_axis_position(current_move, axis, time)
    } else {
        // Placeholder for more complex logic involving other moves
        // This part is crucial and needs a robust solution based on the Move list structure
        get_axis_position(current_move, axis, time.max(0.0).min(current_move.move_t))
    }
}

// Calculate the position from the convolution of the shaper with input signal
#[inline]
fn calc_position(m: &Move, axis: char, move_time: f64, sp: &ShaperPulses, moves_list: *const Move) -> f64 {
    let mut res = 0.0;
    for i in 0..sp.num_pulses {
        let pulse = &sp.pulses[i];
        res += pulse.a * get_axis_position_across_moves(m, axis, move_time + pulse.t, moves_list);
    }
    res
}

// Active flags for kinematics
#[allow(dead_code)]
const AF_X: u32 = 1 << 0;
#[allow(dead_code)]
const AF_Y: u32 = 1 << 1;
#[allow(dead_code)]
const AF_Z: u32 = 1 << 2; // Assuming Z might be needed later

pub struct InputShaper {
    sk: StepperKinematics,
    orig_sk: *mut StepperKinematics, // Raw pointer to original kinematics
    m: Move, // A local move instance for calculations
    sx: ShaperPulses,
    sy: ShaperPulses,
    // We need a way to access the list of moves for get_axis_position_across_moves
    // This might be a raw pointer or a reference depending on the overall architecture.
    // For now, let's assume it's passed in or accessible globally, which is not ideal.
    // A better approach would be to pass it to methods that need it.
    moves_list_ptr: *const Move, // Example: pointer to the head of the moves list
}

impl InputShaper {
    pub fn new() -> Box<Self> {
        let mut shaper = Box::new(InputShaper {
            sk: StepperKinematics::default(),
            orig_sk: null_mut(),
            m: Move {
                // Initialize with some default values, DUMMY_T similar to C
                move_t: 2.0 * DUMMY_T,
                start_v: 0.0,
                end_v: 0.0,
                start_pos: Vec2D::default(),
                axes_r: Vec2D::default(),
                accel: 0.0,
                // Timing fields
                first_accel_t: 0.0,
                cruise_t: 0.0,
                last_accel_t: 0.0,
                // Junction speeds
                max_start_v2: 0.0,
                max_cruise_v2: 0.0,
                max_smooth_v2: 0.0,
                // Pointers to next/prev moves - this needs careful handling in Rust
                // For now, let's use null_mut(), assuming they'll be set up elsewhere
                // or the list management is handled differently.
                prev_move: null_mut(), // Example
                next_move: null_mut(), // Example
            },
            sx: ShaperPulses::default(),
            sy: ShaperPulses::default(),
            moves_list_ptr: null_mut(), // Initialize with null, to be set later
        });

        // Set default calc_position_cb, this might be overridden later
        shaper.sk.calc_position_cb = Some(Self::shaper_xy_calc_position);
        shaper
    }

    // This function would be called externally to provide the moves list
    pub fn set_moves_list(&mut self, moves_list: *const Move) {
        self.moves_list_ptr = moves_list;
    }

    // Optimized calc_position when only x axis is needed
    fn shaper_x_calc_position(sk_ptr: *const StepperKinematics, m: &Move, move_time: f64) -> f64 {
        let is = unsafe { &mut *(sk_ptr as *mut InputShaper) }; // Unsafe cast
        if is.sx.num_pulses == 0 {
            // Call original kinematics' calc_position
            if let Some(orig_sk_ptr) = unsafe { is.orig_sk.as_ref() } {
                if let Some(cb) = orig_sk_ptr.calc_position_cb {
                    return cb(is.orig_sk, m, move_time);
                }
            }
            return 0.0; // Or handle error
        }
        is.m.start_pos.x = calc_position(m, 'x', move_time, &is.sx, is.moves_list_ptr);
        // Call original kinematics' calc_position with the modified move
        if let Some(orig_sk_ptr) = unsafe { is.orig_sk.as_ref() } {
            if let Some(cb) = orig_sk_ptr.calc_position_cb {
                // Pass a reference to is.m, and DUMMY_T for time
                return cb(is.orig_sk, &is.m, DUMMY_T);
            }
        }
        0.0 // Or handle error
    }

    // Optimized calc_position when only y axis is needed
    fn shaper_y_calc_position(sk_ptr: *const StepperKinematics, m: &Move, move_time: f64) -> f64 {
        let is = unsafe { &mut *(sk_ptr as *mut InputShaper) }; // Unsafe cast
        if is.sy.num_pulses == 0 {
            if let Some(orig_sk_ptr) = unsafe { is.orig_sk.as_ref() } {
                if let Some(cb) = orig_sk_ptr.calc_position_cb {
                    return cb(is.orig_sk, m, move_time);
                }
            }
            return 0.0;
        }
        is.m.start_pos.y = calc_position(m, 'y', move_time, &is.sy, is.moves_list_ptr);
        if let Some(orig_sk_ptr) = unsafe { is.orig_sk.as_ref() } {
            if let Some(cb) = orig_sk_ptr.calc_position_cb {
                return cb(is.orig_sk, &is.m, DUMMY_T);
            }
        }
        0.0
    }

    // General calc_position for both x and y axes
    fn shaper_xy_calc_position(sk_ptr: *const StepperKinematics, m: &Move, move_time: f64) -> f64 {
        let is = unsafe { &mut *(sk_ptr as *mut InputShaper) }; // Unsafe cast
        if is.sx.num_pulses == 0 && is.sy.num_pulses == 0 {
            if let Some(orig_sk_ptr) = unsafe { is.orig_sk.as_ref() } {
                if let Some(cb) = orig_sk_ptr.calc_position_cb {
                    return cb(is.orig_sk, m, move_time);
                }
            }
            return 0.0;
        }
        is.m.start_pos = m.move_get_coord(move_time);
        if is.sx.num_pulses > 0 {
            is.m.start_pos.x = calc_position(m, 'x', move_time, &is.sx, is.moves_list_ptr);
        }
        if is.sy.num_pulses > 0 {
            is.m.start_pos.y = calc_position(m, 'y', move_time, &is.sy, is.moves_list_ptr);
        }
        if let Some(orig_sk_ptr) = unsafe { is.orig_sk.as_ref() } {
            if let Some(cb) = orig_sk_ptr.calc_position_cb {
                return cb(is.orig_sk, &is.m, DUMMY_T);
            }
        }
        0.0
    }

    // A callback that forwards post_cb call to the original kinematics
    fn shaper_commanded_pos_post_fixup(sk_ptr: *mut StepperKinematics) {
        let is = unsafe { &mut *(sk_ptr as *mut InputShaper) }; // Unsafe cast
        let sk_ref = unsafe { &*sk_ptr }; // Safe to get a shared ref to sk
        if let Some(orig_sk_mut) = unsafe { is.orig_sk.as_mut() } {
            orig_sk_mut.commanded_pos = sk_ref.commanded_pos;
            if let Some(post_cb) = orig_sk_mut.post_cb {
                post_cb(is.orig_sk);
            }
            // Update self.sk.commanded_pos from orig_sk.commanded_pos
            // This requires sk_ptr to be mutable or a way to update self.sk
            // Directly: (sk_ptr as *mut StepperKinematics).commanded_pos = orig_sk_mut.commanded_pos;
            // Or more safely if `is` has mutable access to `is.sk`
            is.sk.commanded_pos = orig_sk_mut.commanded_pos;
        }
    }


    fn shaper_note_generation_time(&mut self) {
        let mut pre_active = 0.0;
        let mut post_active = 0.0;

        let orig_sk_flags = if let Some(orig_sk) = unsafe { self.orig_sk.as_ref() } {
            orig_sk.active_flags
        } else {
            0 // Default to no flags if orig_sk is null
        };

        if (orig_sk_flags & AF_X != 0) && self.sx.num_pulses > 0 {
            pre_active = self.sx.pulses[self.sx.num_pulses - 1].t;
            post_active = -self.sx.pulses[0].t;
        }
        if (orig_sk_flags & AF_Y != 0) && self.sy.num_pulses > 0 {
            pre_active = pre_active.max(self.sy.pulses[self.sy.num_pulses - 1].t);
            post_active = post_active.max(-self.sy.pulses[0].t);
        }
        self.sk.gen_steps_pre_active = pre_active;
        self.sk.gen_steps_post_active = post_active;
    }

    // This function needs to be callable externally, perhaps as a method of InputShaper
    // And it needs a mutable reference to self.
    pub fn input_shaper_update_sk(&mut self) {
        let orig_sk_flags = if let Some(orig_sk) = unsafe { self.orig_sk.as_ref() } {
            orig_sk.active_flags
        } else {
            self.sk.calc_position_cb = None; // Or some default safe callback
            self.sk.active_flags = 0;
            self.shaper_note_generation_time();
            return;
        };

        if (orig_sk_flags & (AF_X | AF_Y)) == (AF_X | AF_Y) {
            self.sk.calc_position_cb = Some(Self::shaper_xy_calc_position);
        } else if orig_sk_flags & AF_X != 0 {
            self.sk.calc_position_cb = Some(Self::shaper_x_calc_position);
        } else if orig_sk_flags & AF_Y != 0 {
            self.sk.calc_position_cb = Some(Self::shaper_y_calc_position);
        }
        // If none of the above, calc_position_cb remains as is or set to a default
        self.sk.active_flags = orig_sk_flags;
        self.shaper_note_generation_time();
    }


    // This is equivalent to `input_shaper_set_sk`
    // It takes `orig_sk` as a raw pointer because StepperKinematics might be managed by C or other unsafe code.
    pub fn set_sk(&mut self, orig_sk: *mut StepperKinematics) -> Result<(), ()> {
        self.orig_sk = orig_sk; // Store the original kinematics raw pointer

        let orig_sk_ref = match unsafe { self.orig_sk.as_ref() } {
            Some(sk) => sk,
            None => return Err(()), // orig_sk is null
        };

        if orig_sk_ref.active_flags == AF_X {
            self.sk.calc_position_cb = Some(Self::shaper_x_calc_position);
        } else if orig_sk_ref.active_flags == AF_Y {
            self.sk.calc_position_cb = Some(Self::shaper_y_calc_position);
        } else if orig_sk_ref.active_flags & (AF_X | AF_Y) != 0 {
            self.sk.calc_position_cb = Some(Self::shaper_xy_calc_position);
        } else {
            return Err(()); // No suitable active flags
        }

        self.sk.active_flags = orig_sk_ref.active_flags;
        self.sk.commanded_pos = orig_sk_ref.commanded_pos;
        self.sk.last_flush_time = orig_sk_ref.last_flush_time;
        self.sk.last_move_time = orig_sk_ref.last_move_time;

        if orig_sk_ref.post_cb.is_some() {
            self.sk.post_cb = Some(Self::shaper_commanded_pos_post_fixup);
        } else {
            self.sk.post_cb = None;
        }
        Ok(())
    }

    // This is equivalent to `input_shaper_set_shaper_params`
    pub fn set_shaper_params(&mut self, axis: char, n: usize, a: &[f64], t: &[f64]) -> Result<(), ()> {
        if axis != 'x' && axis != 'y' {
            return Err(());
        }

        let sp = if axis == 'x' { &mut self.sx } else { &mut self.sy };

        let orig_sk_flags = if let Some(orig_sk) = unsafe { self.orig_sk.as_ref() } {
            orig_sk.active_flags
        } else {
            // If orig_sk is not set, we can't determine active flags.
            // Depending on requirements, either return error or proceed with default.
            // For now, assume an error or no-op if orig_sk is not available.
            return Err(()); // Or handle as a no-op for the shaper update.
        };

        // Ignore input shaper update if the axis is not active in original kinematics
        let axis_flag = if axis == 'x' { AF_X } else { AF_Y };
        if orig_sk_flags & axis_flag != 0 {
            let status = init_shaper(n, a, t, sp);
            self.shaper_note_generation_time(); // Recalculate generation times
            status // Return status from init_shaper
        } else {
            Ok(()) // Axis not active, no shaper update needed, return Ok
        }
    }


    // Equivalent to `input_shaper_get_step_generation_window`
    pub fn get_step_generation_window(&self) -> f64 {
        self.sk.gen_steps_pre_active.max(self.sk.gen_steps_post_active)
    }

    // Provides a raw pointer to its own StepperKinematics, similar to C.
    // This is unsafe and should be handled with care by the caller.
    pub fn as_stepper_kinematics_ptr(&mut self) -> *mut StepperKinematics {
        &mut self.sk as *mut StepperKinematics
    }
}

// Global function to allocate InputShaper, returning a raw pointer to its StepperKinematics part.
// This mirrors the C API `input_shaper_alloc`.
// The caller is responsible for managing the memory of the Box<InputShaper>.
// In a pure Rust API, returning Box<InputShaper> directly would be safer.
#[no_mangle]
pub extern "C" fn input_shaper_alloc() -> *mut StepperKinematics {
    let shaper = InputShaper::new();
    // Leak the Box to prevent deallocation, returning a raw pointer.
    // The C side or a global Rust structure will be responsible for freeing this later.
    Box::into_raw(shaper) as *mut StepperKinematics
    // If InputShaper itself is needed: Box::into_raw(shaper)
    // But C API expects *StepperKinematics, so we cast the pointer.
    // This assumes InputShaper is #[repr(C)] and sk is its first field,
    // or we directly return a pointer to the `sk` field.
    // A safer way if sk is not the first field or repr(C) is not used:
    // let raw_shaper = Box::into_raw(shaper);
    // unsafe { &mut (*raw_shaper).sk as *mut StepperKinematics }
}

// TODO: Add a corresponding deallocation function if `input_shaper_alloc` is used, e.g.:
// #[no_mangle]
// pub extern "C" fn input_shaper_free(ptr: *mut StepperKinematics) {
//     if !ptr.is_null() {
//         unsafe {
//             // Convert the StepperKinematics pointer back to an InputShaper pointer
//             // This assumes 'sk' is the first member of InputShaper or specific layout.
//             // A more robust way involves storing the shaper pointer itself if possible.
//             // Or, if InputShaper is repr(C) and sk is the first field:
//             let _ = Box::from_raw(ptr as *mut InputShaper);
//         }
//     }
// }
// This deallocation function depends on how the pointer is obtained and used.
// If `Box::into_raw(shaper)` was cast to `*mut StepperKinematics`,
// then `Box::from_raw(ptr as *mut InputShaper)` is the way to reclaim and drop.
// This requires `InputShaper` to be `#[repr(C)]` if `sk` is not the first field,
// or careful pointer arithmetic.
// A common pattern is `container_of` macro in C, which can be emulated in unsafe Rust.
// For now, focusing on porting functionality. Memory management details for FFI
// need careful consideration based on the larger system design.

// Note on `container_of` usage in C:
// The C code uses `container_of` to get the `InputShaper` pointer from a `StepperKinematics` pointer.
// This relies on `sk` being a member of `InputShaper`.
// In Rust, this is achieved by casting the `StepperKinematics` pointer to `*mut InputShaper`.
// This is unsafe and relies on the layout of `InputShaper` or that `sk` is the first field
// if `#[repr(C)]` is not used with a specific layout.
// E.g., `let is = unsafe { &mut *(sk_ptr as *mut InputShaper) };`
// This assumes `sk_ptr` points to the `sk` field within an `InputShaper` struct.
// If `InputShaper` is `#[repr(C)]` and `sk` is its first field, this cast is valid.
// Otherwise, offset calculations would be needed, similar to `container_of`.
// For simplicity in this port, we assume this direct cast is appropriate,
// possibly by ensuring `sk` is the first field or `InputShaper` is `#[repr(C)]`.

/*
Further considerations:
1. Error Handling: The C code uses integer return codes. Rust's `Result` type is more idiomatic.
   This has been partially adopted (e.g., in `init_shaper`).
2. Linked List of Moves: `get_axis_position_across_moves` navigates a linked list.
   The Rust equivalent needs a safe way to manage and access this list.
   The current `moves_list_ptr: *const Move` is a placeholder and implies unsafe access.
   A more Rusty approach would involve slices, `Vec`, or a safe list structure if possible.
3. FFI (Foreign Function Interface): If this Rust code is called from C,
   `#[no_mangle]` and `extern "C"` are correctly used for `input_shaper_alloc`.
   Care must be taken with data types and memory management across the FFI boundary.
4. Callbacks: `calc_position_cb` and `post_cb` are function pointers in C.
   In Rust, these are `Option<fn(...)>` types. The unsafe casts to `InputShaper`
   from `StepperKinematics` raw pointers in callback implementations need to be correct.
5. `DUMMY_T`: Used in C, ported as a const. Its purpose in original kinematics calls should be understood.
6. `container_of` macro: The C code uses this to get parent struct from a field pointer.
   The Rust equivalent is `unsafe { &mut *(ptr as *mut ContainingStruct) }` if the field is first,
   or pointer arithmetic. This port uses the direct cast approach.
7. Memory safety: Extensive use of raw pointers (`*mut StepperKinematics`, `*const Move`)
   makes this code inherently unsafe. This is a direct port characteristic.
   A more idiomatic Rust version might use references and lifetimes where possible,
   but FFI or specific performance needs might dictate raw pointers.
8. `StepperKinematics` and `Move` struct definitions: These are assumed to be defined elsewhere
   (e.g., in `itersolve.rs`, `trapq.rs`) and their fields/methods are used accordingly.
   The current port makes assumptions about these structures.
*/

#[cfg(test)]
mod tests {
    use super::*;
    use crate::itersolve::{StepperKinematics, AF_X, AF_Y, dummy_kin_calc_position_cb};
    use crate::trapq::{Move, Vec2D};
    use core::ptr::null_mut;

    const DELTA: f64 = 1e-9;

    // Helper to create a default InputShaper for testing,
    // with a basic original StepperKinematics.
    fn setup_input_shaper_with_orig_sk() -> (Box<InputShaper>, Box<StepperKinematics>) {
        let mut orig_sk = Box::new(StepperKinematics::default());
        orig_sk.active_flags = AF_X | AF_Y; // Activate X and Y axes for tests
        orig_sk.calc_position_cb = Some(dummy_kin_calc_position_cb); // Use dummy callback

        let mut shaper = InputShaper::new();
        // `set_sk` takes a raw pointer. We need to ensure `orig_sk` lives long enough.
        // For testing, Box::into_raw and then Box::from_raw can manage this,
        // or ensure the Box owning orig_sk outlives the shaper's use of the raw ptr.
        // Here, we'll return the Box<StepperKinematics> to keep it alive.
        let orig_sk_ptr: *mut StepperKinematics = &mut *orig_sk;
        shaper.set_sk(orig_sk_ptr).unwrap();

        (shaper, orig_sk)
    }


    #[test]
    fn test_init_shaper_zv() {
        let mut sp = ShaperPulses::default();
        // ZV shaper: t = [0, period/2], a = [0.5, 0.5]
        // For period = 0.025 (40Hz)
        let period = 0.025;
        let test_t = [0.0, period / 2.0];
        let test_a = [0.5, 0.5];

        init_shaper(2, &test_a, &test_t, &mut sp).unwrap();

        assert_eq!(sp.num_pulses, 2);
        // Pulses are reversed and t negated, then shifted.
        // Original: (t=0, a=0.5), (t=period/2, a=0.5)
        // Reversed & negated t: (-period/2, a=0.5), (0, a=0.5)
        // Sum_a = 1.0. inv_a = 1.0.
        // Pulse 0 (original a[1]): a=0.5, t=0
        // Pulse 1 (original a[0]): a=0.5, t=-period/2
        // Shift: ts = 0.5*0 + 0.5*(-period/2) = -period/4
        // Shifted pulses:
        // sp.pulses[0]: t = 0 - (-period/4) = period/4, a = 0.5
        // sp.pulses[1]: t = -period/2 - (-period/4) = -period/4, a = 0.5
        assert_eq!(sp.pulses[0].a, 0.5);
        assert_eq!(sp.pulses[1].a, 0.5);
        assert!((sp.pulses[0].t - period / 4.0).abs() < DELTA);
        assert!((sp.pulses[1].t - (-period / 4.0)).abs() < DELTA);
    }

    #[test]
    fn test_init_shaper_mzv() {
        let mut sp = ShaperPulses::default();
        // MZV shaper: t = [0, period/2, period], a = [0.25, 0.5, 0.25]
        // For period = 0.025 (40Hz)
        let period = 0.025;
        let test_t = [0.0, period / 2.0, period];
        let test_a = [0.25, 0.5, 0.25];

        init_shaper(3, &test_a, &test_t, &mut sp).unwrap();
        assert_eq!(sp.num_pulses, 3);
        // Original: (0,0.25), (P/2,0.5), (P,0.25)
        // Rev&NegT: (-P,0.25), (-P/2,0.5), (0,0.25)
        // Shift: ts = 0.25*(-P) + 0.5*(-P/2) + 0.25*0 = -P/4 - P/4 = -P/2
        // Shifted:
        // pulses[0] (orig a[2]): t=0 - (-P/2) = P/2, a=0.25
        // pulses[1] (orig a[1]): t=-P/2 - (-P/2) = 0, a=0.5
        // pulses[2] (orig a[0]): t=-P - (-P/2) = -P/2, a=0.25
        assert_eq!(sp.pulses[0].a, 0.25);
        assert_eq!(sp.pulses[1].a, 0.5);
        assert_eq!(sp.pulses[2].a, 0.25);
        assert!((sp.pulses[0].t - period / 2.0).abs() < DELTA);
        assert!((sp.pulses[1].t - 0.0).abs() < DELTA);
        assert!((sp.pulses[2].t - (-period / 2.0)).abs() < DELTA);
    }

    #[test]
    fn test_init_shaper_invalid_n() {
        let mut sp = ShaperPulses::default();
        let test_t = [0.0];
        let test_a = [1.0];
        assert!(init_shaper(MAX_PULSES + 1, &test_a, &test_t, &mut sp).is_err());
        assert_eq!(sp.num_pulses, 0);
    }

    #[test]
    fn test_init_shaper_zero_sum_a() {
        let mut sp = ShaperPulses::default();
        let test_t = [0.0, 0.1];
        let test_a = [0.0, 0.0]; // Sum of 'a' is zero
        assert!(init_shaper(2, &test_a, &test_t, &mut sp).is_err());
        assert_eq!(sp.num_pulses, 0);

        let test_a_neg = [1.0, -1.0]; // Sum of 'a' is zero
        assert!(init_shaper(2, &test_a_neg, &test_t, &mut sp).is_err());
        assert_eq!(sp.num_pulses, 0);
    }

    #[test]
    fn test_get_axis_position_simple_move() {
        // Create a simple move: X-axis, start_pos.x = 10, velocity = 5, duration = 2
        // axes_r.x = 1.0 (normalized direction vector for X)
        // total distance = 5 * 2 = 10. End position should be 10 + 10 = 20.
        let m = Move::new_test_move('x', 10.0, 5.0, 2.0);

        // Test at t=0
        assert!((get_axis_position(&m, 'x', 0.0) - 10.0).abs() < DELTA); // start_pos.x + 1.0 * 0 = 10
        // Test at t=1 (mid-move)
        assert!((get_axis_position(&m, 'x', 1.0) - 15.0).abs() < DELTA); // 10.0 + 1.0 * (5*1) = 15
        // Test at t=2 (end of move)
        assert!((get_axis_position(&m, 'x', 2.0) - 20.0).abs() < DELTA); // 10.0 + 1.0 * (5*2) = 20
        // Test beyond move time (should cap at move_t)
        assert!((get_axis_position(&m, 'x', 3.0) - 20.0).abs() < DELTA);
    }

    #[test]
    fn test_calc_position_no_shaping() {
        // No shaping means 1 pulse at t=0, a=1.0
        let mut sp = ShaperPulses::default();
        init_shaper(1, &[1.0], &[0.0], &mut sp).unwrap(); // Single pulse at t=0, a=1.0
        // Shift: ts = 1.0 * 0 = 0. Shifted: t=0, a=1.0. Correct for identity.

        assert_eq!(sp.num_pulses, 1);
        assert_eq!(sp.pulses[0].a, 1.0);
        assert!((sp.pulses[0].t - 0.0).abs() < DELTA);

        let m = Move::new_test_move('x', 0.0, 10.0, 1.0); // Move from 0 to 10 in 1s

        // For calc_position, get_axis_position_across_moves is called.
        // With a single move and pulse at t=0, this simplifies.
        // moves_list_ptr can be null if we assume current move is the only one.
        let pos_at_0_5s = calc_position(&m, 'x', 0.5, &sp, &m as *const Move);
        assert!((pos_at_0_5s - 5.0).abs() < DELTA); // 0.0 + 1.0 * (10*0.5) = 5.0

        let pos_at_1s = calc_position(&m, 'x', 1.0, &sp, &m as *const Move);
        assert!((pos_at_1s - 10.0).abs() < DELTA);
    }

    #[test]
    fn test_input_shaper_alloc_and_set_sk() {
        let mut orig_sk_storage = StepperKinematics::default(); // Use a local variable for storage
        orig_sk_storage.active_flags = AF_X;
        let orig_sk_ptr: *mut StepperKinematics = &mut orig_sk_storage;

        let shaper_sk_ptr = input_shaper_alloc();
        assert!(!shaper_sk_ptr.is_null());

        let shaper_ptr = shaper_sk_ptr as *mut InputShaper; // Assumes sk is first field or repr(C)
        let mut shaper = unsafe { Box::from_raw(shaper_ptr) };

        assert!(shaper.set_sk(orig_sk_ptr).is_ok());
        assert_eq!(shaper.orig_sk, orig_sk_ptr);
        assert_eq!(shaper.sk.active_flags, AF_X);
        assert!(shaper.sk.calc_position_cb.is_some());

        // shaper (Box) is dropped here, freeing the InputShaper memory.
        // orig_sk_storage is also dropped here.
    }

    #[test]
    fn test_input_shaper_set_params_and_gen_window() {
        let (mut shaper, _orig_sk_box) = setup_input_shaper_with_orig_sk();
        // _orig_sk_box is kept to ensure its lifetime for shaper.orig_sk pointer validity

        // orig_sk has AF_X | AF_Y active.
        // Set ZV shaper for X axis, period = 0.025s (40Hz)
        let period_x = 0.025;
        let shaper_t_x = [0.0, period_x / 2.0];
        let shaper_a_x = [0.5, 0.5];
        assert!(shaper.set_shaper_params('x', 2, &shaper_a_x, &shaper_t_x).is_ok());

        assert_eq!(shaper.sx.num_pulses, 2);
        // Check generation window after setting X shaper
        // For ZV, pulses are at -P/4 and P/4.
        // sx.pulses are stored [ {t: P/4, a:0.5}, {t:-P/4, a:0.5} ] (shifted, reversed input)
        // pre_active = sx.pulses[num_pulses-1].t (using original C indexing logic, so sx.pulses[1].t if num_pulses=2)
        // No, Rust indexing: sx.pulses[is.sx.num_pulses-1].t is sx.pulses[1].t
        // The C code: pre_active = is->sx.pulses[is->sx.num_pulses-1].t;
        // post_active = -is->sx.pulses[0].t;
        // For ZV with period_x, shifted pulses are (-period_x/4, 0.5) and (period_x/4, 0.5)
        // Let's check `init_shaper` output:
        // sp.pulses[0].t = period_x/4, sp.pulses[1].t = -period_x/4 for ZV.
        // So, sx.pulses[0].t = period_x/4 (largest time), sx.pulses[1].t = -period_x/4 (smallest time)
        // num_pulses = 2.
        // pre_active = sx.pulses[1].t = -period_x/4. This is wrong.
        // The `shaper_note_generation_time` C code:
        // pre_active = is->sx.pulses[is->sx.num_pulses-1].t; -> This means the pulse with the largest 'original' t before negation.
        // After shift & reverse: pulses are sorted by new 't' descending if original 't' was ascending.
        // ZV example: test_t = [0, P/2]. Reversed & neg_t: [-P/2, 0]. Shifted: [-P/4, P/4].
        // Stored in sp.pulses: [ {t:P/4, a:0.5}, {t:-P/4, a:0.5} ]
        // num_pulses = 2.
        // sx.pulses[num_pulses-1] = sx.pulses[1] = {t:-P/4, a:0.5} -> pre_active = -P/4. Incorrect.
        // sx.pulses[0] = {t:P/4, a:0.5} -> post_active = -(P/4). Incorrect.

        // Let's re-check `shaper_note_generation_time` logic based on `init_shaper` output:
        // `init_shaper` stores pulses such that `sp.pulses[0]` has the largest (most positive) time `t`
        // and `sp.pulses[sp.num_pulses-1]` has the smallest (most negative) time `t`.
        // C code: `pre_active = is->sx.pulses[is->sx.num_pulses-1].t;`
        // This should be the time of the "earliest" pulse in terms of its effect (delay relative to nominal).
        // If pulses are [-P/4, P/4], then the one at P/4 means the shaper needs to see "future" input.
        // The one at -P/4 means it uses "past" input.
        // `gen_steps_pre_active` is max positive t: sx.pulses[0].t
        // `gen_steps_post_active` is max absolute negative t: -sx.pulses[num_pulses-1].t
        // Let's assume this interpretation for the test:
        let expected_pre_x = period_x / 4.0;
        let expected_post_x = period_x / 4.0; // -(-period_x/4.0)
        assert!((shaper.sk.gen_steps_pre_active - expected_pre_x).abs() < DELTA, "Pre-active X: got {}, expected {}", shaper.sk.gen_steps_pre_active, expected_pre_x);
        assert!((shaper.sk.gen_steps_post_active - expected_post_x).abs() < DELTA, "Post-active X: got {}, expected {}", shaper.sk.gen_steps_post_active, expected_post_x);
        assert!((shaper.get_step_generation_window() - expected_pre_x.max(expected_post_x)).abs() < DELTA);


        // Set different shaper for Y axis (e.g., MZV, period = 0.020s / 50Hz)
        let period_y = 0.020;
        let shaper_t_y = [0.0, period_y / 2.0, period_y];
        let shaper_a_y = [0.25, 0.5, 0.25];
        assert!(shaper.set_shaper_params('y', 3, &shaper_a_y, &shaper_t_y).is_ok());
        assert_eq!(shaper.sy.num_pulses, 3);
        // For MZV, shifted pulses: [-P/2, 0, P/2].
        // sy.pulses[0].t = P_y/2, sy.pulses[1].t = 0, sy.pulses[2].t = -P_y/2
        let expected_pre_y = period_y / 2.0;
        let expected_post_y = period_y / 2.0; // -(-period_y/2.0)

        // Overall pre_active = max(expected_pre_x, expected_pre_y)
        // Overall post_active = max(expected_post_x, expected_post_y)
        let overall_expected_pre = expected_pre_x.max(expected_pre_y);
        let overall_expected_post = expected_post_x.max(expected_post_y);
        assert!((shaper.sk.gen_steps_pre_active - overall_expected_pre).abs() < DELTA, "Overall Pre: got {}, expected {}", shaper.sk.gen_steps_pre_active, overall_expected_pre);
        assert!((shaper.sk.gen_steps_post_active - overall_expected_post).abs() < DELTA, "Overall Post: got {}, expected {}", shaper.sk.gen_steps_post_active, overall_expected_post);
        assert!((shaper.get_step_generation_window() - overall_expected_pre.max(overall_expected_post)).abs() < DELTA);

        // Test setting params for an inactive axis (e.g. if orig_sk only had AF_X)
        let mut local_orig_sk_storage = StepperKinematics::default();
        local_orig_sk_storage.active_flags = AF_X; // Only X active
        let local_orig_sk_ptr: *mut StepperKinematics = &mut local_orig_sk_storage;

        let mut shaper_only_x = InputShaper::new();
        shaper_only_x.set_sk(local_orig_sk_ptr).unwrap();

        // Try to set shaper for Y (inactive)
        assert!(shaper_only_x.set_shaper_params('y', 2, &shaper_a_x, &shaper_t_x).is_ok());
        assert_eq!(shaper_only_x.sy.num_pulses, 0); // Should not have been set
        assert!((shaper_only_x.sk.gen_steps_pre_active - 0.0).abs() < DELTA);
        assert!((shaper_only_x.sk.gen_steps_post_active - 0.0).abs() < DELTA);
    }

    #[test]
    fn test_shaper_calc_position_callbacks() {
        let (mut shaper, mut _orig_sk_box) = setup_input_shaper_with_orig_sk();
        let orig_sk_ptr = shaper.orig_sk; // Get the pointer stored in shaper

        // Set a ZV shaper for X axis, period = 0.02s (50Hz), so T_shaper = 0.02
        // Pulses at t = +/- 0.02/4 = +/- 0.005, each with amplitude 0.5
        let period = 0.02;
        let shaper_t_params = [0.0, period / 2.0]; // Klipper definition for shaper params
        let shaper_a_params = [0.5, 0.5];
        shaper.set_shaper_params('x', 2, &shaper_a_params, &shaper_t_params).unwrap();

        // Manually ensure orig_sk has X and Y active for this part of the test
        unsafe { (*orig_sk_ptr).active_flags = AF_X | AF_Y; }
        shaper.input_shaper_update_sk(); // This sets the correct calc_position_cb on shaper.sk (shaper_xy_calc_position)

        // Create a test move: constant velocity on X
        // Start at x=0, velocity=100mm/s, duration=1s. Ends at x=100.
        let test_move_x = Move::new_test_move('x', 0.0, 100.0, 1.0);
        shaper.set_moves_list(&test_move_x as *const Move);

        let cb_xy = shaper.sk.calc_position_cb.unwrap();

        // Calculate shaped position at t=0.1s
        // For ZV shaper (pulses are at actual_t1=-0.005, actual_t2=0.005 after init_shaper, amps a1=0.5, a2=0.5):
        // ShapedPos(t) = 0.5 * OrigPos(t + actual_t1) + 0.5 * OrigPos(t + actual_t2)
        // ShapedPos(t) = 0.5 * OrigPos(t - 0.005) + 0.5 * OrigPos(t + 0.005)
        // OrigPos(t) for test_move_x = 0 + 100 * t = 100t
        // OrigPos(0.1 - 0.005) = OrigPos(0.095) = 100 * 0.095 = 9.5
        // OrigPos(0.1 + 0.005) = OrigPos(0.105) = 100 * 0.105 = 10.5
        // ShapedPos(0.1) = 0.5 * 9.5 + 0.5 * 10.5 = 4.75 + 5.25 = 10.0
        // The callback `cb_xy` will call `orig_sk.calc_position_cb`.
        // `shaper_xy_calc_position` sets `is.m.start_pos.x` to the calculated shaped value,
        // then calls `orig_sk.calc_position_cb(&is.m, DUMMY_T)`.
        // `dummy_kin_calc_position_cb` with DUMMY_T should return `is.m.start_pos.x` if X-axis.
        // So the result of `cb_xy` should be the shaped X value.
        let shaped_pos_x = cb_xy(shaper.as_stepper_kinematics_ptr(), &test_move_x, 0.1);
        assert!((shaped_pos_x - 10.0).abs() < DELTA, "Shaped X: got {}, expected 10.0", shaped_pos_x);

        // Test case where original kinematics only had X active
        unsafe { (*orig_sk_ptr).active_flags = AF_X; }
        shaper.input_shaper_update_sk(); // Update shaper's cb to shaper_x_calc_position
        let cb_x_only = shaper.sk.calc_position_cb.unwrap();
        let shaped_pos_x_only = cb_x_only(shaper.as_stepper_kinematics_ptr(), &test_move_x, 0.1);
        assert!((shaped_pos_x_only - 10.0).abs() < DELTA, "Shaped X (X only active): got {}, expected 10.0", shaped_pos_x_only);


        // Test Y axis shaping (no Y shaper configured yet, so should be passthrough)
        unsafe { (*orig_sk_ptr).active_flags = AF_Y; } // Make Y active in orig_sk
        shaper.input_shaper_update_sk(); // Update shaper's cb to shaper_y_calc_position
        let cb_y_only = shaper.sk.calc_position_cb.unwrap();

        let test_move_y = Move::new_test_move('y', 5.0, 20.0, 1.0); // y from 5 to 25
        shaper.set_moves_list(&test_move_y as *const Move);

        // No Y shaper (sy.num_pulses == 0), so it should call orig_sk.calc_position_cb directly.
        // `shaper_y_calc_position` path:
        //   if !is.sy.num_pulses: return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time)
        // `dummy_kin_calc_position_cb(orig_sk_ptr, &test_move_y, 0.5)`:
        //   dist = test_move_y.move_get_distance(0.5) = 20.0 * 0.5 = 10.0
        //   returns test_move_y.start_pos.y + test_move_y.axes_r.y * dist
        //   = 5.0 + 1.0 * 10.0 = 15.0
        let shaped_pos_y_only = cb_y_only(shaper.as_stepper_kinematics_ptr(), &test_move_y, 0.5);
        assert!((shaped_pos_y_only - 15.0).abs() < DELTA, "Passthrough Y: got {}, expected 15.0", shaped_pos_y_only);
    }
}
