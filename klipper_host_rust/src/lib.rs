// klipper_host_rust/src/lib.rs

// This crate will contain the Rust port of the Klipper host software (klippy).

// Declare modules corresponding to the created skeleton files
pub mod klippy_main;
pub mod gcode;
pub mod toolhead;
pub mod mcu;
pub mod clocksync;
pub mod configfile;
pub mod pins;
pub mod reactor;
pub mod serialhdl;
pub mod stepper; // Host-side stepper logic
pub mod util;      // Host-side utilities
pub mod webhooks;
pub mod console;
pub mod mathutil;
pub mod msgproto;

// Declare module for the kinematics subdirectory
pub mod kinematics;

// Declare module for the extras subdirectory
pub mod extras;

// Basic test for the library itself
#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
