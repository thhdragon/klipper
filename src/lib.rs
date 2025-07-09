#![no_std]

// core imports
use core::panic::PanicInfo;

// External crate dependencies (will be listed in Cargo.toml)
// use bitflags::bitflags; // Already in stepper.rs
// use heapless; // Already in stepper.rs

// --- Public Modules ---
pub mod board_pins;
pub mod command;
pub mod gcode;
pub mod hal;        // Hardware Abstraction Layer traits
pub mod itersolve;
pub mod kinematics; // Will contain submodules like cartesian, corexy, delta
pub mod mcu;
pub mod sched;
pub mod stepcompress;
pub mod stepper;    // Stepper motor control logic
pub mod trapq;
pub mod utils;      // Utility functions (crc, timing helpers, etc.)


// --- Global/shared items can be re-exported if needed ---
// pub use stepper::Stepper; // Example


// --- Panic Handler ---
#[cfg(not(test))]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // On panic, loop indefinitely.
    // TODO: Consider a more robust panic handler for production firmware
    // (e.g., logging error, resetting MCU, safe state).
    loop {}
}

// --- Tests ---
#[cfg(test)]
mod tests {
    // Basic test to ensure the library structure compiles.
    #[test]
    fn basic_lib_setup_works() {
        assert_eq!(2 + 2, 4);
    }

    // Example of using a function from a submodule (assuming it's public)
    // #[test]
    // fn test_crc_from_utils() {
    //     let data = b"123456789";
    //     // Assuming crc16_ccitt is pub in utils module
    //     assert_eq!(crate::utils::crc16_ccitt(data.as_ptr(), data.len() as u8), 0x6F91);
    // }
}
