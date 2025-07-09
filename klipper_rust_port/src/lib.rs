#![no_std]

// Equivalent of #include "misc.h" for crc16_ccitt declaration
// In Rust, functions are typically public if they are meant to be called from outside the module.
// We will make this function public.

use core::ffi::c_uchar;
use core::panic::PanicInfo;

pub mod trapq; // Add trapq as a public module
pub mod itersolve; // Add itersolve as a public module
pub mod stepcompress; // Add stepcompress as a public module

/// Implement the standard crc "ccitt" algorithm on the given buffer
#[unsafe(no_mangle)]
pub extern "C" fn crc16_ccitt(buf: *const u8, len: c_uchar) -> u16 {
    let mut crc: u16 = 0xffff;
    // Ensure the buffer pointer is not null
    if buf.is_null() {
        // Or handle error appropriately, e.g., return a specific error code if the API allows
        return 0; // Placeholder for error or specific defined behavior
    }
    let mut current_buf = buf;

    for _ in 0..len {
        let mut data_val: u8 = unsafe { *current_buf }; // Use a new variable name to avoid confusion with 'data' in C
        data_val ^= (crc & 0xff) as u8;

        // Emulate C's promotion and truncation for: data ^= data << 4;
        // In C, 'data' (uint8_t) is promoted to 'int' for 'data << 4'.
        // The result of 'data << 4' (an int) is then XORed with 'data',
        // and the final result is truncated back to uint8_t when stored in 'data'.
        let promoted_data_for_shift = data_val as u16; // Promote to a wider type like C's int promotion
        data_val ^= (promoted_data_for_shift << 4) as u8; // Shift, then truncate back to u8 for XOR assignment

        crc = (((data_val as u16) << 8) | (crc >> 8)) ^ (data_val >> 4) as u16
               ^ (data_val as u16) << 3;
        current_buf = unsafe { current_buf.add(1) };
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_ccitt_empty() {
        let data: [u8; 0] = [];
        assert_eq!(crc16_ccitt(data.as_ptr(), data.len() as u8), 0xffff);
    }

    #[test]
    fn test_crc16_ccitt_basic() {
        // Value from running Klipper's C code for "123456789"
        let data1 = b"123456789";
        assert_eq!(crc16_ccitt(data1.as_ptr(), data1.len() as u8), 0x6F91);
    }

    #[test]
    fn test_crc16_ccitt_klipper_example() {
        // Value from running Klipper's C code for {0x01, 0x02, 0x03, 0x04, 0x05}
        let data = [0x01, 0x02, 0x03, 0x04, 0x05];
        assert_eq!(crc16_ccitt(data.as_ptr(), data.len() as u8), 0xDD13);
    }

    #[test]
    fn test_crc16_ccitt_single_byte() {
        // Value from running Klipper's C code for {0xAA}
        let data = [0xAA];
        assert_eq!(crc16_ccitt(data.as_ptr(), data.len() as u8), 0x05D7);
    }

    #[test]
    fn test_crc16_ccitt_all_zeros() {
        // Value from running Klipper's C code for {0x00, 0x00, 0x00, 0x00}
        let data = [0x00, 0x00, 0x00, 0x00];
        assert_eq!(crc16_ccitt(data.as_ptr(), data.len() as u8), 0x0321);
    }
}

/// Panic handler for #![no_std] environment
#[cfg(not(test))] // Only compile for non-test builds (tests run with std)
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
