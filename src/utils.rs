#![cfg_attr(not(test), no_std)]

use core::ffi::c_uchar;

/// Implement the standard crc "ccitt" algorithm on the given buffer
#[no_mangle]
pub extern "C" fn crc16_ccitt(buf: *const u8, len: c_uchar) -> u16 {
    let mut crc: u16 = 0xffff;
    if buf.is_null() {
        return 0;
    }
    let mut current_buf = buf;

    for _ in 0..len {
        let mut data_val: u8 = unsafe { *current_buf };
        data_val ^= (crc & 0xff) as u8;

        let promoted_data_for_shift = data_val as u16;
        data_val ^= (promoted_data_for_shift << 4) as u8;

        crc = (((data_val as u16) << 8) | (crc >> 8)) ^ (data_val >> 4) as u16
               ^ (data_val as u16) << 3;
        current_buf = unsafe { current_buf.add(1) };
    }
    crc
}

// timer_is_before(t1, t2) -> t1 < t2 (considering wraparound for u32 timers)
pub fn timer_is_before(time1: u32, time2: u32) -> bool {
    // Handles timer wraparound for u32 timers.
    // Assumes a difference of up to 2^31 ticks in either direction.
    (time1 as i32).wrapping_sub(time2 as i32) < 0
}

// timer_from_us(us) -> ticks
// This depends on CONFIG_CLOCK_FREQ, which is not available here.
// We'll assume it's provided or calculated elsewhere.
pub fn timer_from_us(us: u32, clock_freq: u32) -> u32 {
    (us as u64 * clock_freq as u64 / 1_000_000) as u32
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
        let data1 = b"123456789";
        assert_eq!(crc16_ccitt(data1.as_ptr(), data1.len() as u8), 0x6F91);
    }

    #[test]
    fn test_crc16_ccitt_klipper_example() {
        let data = [0x01, 0x02, 0x03, 0x04, 0x05];
        assert_eq!(crc16_ccitt(data.as_ptr(), data.len() as u8), 0xDD13);
    }

    #[test]
    fn test_crc16_ccitt_single_byte() {
        let data = [0xAA];
        assert_eq!(crc16_ccitt(data.as_ptr(), data.len() as u8), 0x05D7);
    }

    #[test]
    fn test_crc16_ccitt_all_zeros() {
        let data = [0x00, 0x00, 0x00, 0x00];
        assert_eq!(crc16_ccitt(data.as_ptr(), data.len() as u8), 0x0321);
    }

    #[test]
    fn test_timer_is_before() {
        assert!(timer_is_before(100, 200));
        assert!(!timer_is_before(200, 100));
        assert!(!timer_is_before(100, 100));
        // Test wraparound
        assert!(timer_is_before(u32::MAX - 50, 50)); // MAX-50 is before 50 (wraparound)
        assert!(!timer_is_before(50, u32::MAX - 50)); // 50 is not before MAX-50 (wraparound)
    }

    #[test]
    fn test_timer_from_us() {
        assert_eq!(timer_from_us(1000, 1000000), 1000); // 1MHz clock, 1000us = 1000 ticks
        assert_eq!(timer_from_us(1, 16000000), 16);    // 16MHz clock, 1us = 16 ticks
        assert_eq!(timer_from_us(10, 25000000), 250);  // 25MHz clock, 10us = 250 ticks
    }
}
