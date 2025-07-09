// klipper_rust_port/src/stepcompress.rs
// #![cfg_attr(not(test), no_std)] // Removed from here, rely on lib.rs

#[cfg(all(test, feature = "alloc"))]
extern crate alloc;

#[cfg(test)]
extern crate std;

use crate::itersolve::StepCompress;

#[derive(Debug)]
pub enum StepCompressError {
    QueueFull,
}

/// Gets the current step direction.
#[allow(unused_variables)]
pub fn stepcompress_get_step_dir(sc: *mut StepCompress) -> i32 {
    #[cfg(test)]
    {
        if let Some(dir_override) = tests::TEST_STEP_DIR_OVERRIDE.with(|val| *val.borrow()) {
            return dir_override;
        }
    }
    if sc.is_null() {
        // error
    }
    1
}

/// Appends a step.
#[allow(unused_variables)]
pub fn stepcompress_append(
    sc: *mut StepCompress,
    sdir: i32,
    print_time: f64,
    step_time: f64,
) -> Result<(), StepCompressError> {
    #[cfg(test)]
    {
        tests::TEST_STEP_CALL_RECORDER.with(|recorder| {
            recorder.borrow_mut().push((sdir, print_time, step_time));
        });
    }
    if sc.is_null() {
        // error
    }
    Ok(())
}

/// Commits steps.
#[allow(unused_variables)]
pub fn stepcompress_commit(sc: *mut StepCompress) {
    #[cfg(test)]
    {
        tests::TEST_COMMIT_CALL_COUNT.with(|count| *count.borrow_mut() += 1);
    }
    if sc.is_null() {
        // error
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::thread_local; // Ensure macro is in scope

    #[cfg(feature = "alloc")]
    use alloc::vec::Vec;
    #[cfg(not(feature = "alloc"))]
    type Vec<T> = heapless::Vec<T, 64>;

    thread_local! {
        pub(crate) static TEST_STEP_CALL_RECORDER: RefCell<Vec<(i32, f64, f64)>> = RefCell::new(Vec::new());
        pub(crate) static TEST_STEP_DIR_OVERRIDE: RefCell<Option<i32>> = RefCell::new(None);
        pub(crate) static TEST_COMMIT_CALL_COUNT: RefCell<usize> = RefCell::new(0);
    }

    pub fn test_clear_recorded_step_calls() {
        TEST_STEP_CALL_RECORDER.with(|recorder| recorder.borrow_mut().clear());
    }

    pub fn test_get_recorded_step_calls() -> Vec<(i32, f64, f64)> {
        TEST_STEP_CALL_RECORDER.with(|recorder| recorder.borrow().clone())
    }

    pub fn test_set_step_dir_override(dir: Option<i32>) {
        TEST_STEP_DIR_OVERRIDE.with(|override_val| *override_val.borrow_mut() = dir);
    }

    pub fn test_clear_commit_call_count() {
        TEST_COMMIT_CALL_COUNT.with(|count| *count.borrow_mut() = 0);
    }

    pub fn test_get_commit_call_count() -> usize {
        TEST_COMMIT_CALL_COUNT.with(|count| *count.borrow())
    }

    #[test]
    fn get_dir_stub_default() {
        let mut dummy_sc_state = crate::itersolve::StepCompress;
        test_set_step_dir_override(None);
        assert_eq!(stepcompress_get_step_dir(&mut dummy_sc_state as *mut _), 1);
    }

    #[test]
    fn get_dir_stub_override() {
        let mut dummy_sc_state = crate::itersolve::StepCompress;
        test_set_step_dir_override(Some(0));
        assert_eq!(stepcompress_get_step_dir(&mut dummy_sc_state as *mut _), 0);
        test_set_step_dir_override(Some(-1));
        assert_eq!(stepcompress_get_step_dir(&mut dummy_sc_state as *mut _), -1);
        test_set_step_dir_override(None);
    }

    #[test]
    fn append_stub_records_calls() {
        let mut dummy_sc_state = crate::itersolve::StepCompress;
        test_clear_recorded_step_calls();

        assert!(stepcompress_append(&mut dummy_sc_state as *mut _, 1, 0.1, 0.001).is_ok());
        assert!(stepcompress_append(&mut dummy_sc_state as *mut _, 0, 0.2, 0.002).is_ok());

        let calls = test_get_recorded_step_calls();
        assert_eq!(calls.len(), 2);
        assert_eq!(calls[0], (1, 0.1, 0.001));
        assert_eq!(calls[1], (0, 0.2, 0.002));
        test_clear_recorded_step_calls();
    }

    #[test]
    fn commit_stub_counts_calls() {
        let mut dummy_sc_state = crate::itersolve::StepCompress;
        test_clear_commit_call_count();

        stepcompress_commit(&mut dummy_sc_state as *mut _);
        stepcompress_commit(&mut dummy_sc_state as *mut _);

        assert_eq!(test_get_commit_call_count(), 2);
        test_clear_commit_call_count();
    }
}
