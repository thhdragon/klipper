// klipper_rust_port/src/stepcompress.rs
#![cfg_attr(not(test), no_std)]

use crate::itersolve::StepCompress; // Use the opaque StepCompress from itersolve

// Define an error type for stepcompress operations, if needed.
#[derive(Debug)]
pub enum StepCompressError {
    QueueFull, // Example error
}

// Test hooks for itersolve_generate_steps_tests
// These are public within the crate for tests in other modules to access them.
#[cfg(test)]
use std::cell::RefCell;

#[cfg(test)]
thread_local! {
    pub(crate) static TEST_STEP_CALL_RECORDER: RefCell<Vec<(i32, f64, f64)>> = RefCell::new(Vec::new());
    pub(crate) static TEST_STEP_DIR_OVERRIDE: RefCell<Option<i32>> = RefCell::new(None);
    pub(crate) static TEST_COMMIT_CALL_COUNT: RefCell<usize> = RefCell::new(0);
}

// Helper functions for tests in other modules to interact with the thread_locals.
// These should be in a test utility module or directly in itersolve tests if preferred,
// but placing them here keeps stepcompress self-contained for its test interactions.
#[cfg(test)]
pub fn test_clear_recorded_step_calls() {
    TEST_STEP_CALL_RECORDER.with(|recorder| recorder.borrow_mut().clear());
}

#[cfg(test)]
pub fn test_get_recorded_step_calls() -> Vec<(i32, f64, f64)> {
    TEST_STEP_CALL_RECORDER.with(|recorder| recorder.borrow().clone())
}

#[cfg(test)]
pub fn test_set_step_dir_override(dir: Option<i32>) {
    TEST_STEP_DIR_OVERRIDE.with(|override_val| *override_val.borrow_mut() = dir);
}

#[cfg(test)]
pub fn test_clear_commit_call_count() {
    TEST_COMMIT_CALL_COUNT.with(|count| *count.borrow_mut() = 0);
}

#[cfg(test)]
pub fn test_get_commit_call_count() -> usize {
    TEST_COMMIT_CALL_COUNT.with(|count| *count.borrow())
}


/// Gets the current step direction.
#[allow(unused_variables)]
pub fn stepcompress_get_step_dir(sc: *mut StepCompress) -> i32 {
    #[cfg(test)]
    {
        if let Some(dir_override) = TEST_STEP_DIR_OVERRIDE.with(|val| *val.borrow()) {
            return dir_override;
        }
    }
    // Default non-test behavior or fallback if no override
    if sc.is_null() {
        // Consider panic or error for null pointer in real code if not expected.
    }
    1 // Default direction: forward
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
        TEST_STEP_CALL_RECORDER.with(|recorder| {
            recorder.borrow_mut().push((sdir, print_time, step_time));
        });
    }
    // Default non-test behavior
    if sc.is_null() {
        // This might be an error condition in real code.
    }
    // Real implementation would add to a queue.
    // println!("Stub: stepcompress_append(dir={}, print_t={}, step_t={})", sdir, print_time, step_time);
    Ok(())
}

/// Commits steps.
#[allow(unused_variables)]
pub fn stepcompress_commit(sc: *mut StepCompress) {
    #[cfg(test)]
    {
        TEST_COMMIT_CALL_COUNT.with(|count| *count.borrow_mut() += 1);
    }
    // Default non-test behavior
    if sc.is_null() {
        // Handle error or simply return.
    }
    // println!("Stub: stepcompress_commit()");
}


// Add other necessary stub functions if itersolve needs them.
// For example, from itersolve_set_stepcompress:
// C: stepcompress_set_invert_sdir(sc, stepper_get_invert_sdir(s));
// This implies a `stepcompress_set_invert_sdir` might be needed.
// For now, focusing only on what `itersolve_gen_steps_range` directly calls.

#[cfg(test)]
mod tests {
    use super::*; // To access the stub functions themselves for direct testing.
                  // And also the test helper functions defined above.

    #[test]
    fn get_dir_stub_default() {
        let mut dummy_sc_state = crate::itersolve::StepCompress;
        test_set_step_dir_override(None); // Ensure no override
        assert_eq!(stepcompress_get_step_dir(&mut dummy_sc_state as *mut _), 1);
    }

    #[test]
    fn get_dir_stub_override() {
        let mut dummy_sc_state = crate::itersolve::StepCompress;
        test_set_step_dir_override(Some(0));
        assert_eq!(stepcompress_get_step_dir(&mut dummy_sc_state as *mut _), 0);
        test_set_step_dir_override(Some(-1)); // Assuming -1 is a valid encoding for a direction
        assert_eq!(stepcompress_get_step_dir(&mut dummy_sc_state as *mut _), -1);
        test_set_step_dir_override(None); // Clear override
    }

    #[test]
    fn append_stub_records_calls() {
        let mut dummy_sc_state = crate::itersolve::StepCompress;
        test_clear_recorded_step_calls(); // Clear before test

        assert!(stepcompress_append(&mut dummy_sc_state as *mut _, 1, 0.1, 0.001).is_ok());
        assert!(stepcompress_append(&mut dummy_sc_state as *mut _, 0, 0.2, 0.002).is_ok());

        let calls = test_get_recorded_step_calls();
        assert_eq!(calls.len(), 2);
        assert_eq!(calls[0], (1, 0.1, 0.001));
        assert_eq!(calls[1], (0, 0.2, 0.002));
        test_clear_recorded_step_calls(); // Clear after test
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
