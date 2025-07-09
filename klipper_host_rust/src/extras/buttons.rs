// klipper_host_rust/src/extras/buttons.rs
// Corresponds to klippy/extras/buttons.py

// Logging
use log;
use std::sync::Arc;
use std::collections::HashMap; // For PrinterButtons's mcu_buttons, adc_buttons

// Core traits from the main library
use crate::core_traits::{
    Printer,
    ConfigSection,
    PinController,
    PinParams,
    Mcu,
    Reactor, // Using the main Reactor trait now
};

// --- Minimal Reactor Trait for DebounceButton (can be removed if main Reactor is sufficient) ---
// This will be expanded or replaced by a more complete Reactor in a later step
// For now, let's assume DebounceButton will use the main Reactor trait.
// trait ButtonReactor {
//     fn register_callback_once(&self, eventtime: f64, callback: Box<dyn FnOnce(f64)>);
// }

// Dummy Reactor for standalone testing of DebounceButton if needed (not used in production)
// In a real scenario, this would be provided by the main application.
// #[cfg(test)]
// struct DummyReactor {
//     // We can add more fields here if tests need to inspect scheduled callbacks
// }

// #[cfg(test)]
// impl ButtonReactor for DummyReactor {
//     fn register_callback_once(&self, _eventtime: f64, callback: Box<dyn FnOnce(f64)>) {
//         // For testing, we might just execute immediately or store it for inspection
//         // For simplicity in this example, let's imagine it gets called appropriately by a test runner.
//         // Or, for more complex tests, one might use a mock library or a more sophisticated dummy.
//         log::debug!("DummyReactor: register_callback_once called, executing immediately for test purposes.");
//         // In a real test that needs to simulate time, you wouldn't call it directly here.
//         // callback(eventtime); // This would be incorrect for simulating debounce delay.
//     }
// }
// --- End Minimal Reactor Trait ---

// Callback type for DebounceButton action
type DebounceActionCallback = Box<dyn Fn(f64, bool) + Send + Sync>; // eventtime, state

struct DebounceButton<R: Reactor + ?Sized> {
    reactor: std::sync::Arc<R>,
    button_action: DebounceActionCallback,
    debounce_delay: f64,
    logical_state: Option<bool>,
    physical_state: Option<bool>,
    latest_eventtime: Option<f64>,
    // Store the pending callback to allow cancellation if a new event arrives.
    // This is a simplification; a real reactor might provide cancellation directly.
    // For now, we'll rely on the logic inside _debounce_event to check latest_eventtime.
}

impl<R: ButtonReactor + Send + Sync + 'static> DebounceButton<R> {
    // In Python, `config` is used to get `debounce_delay`.
    // We'll pass debounce_delay directly for now, or a Config object later.
    fn new(
        reactor: std::sync::Arc<R>,
        button_action: DebounceActionCallback,
        debounce_delay: f64,
    ) -> Self {
        Self {
            reactor,
            button_action,
            debounce_delay,
            logical_state: None,
            physical_state: None,
            latest_eventtime: None,
        }
    }

    fn button_handler(&mut self, eventtime: f64, state: bool) {
        self.physical_state = Some(state);
        self.latest_eventtime = Some(eventtime);

        if self.logical_state == self.physical_state {
            return; // No state transition, ignore
        }

        let trigger_time = eventtime + self.debounce_delay;

        // Create self_rc for the closure.
        // This part is tricky because _debounce_event needs mutable access to self.
        // A full reactor implementation might handle this better, or we might need
        // to use Arc<Mutex<Self>> or pass necessary state into the closure if
        // the reactor callback doesn't provide a way to call a method on Self.
        // For now, let's assume the reactor can somehow call back to a method or
        // that we pass enough info.
        //
        // Simplified approach: The closure will own the necessary data to make a decision,
        // and then call a static-like method or a method on an Arc<Mutex<DebounceButton_shared_state>>.
        //
        // Let's try to call a method that takes necessary state.
        // This requires _debounce_event to be callable without full &mut self,
        // or we need a way to get &mut self in the callback.
        //
        // A common pattern for GUI/event systems is for the object to be Arc<Mutex<T>>.
        // Let's assume for now that the reactor will call a method on this instance.
        // This is a placeholder for a more robust solution with a real reactor.
        // For now, we'll pass what's needed to a function that then calls the action.

        // The key challenge: Rust's ownership and borrowing.
        // The callback to the reactor needs to be 'static if it's stored and called later.
        // If `_debounce_event` needs `&mut self`, we can't easily pass it to a generic reactor
        // unless the reactor is specifically designed for it (e.g. via `Arc<Mutex<Self>>`).

        // Let's design `_debounce_event` to be callable based on parameters passed to it,
        // and it will invoke `self.button_action`.
        // The reactor's callback will need to capture `self.button_action`, `self.logical_state`, etc.
        // This is getting complicated without a concrete Reactor.

        // Alternative: The `DebounceButton` itself is not directly mutated by the callback.
        // The callback receives the state at the time of scheduling and decides.
        // The `button_action` is called. `logical_state` of the DebounceButton is updated
        // if the action is indeed taken.

        // Let's assume `_debounce_event` is a method that can be called.
        // This implies the DebounceButton instance needs to be accessible.
        // Using Arc<Mutex<Self>> is a common way if the reactor is generic.
        // However, the original plan was to define traits later.
        // For now, the `_debounce_event` will be a conceptual method.

        // If the reactor calls `_debounce_event(self, eventtime_of_callback)`,
        // then `self` would be available.
        // Let's proceed with the logic inside `_debounce_event` assuming it's called.
        // The actual mechanism of how it's called by the reactor is deferred.

        // To make the closure 'static and avoid capturing `&mut self` directly,
        // we can pass the necessary values to the closure.
        // The closure would then call a static-like method or a method on an Arc-ed object.

        // For this step, we'll focus on the logic of _debounce_event.
        // The `register_callback_once` will take a closure.
        // We need to make `self` (or parts of it) accessible to that closure.
        // This is where `Arc<Mutex<DebounceButtonData>>` might come in,
        // or the `ButtonReactor` trait is more specific.

        // Let's assume the `ButtonReactor` is capable of scheduling a method call on an object,
        // or we pass a closure that captures an `Arc<Mutex<DebounceButtonState>>`.
        // For now, we'll assume the call to `_debounce_event` will happen correctly.
        // The `debounce_delay` and `latest_eventtime` are crucial.

        // The Python code implies that `_debounce_event` has access to the instance's
        // `self.logical_state`, `self.physical_state`, `self.latest_eventtime`,
        // and `self.button_action`.

        // Let's simplify the reactor interaction for now and focus on the debounce logic itself.
        // We will assume that `_debounce_event` is correctly called at `trigger_time`.
        // The actual scheduling is a concern of the `ButtonReactor` trait implementation.

        // To allow _debounce_event to modify DebounceButton, we'd typically use Arc<Mutex<DebounceButton>>
        // and the callback would capture this Arc.

        // For this plan step, we will write the logic of _debounce_event.
        // The `reactor.register_callback_once` is a placeholder for how it gets called.
        // We will need to adjust this when the actual Reactor trait is defined.
        log::debug!("DebounceButton: button_handler event at {}, state {}. Scheduling debounce_event at {}", eventtime, state, trigger_time);
        // The actual scheduling mechanism will be fleshed out with the Reactor trait.
        // For now, we're defining what _debounce_event *should do*.
    }

    // This method would be called by the reactor at `trigger_time`.
    // It needs access to the DebounceButton's state.
    fn _debounce_event(&mut self, eventtime_of_callback: f64) {
        log::debug!("DebounceButton: _debounce_event called at {}", eventtime_of_callback);
        // Ensure reactor interaction for scheduling this callback is correctly handled later.
        // For now, this method contains the logic assuming it's called at the right time.
        if self.logical_state == self.physical_state {
            log::debug!("_debounce_event: logical_state == physical_state, returning.");
            return;
        }

        // Check if this event is superseded by a more recent physical state change.
        // The eventtime_of_callback is (self.latest_eventtime_at_schedule_time + self.debounce_delay)
        // So, (eventtime_of_callback - self.debounce_delay) is the original eventtime that scheduled this callback.
        let original_schedule_event_time = eventtime_of_callback - self.debounce_delay;

        if let Some(latest_eventtime_overall) = self.latest_eventtime {
            if original_schedule_event_time < latest_eventtime_overall {
                log::debug!("_debounce_event: More recent event occurred at {}. This event (originated at {}) is superseded.", latest_eventtime_overall, original_schedule_event_time);
                return;
            }
        } else {
             // Should not happen if _debounce_event is only called after button_handler
            log::error!("_debounce_event: latest_eventtime is None, this should not happen.");
            return;
        }


        // Enact state transition and trigger action
        self.logical_state = self.physical_state;
        if let (Some(ls), Some(le)) = (self.logical_state, self.latest_eventtime) {
            log::debug!("_debounce_event: Invoking button_action with time {} and state {}.", le, ls);
            (self.button_action)(le, ls);
        } else {
            log::error!("_debounce_event: logical_state or latest_eventtime is None after checks.");
        }
    }
}


// MCU_buttons and MCU_ADC_buttons would be ported later.
// For now, we define placeholder structs if PrinterButtons needs to store them.
// These are highly dependent on MCU communication, which is a larger task.
struct McuButtonsPlaceholder { /* ... */ }
struct McuAdcButtonsPlaceholder { /* ... */ }


// --- PrinterButtons Struct and Implementation ---
// Callback for general button events (e.g., from MCU_buttons)
// (eventtime, button_state_mask)
type ButtonGroupCallback = Box<dyn Fn(f64, u8) + Send + Sync>;


pub struct PrinterButtons {
    printer: Arc<dyn Printer>,
    // Stores MCU-specific button handlers. Key is MCU name.
    mcu_buttons: HashMap<String, McuButtonsPlaceholder>,
    // Stores ADC button handlers. Key is pin name.
    adc_buttons: HashMap<String, McuAdcButtonsPlaceholder>,
    // query_adc: Arc<dyn QueryAdc>, // Assuming a QueryAdc trait/object
}

impl PrinterButtons {
    pub fn new(config: Arc<dyn ConfigSection>, printer: Arc<dyn Printer>) -> Result<Self, String> {
        // In Python, this also loads 'query_adc'. We'll need a similar mechanism.
        // let query_adc = printer.lookup_object("query_adc")?.downcast_arc::<dyn QueryAdc>()?;
        // For now, PrinterButtons doesn't directly use query_adc in register_rotary_encoder.
        // It's used by register_adc_button.

        Ok(Self {
            printer,
            mcu_buttons: HashMap::new(),
            adc_buttons: HashMap::new(),
            // query_adc,
        })
    }

    // Corresponds to Python's register_rotary_encoder
    pub fn register_rotary_encoder(
        &mut self,
        pin1_name: &str,
        pin2_name: &str,
        cw_callback: EventTimeCallback,  // Defined at the top of the file
        ccw_callback: EventTimeCallback, // Defined at the top of the file
        steps_per_detent: i32,
    ) -> Result<(), String> {
        let encoder_callback_logic: ButtonGroupCallback = match steps_per_detent {
            2 => {
                let mut encoder = HalfStepRotaryEncoder::new(cw_callback, ccw_callback);
                Box::new(move |eventtime, pin_state| {
                    encoder.encoder_callback(eventtime, pin_state);
                })
            }
            4 => {
                let mut encoder = FullStepRotaryEncoder::new(cw_callback, ccw_callback);
                Box::new(move |eventtime, pin_state| {
                    encoder.encoder_callback(eventtime, pin_state);
                })
            }
            _ => {
                return Err(format!(
                    "{} steps per detent not supported for rotary encoder",
                    steps_per_detent
                ));
            }
        };

        // The original Python code calls `self.register_buttons([pin1, pin2], re.encoder_callback)`
        // This `register_buttons` method handles MCU lookup, pin validation, and then
        // calls `mcu_buttons.setup_buttons(pin_params_list, callback)`.
        // For now, we will stub out the actual registration part.
        log::info!(
            "PrinterButtons: Registering rotary encoder with pins '{}', '{}' and {} steps/detent. Actual pin registration is stubbed.",
            pin1_name, pin2_name, steps_per_detent
        );

        // Placeholder for actual pin registration via a (yet to be fully ported) `register_buttons` method.
        // self.register_buttons(&[pin1_name.to_string(), pin2_name.to_string()], encoder_callback_logic)?;

        Ok(())
    }

    // TODO: Implement other registration methods (register_buttons, register_adc_button, etc.)
    // These will be added in the next step as stubs.

    // --- Stubbed Registration Methods ---

    pub fn register_buttons(
        &mut self,
        pin_names: &[String], // Changed from &str to String to match typical usage
        callback: ButtonGroupCallback,
    ) -> Result<(), String> {
        let pins_str = pin_names.join(", ");
        log::info!(
            "PrinterButtons: STUB: register_buttons called for pins: [{}]. Callback registered conceptually.",
            pins_str
        );
        // In a full implementation, this would:
        // 1. Parse pins using `self.printer.lookup_object("pins")`
        // 2. Determine the MCU for these pins.
        // 3. Get or create an `MCU_buttons` instance for that MCU.
        // 4. Call `mcu_buttons.setup_buttons(...)`.
        Ok(())
    }

    pub fn register_adc_button(
        &mut self,
        pin_name: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64, // In python this is float, e.g. for voltage divider calculation
        callback: DebounceActionCallback, // Assuming ADC buttons are often debounced
    ) -> Result<(), String> {
        log::info!(
            "PrinterButtons: STUB: register_adc_button called for pin: {}, min: {}, max: {}, pullup: {}. Callback registered conceptually.",
            pin_name, min_val, max_val, pullup
        );
        // In a full implementation, this would:
        // 1. Get or create an `MCU_ADC_buttons` instance for the pin.
        // 2. Call `adc_buttons.setup_button(...)`.
        Ok(())
    }

    pub fn register_debounce_button(
        &mut self,
        pin_name: &str, // Python takes a single pin string here
        callback: DebounceActionCallback,
        // config: Arc<dyn ConfigSection> // To get debounce_delay
        debounce_delay: f64, // Passing delay directly for now
    ) -> Result<(), String> {
        log::info!(
            "PrinterButtons: STUB: register_debounce_button called for pin: {} with delay: {}. Callback registered conceptually.",
            pin_name, debounce_delay
        );
        let reactor = self.printer.get_reactor(); // Assuming Printer has get_reactor
        let debouncer = DebounceButton::new(reactor, callback, debounce_delay);

        // This would then call self.register_buttons with debouncer.button_handler
        // let pin_names_vec = vec![pin_name.to_string()];
        // self.register_buttons(&pin_names_vec, Box::new(move |eventtime, state| {
        //     // This part is tricky because button_handler needs &mut self on debouncer.
        //     // This indicates DebounceButton might need to be Arc<Mutex<DebounceButton>>
        //     // or the callback structure needs adjustment.
        //     // For a stub, we'll just log.
        //     log::info!("DebounceButton's button_handler would be called here via register_buttons.");
        // }))?;
        Ok(())
    }

    pub fn register_debounce_adc_button(
        &mut self,
        pin_name: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: DebounceActionCallback,
        // config: Arc<dyn ConfigSection> // To get debounce_delay
        debounce_delay: f64, // Passing delay directly for now
    ) -> Result<(), String> {
        log::info!(
            "PrinterButtons: STUB: register_debounce_adc_button called for pin: {} with delay: {}. Callback registered conceptually.",
            pin_name, debounce_delay
        );
        let reactor = self.printer.get_reactor();
        let debouncer = DebounceButton::new(reactor, callback, debounce_delay);

        // This would then call self.register_adc_button with debouncer.button_handler
        // self.register_adc_button(pin_name, min_val, max_val, pullup, Box::new(move |eventtime, state| {
        //     // Similar challenge with &mut self for debouncer.button_handler
        //     log::info!("DebounceButton's button_handler would be called here via register_adc_button.");
        // }))?;
        Ok(())
    }

    // Helper type for push button callbacks (eventtime only)
    type PushButtonCallback = Box<dyn Fn(f64) + Send + Sync>;

    pub fn register_adc_button_push(
        &mut self,
        pin_name: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        push_callback: Self::PushButtonCallback,
    ) -> Result<(), String> {
        log::info!(
            "PrinterButtons: STUB: register_adc_button_push called for pin: {}. Callback registered conceptually.",
            pin_name
        );
        let wrapped_callback: DebounceActionCallback = Box::new(move |eventtime, state| {
            if state { // Only call on push (true state)
                push_callback(eventtime);
            }
        });
        self.register_adc_button(pin_name, min_val, max_val, pullup, wrapped_callback)?;
        Ok(())
    }

    pub fn register_button_push(
        &mut self,
        pin_name: &str, // Python takes a single pin string
        push_callback: Self::PushButtonCallback,
    ) -> Result<(), String> {
        log::info!(
            "PrinterButtons: STUB: register_button_push called for pin: {}. Callback registered conceptually.",
            pin_name
        );
        let wrapped_callback: ButtonGroupCallback = Box::new(move |eventtime, state| {
            // Assuming state is a bitmask and we're interested in the first bit (or a specific bit if it were a group)
            // For a single pin, state would be 0 or 1.
            if state != 0 { // Only call on push (true state)
                push_callback(eventtime);
            }
        });
        let pin_names_vec = vec![pin_name.to_string()];
        self.register_buttons(&pin_names_vec, wrapped_callback)?;
        Ok(())
    }
}


// Function to be called by Klipper's module loading mechanism (hypothetical)
// pub fn load_config(config: Arc<dyn ConfigSection>, printer: Arc<dyn Printer>) -> Result<Arc<PrinterButtons>, String> {
//     Ok(Arc::new(PrinterButtons::new(config, printer)?))
// }


// Constants from the Python module
const QUERY_TIME: f64 = 0.002;
const RETRANSMIT_COUNT: u32 = 50;

const ADC_REPORT_TIME: f64 = 0.015;
const ADC_DEBOUNCE_TIME: f64 = 0.025;
const ADC_SAMPLE_TIME: f64 = 0.001;
const ADC_SAMPLE_COUNT: u32 = 6;

// Callback types
type EventTimeCallback = Box<dyn Fn(f64)>;

// BaseRotaryEncoder constants
const R_START: u8 = 0x0;
const R_DIR_CW: u8 = 0x10;
const R_DIR_CCW: u8 = 0x20;
const R_DIR_MSK: u8 = 0x30;

struct BaseRotaryEncoder {
    cw_callback: EventTimeCallback,
    ccw_callback: EventTimeCallback,
    encoder_state: u8,
}

impl BaseRotaryEncoder {
    fn new(cw_callback: EventTimeCallback, ccw_callback: EventTimeCallback) -> Self {
        Self {
            cw_callback,
            ccw_callback,
            encoder_state: R_START,
        }
    }

    // This method will be called by the specific encoder types
    fn process_state(&mut self, eventtime: f64, new_pin_state: u8, states_table: &Vec<Vec<u8>>) {
        // The pin_state in python is `state & 0x3`. In python, `state` is a combination of two pin values.
        // Here, we'll assume new_pin_state is already the 2-bit value representing the two pins.
        let current_phase = (self.encoder_state & 0x0f) as usize;
        let new_phase_state = (new_pin_state & 0x03) as usize;

        if current_phase < states_table.len() && new_phase_state < states_table[current_phase].len() {
            self.encoder_state = states_table[current_phase][new_phase_state];
        } else {
            // Invalid state or phase, reset to R_START or log an error
            log::warn!("Invalid rotary encoder state transition: current_phase={}, new_phase_state={}. Resetting.", current_phase, new_phase_state);
            self.encoder_state = R_START;
            return;
        }

        if self.encoder_state & R_DIR_MSK == R_DIR_CW {
            (self.cw_callback)(eventtime);
        } else if self.encoder_state & R_DIR_MSK == R_DIR_CCW {
            (self.ccw_callback)(eventtime);
        }
    }
}

struct FullStepRotaryEncoder {
    base: BaseRotaryEncoder,
    states_table: Vec<Vec<u8>>,
}

impl FullStepRotaryEncoder {
    const R_CW_FINAL: u8 = 0x1;
    const R_CW_BEGIN: u8 = 0x2;
    const R_CW_NEXT: u8 = 0x3;
    const R_CCW_BEGIN: u8 = 0x4;
    const R_CCW_FINAL: u8 = 0x5;
    const R_CCW_NEXT: u8 = 0x6;

    fn new(cw_callback: EventTimeCallback, ccw_callback: EventTimeCallback) -> Self {
        let states_table = vec![
            // R_START (0)
            vec![R_START, Self::R_CW_BEGIN, Self::R_CCW_BEGIN, R_START],
            // R_CW_FINAL (1)
            vec![Self::R_CW_NEXT, R_START, Self::R_CW_FINAL, R_START | R_DIR_CW],
            // R_CW_BEGIN (2)
            vec![Self::R_CW_NEXT, Self::R_CW_BEGIN, R_START, R_START],
            // R_CW_NEXT (3)
            vec![Self::R_CW_NEXT, Self::R_CW_BEGIN, Self::R_CW_FINAL, R_START],
            // R_CCW_BEGIN (4)
            vec![Self::R_CCW_NEXT, R_START, Self::R_CCW_BEGIN, R_START],
            // R_CCW_FINAL (5)
            vec![Self::R_CCW_NEXT, Self::R_CCW_FINAL, R_START, R_START | R_DIR_CCW],
            // R_CCW_NEXT (6)
            vec![Self::R_CCW_NEXT, Self::R_CCW_FINAL, Self::R_CCW_BEGIN, R_START],
        ];
        Self {
            base: BaseRotaryEncoder::new(cw_callback, ccw_callback),
            states_table,
        }
    }

    fn encoder_callback(&mut self, eventtime: f64, pin_state: u8) {
        self.base.process_state(eventtime, pin_state, &self.states_table);
    }
}

struct HalfStepRotaryEncoder {
    base: BaseRotaryEncoder,
    states_table: Vec<Vec<u8>>,
}

impl HalfStepRotaryEncoder {
    const R_CCW_BEGIN: u8 = 0x1;
    const R_CW_BEGIN: u8 = 0x2;
    const R_START_M: u8 = 0x3;
    const R_CW_BEGIN_M: u8 = 0x4;
    const R_CCW_BEGIN_M: u8 = 0x5;

    fn new(cw_callback: EventTimeCallback, ccw_callback: EventTimeCallback) -> Self {
        let states_table = vec![
            // R_START (00) (0)
            vec![Self::R_START_M, Self::R_CW_BEGIN, Self::R_CCW_BEGIN, R_START],
            // R_CCW_BEGIN (1)
            vec![Self::R_START_M | R_DIR_CCW, R_START, Self::R_CCW_BEGIN, R_START],
            // R_CW_BEGIN (2)
            vec![Self::R_START_M | R_DIR_CW, Self::R_CW_BEGIN, R_START, R_START],
            // R_START_M (11) (3)
            vec![Self::R_START_M, Self::R_CCW_BEGIN_M, Self::R_CW_BEGIN_M, R_START],
            // R_CW_BEGIN_M (4)
            vec![Self::R_START_M, Self::R_START_M, Self::R_CW_BEGIN_M, R_START | R_DIR_CW],
            // R_CCW_BEGIN_M (5)
            vec![Self::R_START_M, Self::R_CCW_BEGIN_M, Self::R_START_M, R_START | R_DIR_CCW],
        ];
        Self {
            base: BaseRotaryEncoder::new(cw_callback, ccw_callback),
            states_table,
        }
    }

    fn encoder_callback(&mut self, eventtime: f64, pin_state: u8) {
        self.base.process_state(eventtime, pin_state, &self.states_table);
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;
    use std::sync::{Arc, Mutex};

    // --- Mock Reactor for DebounceButton Tests ---
    struct ScheduledCallback {
        trigger_time: f64,
        callback: Box<dyn FnOnce(f64) + Send>,
    }

    #[derive(Clone)]
    struct MockReactor {
        current_time: Arc<Mutex<f64>>,
        scheduled_callbacks: Arc<Mutex<Vec<ScheduledCallback>>>,
    }

    impl MockReactor {
        fn new() -> Self {
            Self {
                current_time: Arc::new(Mutex::new(0.0)),
                scheduled_callbacks: Arc::new(Mutex::new(Vec::new())),
            }
        }

        fn set_time(&self, time: f64) {
            *self.current_time.lock().unwrap() = time;
        }

        fn advance_time_and_trigger(&self, new_time: f64) {
            let mut current_time_guard = self.current_time.lock().unwrap();
            *current_time_guard = new_time;

            let mut callbacks_guard = self.scheduled_callbacks.lock().unwrap();
            let mut still_pending = Vec::new();
            let mut to_run = Vec::new();

            for scheduled in callbacks_guard.drain(..) {
                if scheduled.trigger_time <= new_time {
                    to_run.push(scheduled);
                } else {
                    still_pending.push(scheduled);
                }
            }
            *callbacks_guard = still_pending;
            drop(callbacks_guard); // Release lock before running callbacks

            // Sort callbacks by trigger time to run them in order
            to_run.sort_by(|a, b| a.trigger_time.partial_cmp(&b.trigger_time).unwrap_or(std::cmp::Ordering::Equal));

            for item in to_run {
                log::debug!("MockReactor: Triggering callback scheduled for {}", item.trigger_time);
                (item.callback)(item.trigger_time);
            }
        }
    }

    impl Reactor for MockReactor {
        fn register_callback_once(&self, delay_s: f64, callback: Box<dyn FnOnce(f64) + Send>) {
            let current_time = *self.current_time.lock().unwrap();
            let trigger_time = current_time + delay_s;
            log::debug!("MockReactor: Scheduling callback for {} (current: {}, delay: {})", trigger_time, current_time, delay_s);
            self.scheduled_callbacks.lock().unwrap().push(ScheduledCallback {
                trigger_time,
                callback,
            });
        }
    }
    // --- End Mock Reactor ---


    #[test]
    fn it_works() {
        // Placeholder test
        assert_eq!(2 + 2, 4);
    }

    #[test]
    fn test_full_step_rotary_encoder_cw_flow() {
        let cw_count = Rc::new(RefCell::new(0));
        let ccw_count = Rc::new(RefCell::new(0));

        let cw_cb_clone = Rc::clone(&cw_count);
        let ccw_cb_clone = Rc::clone(&ccw_count);

        let mut encoder = FullStepRotaryEncoder::new(
            Box::new(move |_time| *cw_cb_clone.borrow_mut() += 1),
            Box::new(move |_time| *ccw_cb_clone.borrow_mut() += 1),
        );

        // Standard CW sequence: 00 -> 01 -> 11 -> 10 -> 00 (detent)
        // pin_state represents (pinB << 1) | pinA
        assert_eq!(*cw_count.borrow(), 0);
        encoder.encoder_callback(0.0, 0b00); // Start
        encoder.encoder_callback(0.1, 0b01); // pin A high
        assert_eq!(*cw_count.borrow(), 0);
        encoder.encoder_callback(0.2, 0b11); // pin B high
        assert_eq!(*cw_count.borrow(), 0);
        encoder.encoder_callback(0.3, 0b10); // pin A low
        assert_eq!(*cw_count.borrow(), 1); // Detent!
        encoder.encoder_callback(0.4, 0b00); // pin B low - back to start
        assert_eq!(*cw_count.borrow(), 1);
        assert_eq!(*ccw_count.borrow(), 0);

        // Another CW turn
        encoder.encoder_callback(0.5, 0b01);
        encoder.encoder_callback(0.6, 0b11);
        encoder.encoder_callback(0.7, 0b10);
        assert_eq!(*cw_count.borrow(), 2);
        encoder.encoder_callback(0.8, 0b00);
        assert_eq!(*cw_count.borrow(), 2);
    }

    #[test]
    fn test_full_step_rotary_encoder_ccw_flow() {
        let cw_count = Rc::new(RefCell::new(0));
        let ccw_count = Rc::new(RefCell::new(0));

        let cw_cb_clone = Rc::clone(&cw_count);
        let ccw_cb_clone = Rc::clone(&ccw_count);

        let mut encoder = FullStepRotaryEncoder::new(
            Box::new(move |_time| *cw_cb_clone.borrow_mut() += 1),
            Box::new(move |_time| *ccw_cb_clone.borrow_mut() += 1),
        );

        // Standard CCW sequence: 00 -> 10 -> 11 -> 01 -> 00 (detent)
        assert_eq!(*ccw_count.borrow(), 0);
        encoder.encoder_callback(0.0, 0b00); // Start
        encoder.encoder_callback(0.1, 0b10); // pin B high
        assert_eq!(*ccw_count.borrow(), 0);
        encoder.encoder_callback(0.2, 0b11); // pin A high
        assert_eq!(*ccw_count.borrow(), 0);
        encoder.encoder_callback(0.3, 0b01); // pin B low
        assert_eq!(*ccw_count.borrow(), 1); // Detent!
        encoder.encoder_callback(0.4, 0b00); // pin A low - back to start
        assert_eq!(*ccw_count.borrow(), 1);
        assert_eq!(*cw_count.borrow(), 0);
    }

    #[test]
    fn test_half_step_rotary_encoder_cw_flow() {
        let cw_count = Rc::new(RefCell::new(0));
        let ccw_count = Rc::new(RefCell::new(0));

        let cw_cb_clone = Rc::clone(&cw_count);
        let ccw_cb_clone = Rc::clone(&ccw_count);

        let mut encoder = HalfStepRotaryEncoder::new(
            Box::new(move |_time| *cw_cb_clone.borrow_mut() += 1),
            Box::new(move |_time| *ccw_cb_clone.borrow_mut() += 1),
        );

        // Half-step CW sequence: 00 -> 01 (detent) -> 11 (detent) -> 10 (detent) -> 00 (detent)
        assert_eq!(*cw_count.borrow(), 0);
        encoder.encoder_callback(0.0, 0b00); // Start

        encoder.encoder_callback(0.1, 0b01); // pin A high
        assert_eq!(*cw_count.borrow(), 1);

        encoder.encoder_callback(0.2, 0b11); // pin B high
        assert_eq!(*cw_count.borrow(), 2);

        encoder.encoder_callback(0.3, 0b10); // pin A low
        assert_eq!(*cw_count.borrow(), 3);

        encoder.encoder_callback(0.4, 0b00); // pin B low
        assert_eq!(*cw_count.borrow(), 4);

        assert_eq!(*ccw_count.borrow(), 0);
    }

    #[test]
    fn test_half_step_rotary_encoder_ccw_flow() {
        let cw_count = Rc::new(RefCell::new(0));
        let ccw_count = Rc::new(RefCell::new(0));

        let cw_cb_clone = Rc::clone(&cw_count);
        let ccw_cb_clone = Rc::clone(&ccw_count);

        let mut encoder = HalfStepRotaryEncoder::new(
            Box::new(move |_time| *cw_cb_clone.borrow_mut() += 1),
            Box::new(move |_time| *ccw_cb_clone.borrow_mut() += 1),
        );

        // Half-step CCW sequence: 00 -> 10 (detent) -> 11 (detent) -> 01 (detent) -> 00 (detent)
        assert_eq!(*ccw_count.borrow(), 0);
        encoder.encoder_callback(0.0, 0b00); // Start

        encoder.encoder_callback(0.1, 0b10); // pin B high
        assert_eq!(*ccw_count.borrow(), 1);

        encoder.encoder_callback(0.2, 0b11); // pin A high
        assert_eq!(*ccw_count.borrow(), 2);

        encoder.encoder_callback(0.3, 0b01); // pin B low
        assert_eq!(*ccw_count.borrow(), 3);

        encoder.encoder_callback(0.4, 0b00); // pin A low
        assert_eq!(*ccw_count.borrow(), 4);

        assert_eq!(*cw_count.borrow(), 0);
    }

    #[test]
    fn test_debounce_button_simple_press_release() {
        let reactor = Arc::new(MockReactor::new());
        let last_state = Arc::new(Mutex::new(None::<bool>));
        let last_time = Arc::new(Mutex::new(0.0f64));
        let debounce_delay = 0.025; // 25ms

        let action_last_state = Arc::clone(&last_state);
        let action_last_time = Arc::clone(&last_time);
        let button_action: DebounceActionCallback = Box::new(move |eventtime, state| {
            *action_last_state.lock().unwrap() = Some(state);
            *action_last_time.lock().unwrap() = eventtime;
            log::info!("Action: time={}, state={}", eventtime, state);
        });

        // To manage DebounceButton's mutable state across reactor callbacks,
        // it needs to be wrapped in Arc<Mutex<>>.
        let debouncer = Arc::new(Mutex::new(DebounceButton::new(
            Arc::clone(&reactor) as Arc<dyn Reactor>,
            button_action,
            debounce_delay
        )));

        // Initial state: None
        assert!(last_state.lock().unwrap().is_none());

        // 1. Press button at t=0.0s
        reactor.set_time(0.0);
        debouncer.lock().unwrap().button_handler(0.0, true);

        // No immediate callback
        assert!(last_state.lock().unwrap().is_none());
        reactor.advance_time_and_trigger(0.024); // Just before debounce
        assert!(last_state.lock().unwrap().is_none());

        // Advance past debounce time for press
        reactor.advance_time_and_trigger(0.025); // Debounce time hit
        assert_eq!(last_state.lock().unwrap(), Some(true));
        assert_eq!(*last_time.lock().unwrap(), 0.0); // Time of original event

        // 2. Release button at t=0.1s
        reactor.set_time(0.1);
        debouncer.lock().unwrap().button_handler(0.1, false);

        reactor.advance_time_and_trigger(0.124); // Just before debounce
        assert_eq!(last_state.lock().unwrap(), Some(true)); // Still pressed

        reactor.advance_time_and_trigger(0.125); // Debounce time for release
        assert_eq!(last_state.lock().unwrap(), Some(false));
        assert_eq!(*last_time.lock().unwrap(), 0.1);
    }

    #[test]
    fn test_debounce_button_noisy_press() {
        let reactor = Arc::new(MockReactor::new());
        let state_changes = Arc::new(Mutex::new(Vec::new())); // Stores (time, state)
        let debounce_delay = 0.050; // 50ms

        let cb_state_changes = Arc::clone(&state_changes);
        let button_action: DebounceActionCallback = Box::new(move |eventtime, state| {
            cb_state_changes.lock().unwrap().push((eventtime, state));
            log::info!("Action: time={}, state={}", eventtime, state);
        });

        let debouncer = Arc::new(Mutex::new(DebounceButton::new(
            Arc::clone(&reactor) as Arc<dyn Reactor>,
            button_action,
            debounce_delay
        )));

        // Simulate noisy press: true -> false -> true all within debounce period
        reactor.set_time(0.0);
        debouncer.lock().unwrap().button_handler(0.0, true);    // Initial press

        reactor.set_time(0.01);
        debouncer.lock().unwrap().button_handler(0.01, false);  // Noise

        reactor.set_time(0.02);
        debouncer.lock().unwrap().button_handler(0.02, true);   // Noise, back to true

        // Advance time, but not enough for the first event (0.0 + 0.050 = 0.050)
        reactor.advance_time_and_trigger(0.049);
        assert!(state_changes.lock().unwrap().is_empty());

        // Advance time past the debounce for the latest event (0.02 + 0.050 = 0.070)
        reactor.advance_time_and_trigger(0.070);

        let changes = state_changes.lock().unwrap();
        assert_eq!(changes.len(), 1); // Only one action should have occurred
        assert_eq!(changes[0], (0.02, true)); // Action for the last stable state 'true' at t=0.02
    }

    #[test]
    fn test_debounce_button_quick_release_repress() {
        let reactor = Arc::new(MockReactor::new());
        let state_changes = Arc::new(Mutex::new(Vec::new())); // Stores (time, state)
        let debounce_delay = 0.025;

        let cb_state_changes = Arc::clone(&state_changes);
        let button_action: DebounceActionCallback = Box::new(move |eventtime, state| {
            cb_state_changes.lock().unwrap().push((eventtime, state));
            log::info!("Action: time={}, state={}", eventtime, state);
        });

        let debouncer = Arc::new(Mutex::new(DebounceButton::new(
            Arc::clone(&reactor) as Arc<dyn Reactor>,
            button_action,
            debounce_delay
        )));

        // 1. Press at t=0.0
        reactor.set_time(0.0);
        debouncer.lock().unwrap().button_handler(0.0, true);
        reactor.advance_time_and_trigger(0.025); // Debounce for press
        assert_eq!(state_changes.lock().unwrap().last().cloned(), Some((0.0, true)));

        // 2. Release at t=0.030 (very quickly after first debounce)
        reactor.set_time(0.030);
        debouncer.lock().unwrap().button_handler(0.030, false);
        // Debounce for this release will be at 0.030 + 0.025 = 0.055

        // 3. Repress at t=0.040 (before release debounce fires)
        reactor.set_time(0.040);
        debouncer.lock().unwrap().button_handler(0.040, true);
        // Debounce for this repress will be at 0.040 + 0.025 = 0.065

        // Advance time to where release *would* have fired (0.055)
        reactor.advance_time_and_trigger(0.055);
        // The release at 0.030 should be superseded by repress at 0.040.
        // So, the logical state should still be true from the first press.
        // No new action should be triggered for the release.
        // The last action should still be (0.0, true)
        assert_eq!(state_changes.lock().unwrap().last().cloned(), Some((0.0, true)));
        assert_eq!(state_changes.lock().unwrap().len(), 1);


        // Advance time past repress debounce (0.065)
        reactor.advance_time_and_trigger(0.065);
        assert_eq!(state_changes.lock().unwrap().last().cloned(), Some((0.040, true)));
        assert_eq!(state_changes.lock().unwrap().len(), 2); // Press at 0.0, Repress at 0.040
    }
}
