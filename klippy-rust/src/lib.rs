pub mod itersolve;
pub mod kinematics;
pub mod stepcompress;
pub mod trapq;

use std::collections::HashMap;

pub mod reactor {
    use crate::Printer;

    pub struct Reactor;
    impl Reactor {
        pub fn new() -> Self {
            Reactor
        }
        pub fn register_callback(&self, _cb: fn(&mut Printer)) {}
        pub fn run(&self) {}
        pub fn end(&self) {}
    }
}

pub mod gcode {
    pub struct GCode;
    impl GCode {
        pub fn new() -> Self {
            GCode
        }
    }
}

pub struct Printer {
    reactor: reactor::Reactor,
    state_message: String,
    in_shutdown_state: bool,
    run_result: Option<String>,
    event_handlers: HashMap<String, Vec<fn()>>,
    objects: HashMap<String, Box<dyn std::any::Any>>,
}

impl Printer {
    pub fn new(main_reactor: reactor::Reactor) -> Self {
        let mut printer = Printer {
            reactor: main_reactor,
            state_message: "Printer is not ready".to_string(),
            in_shutdown_state: false,
            run_result: None,
            event_handlers: HashMap::new(),
            objects: HashMap::new(),
        };
        printer.reactor.register_callback(Printer::connect);
        printer
    }

    fn connect(&mut self) {}

    pub fn run(&mut self) -> Option<String> {
        log::info!("Start printer");
        self.reactor.run();
        self.run_result.clone()
    }
}
