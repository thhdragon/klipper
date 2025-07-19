pub mod itersolve;
pub mod kinematics;
pub mod mcu;
pub mod pins;
pub mod stepcompress;
pub mod toolhead;
pub mod trapq;
pub mod webhooks;

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

pub mod configfile {
    pub struct PrinterConfig;
    impl PrinterConfig {
        pub fn new() -> Self {
            PrinterConfig
        }
        pub fn read_main_config(&self) -> Config {
            Config
        }
        pub fn get_prefix_sections(&self, _prefix: &str) -> Vec<Section> {
            vec![]
        }
    }
    pub struct Config;
    impl Config {
        pub fn get_prefix_sections(&self, _prefix: &str) -> Vec<Section> {
            vec![]
        }
    }
    pub struct Section;
    impl Section {
        pub fn get_name(&self) -> &str {
            ""
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
        let printer = Printer {
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

    fn connect(&mut self) {
        self.read_config();
        // self.send_event("klippy:mcu_identify", &[]);
        // for cb in self.event_handlers.get("klippy:connect").unwrap_or(&vec![]) {
        //     if self.state_message != "Printer is not ready" {
        //         return;
        //     }
        //     cb();
        // }
    }

    fn read_config(&mut self) {
        let pconfig = configfile::PrinterConfig::new();
        let config = pconfig.read_main_config();
        self.objects
            .insert("configfile".to_string(), Box::new(pconfig));
        // for m in [pins, mcu] {
        //     m.add_printer_objects(config);
        // }
        for section_config in config.get_prefix_sections("") {
            self.load_object(&config, section_config.get_name());
        }
        // for m in [toolhead] {
        //     m.add_printer_objects(config);
        // }
        // pconfig.check_unused_options(config);
    }

    pub fn run(&mut self) -> Option<String> {
        log::info!("Start printer");
        self.reactor.run();
        self.run_result.clone()
    }

    fn load_object(&mut self, _config: &configfile::Config, section: &str) {
        if self.objects.contains_key(section) {
            return;
        }
        let module_parts: Vec<&str> = section.split(' ').collect();
        let module_name = module_parts[0];
        // TODO: Implement module loading
        println!("Loading module {}", module_name);
    }

    fn add_object(&mut self, name: &str, obj: Box<dyn std::any::Any>) {
        if self.objects.contains_key(name) {
            panic!("Printer object '{}' already created", name);
        }
        self.objects.insert(name.to_string(), obj);
    }

    fn lookup_object(&self, name: &str) -> Option<&Box<dyn std::any::Any>> {
        self.objects.get(name)
    }

    fn register_event_handler(&mut self, event: &str, callback: fn()) {
        self.event_handlers
            .entry(event.to_string())
            .or_insert_with(Vec::new)
            .push(callback);
    }

    fn send_event(&self, event: &str, _params: &[&str]) {
        if let Some(handlers) = self.event_handlers.get(event) {
            for handler in handlers {
                handler();
            }
        }
    }

    pub fn invoke_shutdown(&mut self, msg: &str) {
        if self.in_shutdown_state {
            return;
        }
        log::error!("Transition to shutdown state: {}", msg);
        self.in_shutdown_state = true;
        self.state_message = msg.to_string();
        self.send_event("klippy:shutdown", &[]);
    }

    pub fn request_exit(&mut self, result: &str) {
        if self.run_result.is_none() {
            self.run_result = Some(result.to_string());
        }
        self.reactor.end();
    }

    pub fn is_shutdown(&self) -> bool {
        self.in_shutdown_state
    }

    pub fn get_run_result(&self) -> Option<String> {
        self.run_result.clone()
    }
}
