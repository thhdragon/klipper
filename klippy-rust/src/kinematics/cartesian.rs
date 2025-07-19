use crate::configfile::Config;
use crate::itersolve::StepperKinematics;
use crate::toolhead::Move;

// Placeholders
pub struct Toolhead;
pub struct HomingState;
pub struct Printer;
mod idex_modes {
    pub struct DualCarriages;
}
mod stepper {
    use crate::itersolve::StepperKinematics;
    pub struct LookupMultiRail;
    impl LookupMultiRail {
        pub fn new() -> Self {
            LookupMultiRail
        }
        pub fn setup_itersolve(&self, _name: &str, _axis: &[u8]) {}
        pub fn get_range(&self) -> (f64, f64) {
            (0.0, 0.0)
        }
        pub fn get_steppers(&self) -> Vec<StepperKinematics> {
            vec![]
        }
        pub fn set_trapq(&self, _tq: *mut crate::trapq::TrapQ) {}
    }
}

pub struct CartKinematics {
    rails: Vec<stepper::LookupMultiRail>,
    limits: Vec<(f64, f64)>,
    max_z_velocity: f64,
    max_z_accel: f64,
    dc_module: Option<idex_modes::DualCarriages>,
}

impl CartKinematics {
    pub fn new(_toolhead: &Toolhead, _config: &Config) -> Self {
        // let printer = config.get_printer();
        let rails = vec![
            stepper::LookupMultiRail::new(),
            stepper::LookupMultiRail::new(),
            stepper::LookupMultiRail::new(),
        ];
        for (rail, axis) in rails.iter().zip("xyz".bytes()) {
            rail.setup_itersolve("cartesian_stepper_alloc", &[axis]);
        }
        // let ranges = rails.iter().map(|r| r.get_range()).collect::<Vec<_>>();
        // let axes_min = toolhead.coord(&[ranges[0].0, ranges[1].0, ranges[2].0, 0.0]);
        // let axes_max = toolhead.coord(&[ranges[0].1, ranges[1].1, ranges[2].1, 0.0]);
        let dc_module = None;
        // if config.has_section("dual_carriage") {
        //     let dc_config = config.get_section("dual_carriage");
        //     let dc_axis = dc_config.get_choice("axis", &["x", "y"]);
        //     let dual_carriage_axis = if dc_axis == "x" { 0 } else { 1 };
        //     rails.push(stepper::LookupMultiRail::new(dc_config));
        //     rails[3].setup_itersolve("cartesian_stepper_alloc", dc_axis.as_bytes());
        //     dc_module = Some(idex_modes::DualCarriages::new(
        //         printer,
        //         &[rails[dual_carriage_axis]],
        //         &[rails[3]],
        //         &[dual_carriage_axis],
        //         dc_config.get_float("safe_distance", None, Some(0.0)),
        //     ));
        // }
        // for s in self.get_steppers() {
        //     s.set_trapq(toolhead.get_trapq());
        //     toolhead.register_step_generator(s.generate_steps);
        // }
        // let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let _max_velocity = 0.0;
        let _max_accel = 0.0;
        let max_z_velocity = 0.0; //config.get_float("max_z_velocity", max_velocity, Some(0.0), Some(max_velocity));
        let max_z_accel = 0.0; //config.get_float("max_z_accel", max_accel, Some(0.0), Some(max_accel));
        CartKinematics {
            rails,
            limits: vec![(1.0, -1.0), (1.0, -1.0), (1.0, -1.0)],
            max_z_velocity,
            max_z_accel,
            dc_module,
        }
    }

    pub fn get_steppers(&self) -> Vec<StepperKinematics> {
        vec![]
    }

    pub fn calc_position(&self, _stepper_positions: &Vec<f64>) -> Vec<f64> {
        vec![]
    }

    pub fn set_position(&mut self, _newpos: &[f64; 3], _homing_axes: &[char]) {}

    pub fn home(&self, _homing_state: &mut HomingState) {}

    pub fn check_move(&self, _move: &mut Move) {}
}

pub fn load_kinematics(toolhead: &Toolhead, config: &Config) -> CartKinematics {
    CartKinematics::new(toolhead, config)
}
