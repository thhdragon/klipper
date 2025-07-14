#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use cortex_m_rt::entry;
use core::panic::PanicInfo;
use core::cell::RefCell;
use cortex_m::interrupt::{Mutex as InterruptMutex, free as interrupt_free};

// HAL and PAC
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock, SystemClock},
    pac::{self, interrupt, NVIC},
    sio::Sio,
    watchdog::Watchdog,
    timer::Timer as RpHalTimer,
    timer::{Alarm0, Alarm1, Alarm2},
    usb::UsbBus,
    gpio::Pins as RpHalPins,
    adc::Adc as RpHalAdc,
    pwm::Slices as RpHalPwmSlices,
};
use rp2040_hal::clocks::UsbClock;
use rp2040_hal::gpio::{PinId, FunctionPwm};
use rp2040_hal::pwm::{Slice as RpSlice, FreeRunning, SliceId as RpSliceId, ValidSliceMode, Channel as RpPwmChannelExt, PwmA, PwmB};


// Klipper HAL Traits and Implementations
use klipper_mcu_lib::{
    hal::{Timer as KlipperHalTimer, PullType, AdcChannel, AdcError, PwmChannel, PwmError, StepEventResult},
    sched::{SchedulerState, KlipperSchedulerTrait},
    gpio_manager::GpioManager,
    command_parser::{parse_gcode_arguments, CommandArgValue, ArgParseError, ParsedArgs},
    stepper::{Stepper, StepperDirection, TrapezoidalMove, MovePlanner},
};
use klipper_mcu_lib::rp2040_hal_impl::timer::Rp2040Timer;
use klipper_mcu_lib::rp2040_hal_impl::adc::Rp2040AdcChannel;
use klipper_mcu_lib::rp2040_hal_impl::pwm::Rp2040PwmChannel;


// USB
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

// --- Panic Handler ---
#[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {}", Debug2Format(info));
    loop { cortex_m::asm::nop(); }
}

// --- Globals ---
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
const MAX_LINE_LENGTH: usize = 256;
static mut LINE_BUFFER: heapless::String<MAX_LINE_LENGTH> = heapless::String::new();
static TEST_TIMER0: InterruptMutex<RefCell<Option<Rp2040Timer<Alarm0>>>> = InterruptMutex::new(RefCell::new(None));
const TEST_TIMER0_ID: u32 = 0;
static TEST_TIMER1: InterruptMutex<RefCell<Option<Rp2040Timer<Alarm2>>>> = InterruptMutex::new(RefCell::new(None));
const TEST_TIMER1_ID: u32 = 1;
const SCHEDULER_MASTER_TIMER_ID: u32 = 99;
static STEPPER_X: InterruptMutex<RefCell<Option<Stepper>>> = InterruptMutex::new(RefCell::new(None));
const STEPPER_X_ID: u32 = 2;
const STEPPER_X_SCHEDULER_TASK_ID: u32 = 2;
const STEPPER_X_STEP_PIN: u8 = 16;
const STEPPER_X_DIR_PIN: u8 = 17;
const LED_PIN_ID: u8 = 25;
static SCHEDULER: InterruptMutex<RefCell<Option<SchedulerState<Alarm1>>>> = InterruptMutex::new(RefCell::new(None));
static GPIO_MANAGER: InterruptMutex<RefCell<Option<GpioManager>>> = InterruptMutex::new(RefCell::new(None));
static ADC_PERIPHERAL: InterruptMutex<RefCell<Option<RpHalAdc>>> = InterruptMutex::new(RefCell::new(None));
static PWM_SLICES: InterruptMutex<RefCell<Option<RpHalPwmSlices>>> = InterruptMutex::new(RefCell::new(None));
static SYSTEM_CLOCK_FREQ: InterruptMutex<RefCell<Option<u32>>> = InterruptMutex::new(RefCell::new(None));

// --- Callbacks and Dispatchers ---
// ... (All callback and dispatcher functions as before) ...
fn test_timer0_callback(timer: &mut Rp2040Timer<Alarm0>) -> StepEventResult { info!("T0 Fire"); let nt = timer.get_waketime().wrapping_add(1_000_000); timer.set_hw_irq_active(false); timer.set_waketime(nt); interrupt_free(|cs| SCHEDULER.borrow(cs).borrow_mut().as_mut().unwrap().add_timer(timer)); debug!("T0 Resched {}", nt); () }
fn test_timer1_callback(timer: &mut Rp2040Timer<Alarm2>) -> StepEventResult { info!("T1 Fire"); let nt = timer.get_waketime().wrapping_add(1_500_000); timer.set_hw_irq_active(false); timer.set_waketime(nt); interrupt_free(|cs| SCHEDULER.borrow(cs).borrow_mut().as_mut().unwrap().add_timer(timer)); debug!("T1 Resched {}", nt); () }
fn master_scheduler_timer_callback(_timer: &mut Rp2040Timer<Alarm1>) -> StepEventResult { interrupt_free(|cs| SCHEDULER.borrow(cs).borrow_mut().as_mut().unwrap().service_master_timer()); () }
pub fn dispatch_klipper_task_from_scheduler(task_id: u32) {
    match task_id {
        TEST_TIMER0_ID => { /* ... */ },
        TEST_TIMER1_ID => { /* ... */ },
        STEPPER_X_SCHEDULER_TASK_ID => {
            interrupt_free(|cs| {
                let mut scheduler_opt = SCHEDULER.borrow(cs).borrow_mut();
                let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
                let mut stepper_x_opt = STEPPER_X.borrow(cs).borrow_mut();
                if let (Some(ref mut scheduler), Some(ref mut gpio_manager), Some(ref mut stepper_x)) =
                    (scheduler_opt.as_mut(), gpio_manager_opt.as_mut(), stepper_x_opt.as_mut()) {
                    stepper_x.step_event_callback(scheduler, gpio_manager);
                } else { warn!("dispatch: Failed to get resources for STEPPER_X_SCHEDULER_TASK_ID ({})", task_id); }
            });
        }
        _ => warn!("Scheduler: Dispatch for unknown task ID {}", task_id),
    }
}

// --- Entry Point & Main Loop ---
#[entry]
fn main() -> ! {
    // ... (Peripheral setup, globals init, timer/scheduler/stepper setup as before) ...
    info!("Setup complete, entering main loop.");
    let mut loop_count = 0u32;
    let mut led_state = false;
    let mut delay = /* ... */;
    loop { /* Main loop body as before */ }
}

// --- Interrupt Handlers ---
#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_1() { /* ... */ }

// --- Helper Functions ---
fn get_pwm_slice_channel_for_pin(pin_id: u8) -> Option<(u8, bool)> { /* ... */ }
fn calculate_pwm_settings(sys_clk_hz: u32, target_freq_hz: u32) -> (u8, u8, u16) { /* ... */ }
fn serial_write_line(serial: &mut SerialPort<UsbBus>, data: &str) { /* ... */ }


// --- process_command (with MOVE command) ---
fn process_command(line: &str, serial: &mut SerialPort<UsbBus>) {
    let mut parts = line.trim().splitn(2, |c: char| c.is_whitespace());
    let command = parts.next().unwrap_or("").to_ascii_uppercase();
    let args_str = parts.next().unwrap_or("");
    let parsed_args_result = if !args_str.is_empty() { parse_gcode_arguments(args_str) } else { Ok(ParsedArgs::new()) };

    match parsed_args_result {
        Ok(args) => {
            let cmd_result : Result<(), &'static str> = if command == "PING" { /* ... */ Ok(()) }
            else if command == "ID" { /* ... */ Ok(()) }
            else if command == "ECHO" { /* ... */ Ok(()) }
            else if command == "SET_PIN" { /* ... */ Ok(()) }
            else if command == "GET_PIN" { /* ... */ Ok(()) }
            else if command == "QUERY_ADC" { /* ... */ Ok(()) }
            else if command == "SET_PWM" { /* ... */ Ok(()) }
            else if command == "MOVE" { // New MOVE command
                // Parse arguments
                let steps_opt = args.get(&'S').and_then(|v| match v { CommandArgValue::UInteger(u) => Some(*u), _ => None });
                let accel_opt = args.get(&'A').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                let vel_opt = args.get(&'V').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None });
                let start_vel_opt = args.get(&'V').and_then(|v| match v { CommandArgValue::Float(f) => Some(*f), CommandArgValue::UInteger(u) => Some(*u as f32), _ => None }).unwrap_or(0.0);
                let dir_opt = args.get(&'D').and_then(|v| match v { CommandArgValue::UInteger(u) => Some(*u == 1), _ => None });

                if let (Some(steps), Some(accel), Some(vel)) = (steps_opt, accel_opt, vel_opt) {
                    let mov = TrapezoidalMove {
                        total_steps: steps,
                        acceleration: accel,
                        start_velocity: start_vel_opt,
                        cruise_velocity: vel,
                    };

                    interrupt_free(|cs| {
                        let mut scheduler_opt = SCHEDULER.borrow(cs).borrow_mut();
                        let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
                        let mut stepper_x_opt = STEPPER_X.borrow(cs).borrow_mut();
                        let sys_clk_opt = SYSTEM_CLOCK_FREQ.borrow(cs).borrow();

                        if let (Some(ref mut scheduler), Some(ref mut gpio_manager), Some(ref mut stepper_x), Some(clock_freq)) =
                            (scheduler_opt.as_mut(), gpio_manager_opt.as_mut(), stepper_x_opt.as_mut(), sys_clk_opt.as_ref()) {

                            if stepper_x.current_move.is_some() {
                                return Err("Stepper X is already moving");
                            }

                            if let Some(dir_is_ccw) = dir_opt {
                                let dir = if dir_is_ccw { StepperDirection::CounterClockwise } else { StepperDirection::Clockwise };
                                stepper_x.set_direction(dir, gpio_manager)?;
                            }

                            match MovePlanner::new(&mov, *clock_freq) {
                                Ok(move_plan) => {
                                    stepper_x.current_move = Some(move_plan);
                                    stepper_x.current_step_num = 0;
                                    stepper_x.is_pulsing_high = false;
                                    stepper_x.step_period_ticks = move_plan.initial_period_ticks;

                                    let now = scheduler.read_time();
                                    stepper_x.next_step_waketime = now.wrapping_add(100); // Small delay to start
                                    scheduler.schedule_task(stepper_x.timer_id_for_scheduler, stepper_x.next_step_waketime);

                                    info!("MOVE command initiated: {} steps", mov.total_steps);
                                    Ok(())
                                }
                                Err(e) => Err(e),
                            }
                        } else {
                            Err("Stepper/Scheduler/GPIO not initialized")
                        }
                    })
                } else {
                    Err("Missing required S, A, or V arguments for MOVE")
                }
            }
            else if command.is_empty() { Ok(()) }
            else { Err("Unknown command") }
        }
        Err(e) => { Err("Argument parsing failed") }
    };
    // ... (handle cmd_result and send response) ...
}
