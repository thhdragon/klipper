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
    gpio::{Pins as RpHalPins, Pin, FunctionSio, SioInput, PullDown, PushPullOutput, FloatingInput, PullUpInput, FunctionSpi},
    adc::Adc as RpHalAdc,
    pwm::Slices as RpHalPwmSlices,
    spi::Spi as RpHalSpi,
};
use rp2040_hal::clocks::UsbClock;
use rp2040_hal::gpio::PinId;
use rp2040_hal::pwm::{Slice as RpSlice, FreeRunning, SliceId as RpSliceId, ValidSliceMode, Channel as RpPwmChannelExt, PwmA, PwmB};
use rp2040_hal::spi::Enabled;
use embedded_hal::digital::v2::{OutputPin, InputPin};
use fugit::RateExtU32;

// Klipper HAL Traits and Implementations
use klipper_mcu_lib::{
    hal::{Timer as KlipperHalTimer, PullType, AdcChannel, AdcError, PwmChannel, PwmError, Spi as KlipperSpi, SpiError, StepEventResult},
    sched::{SchedulerState, KlipperSchedulerTrait},
    gpio_manager::GpioManager,
    command_parser::{parse_gcode_arguments, CommandArgValue, ArgParseError, ParsedArgs, MAX_STRING_ARG_LEN},
    stepper::{Stepper, StepperDirection, TrapezoidalMove, MovePlanner},
};
use klipper_mcu_lib::rp2040_hal_impl::timer::Rp2040Timer;
use klipper_mcu_lib::rp2040_hal_impl::adc::Rp2040AdcChannel;
use klipper_mcu_lib::rp2040_hal_impl::pwm::Rp2040PwmChannel;
use klipper_mcu_lib::rp2040_hal_impl::spi::Rp2040Spi;


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

// SPI Globals
type Spi0SckPin = Pin<rp2040_hal::gpio::bank0::Gpio18, FunctionSpi, PullDown>;
type Spi0MosiPin = Pin<rp2040_hal::gpio::bank0::Gpio19, FunctionSpi, PullDown>;
type Spi0MisoPin = Pin<rp2040_hal::gpio::bank0::Gpio20, FunctionSpi, PullDown>;
type Spi0Pins = (Spi0MosiPin, Spi0MisoPin, Spi0SckPin);
type RpHalSpi0 = RpHalSpi<Enabled, pac::SPI0, Spi0Pins, 8>;
static SPI0_PERIPHERAL: InterruptMutex<RefCell<Option<Rp2040Spi<RpHalSpi0>>>> = InterruptMutex::new(RefCell::new(None));

// --- Callbacks and Dispatchers ---
// ... (All callback and dispatcher functions as before) ...
fn test_timer0_callback(timer: &mut Rp2040Timer<Alarm0>) -> StepEventResult { /* ... */ }
fn test_timer1_callback(timer: &mut Rp2040Timer<Alarm2>) -> StepEventResult { /* ... */ }
fn master_scheduler_timer_callback(_timer: &mut Rp2040Timer<Alarm1>) -> StepEventResult { /* ... */ }
pub fn dispatch_klipper_task_from_scheduler(task_id: u32) { /* ... */ }

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

fn hex_str_to_bytes(hex_str: &str) -> Result<heapless::Vec<u8, 32>, &'static str> {
    let mut bytes = heapless::Vec::new();
    if hex_str.len() % 2 != 0 { return Err("Hex string must have even length"); }
    for i in (0..hex_str.len()).step_by(2) {
        let byte_str = &hex_str[i..i+2];
        let byte = u8::from_str_radix(byte_str, 16).map_err(|_| "Invalid hex character")?;
        if bytes.push(byte).is_err() { return Err("Hex data too long"); }
    }
    Ok(bytes)
}

// --- process_command (with DEBUG_SPI command) ---
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
            else if command == "MOVE" { /* ... */ Ok(()) }
            else if command == "DEBUG_SPI" {
                let write_data_opt = args.get(&'W').and_then(|val| match val { CommandArgValue::String(s) => Some(s.clone()), _ => None });
                let read_len_opt = args.get(&'R').and_then(|val| match val { CommandArgValue::UInteger(u) => Some(*u as usize), _ => None });
                let cs_pin_opt = args.get(&'C').and_then(|val| match val { CommandArgValue::UInteger(u) => Some(*u as u8), _ => None });

                if let Some(write_hex_str) = write_data_opt {
                    interrupt_free(|cs| {
                        // Get resources
                        let mut spi_opt = SPI0_PERIPHERAL.borrow(cs).borrow_mut();
                        let mut gpio_manager_opt = GPIO_MANAGER.borrow(cs).borrow_mut();
                        if let (Some(ref mut spi), Some(ref mut manager)) = (spi_opt.as_mut(), gpio_manager_opt.as_mut()) {

                            // 1. Handle CS pin if provided
                            let mut cs_pin_typed : Option<Pin<_, PushPullOutput>> = None;
                            if let Some(cs_pin_num) = cs_pin_opt {
                                // This requires a big match statement for pin_num to get the typed pin
                                // For now, let's just support one CS pin, e.g. GPIO17
                                if cs_pin_num == 17 {
                                    if let Some(pin) = manager.Gpio17.take() {
                                        let mut pin_out = pin.into_push_pull_output();
                                        let _ = pin_out.set_high(); // De-assert CS initially
                                        cs_pin_typed = Some(pin_out);
                                    } else { return Err("CS Pin 17 is busy"); }
                                } else { return Err("Unsupported CS Pin"); }
                            }

                            // 2. Parse hex data
                            let mut write_bytes = match hex_str_to_bytes(&write_hex_str) {
                                Ok(b) => b,
                                Err(e) => return Err(e),
                            };

                            // 3. Assert CS (pull low)
                            if let Some(ref mut cs) = cs_pin_typed { let _ = cs.set_low(); }

                            // 4. Perform SPI transaction
                            let spi_result = if let Some(read_len) = read_len_opt {
                                // Perform a transfer
                                if write_bytes.len() < read_len {
                                    // Pad write_bytes with 0s if we need to read more than we write
                                    if write_bytes.resize_default(read_len).is_err() {
                                        return Err("Read length exceeds buffer capacity");
                                    }
                                }
                                spi.transfer(&mut write_bytes[..read_len])
                                    .map(|read_slice| {
                                        let mut response = heapless::String::<192>::new(); // 3*64
                                        use core::fmt::Write;
                                        write!(response, "READ_HEX: ").unwrap();
                                        for byte in read_slice {
                                            write!(response, "{:02X}", byte).unwrap();
                                        }
                                        write!(response, "\r\n").unwrap();
                                        serial_write_line(serial, response.as_str());
                                        Ok(())
                                    })
                                    .map_err(|e| { error!("SPI Transfer Error: {:?}", e); "SPI Transfer Error"})
                            } else {
                                // Perform a write-only
                                spi.write(&write_bytes).map_err(|e| { error!("SPI Write Error: {:?}", e); "SPI Write Error"})
                            };

                            // 5. De-assert CS (pull high)
                            if let Some(ref mut cs) = cs_pin_typed { let _ = cs.set_high(); }

                            // 6. Return CS pin to manager
                            if let Some(cs) = cs_pin_typed {
                                manager.Gpio17.replace(cs.into_mode());
                            }

                            if spi_result.is_ok() {
                                serial_write_line(serial, "ok\r\n");
                                Ok(())
                            } else {
                                spi_result
                            }
                        } else { Err("SPI or GpioManager not initialized") }
                    })
                } else { Err("Missing required W (WRITE) argument") }
            }
            else if command.is_empty() { /* Ignore */ }
            else { Err("Unknown command") }
        }
        Err(e) => { Err("Argument parsing failed") }
    };
    // ... (handle cmd_result and send response) ...
}
