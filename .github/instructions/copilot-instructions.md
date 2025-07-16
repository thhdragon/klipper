# Copilot Instructions for Klipper Codebase

## Overview
- **Klipper** is a 3D printer firmware system that splits logic between a host (general-purpose computer, Python) and one or more microcontrollers (MCUs, C/Rust). The host runs "klippy" (Python), which communicates with MCUs for real-time control.
- The project is in the process of a Rust port (`klipper-rs`), mirroring the Python structure in `klipper-py`.
- Major directories:
  - `klipper-py/klippy/`: Main Python host firmware logic
  - `klipper-py/scripts/`: Utilities, build/test scripts, requirements
  - `klipper-rs/klippy/`: Rust port (WIP, mirrors Python modules)

## Architecture & Patterns
- **Host firmware entrypoint:** `klipper-py/klippy/klippy.py` (main loop, config, event system)
- **Config system:** `klippy/configfile.py` parses and validates printer configs, supports includes and autosave blocks. Use `PrinterConfig` and `ConfigWrapper` for config access.
- **Component pattern:** Each major printer subsystem (e.g., `gcode`, `mcu`, `pins`, `toolhead`) is a module with `add_printer_objects` and/or `add_early_printer_objects` for registration.
- **Dynamic module loading:** Extra features are loaded from `klippy/extras/` and `klippy/kinematics/` at runtime via config sections.
- **Event/callback system:** Printer state changes and events are handled via `register_event_handler` and `send_event`.
- **MCU protocol:** Host communicates with MCUs using protocol dictionaries (see `msgproto.py`, `serialhdl.py`).

## Developer Workflows
- **Install Python dependencies:**
  ```sh
  pip install -r klipper-py/scripts/klippy-requirements.txt
  ```
- **Run host firmware:**
  ```sh
  python3 klipper-py/klippy/klippy.py <config.cfg>
  ```
  - Use `-v` for debug, `-l <logfile>` for logs, `--import-test` to check module imports.
- **Run tests:**
  ```sh
  python3 klipper-py/scripts/test_klippy.py <test_case>
  ```
- **Build system:**
  - MCU firmware builds use the Makefile in `klipper-py/`.
  - Rust port: see `klipper-rs/` (WIP, structure mirrors Python).

## Project Conventions
- **Config files** support includes and autosave blocks (see `SAVE_CONFIG` command and `AUTOSAVE_HEADER` in `configfile.py`).
- **Python 3.7+** is required for host code.
- **Module registration**: Use `add_printer_objects`/`add_early_printer_objects` for new subsystems.
- **Tests** are defined as scripts in `klipper-py/scripts/`.
- **No .github/copilot-instructions.md existed previously; this is the canonical version.**

## Key Files & Directories
- `klipper-py/klippy/klippy.py`: Host firmware main loop
- `klipper-py/klippy/configfile.py`: Config parsing/validation
- `klipper-py/scripts/klippy-requirements.txt`: Python dependencies
- `klipper-py/scripts/test_klippy.py`: Test runner
- `klipper-rs/klippy/`: Rust port (mirrors Python modules)

## Integration Points
- **MCU communication:** via protocol dictionaries and serial/canbus (see `serialhdl.py`, `msgproto.py`)
- **Dynamic features:** loaded from `klippy/extras/` and `klippy/kinematics/` based on config

---

For more, see the [official documentation](https://www.klipper3d.org/). If any section is unclear or incomplete, please provide feedback for improvement.
