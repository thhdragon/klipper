//! Rust port of Python extras/display/__init__.py
// Exposes load_config and load_config_prefix, delegating to display.rs

use anyhow::{Result, bail};

use super::display;

/// Load the display config (delegates to display.rs)
pub fn load_config<C: ConfigLike>(config: &C) -> Result<()> {
    display::load_config(config)
}

/// Load a display config with prefix validation (mirrors load_config_prefix in Python)
pub fn load_config_prefix<C: ConfigLike>(config: &C) -> Result<()> {
    if !config.has_section("display") {
        bail!("A primary [display] section must be defined in printer.cfg to use auxilary displays");
    }
    let name = config.get_name().split_whitespace().last().unwrap_or("");
    if name == "display" {
        bail!("Section name [display display] is not valid. Please choose a different postfix.");
    }
    display::load_config(config)
}

/// Trait for config objects (to be implemented by actual config type)
pub trait ConfigLike {
    fn has_section(&self, section: &str) -> bool;
    fn get_name(&self) -> &str;
}
