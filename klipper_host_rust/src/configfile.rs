// klipper_host_rust/src/configfile.rs
// Corresponds to klippy/configfile.py - Reads and processes printer configuration.

use std::collections::HashMap;
use std::num::{ParseFloatError, ParseIntError};

#[derive(Debug, Clone, PartialEq)]
pub enum ConfigError {
    SectionNotFound(String),
    OptionNotFound(String, String),
    ParseError(String, String, String), // option, value, type_expected
    ValidationError(String), // General validation like min/max
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::SectionNotFound(s) => write!(f, "Section not found: [{}]", s),
            ConfigError::OptionNotFound(s, o) => write!(f, "Option '{}' not found in section '[{}]'", o, s),
            ConfigError::ParseError(o, v, t) => write!(f, "Failed to parse option '{}' value '{}' as {}", o, v, t),
            ConfigError::ValidationError(msg) => write!(f, "Validation error: {}", msg),
        }
    }
}

impl std::error::Error for ConfigError {}

#[derive(Debug, Default, Clone)]
pub struct Configfile {
    // Stores config data as: section_name -> { option_name -> option_value_as_string }
    // Section and option names are stored as lowercase.
    data: HashMap<String, HashMap<String, String>>,
    filename: Option<String>, // Optional: Store the filename it was loaded from
}

impl Configfile {
    pub fn new(filename: Option<String>) -> Self {
        Configfile {
            data: HashMap::new(),
            filename,
        }
    }

    /// Parses an INI-style configuration string.
    pub fn parse(&mut self, config_content: &str) -> Result<(), String> { // Using String for simple error reporting for now
        self.data.clear();
        let mut current_section_name: Option<String> = None;

        for line_raw in config_content.lines() {
            // 1. Strip comments (everything after '#' or ';')
            let line_no_comment = match line_raw.find(|c| c == '#' || c == ';') {
                Some(index) => &line_raw[..index],
                None => line_raw,
            };
            let line = line_no_comment.trim();

            if line.is_empty() {
                continue; // Skip empty lines and comment-only lines
            }

            // 2. Check for section header
            if line.starts_with('[') && line.ends_with(']') {
                let section_name_raw = line[1..line.len() - 1].trim().to_lowercase();
                // Klipper allows "section_type section_name", we'll simplify to just section_name for now.
                // TODO: Handle section type vs name if needed later.
                if section_name_raw.is_empty() {
                    return Err("Empty section name found".to_string());
                }
                current_section_name = Some(section_name_raw.clone());
                self.data.entry(section_name_raw).or_insert_with(HashMap::new);
            }
            // 3. Check for key-value pair
            else if let Some(ref section_name) = current_section_name {
                let parts: Vec<&str> = line.splitn(2, '=').collect();
                if parts.len() == 2 {
                    let key = parts[0].trim().to_lowercase();
                    let value = parts[1].trim().to_string(); // Store original case for value, or also lowercase? Klipper seems to store as is.
                    if key.is_empty() {
                        return Err(format!("Empty key found in section [{}]", section_name));
                    }
                    self.data.get_mut(section_name).unwrap().insert(key, value);
                } else {
                    // Also support "key: value" format
                    let parts_colon: Vec<&str> = line.splitn(2, ':').collect();
                    if parts_colon.len() == 2 {
                        let key = parts_colon[0].trim().to_lowercase();
                        let value = parts_colon[1].trim().to_string();
                        if key.is_empty() {
                             return Err(format!("Empty key found in section [{}]", section_name));
                        }
                        self.data.get_mut(section_name).unwrap().insert(key, value);
                    } else if !line.is_empty() {
                        // Line is not a section, not a valid key-value, and not empty/comment
                        return Err(format!("Malformed line in section [{}]: {}", section_name, line_raw));
                    }
                }
            } else if !line.is_empty() {
                // Line outside of any section (and not a section header itself)
                return Err(format!("Line outside of section: {}", line_raw));
            }
        }
        Ok(())
    }

    // Helper to get a raw string value
    pub fn get_str(&self, section: &str, option: &str) -> Result<&String, ConfigError> {
        let section_lower = section.to_lowercase();
        let option_lower = option.to_lowercase();
        self.data
            .get(&section_lower)
            .ok_or_else(|| ConfigError::SectionNotFound(section.to_string()))?
            .get(&option_lower)
            .ok_or_else(|| ConfigError::OptionNotFound(section.to_string(), option.to_string()))
    }

    // Generic getter that takes a default Option
    pub fn get<'a>(&'a self, section: &str, option: &str, default: Option<&'a str>) -> Result<String, ConfigError> {
        match self.get_str(section, option) {
            Ok(s_ref) => Ok(s_ref.clone()),
            Err(ConfigError::OptionNotFound(_, _)) => {
                if let Some(d) = default {
                    Ok(d.to_string())
                } else {
                    Err(ConfigError::OptionNotFound(section.to_string(), option.to_string()))
                }
            }
            Err(e) => Err(e), // Propagate other errors like SectionNotFound
        }
    }

    // getfloat with optional default, min, max checks
    pub fn getfloat(&self, section: &str, option: &str, default: Option<f64>, minval: Option<f64>, maxval: Option<f64>) -> Result<f64, ConfigError> {
        let val_str_res = self.get_str(section, option);

        match val_str_res {
            Ok(s_val) => {
                let parsed_val = s_val.parse::<f64>().map_err(|_| {
                    ConfigError::ParseError(option.to_string(), s_val.clone(), "float".to_string())
                })?;
                if let Some(min) = minval {
                    if parsed_val < min {
                        return Err(ConfigError::ValidationError(format!(
                            "Option '{}' in section '[{}]' ({}) must be >= {}", option, section, parsed_val, min
                        )));
                    }
                }
                if let Some(max) = maxval {
                    if parsed_val > max {
                        return Err(ConfigError::ValidationError(format!(
                            "Option '{}' in section '[{}]' ({}) must be <= {}", option, section, parsed_val, max
                        )));
                    }
                }
                Ok(parsed_val)
            }
            Err(ConfigError::OptionNotFound(_, _)) => {
                if let Some(d) = default {
                    Ok(d)
                } else {
                    Err(ConfigError::OptionNotFound(section.to_string(), option.to_string()))
                }
            }
            Err(e) => Err(e),
        }
    }

    // getint with optional default, min, max checks
    pub fn getint(&self, section: &str, option: &str, default: Option<i64>, minval: Option<i64>, maxval: Option<i64>) -> Result<i64, ConfigError> {
        let val_str_res = self.get_str(section, option);
        match val_str_res {
            Ok(s_val) => {
                let parsed_val = s_val.parse::<i64>().map_err(|_| {
                    ConfigError::ParseError(option.to_string(), s_val.clone(), "integer".to_string())
                })?;
                 if let Some(min) = minval {
                    if parsed_val < min {
                        return Err(ConfigError::ValidationError(format!(
                            "Option '{}' in section '[{}]' ({}) must be >= {}", option, section, parsed_val, min
                        )));
                    }
                }
                if let Some(max) = maxval {
                    if parsed_val > max {
                        return Err(ConfigError::ValidationError(format!(
                            "Option '{}' in section '[{}]' ({}) must be <= {}", option, section, parsed_val, max
                        )));
                    }
                }
                Ok(parsed_val)
            }
            Err(ConfigError::OptionNotFound(_, _)) => {
                if let Some(d) = default {
                    Ok(d)
                } else {
                    Err(ConfigError::OptionNotFound(section.to_string(), option.to_string()))
                }
            }
            Err(e) => Err(e),
        }
    }

    // getboolean with optional default
    pub fn getboolean(&self, section: &str, option: &str, default: Option<bool>) -> Result<bool, ConfigError> {
        let val_str_res = self.get_str(section, option);
        match val_str_res {
            Ok(s_val) => {
                match s_val.to_lowercase().as_str() {
                    "true" | "yes" | "on" | "1" => Ok(true),
                    "false" | "no" | "off" | "0" => Ok(false),
                    _ => Err(ConfigError::ParseError(option.to_string(), s_val.clone(), "boolean".to_string())),
                }
            }
            Err(ConfigError::OptionNotFound(_, _)) => {
                if let Some(d) = default {
                    Ok(d)
                } else {
                    Err(ConfigError::OptionNotFound(section.to_string(), option.to_string()))
                }
            }
            Err(e) => Err(e),
        }
    }

    // Methods for ToolHead::new() to easily add sections/options for testing klippy_main.rs
    // In a real scenario, these would not be needed as config is parsed from string/file.
    #[cfg(test)]
    pub fn add_section(&mut self, section_name: &str) {
        self.data.entry(section_name.to_lowercase()).or_insert_with(HashMap::new);
    }

    #[cfg(test)]
    pub fn set(&mut self, section: &str, option: &str, value: &str) {
        let section_lower = section.to_lowercase();
        let option_lower = option.to_lowercase();
        self.data.entry(section_lower).or_insert_with(HashMap::new).insert(option_lower, value.to_string());
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_parsed_config(content: &str) -> Configfile {
        let mut cf = Configfile::new(None);
        cf.parse(content).unwrap();
        cf
    }

    #[test]
    fn test_parse_simple_config() {
        let content = "[section1]\nkey1 = value1\nkey2:value2\n\n[section2]\nkeyA=valueA";
        let cf = create_parsed_config(content);

        assert_eq!(cf.get("section1", "key1", None).unwrap(), "value1");
        assert_eq!(cf.get("section1", "KEY2", None).unwrap(), "value2"); // Case insensitive option
        assert_eq!(cf.get("SECTION2", "keya", None).unwrap(), "valueA"); // Case insensitive section
    }

    #[test]
    fn test_parse_with_comments_and_whitespace() {
        let content = r#"
# This is a full line comment
[section_alpha] ; another comment style
  key_one = value_one # trailing comment
  key_two:    value_two
  ; key_three = commented_out_value
[section_beta]
val = 123
"#;
        let cf = create_parsed_config(content);
        assert_eq!(cf.get("section_alpha", "key_one", None).unwrap(), "value_one");
        assert_eq!(cf.get("section_alpha", "key_two", None).unwrap(), "value_two");
        assert!(cf.get("section_alpha", "key_three", None).is_err());
        assert_eq!(cf.getint("section_beta", "val", None, None, None).unwrap(), 123);
    }

    #[test]
    fn test_getters_with_types_and_defaults() {
        let content = "[types]\nmyfloat = 3.14\nmyint = 42\nmybool_true = true\nmybool_false = No";
        let cf = create_parsed_config(content);

        assert_eq!(cf.getfloat("types", "myfloat", None, None, None).unwrap(), 3.14);
        assert_eq!(cf.getint("types", "myint", None, None, None).unwrap(), 42);
        assert_eq!(cf.getboolean("types", "mybool_true", None).unwrap(), true);
        assert_eq!(cf.getboolean("types", "mybool_false", None).unwrap(), false);

        // Test defaults
        assert_eq!(cf.get("types", "nonexistent", Some("default_val")).unwrap(), "default_val");
        assert_eq!(cf.getfloat("types", "nonexistent_float", Some(1.23), None, None).unwrap(), 1.23);
        assert_eq!(cf.getint("types", "nonexistent_int", Some(100), None, None).unwrap(), 100);
        assert_eq!(cf.getboolean("types", "nonexistent_bool", Some(true)).unwrap(), true);

        // Test missing without default
        assert!(cf.get("types", "required_missing", None).is_err());
        assert!(cf.getfloat("types", "required_float_missing", None, None, None).is_err());
    }

    #[test]
    fn test_getfloat_with_validation() {
        let content = "[validation]\nval = 10.0";
        let cf = create_parsed_config(content);
        assert!(cf.getfloat("validation", "val", None, Some(0.0), Some(20.0)).is_ok());
        assert_eq!(cf.getfloat("validation", "val", None, Some(0.0), Some(20.0)).unwrap(), 10.0);

        let res_min_err = cf.getfloat("validation", "val", None, Some(15.0), Some(20.0));
        assert!(res_min_err.is_err());
        if let Err(ConfigError::ValidationError(msg)) = res_min_err {
            assert!(msg.contains("must be >= 15"));
        } else { panic!("Expected ValidationError for minval"); }

        let res_max_err = cf.getfloat("validation", "val", None, Some(0.0), Some(5.0));
        assert!(res_max_err.is_err());
         if let Err(ConfigError::ValidationError(msg)) = res_max_err {
            assert!(msg.contains("must be <= 5"));
        } else { panic!("Expected ValidationError for maxval"); }
    }

    #[test]
    fn test_parse_errors() {
        let mut cf_empty_section = Configfile::new(None);
        assert!(cf_empty_section.parse("[]\nkey=val").is_err());

        let mut cf_empty_key = Configfile::new(None);
        assert!(cf_empty_key.parse("[sec]\n=val").is_err());

        let mut cf_malformed_line = Configfile::new(None);
        assert!(cf_malformed_line.parse("[sec]\njustavalue").is_err());

        let mut cf_outside_section = Configfile::new(None);
        assert!(cf_outside_section.parse("key=val").is_err());
    }
     #[test]
    fn test_parse_various_bool_strings() {
        let content = "[bools]\nt1=True\nt2=yes\nt3=ON\nt4=1\nf1=False\nf2=NO\nf3=off\nf4=0";
        let cf = create_parsed_config(content);
        assert_eq!(cf.getboolean("bools", "t1", None).unwrap(), true);
        assert_eq!(cf.getboolean("bools", "t2", None).unwrap(), true);
        assert_eq!(cf.getboolean("bools", "t3", None).unwrap(), true);
        assert_eq!(cf.getboolean("bools", "t4", None).unwrap(), true);
        assert_eq!(cf.getboolean("bools", "f1", None).unwrap(), false);
        assert_eq!(cf.getboolean("bools", "f2", None).unwrap(), false);
        assert_eq!(cf.getboolean("bools", "f3", None).unwrap(), false);
        assert_eq!(cf.getboolean("bools", "f4", None).unwrap(), false);
    }
}
