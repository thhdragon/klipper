// src/command_parser.rs
#![cfg_attr(not(test), no_std)]

use heapless::{String, FnvIndexMap};
use defmt::Format; // For deriving defmt::Format on CommandArgValue

// Maximum number of arguments a single command can have (e.g., G1 X10 Y20 Z5 F3000 -> 4 args)
pub const MAX_ARGS_PER_COMMAND: usize = 8;
// Maximum length for string argument values, if we support them.
// const MAX_STRING_ARG_LEN: usize = 64;

/// Represents the value of a parsed command argument.
#[derive(Clone, Debug, Format, PartialEq)] // PartialEq for testing
pub enum CommandArgValue {
    Float(f32),
    Integer(i32), // Using i32 to accommodate potential negative values if needed
    UInteger(u32),
    // String(String<MAX_STRING_ARG_LEN>), // Add later if needed
    // Bool(bool), // Often represented as UInteger(0) or UInteger(1)
}

// Type alias for the structure that holds parsed arguments.
// Maps a character key (e.g., 'X', 'Y', 'P', 'S') to its CommandArgValue.
pub type ParsedArgs = FnvIndexMap<char, CommandArgValue, MAX_ARGS_PER_COMMAND>;

// Helper methods for ParsedArgs could be added here or as trait impls if needed,
// e.g., to get a value as a specific type with error handling or defaults.
// Example:
// impl ParsedArgs {
//     pub fn get_f32(&self, key: char) -> Option<f32> {
//         self.get(&key).and_then(|val| match val {
//             CommandArgValue::Float(f) => Some(*f),
//             _ => None,
//         })
//     }
//     // Similar for get_i32, get_u32 etc.
// }

// Error type for parsing
#[derive(Debug, Format, PartialEq, Eq)]
pub enum ArgParseError {
    MalformedArgument,      // e.g., "X" without a value, or "X=Y"
    ValueParseError,        // e.g., "Xabc" when number expected
    TooManyArguments,       // Exceeded MAX_ARGS_PER_COMMAND
    InvalidArgumentKey,     // Key is not a valid char or format
    // Add more specific errors as needed
}

// Basic parser function declaration (implementation in next step)
pub fn parse_gcode_arguments(arg_str: &str) -> Result<ParsedArgs, ArgParseError> {
    let mut parsed_args = ParsedArgs::new();
    let mut current_key: Option<char> = None;
    let mut current_value_str = String::<32>::new(); // Buffer for value part, 32 chars max for a value

    for char_code in arg_str.chars() {
        match char_code {
            'A'..='Z' | 'a'..='z' => { // Start of a new key
                // Process previous key-value pair if any
                if let Some(key) = current_key {
                    if current_value_str.is_empty() {
                        return Err(ArgParseError::MalformedArgument); // Key without value e.g. "X Y"
                    }
                    let value = parse_value(&current_value_str)?;
                    if parsed_args.insert(key.to_ascii_uppercase(), value).is_err() {
                        return Err(ArgParseError::TooManyArguments);
                    }
                    current_value_str.clear();
                }
                current_key = Some(char_code.to_ascii_uppercase());
            }
            '.' | '-' | '+' | '0'..='9' => { // Part of a value
                if current_key.is_none() {
                    // Value char before any key, e.g. " 123" or "G1 123"
                    // Depending on strictness, this could be an error or ignored.
                    // For G-code, usually values are attached to keys. Let's error.
                    if !char_code.is_whitespace() { // Ignore leading whitespace if no key yet
                        // return Err(ArgParseError::MalformedArgument); // Value without preceding key
                    }
                }
                if current_value_str.push(char_code).is_err() {
                    // Value string too long, should be handled by parse_value if it truncates
                    // or return an error specific to value length.
                    // For now, assume parse_value handles this or it's an implicit error.
                    // This buffer is small (32), real values might be longer.
                    // Let's return an error here.
                    return Err(ArgParseError::ValueParseError); // Or a new "ValueTooLong"
                }
            }
            ' ' | '\t' | '\r' | '\n' => { // Whitespace, acts as separator or end of value
                if let Some(key) = current_key {
                    if !current_value_str.is_empty() {
                        let value = parse_value(&current_value_str)?;
                        if parsed_args.insert(key.to_ascii_uppercase(), value).is_err() {
                            return Err(ArgParseError::TooManyArguments);
                        }
                        current_value_str.clear();
                        current_key = None; // Reset key after processing value
                    } else {
                        // Key followed by whitespace without value, e.g. "X "
                        // This could be an error if values are mandatory for keys.
                        // Or it could mean the key is a flag (not supported by CommandArgValue yet)
                        // For now, if a key was set, whitespace means end of its value.
                        // If value_str is empty, it means error like "X Y" handled above.
                    }
                }
                // Ignore whitespace if no current key.
            }
            _ => {
                // Invalid character in argument string
                return Err(ArgParseError::MalformedArgument);
            }
        }
    }

    // Process the last key-value pair if any
    if let Some(key) = current_key {
        if current_value_str.is_empty() {
            return Err(ArgParseError::MalformedArgument); // Key at end of string without value
        }
        let value = parse_value(&current_value_str)?;
        if parsed_args.insert(key.to_ascii_uppercase(), value).is_err() {
            return Err(ArgParseError::TooManyArguments);
        }
    }

    Ok(parsed_args)
}

/// Parses a string value into a CommandArgValue.
/// Heuristic: if it contains '.', parse as f32. Otherwise, try i32 then u32.
fn parse_value(value_str: &str) -> Result<CommandArgValue, ArgParseError> {
    if value_str.is_empty() {
        return Err(ArgParseError::ValueParseError); // Should not happen if called correctly
    }

    // Trim whitespace just in case, though outer parser should handle most.
    let trimmed_val = value_str.trim();
    if trimmed_val.is_empty() {
        return Err(ArgParseError::ValueParseError);
    }

    // Heuristic for float: presence of '.'
    if trimmed_val.contains('.') {
        if let Ok(f_val) = trimmed_val.parse::<f32>() {
            return Ok(CommandArgValue::Float(f_val));
        } else {
            return Err(ArgParseError::ValueParseError); // Failed to parse as f32
        }
    }

    // Try parsing as i32 (allows negative integers)
    if let Ok(i_val) = trimmed_val.parse::<i32>() {
        return Ok(CommandArgValue::Integer(i_val));
    }

    // Try parsing as u32 (for positive integers, e.g. pin numbers, speeds)
    if let Ok(u_val) = trimmed_val.parse::<u32>() {
        return Ok(CommandArgValue::UInteger(u_val));
    }

    Err(ArgParseError::ValueParseError) // Could not parse as any known numeric type
}
