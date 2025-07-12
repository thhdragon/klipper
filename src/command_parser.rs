// src/command_parser.rs
#![cfg_attr(not(test), no_std)]

use heapless::{String, FnvIndexMap};
use defmt::Format;

pub const MAX_ARGS_PER_COMMAND: usize = 8;
pub const MAX_STRING_ARG_LEN: usize = 64;

#[derive(Clone, Debug, Format, PartialEq)]
pub enum CommandArgValue {
    Float(f32),
    Integer(i32),
    UInteger(u32),
    String(String<MAX_STRING_ARG_LEN>),
}

pub type ParsedArgs = FnvIndexMap<char, CommandArgValue, MAX_ARGS_PER_COMMAND>;

#[derive(Debug, Format, PartialEq, Eq)]
pub enum ArgParseError {
    MalformedArgument,
    ValueParseError,
    TooManyArguments,
    InvalidArgumentKey,
}

pub fn parse_gcode_arguments(arg_str: &str) -> Result<ParsedArgs, ArgParseError> {
    let mut parsed_args = ParsedArgs::new();
    let mut chars = arg_str.trim().chars().peekable();

    while let Some(key_char_candidate) = chars.next() {
        if key_char_candidate.is_whitespace() {
            continue;
        }

        let key = key_char_candidate.to_ascii_uppercase();

        if key == '*' {
            let mut value_buffer_for_star = String::<MAX_STRING_ARG_LEN>::new();
            for rest_char in chars {
                if value_buffer_for_star.push(rest_char).is_err() {
                    // String collected so far might be useful for error message, but ValueParseError is generic
                    return Err(ArgParseError::ValueParseError); // String too long
                }
            }

            let trimmed_star_value_str = value_buffer_for_star.as_str().trim();
            let final_star_value = String::from_str(trimmed_star_value_str)
                .map_err(|_| ArgParseError::ValueParseError)?; // Error if trim results in > MAX_LEN (unlikely) or other issue

            if parsed_args.insert(key, CommandArgValue::String(final_star_value)).is_err() {
                return Err(ArgParseError::TooManyArguments);
            }
            break;
        } else if key.is_ascii_alphabetic() {
            let mut value_str_buffer = String::<32>::new();

            while let Some(&peek_char) = chars.peek() {
                if peek_char.is_whitespace() || peek_char.is_ascii_alphabetic() || peek_char == '*' {
                    break;
                }
                if value_str_buffer.push(chars.next().unwrap()).is_err() {
                    return Err(ArgParseError::ValueParseError);
                }
            }

            if value_str_buffer.is_empty() {
                return Err(ArgParseError::MalformedArgument);
            }

            let value = parse_value(&value_str_buffer)?;
            if parsed_args.insert(key, value).is_err() {
                return Err(ArgParseError::TooManyArguments);
            }
        } else {
            if !key_char_candidate.is_whitespace() {
                 return Err(ArgParseError::InvalidArgumentKey);
            }
        }
    }
    Ok(parsed_args)
}

fn parse_value(value_str: &str) -> Result<CommandArgValue, ArgParseError> {
    if value_str.is_empty() { return Err(ArgParseError::ValueParseError); }
    let trimmed_val = value_str.trim();
    if trimmed_val.is_empty() { return Err(ArgParseError::ValueParseError); }

    if trimmed_val.contains('.') {
        if let Ok(f_val) = trimmed_val.parse::<f32>() {
            return Ok(CommandArgValue::Float(f_val));
        }
        // If it has a '.' but doesn't parse as f32, it will fall through to string.
    }

    if !trimmed_val.contains('.') { // Only try int/uint if no decimal point
        if let Ok(i_val) = trimmed_val.parse::<i32>() {
            return Ok(CommandArgValue::Integer(i_val));
        }
        if let Ok(u_val) = trimmed_val.parse::<u32>() {
            return Ok(CommandArgValue::UInteger(u_val));
        }
    }

    match String::<MAX_STRING_ARG_LEN>::from_str(trimmed_val) {
        Ok(s) => Ok(CommandArgValue::String(s)),
        Err(_) => Err(ArgParseError::ValueParseError),
    }
}
