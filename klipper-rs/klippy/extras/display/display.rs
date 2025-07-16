//! Rust port of klippy/extras/display/display.py
//! Main LCD display support (idiomatic Rust)
//
// This module provides types and functions to parse and use Klipper's display.cfg
// in a strongly-typed, idiomatic Rust way.

use anyhow::{Result, Context};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

/// Represents a [display_template ...] section in display.cfg
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayTemplate {
    pub name: String,
    pub params: HashMap<String, String>,
    pub text: String,
}

/// Represents a [display_data ...] section in display.cfg
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayData {
    pub group: String,
    pub item: String,
    pub position: (u32, u32),
    pub text: String,
}

/// Registry for all display templates, groups, and glyphs, similar to Python's PrinterDisplayTemplate.
#[derive(Debug, Clone)]
pub struct PrinterDisplay {
    pub templates: HashMap<String, DisplayTemplate>,
    pub groups: HashMap<String, DisplayGroup>,
    pub glyphs: HashMap<String, DisplayGlyph>,
}

impl PrinterDisplay {
    /// Build a PrinterDisplay from a DisplayConfig.
    pub fn from_config(cfg: &DisplayConfig) -> Self {
        let mut groups = HashMap::new();
        for (group, items) in &cfg.data {
            groups.insert(group.clone(), DisplayGroup::from_items(items));
        }
        Self {
            templates: cfg.templates.clone(),
            groups,
            glyphs: cfg.glyphs.clone(),
        }
    }

    /// Lookup a display template by name.
    pub fn template(&self, name: &str) -> Option<&DisplayTemplate> {
        self.templates.get(name)
    }

    /// Lookup a display group by name.
    pub fn group(&self, name: &str) -> Option<&DisplayGroup> {
        self.groups.get(name)
    }

    /// Lookup a display glyph by name.
    pub fn glyph(&self, name: &str) -> Option<&DisplayGlyph> {
        self.glyphs.get(name)
    }
}

/// A group of display data items, sorted by position and item name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayGroup {
    pub items: Vec<DisplayData>,
}

impl DisplayGroup {
    /// Build a DisplayGroup from a list of DisplayData items.
    pub fn from_items(items: &[DisplayData]) -> Self {
        let mut sorted = items.to_vec();
        sorted.sort_by(|a, b| a.position.cmp(&b.position).then(a.item.cmp(&b.item)));
        Self { items: sorted }
    }

    /// Get all items in this group, sorted by position and item name.
    pub fn items(&self) -> &[DisplayData] {
        &self.items
    }

    /// Get a specific item by item name.
    pub fn get(&self, item: &str) -> Option<&DisplayData> {
        self.items.iter().find(|d| d.item == item)
    }
}

/// Manages template environment and rendering for display templates.
///
/// # Example
/// ```rust
/// let cfg = load_display_config("klippy/extras/display/display.cfg")?;
/// let engine = DisplayTemplateEngine::from_config(&cfg)?;
/// let ctx = serde_json::json!({ "printer": { "foo": 1 } });
/// let params = std::collections::HashMap::new();
/// let rendered = engine.render_template("_heater_temperature", &ctx, &params)?;
/// println!("{}", rendered);
/// ```
pub struct DisplayTemplateEngine {
    env: minijinja::Environment<'static>,
}

impl DisplayTemplateEngine {
    /// Create a new engine and load all templates from DisplayConfig.
    pub fn from_config(cfg: &DisplayConfig) -> Result<Self> {
        let mut env = minijinja::Environment::new();
        for (name, template) in &cfg.templates {
            env.add_template(name, &template.text)
                .with_context(|| format!("Failed to add template '{}': {}", name, template.text))?;
        }
        Ok(Self { env })
    }

    /// Render a template by name with the given context and parameters.
    pub fn render_template(&self, name: &str, ctx: &serde_json::Value, params: &HashMap<String, serde_json::Value>) -> Result<String> {
        let tmpl = self.env.get_template(name)
            .with_context(|| format!("Template '{}' not found", name))?;
        // Merge context and params into one context
        let mut merged = ctx.clone();
        if let Some(obj) = merged.as_object_mut() {
            for (k, v) in params {
                obj.insert(k.clone(), v.clone());
            }
        }
        tmpl.render(minijinja::value::Value::from_serializable(&merged))
            .with_context(|| format!("Failed to render template '{}'", name))
    }
}

/// State for the display, including group switching and redraw logic.
///
/// Use this struct to manage the current display group, handle redraw requests,
/// and run the async update loop for the display.
///
/// # Example
/// ```rust,ignore
/// let mut state = DisplayState::new("_default_16x4", groups, engine);
/// state.set_group("_default_16x4").unwrap();
/// state.request_redraw();
/// tokio::spawn(async move {
///     state.run_update_loop(|group, engine| {
///         // draw logic here
///         Ok(())
///     }).await;
/// });
/// ```
pub struct DisplayState {
    pub current_group: String,
    pub redraw_request_pending: bool,
    pub redraw_time: Instant,
    pub last_update: Instant,
    pub display_groups: Arc<HashMap<String, DisplayGroup>>,
    pub template_engine: Arc<DisplayTemplateEngine>,
}

impl DisplayState {
    pub fn new(
        default_group: &str,
        display_groups: Arc<HashMap<String, DisplayGroup>>,
        template_engine: Arc<DisplayTemplateEngine>,
    ) -> Self {
        let now = Instant::now();
        Self {
            current_group: default_group.to_string(),
            redraw_request_pending: false,
            redraw_time: now,
            last_update: now,
            display_groups,
            template_engine,
        }
    }

    /// Request a redraw at the next minimum interval.
    pub fn request_redraw(&mut self) {
        if !self.redraw_request_pending {
            self.redraw_request_pending = true;
            self.redraw_time = Instant::now() + Duration::from_secs_f32(REDRAW_MIN_TIME);
        }
    }

    /// Switch the active display group.
    pub fn set_group(&mut self, group: &str) -> Result<()> {
        if self.display_groups.contains_key(group) {
            self.current_group = group.to_string();
            self.request_redraw();
            Ok(())
        } else {
            anyhow::bail!("Unknown display_data group '{}'.", group)
        }
    }

    /// Async display update loop. Calls the provided callback to actually draw.
    pub async fn run_update_loop<F>(&mut self, mut draw_cb: F)
    where
        F: FnMut(&DisplayGroup, &DisplayTemplateEngine) -> Result<()> + Send + 'static,
    {
        let mut interval = time::interval(Duration::from_secs_f32(REDRAW_TIME));
        loop {
            interval.tick().await;
            let now = Instant::now();
            if self.redraw_request_pending && now >= self.redraw_time {
                self.redraw_request_pending = false;
            }
            if let Some(group) = self.display_groups.get(&self.current_group) {
                let _ = draw_cb(group, &self.template_engine);
            }
            self.last_update = now;
        }
    }
}

/// Represents a [display_glyph ...] section in display.cfg.
///
/// Provides methods to parse and validate glyph data for both 16x16 and 5x8 formats.
///
/// # Example
/// ```rust
/// let glyph = cfg.glyphs.get("extruder").unwrap();
/// let icon16 = glyph.parse_icon16x16().unwrap().unwrap();
/// assert_eq!(icon16.len(), 16);
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayGlyph {
    pub name: String,
    pub data: Option<Vec<String>>, // original lines
    pub hd44780_slot: Option<u8>,
    pub hd44780_data: Option<Vec<String>>,
}

impl DisplayGlyph {
    /// Parse the dot/star 16x16 glyph data into a vector of u16 rows.
    pub fn parse_icon16x16(&self) -> Result<Option<Vec<u16>>> {
        if let Some(lines) = &self.data {
            let mut out = Vec::new();
            for line in lines {
                let line = line.trim().replace('.', "0").replace('*', "1");
                if line.is_empty() { continue; }
                if line.len() != 16 || line.chars().any(|c| c != '0' && c != '1') {
                    anyhow::bail!("Invalid glyph line in {}: {}", self.name, line);
                }
                let val = u16::from_str_radix(&line, 2)?;
                out.push(val);
            }
            if out.len() != 16 {
                anyhow::bail!("Glyph {} has {} lines, expected 16", self.name, out.len());
            }
            Ok(Some(out))
        } else {
            Ok(None)
        }
    }

    /// Parse the hd44780 5x8 glyph data into a vector of u8 rows.
    pub fn parse_icon5x8(&self) -> Result<Option<Vec<u8>>> {
        if let Some(lines) = &self.hd44780_data {
            let mut out = Vec::new();
            for line in lines {
                let line = line.trim().replace('.', "0").replace('*', "1");
                if line.is_empty() { continue; }
                if line.len() != 5 || line.chars().any(|c| c != '0' && c != '1') {
                    anyhow::bail!("Invalid hd44780 glyph line in {}: {}", self.name, line);
                }
                let val = u8::from_str_radix(&line, 2)?;
                out.push(val);
            }
            if out.len() != 8 {
                anyhow::bail!("hd44780 glyph {} has {} lines, expected 8", self.name, out.len());
            }
            Ok(Some(out))
        } else {
            Ok(None)
        }
    }
}

/// The full parsed display config
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayConfig {
    pub templates: HashMap<String, DisplayTemplate>,
    pub data: HashMap<String, Vec<DisplayData>>, // group -> items
    pub glyphs: HashMap<String, DisplayGlyph>,
}

/// A collection of display groups, each containing sorted DisplayData items.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayGroups {
    pub groups: HashMap<String, Vec<DisplayData>>, // group -> sorted items
}

impl DisplayGroups {
    /// Build DisplayGroups from the data field of DisplayConfig.
    pub fn from_data(data: &HashMap<String, Vec<DisplayData>>) -> Self {
        let mut groups = HashMap::new();
        for (group, items) in data {
            let mut sorted = items.clone();
            // Sort by (row, col) position, then by item name
            sorted.sort_by(|a, b| {
                a.position.cmp(&b.position).then(a.item.cmp(&b.item))
            });
            groups.insert(group.clone(), sorted);
        }
        Self { groups }
    }

    /// Get all group names.
    pub fn group_names(&self) -> impl Iterator<Item = &String> {
        self.groups.keys()
    }

    /// Get all items for a group, sorted by position.
    pub fn items_for_group(&self, group: &str) -> Option<&[DisplayData]> {
        self.groups.get(group).map(|v| v.as_slice())
    }

    /// Get a specific item by group and item name.
    pub fn get_item(&self, group: &str, item: &str) -> Option<&DisplayData> {
        self.groups.get(group)?.iter().find(|d| d.item == item)
    }
}

/// Loads and parses a display.cfg file into a strongly-typed DisplayConfig.
///
/// # Example
/// ```rust
/// let cfg = load_display_config("klippy/extras/display/display.cfg")?;
/// let groups = DisplayGroups::from_data(&cfg.data);
/// for group in groups.group_names() {
///     println!("Group: {}", group);
///     for item in groups.items_for_group(group).unwrap() {
///         println!("  Item: {} at {:?}", item.item, item.position);
///     }
/// }
/// ```
pub fn load_display_config<P: AsRef<Path>>(path: P) -> Result<DisplayConfig> {
    let content = fs::read_to_string(&path)
        .with_context(|| format!("Failed to read display config file: {}", path.as_ref().display()))?;
    parse_display_cfg(&content)
}

fn parse_display_cfg(cfg: &str) -> Result<DisplayConfig> {
    let mut templates = HashMap::new();
    let mut data: HashMap<String, Vec<DisplayData>> = HashMap::new();
    let mut glyphs = HashMap::new();

    let mut section: Option<(String, HashMap<String, String>)> = None;
    let mut multiline_key: Option<String> = None;
    let mut multiline_val = String::new();

    for line in cfg.lines() {
        let line = line.trim_end();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if line.starts_with('[') && line.ends_with(']') {
            // Save previous section
            if let Some((sec, mut map)) = section.take() {
                if let Some(key) = multiline_key.take() {
                    map.insert(key, multiline_val.trim_end().to_string());
                    multiline_val.clear();
                }
                parse_section(&sec, map, &mut templates, &mut data, &mut glyphs)?;
            }
            section = Some((line[1..line.len()-1].to_string(), HashMap::new()));
        } else if let Some((_, ref mut map)) = section {
            if let Some(key) = &multiline_key {
                // Continue multiline
                if line.starts_with(' ') || line.starts_with('\t') || line.is_empty() {
                    multiline_val.push_str(line.trim_start());
                    multiline_val.push('\n');
                    continue;
                } else {
                    // End multiline
                    map.insert(key.clone(), multiline_val.trim_end().to_string());
                    multiline_key = None;
                    multiline_val.clear();
                }
            }
            // Key: value or Key:
            if let Some(idx) = line.find(':') {
                let key = line[..idx].trim().to_string();
                let val = line[idx+1..].trim();
                if val.is_empty() {
                    // Start multiline
                    multiline_key = Some(key);
                    multiline_val.clear();
                } else {
                    map.insert(key, val.to_string());
                }
            }
        }
    }
    // Save last section
    if let Some((sec, mut map)) = section.take() {
        if let Some(key) = multiline_key.take() {
            map.insert(key, multiline_val.trim_end().to_string());
        }
        parse_section(&sec, map, &mut templates, &mut data, &mut glyphs)?;
    }

    Ok(DisplayConfig { templates, data, glyphs })
}

fn parse_section(
    sec: &str,
    map: HashMap<String, String>,
    templates: &mut HashMap<String, DisplayTemplate>,
    data: &mut HashMap<String, Vec<DisplayData>>,
    glyphs: &mut HashMap<String, DisplayGlyph>,
) -> Result<()> {
    if let Some(rest) = sec.strip_prefix("display_template ") {
        // Parse params (param_*) and text
        let mut params = HashMap::new();
        let mut text = String::new();
        for (k, v) in &map {
            if k.starts_with("param_") {
                params.insert(k[6..].to_string(), v.clone());
            } else if k == "text" {
                text = v.clone();
            }
        }
        templates.insert(rest.to_string(), DisplayTemplate {
            name: rest.to_string(),
            params,
            text,
        });
    } else if let Some(rest) = sec.strip_prefix("display_data ") {
        // Section name: group item
        let mut parts = rest.split_whitespace();
        let group = parts.next().unwrap_or("").to_string();
        let item = parts.next().unwrap_or("").to_string();
        let position = map.get("position")
            .and_then(|s| {
                let mut it = s.split(',').map(|v| v.trim().parse::<u32>());
                Some((it.next()??, it.next()??))
            })
            .unwrap_or((0, 0));
        let text = map.get("text").cloned().unwrap_or_default();
        data.entry(group.clone()).or_default().push(DisplayData {
            group,
            item,
            position,
            text,
        });
    } else if let Some(rest) = sec.strip_prefix("display_glyph ") {
        let data_lines = map.get("data").map(|d| d.lines().map(|l| l.trim().to_string()).collect());
        let hd44780_data = map.get("hd44780_data").map(|d| d.lines().map(|l| l.trim().to_string()).collect());
        let hd44780_slot = map.get("hd44780_slot").and_then(|s| s.parse().ok());
        glyphs.insert(rest.to_string(), DisplayGlyph {
            name: rest.to_string(),
            data: data_lines,
            hd44780_slot,
            hd44780_data,
        });
    }
    Ok(())
}

/// Default and minimum redraw intervals (seconds)
const REDRAW_TIME: f32 = 0.500;
const REDRAW_MIN_TIME: f32 = 0.100;

/// State for the display, including group switching and redraw logic.
pub struct DisplayState {
    pub current_group: String,
    pub redraw_request_pending: bool,
    pub redraw_time: Instant,
    pub last_update: Instant,
    pub display_groups: Arc<HashMap<String, DisplayGroup>>,
    pub template_engine: Arc<DisplayTemplateEngine>,
}

impl DisplayState {
    pub fn new(
        default_group: &str,
        display_groups: Arc<HashMap<String, DisplayGroup>>,
        template_engine: Arc<DisplayTemplateEngine>,
    ) -> Self {
        let now = Instant::now();
        Self {
            current_group: default_group.to_string(),
            redraw_request_pending: false,
            redraw_time: now,
            last_update: now,
            display_groups,
            template_engine,
        }
    }

    /// Request a redraw at the next minimum interval.
    pub fn request_redraw(&mut self) {
        if !self.redraw_request_pending {
            self.redraw_request_pending = true;
            self.redraw_time = Instant::now() + Duration::from_secs_f32(REDRAW_MIN_TIME);
        }
    }

    /// Switch the active display group.
    pub fn set_group(&mut self, group: &str) -> Result<()> {
        if self.display_groups.contains_key(group) {
            self.current_group = group.to_string();
            self.request_redraw();
            Ok(())
        } else {
            anyhow::bail!("Unknown display_data group '{}'.", group)
        }
    }

    /// Async display update loop. Calls the provided callback to actually draw.
    pub async fn run_update_loop<F>(&mut self, mut draw_cb: F)
    where
        F: FnMut(&DisplayGroup, &DisplayTemplateEngine) -> Result<()> + Send + 'static,
    {
        let mut interval = time::interval(Duration::from_secs_f32(REDRAW_TIME));
        loop {
            interval.tick().await;
            let now = Instant::now();
            if self.redraw_request_pending && now >= self.redraw_time {
                self.redraw_request_pending = false;
            }
            if let Some(group) = self.display_groups.get(&self.current_group) {
                let _ = draw_cb(group, &self.template_engine);
            }
            self.last_update = now;
        }
    }
}

/// Represents a [display_glyph ...] section in display.cfg.
///
/// Provides methods to parse and validate glyph data for both 16x16 and 5x8 formats.
///
/// # Example
/// ```rust
/// let glyph = cfg.glyphs.get("extruder").unwrap();
/// let icon16 = glyph.parse_icon16x16().unwrap().unwrap();
/// assert_eq!(icon16.len(), 16);
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayGlyph {
    pub name: String,
    pub data: Option<Vec<String>>, // original lines
    pub hd44780_slot: Option<u8>,
    pub hd44780_data: Option<Vec<String>>,
}

impl DisplayGlyph {
    /// Parse the dot/star 16x16 glyph data into a vector of u16 rows.
    pub fn parse_icon16x16(&self) -> Result<Option<Vec<u16>>> {
        if let Some(lines) = &self.data {
            let mut out = Vec::new();
            for line in lines {
                let line = line.trim().replace('.', "0").replace('*', "1");
                if line.is_empty() { continue; }
                if line.len() != 16 || line.chars().any(|c| c != '0' && c != '1') {
                    anyhow::bail!("Invalid glyph line in {}: {}", self.name, line);
                }
                let val = u16::from_str_radix(&line, 2)?;
                out.push(val);
            }
            if out.len() != 16 {
                anyhow::bail!("Glyph {} has {} lines, expected 16", self.name, out.len());
            }
            Ok(Some(out))
        } else {
            Ok(None)
        }
    }

    /// Parse the hd44780 5x8 glyph data into a vector of u8 rows.
    pub fn parse_icon5x8(&self) -> Result<Option<Vec<u8>>> {
        if let Some(lines) = &self.hd44780_data {
            let mut out = Vec::new();
            for line in lines {
                let line = line.trim().replace('.', "0").replace('*', "1");
                if line.is_empty() { continue; }
                if line.len() != 5 || line.chars().any(|c| c != '0' && c != '1') {
                    anyhow::bail!("Invalid hd44780 glyph line in {}: {}", self.name, line);
                }
                let val = u8::from_str_radix(&line, 2)?;
                out.push(val);
            }
            if out.len() != 8 {
                anyhow::bail!("hd44780 glyph {} has {} lines, expected 8", self.name, out.len());
            }
            Ok(Some(out))
        } else {
            Ok(None)
        }
    }
}

/// The full parsed display config
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayConfig {
    pub templates: HashMap<String, DisplayTemplate>,
    pub data: HashMap<String, Vec<DisplayData>>, // group -> items
    pub glyphs: HashMap<String, DisplayGlyph>,
}

/// A collection of display groups, each containing sorted DisplayData items.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayGroups {
    pub groups: HashMap<String, Vec<DisplayData>>, // group -> sorted items
}

impl DisplayGroups {
    /// Build DisplayGroups from the data field of DisplayConfig.
    pub fn from_data(data: &HashMap<String, Vec<DisplayData>>) -> Self {
        let mut groups = HashMap::new();
        for (group, items) in data {
            let mut sorted = items.clone();
            // Sort by (row, col) position, then by item name
            sorted.sort_by(|a, b| {
                a.position.cmp(&b.position).then(a.item.cmp(&b.item))
            });
            groups.insert(group.clone(), sorted);
        }
        Self { groups }
    }

    /// Get all group names.
    pub fn group_names(&self) -> impl Iterator<Item = &String> {
        self.groups.keys()
    }

    /// Get all items for a group, sorted by position.
    pub fn items_for_group(&self, group: &str) -> Option<&[DisplayData]> {
        self.groups.get(group).map(|v| v.as_slice())
    }

    /// Get a specific item by group and item name.
    pub fn get_item(&self, group: &str, item: &str) -> Option<&DisplayData> {
        self.groups.get(group)?.iter().find(|d| d.item == item)
    }
}

/// Loads and parses a display.cfg file into a strongly-typed DisplayConfig.
///
/// # Example
/// ```rust
/// let cfg = load_display_config("klippy/extras/display/display.cfg")?;
/// let groups = DisplayGroups::from_data(&cfg.data);
/// for group in groups.group_names() {
///     println!("Group: {}", group);
///     for item in groups.items_for_group(group).unwrap() {
///         println!("  Item: {} at {:?}", item.item, item.position);
///     }
/// }
/// ```
pub fn load_display_config<P: AsRef<Path>>(path: P) -> Result<DisplayConfig> {
    let content = fs::read_to_string(&path)
        .with_context(|| format!("Failed to read display config file: {}", path.as_ref().display()))?;
    parse_display_cfg(&content)
}

fn parse_display_cfg(cfg: &str) -> Result<DisplayConfig> {
    let mut templates = HashMap::new();
    let mut data: HashMap<String, Vec<DisplayData>> = HashMap::new();
    let mut glyphs = HashMap::new();

    let mut section: Option<(String, HashMap<String, String>)> = None;
    let mut multiline_key: Option<String> = None;
    let mut multiline_val = String::new();

    for line in cfg.lines() {
        let line = line.trim_end();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if line.starts_with('[') && line.ends_with(']') {
            // Save previous section
            if let Some((sec, mut map)) = section.take() {
                if let Some(key) = multiline_key.take() {
                    map.insert(key, multiline_val.trim_end().to_string());
                    multiline_val.clear();
                }
                parse_section(&sec, map, &mut templates, &mut data, &mut glyphs)?;
            }
            section = Some((line[1..line.len()-1].to_string(), HashMap::new()));
        } else if let Some((_, ref mut map)) = section {
            if let Some(key) = &multiline_key {
                // Continue multiline
                if line.starts_with(' ') || line.starts_with('\t') || line.is_empty() {
                    multiline_val.push_str(line.trim_start());
                    multiline_val.push('\n');
                    continue;
                } else {
                    // End multiline
                    map.insert(key.clone(), multiline_val.trim_end().to_string());
                    multiline_key = None;
                    multiline_val.clear();
                }
            }
            // Key: value or Key:
            if let Some(idx) = line.find(':') {
                let key = line[..idx].trim().to_string();
                let val = line[idx+1..].trim();
                if val.is_empty() {
                    // Start multiline
                    multiline_key = Some(key);
                    multiline_val.clear();
                } else {
                    map.insert(key, val.to_string());
                }
            }
        }
    }
    // Save last section
    if let Some((sec, mut map)) = section.take() {
        if let Some(key) = multiline_key.take() {
            map.insert(key, multiline_val.trim_end().to_string());
        }
        parse_section(&sec, map, &mut templates, &mut data, &mut glyphs)?;
    }

    Ok(DisplayConfig { templates, data, glyphs })
}

fn parse_section(
    sec: &str,
    map: HashMap<String, String>,
    templates: &mut HashMap<String, DisplayTemplate>,
    data: &mut HashMap<String, Vec<DisplayData>>,
    glyphs: &mut HashMap<String, DisplayGlyph>,
) -> Result<()> {
    if let Some(rest) = sec.strip_prefix("display_template ") {
        // Parse params (param_*) and text
        let mut params = HashMap::new();
        let mut text = String::new();
        for (k, v) in &map {
            if k.starts_with("param_") {
                params.insert(k[6..].to_string(), v.clone());
            } else if k == "text" {
                text = v.clone();
            }
        }
        templates.insert(rest.to_string(), DisplayTemplate {
            name: rest.to_string(),
            params,
            text,
        });
    } else if let Some(rest) = sec.strip_prefix("display_data ") {
        // Section name: group item
        let mut parts = rest.split_whitespace();
        let group = parts.next().unwrap_or("").to_string();
        let item = parts.next().unwrap_or("").to_string();
        let position = map.get("position")
            .and_then(|s| {
                let mut it = s.split(',').map(|v| v.trim().parse::<u32>());
                Some((it.next()??, it.next()??))
            })
            .unwrap_or((0, 0));
        let text = map.get("text").cloned().unwrap_or_default();
        data.entry(group.clone()).or_default().push(DisplayData {
            group,
            item,
            position,
            text,
        });
    } else if let Some(rest) = sec.strip_prefix("display_glyph ") {
        let data_lines = map.get("data").map(|d| d.lines().map(|l| l.trim().to_string()).collect());
        let hd44780_data = map.get("hd44780_data").map(|d| d.lines().map(|l| l.trim().to_string()).collect());
        let hd44780_slot = map.get("hd44780_slot").and_then(|s| s.parse().ok());
        glyphs.insert(rest.to_string(), DisplayGlyph {
            name: rest.to_string(),
            data: data_lines,
            hd44780_slot,
            hd44780_data,
        });
    }
    Ok(())
}

/// Default and minimum redraw intervals (seconds)
const REDRAW_TIME: f32 = 0.500;
const REDRAW_MIN_TIME: f32 = 0.100;

/// State for the display, including group switching and redraw logic.
pub struct DisplayState {
    pub current_group: String,
    pub redraw_request_pending: bool,
    pub redraw_time: Instant,
    pub last_update: Instant,
    pub display_groups: Arc<HashMap<String, DisplayGroup>>,
    pub template_engine: Arc<DisplayTemplateEngine>,
}

impl DisplayState {
    pub fn new(
        default_group: &str,
        display_groups: Arc<HashMap<String, DisplayGroup>>,
        template_engine: Arc<DisplayTemplateEngine>,
    ) -> Self {
        let now = Instant::now();
        Self {
            current_group: default_group.to_string(),
            redraw_request_pending: false,
            redraw_time: now,
            last_update: now,
            display_groups,
            template_engine,
        }
    }

    /// Request a redraw at the next minimum interval.
    pub fn request_redraw(&mut self) {
        if !self.redraw_request_pending {
            self.redraw_request_pending = true;
            self.redraw_time = Instant::now() + Duration::from_secs_f32(REDRAW_MIN_TIME);
        }
    }

    /// Switch the active display group.
    pub fn set_group(&mut self, group: &str) -> Result<()> {
        if self.display_groups.contains_key(group) {
            self.current_group = group.to_string();
            self.request_redraw();
            Ok(())
        } else {
            anyhow::bail!("Unknown display_data group '{}'.", group)
        }
    }

    /// Async display update loop. Calls the provided callback to actually draw.
    pub async fn run_update_loop<F>(&mut self, mut draw_cb: F)
    where
        F: FnMut(&DisplayGroup, &DisplayTemplateEngine) -> Result<()> + Send + 'static,
    {
        let mut interval = time::interval(Duration::from_secs_f32(REDRAW_TIME));
        loop {
            interval.tick().await;
            let now = Instant::now();
            if self.redraw_request_pending && now >= self.redraw_time {
                self.redraw_request_pending = false;
            }
            if let Some(group) = self.display_groups.get(&self.current_group) {
                let _ = draw_cb(group, &self.template_engine);
            }
            self.last_update = now;
        }
    }
}
