extern crate cbindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let package_name = env::var("CARGO_PKG_NAME").unwrap();
    let output_file = PathBuf::from(&crate_dir)
        .join(format!("{}.h", package_name).replace("-", "_")); // Replace hyphens with underscores for C compatibility

    // Configure cbindgen
    let config = cbindgen::Config {
        // Set an include guard, e.g., based on the package name
        include_guard: Some(format!("{}_H", package_name.to_uppercase().replace("-", "_"))),
        language: cbindgen::Language::C,
        // Add more configurations as needed, e.g., for comments, style, etc.
        // For example, to include documentation comments as C comments:
        // parse: cbindgen::ParseConfig {
        //     parse_deps: true,
        //     include: Some(vec![package_name.clone()]),
        //     ..Default::default()
        // },
        // export: cbindgen::ExportConfig {
        //     include: vec![String::from("crc16_ccitt")], // Only export specified items
        //     ..Default::default()
        // },
        ..Default::default()
    };

    match cbindgen::generate_with_config(&crate_dir, config) {
        Ok(bindings) => {
            bindings.write_to_file(&output_file);
            println!("Generated C header: {}", output_file.display());
        }
        Err(err) => {
            eprintln!("Failed to generate C bindings: {}", err);
            std::process::exit(1);
        }
    }
}
