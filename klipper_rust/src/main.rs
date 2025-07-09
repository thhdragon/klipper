// This file is now primarily for simple standalone testing of the util library.
// PyO3 bindings have been removed to align with the pure Rust endgoal.

mod util;

fn main() {
    println!("--- Klipper Rust Util - Standalone Test ---");

    println!("\nFetching CPU Info...");
    let cpu_info = util::get_cpu_info();
    println!("CPU Info: {}", cpu_info);

    println!("\nFetching Git Version (from file if possible)...");
    match util::get_git_version(true) {
        Ok((version, file_status, branch, remote, url)) => {
            println!("  Version: {}", version);
            println!("  Branch: {}", branch);
            println!("  Remote: {}", remote);
            println!("  URL: {}", url);
            if !file_status.is_empty() {
                println!("  File Status:");
                for (status, file) in file_status {
                    println!("    {} {}", status, file);
                }
            } else {
                println!("  File Status: Clean or no details");
            }
        }
        Err(e) => println!("Error getting git version: {}", e),
    }

    println!("\nFetching Git Version (forcing git commands, no file fallback)...");
    match util::get_git_version(false) {
         Ok((version, file_status, branch, remote, url)) => {
            println!("  Version: {}", version);
            println!("  Branch: {}", branch);
            println!("  Remote: {}", remote);
            println!("  URL: {}", url);
            if !file_status.is_empty() {
                println!("  File Status:");
                for (status, file) in file_status {
                    println!("    {} {}", status, file);
                }
            } else {
                println!("  File Status: Clean or no details");
            }
        }
        Err(e) => println!("Error getting git version (no file fallback): {}", e),
    }

    println!("\n--- Test Complete ---");
}
