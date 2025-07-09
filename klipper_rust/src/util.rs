use std::fs;
use std::process::Command;
// use std::path::Path; // Unused

#[cfg(target_os = "linux")]
pub fn get_cpu_info() -> String {
    let cpuinfo = match fs::read_to_string("/proc/cpuinfo") {
        Ok(info) => info,
        Err(_) => return "?".to_string(),
    };

    let mut core_count = 0;
    let mut model_name = "?".to_string();

    for line in cpuinfo.lines() {
        if line.starts_with("processor") {
            core_count += 1;
        }
        if line.starts_with("model name") {
            if let Some(name) = line.split(':').nth(1) {
                model_name = name.trim().to_string();
            }
        }
    }

    format!("{} core {}", core_count, model_name)
}

#[cfg(not(target_os = "linux"))]
pub fn get_cpu_info() -> String {
    "?".to_string()
}

pub fn get_git_version(from_file: bool) -> Result<(String, Vec<(String, String)>, String, String, String), String> {
    let mut version = "?".to_string();
    let mut file_status: Vec<(String, String)> = Vec::new();
    let branch = "?".to_string();
    let remote = "?".to_string();
    let url = "?".to_string();

    // Assuming the klipper_rust directory is a sibling of the klippy directory
    let base_path = std::env::current_dir().unwrap_or_default();
    let klippy_src_path = base_path.parent().unwrap_or(&base_path).join("klippy");
    let gitdir = klippy_src_path.parent().unwrap_or(&klippy_src_path);


    // Obtain version info from "git" program
    let output = Command::new("git")
        .args(["-C", gitdir.to_str().unwrap_or("."), "describe", "--always", "--tags", "--long", "--dirty"])
        .output();

    match output {
        Ok(output) => {
            if output.status.success() {
                version = String::from_utf8_lossy(&output.stdout).trim().to_string();

                let status_output = Command::new("git")
                    .args(["-C", gitdir.to_str().unwrap_or(""), "status", "--porcelain", "--ignored"])
                    .output();

                if let Ok(status_output) = status_output {
                    if status_output.status.success() {
                        let status_str = String::from_utf8_lossy(&status_output.stdout);
                        file_status = status_str
                            .lines()
                            .filter_map(|line| {
                                let parts: Vec<&str> = line.trim().splitn(2, ' ').collect();
                                if parts.len() == 2 {
                                    Some((parts[0].to_string(), parts[1].to_string()))
                                } else {
                                    None
                                }
                            })
                            .collect();
                    }
                }
                // TODO: Implement _get_repo_info equivalent in Rust
            } else {
                return Err(String::from_utf8_lossy(&output.stderr).trim().to_string());
            }
        }
        Err(e) => return Err(e.to_string()),
    }


    if from_file && version == "?" {
        if let Ok(contents) = fs::read_to_string(klippy_src_path.join(".version")) {
            version = contents.trim().to_string();
        }
    }

    Ok((version, file_status, branch, remote, url))
}

// Placeholder for other functions from util.py
// pub fn fix_sigint() {}
// pub fn set_nonblock(fd: i32) {}
// pub fn clear_hupcl(fd: i32) {}
// pub fn create_pty(ptyname: &str) -> i32 { 0 }
// pub fn dump_mcu_build() {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_cpu_info() {
        let cpu_info = get_cpu_info();
        // Basic check, as the exact output depends on the system
        assert!(cpu_info.contains("core"));
    }

    #[test]
    fn test_get_git_version() {
        let git_installed = Command::new("git").arg("--version").output().is_ok();
        // Adjust path for test environment. current_dir is klipper_rust, so .git is in ../
        let in_git_repo = Path::new("../.git").exists();

        // Test with from_file = true
        let result_true = get_git_version(true);
        if git_installed && in_git_repo {
            assert!(result_true.is_ok(), "Expected Ok, got Err: {:?}", result_true.err());
            assert_ne!(result_true.as_ref().unwrap().0, "?", "Version should not be '?'");
        } else if git_installed && !in_git_repo {
            // If git is installed but not in a repo, it should error or return "?" if from_file is true and .version exists
            if result_true.is_err() {
                assert!(result_true.unwrap_err().contains("not a git repository"));
            } else {
                assert_eq!(result_true.as_ref().unwrap().0, "?")
            }
        }
        else {
            // If git is not installed, it should error or return "?" if from_file is true and .version exists
             if result_true.is_err() {
                assert!(result_true.unwrap_err().contains("No such file or directory"));
            } else {
                assert_eq!(result_true.as_ref().unwrap().0, "?")
            }
        }

        // Test with from_file = false
        let result_false = get_git_version(false);
        if git_installed && in_git_repo {
            assert!(result_false.is_ok(), "Expected Ok, got Err: {:?}", result_false.err());
            assert_ne!(result_false.as_ref().unwrap().0, "?", "Version should not be '?'");
        } else {
            assert!(result_false.is_err(), "Expected Err, got Ok: {:?}", result_false.ok());
        }
    }
}
