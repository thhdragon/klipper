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
                // Call _get_repo_info equivalent
                match get_repo_info_internal(gitdir.to_str().unwrap_or(".")) {
                    Ok(repo_info_tuple) => {
                        // version, file_status are already set
                        // branch, remote, url need to be updated from repo_info_tuple
                        return Ok((version, file_status, repo_info_tuple.0, repo_info_tuple.1, repo_info_tuple.2));
                    }
                    Err(e) => {
                        // Log error like Python version, but continue
                        eprintln!("Error getting repo info: {}", e);
                        // Fall through to return with default branch/remote/url if from_file is false or .version doesn't exist
                    }
                }
            } else {
                // If git describe fails, try to read from file if allowed
                if from_file {
                    if let Ok(contents) = fs::read_to_string(klippy_src_path.join(".version")) {
                        version = contents.trim().to_string();
                        // Still return default for other git specific parts
                        return Ok((version, file_status, branch, remote, url));
                    }
                }
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

fn get_repo_info_internal(gitdir_str: &str) -> Result<(String, String, String), String> {
    let mut repo_branch = "?".to_string();
    let mut repo_remote = "?".to_string();
    let mut repo_url = "?".to_string();

    // Get current branch
    let branch_output = Command::new("git")
        .args(["-C", gitdir_str, "branch", "--no-color"])
        .output();

    match branch_output {
        Ok(output) => {
            if output.status.success() {
                let branch_list = String::from_utf8_lossy(&output.stdout);
                for line in branch_list.lines() {
                    if line.starts_with('*') {
                        repo_branch = line[1..].trim().to_string();
                        break;
                    }
                }
            } else {
                return Err(format!("git branch error: {}", String::from_utf8_lossy(&output.stderr)));
            }
        }
        Err(e) => return Err(format!("Failed to execute git branch: {}", e)),
    }

    if repo_branch == "?" {
        return Ok((repo_branch, repo_remote, repo_url)); // Early exit if branch not found
    }

    // Determine remote name
    if repo_branch.starts_with("(HEAD detached") {
        // Format is typically "(HEAD detached at origin/some-branch)" or "(HEAD detached at commit-hash)"
        let parts: Vec<&str> = repo_branch.split_whitespace().collect();
        if parts.len() >= 4 && parts[3].contains('/') { // detached at remote/branch
            repo_remote = parts[3].split('/').next().unwrap_or("?").to_string();
        } else {
            // Detached at a commit, remote might not be directly inferable this way.
            // Python version seems to try and get it later anyway.
            // For now, we'll leave it as "?" and let the next step try to find it.
            // A more robust way might be `git rev-parse --abbrev-ref HEAD@{upstream}`
            // but that assumes an upstream is set.
        }
    } else {
        let remote_key = format!("branch.{}.remote", repo_branch);
        let config_output = Command::new("git")
            .args(["-C", gitdir_str, "config", "--get", &remote_key])
            .output();

        match config_output {
            Ok(output) => {
                if output.status.success() {
                    repo_remote = String::from_utf8_lossy(&output.stdout).trim().to_string();
                } else {
                     eprintln!("git config get {} error: {}", remote_key, String::from_utf8_lossy(&output.stderr));
                    // Python version continues, so we do too.
                }
            }
            Err(e) => eprintln!("Failed to execute git config get {}: {}", remote_key, e),
        }
    }

    if repo_remote == "?" {
         // Attempt to find a remote if still unknown, e.g. for detached heads or if above failed
        let remote_names_output = Command::new("git")
            .args(["-C", gitdir_str, "remote"])
            .output();
        if let Ok(output) = remote_names_output {
            if output.status.success() {
                let remotes_str = String::from_utf8_lossy(&output.stdout);
                if let Some(first_remote) = remotes_str.lines().next() {
                    repo_remote = first_remote.trim().to_string();
                }
            }
        }
    }


    if repo_remote != "?" {
        // Get remote URL
        let url_output = Command::new("git")
            .args(["-C", gitdir_str, "remote", "get-url", &repo_remote])
            .output();

        match url_output {
            Ok(output) => {
                if output.status.success() {
                    repo_url = String::from_utf8_lossy(&output.stdout).trim().to_string();
                } else {
                    eprintln!("git remote get-url {} error: {}", repo_remote, String::from_utf8_lossy(&output.stderr));
                }
            }
            Err(e) => eprintln!("Failed to execute git remote get-url {}: {}", repo_remote, e),
        }
    }

    Ok((repo_branch, repo_remote, repo_url))
}


// Python2 wrappers are not needed in Rust.

#[cfg(test)]
mod tests {
    use super::*;
    // use std::path::Path; // No longer needed after removing PTY tests
    // tempfile might not be needed if fd_operations_smoke is removed or simplified further
    // use tempfile;

    #[test]
    fn test_get_cpu_info() {
        let cpu_info = get_cpu_info();
        // Basic check, as the exact output depends on the system
        assert!(cpu_info.contains("core") || cpu_info == "?");
    }

    #[test]
    fn test_get_git_version() {
        let git_installed = Command::new("git").arg("--version").output().is_ok();
        let mut project_root = std::env::current_dir().unwrap_or_default(); // klipper_rust
        if !project_root.ends_with("klipper_rust") { // If current_dir is not klipper_rust (e.g. root)
            project_root.push("klipper_rust");
        }
        project_root.pop(); // klipper (project root)

        let in_git_repo = project_root.join(".git").exists();
        println!("Test get_git_version: git_installed={}, in_git_repo={}, project_root={}", git_installed, in_git_repo, project_root.display());


        // Test with from_file = true
        let result_true = get_git_version(true);
        if git_installed && in_git_repo {
            assert!(result_true.is_ok(), "Expected Ok from get_git_version(true) in repo, got Err: {:?}", result_true.err());
            let (version, _, branch, remote, url) = result_true.as_ref().unwrap();
            assert_ne!(version, "?", "Version should not be '?' in repo");
            assert_ne!(branch, "?", "Branch should not be '?' in repo");
            println!("Git version (from_file=true, in repo): V={}, B={}, R={}, U={}", version, branch, remote, url);
        } else {
            // Not in a git repo or git not installed.
            // Create a dummy .version file to test the fallback.
            let klippy_dir = project_root.join("klippy");
            let dummy_version_path = klippy_dir.join(".version");
            let _ = fs::create_dir_all(&klippy_dir); // Ensure klippy directory exists
            let pre_existing_version = fs::read_to_string(&dummy_version_path).ok();
            let _ = fs::write(&dummy_version_path, "test-version-from-file");

            let result_fallback = get_git_version(true);
            assert!(result_fallback.is_ok(), "Expected Ok from get_git_version(true) with fallback, got Err: {:?}", result_fallback.err());
            assert_eq!(result_fallback.as_ref().unwrap().0, "test-version-from-file", "Version should be from file");

            // Clean up: remove dummy .version or restore original
            if let Some(original_content) = pre_existing_version {
                let _ = fs::write(&dummy_version_path, original_content);
            } else {
                let _ = fs::remove_file(&dummy_version_path);
            }
            // Attempt to remove klippy dir only if it was created by this test and is empty
            if fs::read_dir(&klippy_dir).map_or(false, |mut d| d.next().is_none()) {
                 let _ = fs::remove_dir(&klippy_dir);
            }
             println!("Git version (from_file=true, no repo/git): {:?}", result_fallback.as_ref().unwrap());
        }

        // Test with from_file = false
        let result_false = get_git_version(false);
        if git_installed && in_git_repo {
            assert!(result_false.is_ok(), "Expected Ok from get_git_version(false) in repo, got Err: {:?}", result_false.err());
            let (version, _, branch, _, _) = result_false.as_ref().unwrap();
            assert_ne!(version, "?", "Version should not be '?' in repo (from_file=false)");
            assert_ne!(branch, "?", "Branch should not be '?' in repo (from_file=false)");
        } else {
            // If not in a git repo or git not installed, and from_file is false, it should error.
            assert!(result_false.is_err(), "Expected Err from get_git_version(false) if not in repo or no git, got Ok: {:?}", result_false.ok());
        }
    }

    // The following smoke tests are removed as the functions themselves are removed.
    // #[test]
    // fn test_fix_sigint_smoke() {
    //     fix_sigint(); // Just ensure it runs without panic
    // }

    // #[test]
    // #[cfg(unix)] // These tests are Unix-specific
    // fn test_fd_operations_smoke() {
    //     use std::os::unix::io::AsRawFd;

    //     // Test set_nonblock
    //     let file_for_nonblock = tempfile::tempfile().expect("Failed to create tempfile for set_nonblock test");
    //     let fd_nonblock = file_for_nonblock.as_raw_fd();
    //     assert!(set_nonblock(fd_nonblock).is_ok(), "set_nonblock failed");

    //     // Test clear_hupcl
    //     let file_for_hupcl = tempfile::tempfile().expect("Failed to create tempfile for clear_hupcl non-TTY test");
    //     let fd_hupcl_nontty = file_for_hupcl.as_raw_fd();
    //     assert!(clear_hupcl(fd_hupcl_nontty).is_ok(), "clear_hupcl on non-TTY fd failed");

    //     let stdin_fd = std::io::stdin().as_raw_fd();
    //     if nix::unistd::isatty(stdin_fd).unwrap_or(false) {
    //         println!("Attempting clear_hupcl on stdin (TTY)");
    //         let borrowed_stdin = unsafe { BorrowedFd::borrow_raw(stdin_fd) };
    //         let original_termios = nix::sys::termios::tcgetattr(borrowed_stdin).ok();
    //         let _ = clear_hupcl(stdin_fd);
    //         if let Some(orig) = original_termios {
    //             let _ = nix::sys::termios::tcsetattr(borrowed_stdin, nix::sys::termios::SetArg::TCSADRAIN, &orig);
    //         }
    //     } else {
    //         println!("Skipping clear_hupcl TTY-specific part as stdin is not a TTY");
    //     }

    //     // Test create_pty
    //     let pty_symlink_name = format!("test_pty_symlink_{}", std::process::id());
    //     let pty_path = std::env::temp_dir().join(&pty_symlink_name);
    //     let ptyname_str = pty_path.to_str().expect("Failed to create temp pty name");

    //     match create_pty(ptyname_str) {
    //         Ok(master_fd) => {
    //             assert!(master_fd > 0, "Master FD from create_pty should be positive");
    //             assert!(Path::new(ptyname_str).exists(), "PTY symlink not created at {}", ptyname_str);
    //             assert!(Path::new(ptyname_str).is_symlink(), "PTY path {} is not a symlink", ptyname_str);
    //             let _ = nix::unistd::close(master_fd);
    //             let _ = std_fs::remove_file(ptyname_str);
    //         }
    //         Err(e) => {
    //             if e.contains("Operation not permitted") || e.contains("such file or directory") || e.contains("function not implemented") {
    //                 println!("Skipping PTY creation test due to environment restrictions: {}", e);
    //             } else {
    //                 panic!("create_pty failed: {}", e);
    //             }
    //         }
    //     }
    // }

    // #[test]
    // fn test_dump_mcu_build_smoke() {
    //     let mut build_dir_path = std::env::current_dir().unwrap_or_default();
    //     build_dir_path.pop();

    //     let _ = std_fs::create_dir_all(build_dir_path.join("out"));
    //     let config_path = build_dir_path.join(".config");
    //     let dict_path = build_dir_path.join("out/klipper.dict");
    //     let elf_path = build_dir_path.join("out/klipper.elf");

    //     let _ = std_fs::write(&config_path, "CONFIG_TEST=y\n");
    //     let _ = std_fs::write(&dict_path, r#"{"version": "test-123-smoke", "build_versions": "tools-v1-smoke", "config": {"CONFIG_SMOKE":"1"}}"#);
    //     let _ = std_fs::write(&elf_path, "dummy elf data for smoke test");
    //     dump_mcu_build();

    //     let _ = std_fs::remove_file(&config_path);
    //     let _ = std_fs::remove_file(&dict_path);
    //     let _ = std_fs::remove_file(&elf_path);
    //     if std_fs::read_dir(build_dir_path.join("out")).map_or(false, |mut d| d.next().is_none()) {
    //         let _ = std_fs::remove_dir(build_dir_path.join("out"));
    //     }
    // }
}
