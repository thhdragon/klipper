# Low level unix utility functions
#
# Copyright (C) 2016-2023  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, pty, fcntl, termios, signal, logging, json, time
import subprocess, traceback, shlex

# Attempt to import Rust implementations
klipper_rust = None
try:
    import klipper_rust
except ImportError as e:
    logging.debug("Unable to load klipper_rust module: %s", e)


######################################################################
# Low-level Unix commands
######################################################################

# Return the SIGINT interrupt handler back to the OS default
def _fix_sigint_python():
    signal.signal(signal.SIGINT, signal.SIG_DFL)

def fix_sigint():
    if klipper_rust:
        try:
            return klipper_rust.fix_sigint_rs()
        except Exception as e:
            logging.exception("Error calling klipper_rust.fix_sigint_rs (falling back to Python): %s", e)
    return _fix_sigint_python()

fix_sigint() # Initial call

# Set a file-descriptor as non-blocking
def _set_nonblock_python(fd):
    fcntl.fcntl(fd, fcntl.F_SETFL
                , fcntl.fcntl(fd, fcntl.F_GETFL) | os.O_NONBLOCK)

def set_nonblock(fd):
    if klipper_rust:
        try:
            return klipper_rust.set_nonblock_rs(fd)
        except Exception as e:
            logging.exception("Error calling klipper_rust.set_nonblock_rs (falling back to Python): %s", e)
    return _set_nonblock_python(fd)

# Clear HUPCL flag
def _clear_hupcl_python(fd):
    if not os.isatty(fd): # Match Rust behavior: only proceed if it's a TTY
        return
    try:
        attrs = termios.tcgetattr(fd)
        attrs[2] = attrs[2] & ~termios.HUPCL # c_cflag index is 2
        termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
    except termios.error:
        pass # Ignore error, as is done in Rust version and original Python

def clear_hupcl(fd):
    if klipper_rust:
        try:
            return klipper_rust.clear_hupcl_rs(fd)
        except Exception as e:
            logging.exception("Error calling klipper_rust.clear_hupcl_rs (falling back to Python): %s", e)
    return _clear_hupcl_python(fd)


# Support for creating a pseudo-tty for emulating a serial port
def _create_pty_python(ptyname):
    mfd, sfd = pty.openpty()
    try:
        os.unlink(ptyname)
    except OSError: # Changed from os.error for Python 3 compatibility
        pass
    filename = os.ttyname(sfd)
    os.chmod(filename, 0o660) # Python 3 prefers 0o prefix for octal
    os.symlink(filename, ptyname)
    set_nonblock(mfd) # Uses the potentially Rust-backed version
    if os.isatty(mfd): # Match Rust behavior
        old = termios.tcgetattr(mfd)
        old[3] = old[3] & ~termios.ECHO # c_lflag index is 3
        termios.tcsetattr(mfd, termios.TCSADRAIN, old)
    return mfd

def create_pty(ptyname):
    if klipper_rust:
        try:
            return klipper_rust.create_pty_rs(ptyname)
        except Exception as e:
            logging.exception("Error calling klipper_rust.create_pty_rs (falling back to Python): %s", e)
    return _create_pty_python(ptyname)


######################################################################
# Helper code for extracting mcu build info
######################################################################

def _dump_file_stats_python(build_dir, filename):
    # This helper is internal to _dump_mcu_build_python, so no Rust fallback here.
    fname = os.path.join(build_dir, filename)
    try:
        mtime = os.path.getmtime(fname)
        fsize = os.path.getsize(fname)
        timestr = time.asctime(time.localtime(mtime))
        logging.info("Build file %s(%d): %s (Python)", fname, fsize, timestr)
    except:
        logging.info("No build file %s (Python)", fname)

def _dump_mcu_build_python():
    build_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
    logging.info("Using build directory: %s (Python)", build_dir)
    # Try to log last mcu config
    _dump_file_stats_python(build_dir, '.config')
    try:
        with open(os.path.join(build_dir, '.config'), 'r') as f:
            data = f.read(32*1024)
        logging.info("========= Last MCU build config (Python) =========\n%s"
                     "=======================", data)
    except:
        pass
    # Try to log last mcu build version
    _dump_file_stats_python(build_dir, 'out/klipper.dict')
    try:
        with open(os.path.join(build_dir, 'out/klipper.dict'), 'r') as f:
            data = f.read(32*1024)
        parsed_data = json.loads(data)
        logging.info("Last MCU build version (Python): %s", parsed_data.get('version', ''))
        logging.info("Last MCU build tools (Python): %s", parsed_data.get('build_versions', ''))
        cparts = ["%s=%s" % (k, v) for k, v in parsed_data.get('config', {}).items()]
        logging.info("Last MCU build config (Python): %s", " ".join(cparts))
    except:
        pass
    _dump_file_stats_python(build_dir, 'out/klipper.elf')

def dump_mcu_build():
    if klipper_rust:
        try:
            return klipper_rust.dump_mcu_build_rs()
        except Exception as e:
            logging.exception("Error calling klipper_rust.dump_mcu_build_rs (falling back to Python): %s", e)
    return _dump_mcu_build_python()


######################################################################
# Python2 wrapper hacks
######################################################################

def setup_python2_wrappers():
    if sys.version_info.major >= 3:
        return
    # Add module hacks so that common Python3 module imports work in Python2
    import ConfigParser, Queue, io, StringIO, time # type: ignore
    sys.modules["configparser"] = ConfigParser      # type: ignore
    sys.modules["queue"] = Queue                    # type: ignore
    io.StringIO = StringIO.StringIO                 # type: ignore
    time.process_time = time.clock                  # type: ignore
setup_python2_wrappers()


######################################################################
# General system and software information
######################################################################

def _get_cpu_info_python():
    try:
        with open('/proc/cpuinfo', 'r') as f:
            data = f.read()
    except (IOError, OSError): # type: ignore
        logging.debug("Exception on read /proc/cpuinfo (Python): %s",
                      traceback.format_exc())
        return "?"
    lines = [l.split(':', 1) for l in data.split('\n')]
    lines = [(l[0].strip(), l[1].strip()) for l in lines if len(l) == 2]
    core_count = [k for k, v in lines].count("processor")
    model_name = dict(lines).get("model name", "?")
    return "%d core %s" % (core_count, model_name)

def get_cpu_info():
    if klipper_rust:
        try:
            return klipper_rust.get_cpu_info_rs()
        except Exception as e:
            logging.exception("Error calling klipper_rust.get_cpu_info_rs (falling back to Python): %s", e)
    return _get_cpu_info_python()


def get_version_from_file(klippy_src_dir): # Renamed for clarity
    # This is a helper for _get_git_version_python, no Rust fallback needed directly.
    try:
        with open(os.path.join(klippy_src_dir, '.version')) as h:
            return h.read().rstrip()
    except IOError: # type: ignore
        pass
    return "?"

def _get_repo_info_python(gitdir): # Renamed for clarity and consistency
    # This is a helper for _get_git_version_python, no Rust fallback needed directly.
    repo_info = {"branch": "?", "remote": "?", "url": "?"}
    prog_branch = ('git', '-C', gitdir, 'branch', '--no-color')
    try:
        process = subprocess.Popen(prog_branch, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        branch_list_bytes, err_bytes = process.communicate()
        retcode = process.wait()
        branch_list = branch_list_bytes.decode(errors='ignore')
        err = err_bytes.decode(errors='ignore')
        if retcode != 0:
            logging.debug("Error running git branch (Python): %s", err)
            return repo_info
        lines = str(branch_list.strip()).split("\n")
        for line in lines:
            if line.startswith("*"):
                repo_info["branch"] = line[1:].strip()
                break
        else:
            logging.debug("Unable to find current branch (Python):\n%s", branch_list)
            return repo_info

        if repo_info["branch"].startswith("(HEAD detached"):
            # Try to get remote from detached head string e.g. (HEAD detached at origin/my-branch)
            parts = repo_info["branch"].strip("()").split()
            if len(parts) >= 4 and "/" in parts[-1] : # Should be 'HEAD detached at remote/branch'
                 repo_info["remote"] = parts[-1].split("/",1)[0]
            # If not, remote remains '?' - Rust version also has logic for this.
        else:
            key = "branch.%s.remote" % (repo_info["branch"],)
            prog_config = ('git', '-C', gitdir, 'config', '--get', key)
            process = subprocess.Popen(prog_config, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE)
            remote_info_bytes, err_bytes = process.communicate()
            retcode = process.wait()
            remote_info = remote_info_bytes.decode(errors='ignore')
            err = err_bytes.decode(errors='ignore')
            if retcode == 0:
                repo_info["remote"] = str(remote_info.strip())
            else:
                logging.debug("Error running git config for %s (Python): %s", key, err)
                # Fallback: try to get first remote if specific one fails (like Rust version)
                prog_remotes = ('git', '-C', gitdir, 'remote')
                process_remotes = subprocess.Popen(prog_remotes, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                remotes_out_bytes, _ = process_remotes.communicate()
                if process_remotes.wait() == 0:
                    first_remote = remotes_out_bytes.decode(errors='ignore').split('\n')[0].strip()
                    if first_remote:
                        repo_info["remote"] = first_remote

        if repo_info["remote"] != "?":
            prog_remote_url = (
                'git', '-C', gitdir, 'remote', 'get-url', repo_info["remote"])
            process = subprocess.Popen(prog_remote_url, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE)
            remote_url_bytes, err_bytes = process.communicate()
            retcode = process.wait()
            remote_url = remote_url_bytes.decode(errors='ignore')
            err = err_bytes.decode(errors='ignore')
            if retcode == 0:
                repo_info["url"] = str(remote_url.strip())
            else:
                logging.debug("Error running git remote get-url for %s (Python): %s", repo_info["remote"], err)
    except Exception:
        logging.debug("Error fetching repo info (Python): %s", traceback.format_exc())
    return repo_info

def _get_git_version_python(from_file=True):
    git_info = {
        "version": "?",
        "file_status": [], # list of tuples (status_char, filename)
        "branch": "?",
        "remote": "?",
        "url": "?"
    }
    klippy_src_dir = os.path.dirname(os.path.abspath(__file__))
    gitdir = os.path.join(klippy_src_dir, '..')

    prog_desc = ('git', '-C', gitdir, 'describe', '--always',
                 '--tags', '--long', '--dirty')
    prog_status = ('git', '-C', gitdir, 'status', '--porcelain', '--ignored')
    try:
        process = subprocess.Popen(prog_desc, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        ver_bytes, err_bytes = process.communicate()
        retcode = process.wait()
        ver = ver_bytes.decode(errors='ignore')
        err = err_bytes.decode(errors='ignore')

        if retcode == 0:
            git_info["version"] = str(ver.strip())
            process = subprocess.Popen(prog_status, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE)
            stat_bytes, err_stat_bytes = process.communicate()
            retcode_stat = process.wait()
            stat = stat_bytes.decode(errors='ignore')
            err_stat = err_stat_bytes.decode(errors='ignore')

            if retcode_stat == 0:
                status_lines = str(stat.strip()).split('\n')
                parsed_status = []
                for line in status_lines:
                    if not line.strip(): continue # Skip empty lines
                    # Split status and filename, handling cases like "?? file with spaces"
                    status_char = line[:2].strip() # Status is usually 1 or 2 chars e.g., " M", "??", "A "
                    filename = line[3:] # Filename starts after the status and a space
                    parsed_status.append((status_char, filename))
                git_info["file_status"] = parsed_status
            else:
                logging.debug("Error getting git status (Python): %s", err_stat)
            git_info.update(_get_repo_info_python(gitdir)) # Use the renamed helper
            return git_info
        else:
            logging.debug("Error getting git version (git describe) (Python): %s", err)
    except Exception:
        logging.debug("Exception on run git (Python): %s", traceback.format_exc())

    if from_file:
        git_info["version"] = get_version_from_file(klippy_src_dir)
    return git_info

def get_git_version(from_file=True):
    if klipper_rust:
        try:
            # Rust function returns (version, file_status_list, branch, remote, url)
            # file_status_list is already a list of tuples from Rust
            version, file_status, branch, remote, url = klipper_rust.get_git_version_rs(from_file)
            return {
                "version": version,
                "file_status": file_status,
                "branch": branch,
                "remote": remote,
                "url": url
            }
        except Exception as e:
            logging.exception("Error calling klipper_rust.get_git_version_rs (falling back to Python): %s", e)
    return _get_git_version_python(from_file)
