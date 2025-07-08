import subprocess
import unittest
import os

class TestRustUtil(unittest.TestCase):
    def test_rust_binary_output(self):
        # Path to the compiled Rust binary
        # Assuming this test is run from the main klipper directory
        binary_name = "klipper_rust_bin"
        # On Windows, binaries have .exe extension
        if os.name == 'nt':
            binary_name += ".exe"

        binary_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "klipper_rust", "target", "release", binary_name))

        if not os.path.exists(binary_path):
            self.fail(f"Rust binary not found at {binary_path}. Make sure it's built.")

        try:
            result = subprocess.run([binary_path], capture_output=True, text=True, check=True)
            output = result.stdout.strip()

            self.assertIn("CPU Info:", output)
            self.assertIn("core", output) # From get_cpu_info

            # Check for git version or error
            if "Git Version:" in output:
                self.assertNotIn("?", output.split("Git Version:")[1].strip(), "Git version should not be '?' if present")
            elif "Git Version Error:" in output:
                self.assertTrue("not a git repository" in output or ".version" in output or "No such file or directory" in output)
            else:
                self.fail("Output did not contain expected Git version information.")

        except subprocess.CalledProcessError as e:
            self.fail(f"Running Rust binary failed: {e}\nStdout: {e.stdout}\nStderr: {e.stderr}")
        except FileNotFoundError:
            self.fail(f"Rust binary not found at {binary_path}. Make sure it's built and executable.")

if __name__ == '__main__':
    unittest.main()
