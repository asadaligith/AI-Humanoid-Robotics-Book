import argparse
import os
import subprocess
import sys
from typing import List

# Utility function to check if a command exists
def command_exists(cmd: str) -> bool:
    return subprocess.call(f"type {cmd}", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE) == 0

def validate_urdf(urdf_file_path: str, robot_name: str = "") -> bool:
    """
    Validates a URDF file using check_urdf and prints its model using rviz2.

    Args:
        urdf_file_path: Absolute path to the URDF file.
        robot_name: Optional, a name for the robot for display purposes.

    Returns:
        True if the URDF is valid and visualization is attempted, False otherwise.
    """
    print(f"\n--- Validating URDF: {urdf_file_path} ---")
    if not os.path.exists(urdf_file_path):
        print(f"❌ Error: URDF file not found at {urdf_file_path}")
        return False

    # Check if ROS 2 environment is sourced
    if not command_exists("check_urdf"):
        print("❌ Error: 'check_urdf' command not found.")
        print("       Please ensure ROS 2 environment is sourced (e.g., source /opt/ros/humble/setup.bash)")
        return False

    # --- Step 1: Validate URDF using check_urdf ---
    print("✅ Running check_urdf for syntax and basic structure...")
    try:
        # check_urdf outputs to stderr, so capture it
        result = subprocess.run(
            ["check_urdf", urdf_file_path],
            capture_output=True,
            text=True,
            check=False  # Don't raise exception for non-zero exit code immediately
        )
        if result.returncode == 0:
            print("✅ check_urdf passed: URDF syntax is valid.")
        else:
            print(f"❌ check_urdf failed:\n{result.stderr}")
            return False
    except Exception as e:
        print(f"❌ An error occurred during check_urdf: {e}")
        return False

    # --- Step 2: Attempt visualization in RViz2 (optional but highly recommended) ---
    print("\n--- Attempting visualization in RViz2 ---")
    if not command_exists("ros2"):
        print("⚠️ Warning: 'ros2' command not found. Skipping RViz2 visualization.")
        print("       Ensure ROS 2 environment is sourced.")
        return True # URDF is valid, but cannot visualize

    if not command_exists("rviz2"):
        print("⚠️ Warning: 'rviz2' command not found. Skipping RViz2 visualization.")
        print("       Ensure RViz2 is installed (e.g., sudo apt install ros-humble-rviz2).")
        return True # URDF is valid, but cannot visualize

    # Export ROS_PACKAGE_PATH if the URDF is within a package to help RViz find meshes
    # This part assumes a standard ROS workspace structure
    package_path = os.path.abspath(os.path.join(urdf_file_path, "..", ".."))
    if "src" in package_path and os.path.exists(os.path.join(package_path, "share")):
        print(f"💡 Setting ROS_PACKAGE_PATH for mesh discovery: {package_path}")
        os.environ["ROS_PACKAGE_PATH"] = package_path + os.pathsep + os.environ.get("ROS_PACKAGE_PATH", "")

    # Launch robot_state_publisher and RViz2
    print("💡 Launching robot_state_publisher and RViz2...")
    print("   - RViz2 window should open. If not, check terminal for errors.")
    print("   - In RViz2, add a 'RobotModel' display and ensure 'Fixed Frame' is set to 'base_link'.")
    print("   - You may need to manually add 'robot_state_publisher' to your launch file for complex setups.")
    print("   - Press Ctrl+C in this terminal to stop RViz2.")

    try:
        # Start robot_state_publisher in the background
        # Use 'stdbuf -oL' to ensure immediate output for better logging
        robot_state_publisher_cmd = [
            "ros2", "run", "robot_state_publisher", "robot_state_publisher",
            "--ros-args", "-p", f"robot_description:='$(cat {urdf_file_path})'",
        ]
        # The `robot_description` parameter expects the XML content directly.
        # This approach reads the file content and passes it as a parameter.
        print(f"    Starting: {' '.join(robot_state_publisher_cmd)}")
        robot_state_process = subprocess.Popen(
            robot_state_publisher_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1, # Line-buffered
            universal_newlines=True # Ensure text mode for output
        )
        # Read and print output from robot_state_publisher in real-time
        print("    [robot_state_publisher output]:")
        for line in iter(robot_state_process.stdout.readline, ''):
            sys.stdout.write(f"        {line}")
            if "started" in line.lower():
                break # Wait for it to start before launching rviz2
        robot_state_process.stdout.close()

        # Start RViz2
        rviz_cmd = [
            "rviz2",
            "-d", "$(ros2 pkg prefix rviz2_default_plugins)/share/rviz2_default_plugins/rviz/default.rviz", # Use default config
            "-f", "base_link" # Set fixed frame
        ]
        print(f"    Starting: {' '.join(rviz_cmd)}")
        rviz_process = subprocess.run(rviz_cmd, check=False)

        # Clean up robot_state_publisher
        robot_state_process.terminate()
        robot_state_process.wait()

        if rviz_process.returncode != 0:
            print("❌ RViz2 exited with an error. Check the RViz2 window or terminal for details.")
            return True # URDF is valid, but visualization failed

    except FileNotFoundError:
        print("❌ Error: Make sure 'ros2' and 'rviz2' commands are in your PATH.")
        return True # URDF is valid, but visualization failed
    except Exception as e:
        print(f"❌ An unexpected error occurred during RViz2 visualization: {e}")
        return True # URDF is valid, but visualization failed

    print("✅ URDF visualization attempt completed.")
    print("    (If RViz2 launched, you should see your robot model. Close RViz2 to continue.)")
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Validate a URDF file and optionally visualize it in RViz2."
    )
    parser.add_argument(
        "urdf_file",
        type=str,
        help="Absolute path to the URDF file to validate."
    )
    parser.add_argument(
        "--robot-name",
        type=str,
        default="",
        help="Optional: A name for the robot for display purposes."
    )
    args = parser.parse_args()

    if not validate_urdf(args.urdf_file, args.robot_name):
        sys.exit(1)
    sys.exit(0)
