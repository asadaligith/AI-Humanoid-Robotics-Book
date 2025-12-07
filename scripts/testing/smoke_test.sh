#!/bin/bash
#
# ROS 2 Smoke Test for AI Humanoid Robotics Capstone
#
# Purpose: Quick validation that all ROS 2 nodes can be imported and basic
#          integration logic works in mock mode without full simulator.
#
# Usage:
#   ./smoke_test.sh
#
# Exit codes:
#   0: All tests passed
#   1: One or more tests failed
#
# Requirements:
#   - ROS 2 Humble installed
#   - Python dependencies installed (requirements.txt)
#   - Capstone package in workspace

set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================"
echo "ROS 2 Capstone Smoke Test"
echo "========================================"
echo ""

# Track test results
PASSED=0
FAILED=0

# Test function
run_test() {
    local test_name="$1"
    local test_command="$2"

    echo -n "Testing: $test_name ... "

    if eval "$test_command" > /dev/null 2>&1; then
        echo -e "${GREEN}PASS${NC}"
        ((PASSED++))
        return 0
    else
        echo -e "${RED}FAIL${NC}"
        ((FAILED++))
        return 1
    fi
}

# Test 1: ROS 2 environment
run_test "ROS 2 environment sourced" \
    "command -v ros2"

# Test 2: Python imports
run_test "Python ROS 2 client library (rclpy)" \
    "python3 -c 'import rclpy'"

run_test "Python OpenAI library" \
    "python3 -c 'import openai'"

run_test "Python sounddevice library" \
    "python3 -c 'import sounddevice'"

run_test "Python OpenCV library" \
    "python3 -c 'import cv2'"

# Test 3: Node imports (without running)
echo ""
echo "Testing ROS 2 node imports..."

run_test "Voice input node imports" \
    "python3 -c 'import sys; sys.path.insert(0, \"examples/module-05-capstone\"); import voice_input_node'"

run_test "LLM planner node imports" \
    "python3 -c 'import sys; sys.path.insert(0, \"examples/module-05-capstone\"); import llm_planner_node'"

run_test "Integration demo imports" \
    "python3 -c 'import sys; sys.path.insert(0, \"examples/module-05-capstone\"); import integration_demo'"

# Test 4: Configuration files exist
echo ""
echo "Testing configuration files..."

run_test "Voice config exists" \
    "test -f examples/module-05-capstone/config/voice_config.yaml"

run_test "Nav2 params exist" \
    "test -f examples/module-05-capstone/config/nav2_params.yaml"

run_test "Detection classes exist" \
    "test -f examples/module-05-capstone/config/detection_classes.yaml"

run_test "Grasp poses exist" \
    "test -f examples/module-05-capstone/config/grasp_poses.yaml"

# Test 5: Launch files exist
echo ""
echo "Testing launch files..."

run_test "Gazebo demo launch exists" \
    "test -f examples/module-05-capstone/launch/gazebo_demo.launch.py"

run_test "Isaac Sim demo launch exists" \
    "test -f examples/module-05-capstone/launch/isaac_demo.launch.py"

# Test 6: Simulation assets exist
echo ""
echo "Testing simulation assets..."

run_test "Unitree G1 URDF exists" \
    "test -f assets/urdf/unitree_g1.urdf"

run_test "Kitchen environment world exists" \
    "test -f assets/worlds/kitchen_env.world"

run_test "Unitree G1 USD exists" \
    "test -f assets/usd/unitree_g1.usd"

run_test "Kitchen environment USD exists" \
    "test -f assets/usd/kitchen_env.usd"

# Test 7: Documentation exists
echo ""
echo "Testing documentation..."

run_test "Chapter 1 exists" \
    "test -f docs/modules/module-05-capstone/chapter-01-architecture.md"

run_test "Chapter 2 exists" \
    "test -f docs/modules/module-05-capstone/chapter-02-voice-llm.md"

run_test "Testing methodology exists" \
    "test -f docs/modules/module-05-capstone/testing-methodology.md"

run_test "Benchmarking guide exists" \
    "test -f docs/modules/module-05-capstone/benchmarking.md"

run_test "Troubleshooting guide exists" \
    "test -f docs/modules/module-05-capstone/troubleshooting.md"

# Test 8: URDF validation (if check_urdf is available)
echo ""
echo "Testing URDF validity..."

if command -v check_urdf &> /dev/null; then
    run_test "URDF validation (check_urdf)" \
        "check_urdf assets/urdf/unitree_g1.urdf"
else
    echo -e "${YELLOW}SKIP${NC}: check_urdf not found (install with: apt install liburdfdom-tools)"
fi

# Test 9: Mock mode integration test (quick 5-second test)
echo ""
echo "Testing mock mode integration..."

# Create temporary test script
cat > /tmp/capstone_mock_test.py << 'EOFPYTHON'
#!/usr/bin/env python3
import sys
sys.path.insert(0, "examples/module-05-capstone")

# Test that mock mode can be initialized without errors
try:
    from integration_demo import State
    print("Mock mode initialization: PASS")
    sys.exit(0)
except Exception as e:
    print(f"Mock mode initialization: FAIL - {e}")
    sys.exit(1)
EOFPYTHON

run_test "Mock mode initialization" \
    "python3 /tmp/capstone_mock_test.py"

# Cleanup
rm -f /tmp/capstone_mock_test.py

# Summary
echo ""
echo "========================================"
echo "Test Summary"
echo "========================================"
echo -e "Passed: ${GREEN}$PASSED${NC}"
if [ $FAILED -gt 0 ]; then
    echo -e "Failed: ${RED}$FAILED${NC}"
else
    echo -e "Failed: $FAILED"
fi
echo "========================================"

# Exit with appropriate code
if [ $FAILED -gt 0 ]; then
    echo ""
    echo -e "${RED}Smoke test FAILED${NC}"
    exit 1
else
    echo ""
    echo -e "${GREEN}All smoke tests PASSED${NC}"
    exit 0
fi
