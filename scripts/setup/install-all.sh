#!/bin/bash
# Complete Installation Script - All Modules
# Usage: ./install-all.sh [--skip-isaac]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKIP_ISAAC=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-isaac)
            SKIP_ISAAC=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--skip-isaac]"
            exit 1
            ;;
    esac
done

echo "=== AI & Humanoid Robotics Course - Complete Setup ==="
echo ""
echo "This will install:"
echo "  - ROS 2 Humble"
echo "  - Gazebo Fortress"
if [ "$SKIP_ISAAC" = false ]; then
    echo "  - Isaac Sim (guide)"
fi
echo "  - Python dependencies"
echo ""
read -p "Continue? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled"
    exit 1
fi

# Install ROS 2 Humble
echo ""
echo "==== Installing ROS 2 Humble ===="
if [ -f "$SCRIPT_DIR/install-ros2-humble.sh" ]; then
    bash "$SCRIPT_DIR/install-ros2-humble.sh"
else
    echo "ERROR: install-ros2-humble.sh not found"
    exit 1
fi

# Install Gazebo Fortress
echo ""
echo "==== Installing Gazebo Fortress ===="
if [ -f "$SCRIPT_DIR/install-gazebo-fortress.sh" ]; then
    bash "$SCRIPT_DIR/install-gazebo-fortress.sh"
else
    echo "ERROR: install-gazebo-fortress.sh not found"
    exit 1
fi

# Isaac Sim guide
if [ "$SKIP_ISAAC" = false ]; then
    echo ""
    echo "==== Isaac Sim Installation Guide ===="
    if [ -f "$SCRIPT_DIR/install-isaac-sim.sh" ]; then
        bash "$SCRIPT_DIR/install-isaac-sim.sh"
    fi
fi

# Install Python dependencies
echo ""
echo "==== Installing Python Dependencies ===="
pip3 install --user openai-whisper anthropic rclpy

echo ""
echo "=== Installation Complete ==="
echo ""
echo "Next steps:"
echo "1. Source ROS 2: source ~/.bashrc"
echo "2. Test ROS 2: ros2 run demo_nodes_cpp talker"
echo "3. Test Gazebo: gz sim shapes.sdf"
echo "4. Start Module 1: https://asadaligith.github.io/AI-Humanoid-Robotics-Book/docs/modules/module-01-ros2-fundamentals"
