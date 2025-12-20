#!/bin/bash
# Docker entrypoint script for ROS 2 containers
# Ensures proper environment setup before running commands

set -e

# Source ROS 2 environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Sourcing ROS 2 ${ROS_DISTRO} environment..."
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace if it exists
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    echo "Sourcing workspace environment..."
    source "/root/ros2_ws/install/setup.bash"
fi

# Alternative workspace path for Isaac ROS
if [ -f "/root/isaac_ws/install/setup.bash" ]; then
    echo "Sourcing Isaac workspace environment..."
    source "/root/isaac_ws/install/setup.bash"
fi

# Print environment info
echo ""
echo "==================================="
echo "ROS 2 Container Environment Ready"
echo "==================================="
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "ROS_VERSION: $([ -n "${ROS_VERSION}" ] && echo "${ROS_VERSION}" || echo "2")"
echo "Workspace: $([ -d "/root/ros2_ws" ] && echo "/root/ros2_ws" || [ -d "/root/isaac_ws" ] && echo "/root/isaac_ws" || echo "None")"
echo ""

# Execute the provided command or start bash
exec "$@"
