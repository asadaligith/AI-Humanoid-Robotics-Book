#!/bin/bash
# ROS 2 Humble Installation Script for Ubuntu 22.04
# Usage: ./install-ros2-humble.sh

set -e  # Exit on error

echo "=== ROS 2 Humble Installation Script ==="
echo "This will install ROS 2 Humble Desktop on Ubuntu 22.04"
echo ""

# Check Ubuntu version
if [ "$(lsb_release -cs)" != "jammy" ]; then
    echo "Error: This script requires Ubuntu 22.04 (Jammy Jellyfish)"
    echo "Current version: $(lsb_release -ds)"
    exit 1
fi

# Set locale
echo "[1/5] Setting up locale..."
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
echo "[2/5] Adding ROS 2 apt repository..."
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
echo "[3/5] Updating package list..."
sudo apt update

# Install ROS 2 Humble Desktop
echo "[4/5] Installing ROS 2 Humble Desktop (this may take 5-10 minutes)..."
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y

# Environment setup
echo "[5/5] Setting up environment..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "Added ROS 2 setup to ~/.bashrc"
fi

# Source for current session
source /opt/ros/humble/setup.bash

# Verify installation
echo ""
echo "=== Installation Complete ==="
echo "ROS 2 version: $(ros2 --version)"
echo ""
echo "To apply changes, run: source ~/.bashrc"
echo "Or open a new terminal"
echo ""
echo "Test with: ros2 run demo_nodes_cpp talker"
