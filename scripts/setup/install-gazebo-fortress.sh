#!/bin/bash
# Gazebo Fortress Installation Script for Ubuntu 22.04
# Usage: ./install-gazebo-fortress.sh

set -e

echo "=== Gazebo Fortress Installation Script ==="
echo ""

# Check Ubuntu version
if [ "$(lsb_release -cs)" != "jammy" ]; then
    echo "Error: This script requires Ubuntu 22.04"
    exit 1
fi

# Add Gazebo repository
echo "[1/3] Adding Gazebo repository..."
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install
echo "[2/3] Installing Gazebo Fortress..."
sudo apt update
sudo apt install gz-fortress -y

# Install ROS 2 - Gazebo bridge
echo "[3/3] Installing ROS 2 - Gazebo bridge..."
sudo apt install ros-humble-ros-gz -y

echo ""
echo "=== Installation Complete ==="
echo "Gazebo version: $(gz sim --version | head -n 1)"
echo ""
echo "Test with: gz sim shapes.sdf"
