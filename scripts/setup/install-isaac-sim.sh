#!/bin/bash
# Isaac Sim Installation Guide for Ubuntu 22.04
# This is a guide script - manual steps required for Omniverse
# Usage: ./install-isaac-sim.sh

set -e

echo "=== Isaac Sim Installation Guide ==="
echo ""
echo "PREREQUISITES:"
echo "- NVIDIA RTX GPU (2060 or better)"
echo "- 6GB+ VRAM"
echo "- NVIDIA Driver 535+"
echo ""

# Check GPU
if ! command -v nvidia-smi &> /dev/null; then
    echo "ERROR: nvidia-smi not found. NVIDIA driver not installed."
    echo ""
    echo "Install driver with:"
    echo "  sudo apt install nvidia-driver-535 -y"
    echo "  sudo reboot"
    exit 1
fi

echo "GPU detected:"
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
echo ""

# Check VRAM
VRAM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits)
if [ "$VRAM" -lt 6000 ]; then
    echo "WARNING: GPU has less than 6GB VRAM ($VRAM MB)"
    echo "Isaac Sim may not run properly"
fi

echo ""
echo "=== Installation Steps ==="
echo ""
echo "1. Download NVIDIA Omniverse Launcher:"
echo "   https://www.nvidia.com/en-us/omniverse/"
echo ""
echo "2. Install Omniverse Launcher (AppImage or .deb)"
echo ""
echo "3. Within Omniverse Launcher, install Isaac Sim 2023.1.1+"
echo ""
echo "4. Install Isaac ROS packages:"
echo ""
echo "   sudo apt install ros-humble-isaac-ros-visual-slam \\"
echo "                    ros-humble-isaac-ros-apriltag \\"
echo "                    ros-humble-isaac-ros-dnn-inference -y"
echo ""
echo "5. Verify installation:"
echo "   Launch Isaac Sim from Omniverse Launcher"
echo ""
echo "ALTERNATIVE: NVIDIA Omniverse Cloud (no local GPU required)"
echo "   https://www.nvidia.com/en-us/omniverse/cloud/"
echo ""
echo "For detailed instructions, see:"
echo "https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html"
