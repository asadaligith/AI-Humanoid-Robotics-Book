---
title: Installation Guide
description: Step-by-step installation of ROS 2, Gazebo, Isaac Sim, and course dependencies
sidebar_position: 2
---

# Installation Guide

Complete installation instructions for all course modules. Install components as you progress through modules.

## Module 1: ROS 2 Humble Installation

### Method 1: Automated Script (Recommended)

```bash
# Download and run installation script
wget https://raw.githubusercontent.com/asadaligith/AI-Humanoid-Robotics-Book/master/scripts/setup/install-ros2-humble.sh
chmod +x install-ros2-humble.sh
./install-ros2-humble.sh
```

### Method 2: Manual Installation

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verification

```bash
# Check ROS 2 installation
ros2 --version  # Should output: ros2 version humble

# Test with demo nodes
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
# You should see messages being published and received
```

## Module 2: Gazebo Fortress Installation

### Automated Installation

```bash
wget https://raw.githubusercontent.com/asadaligith/AI-Humanoid-Robotics-Book/master/scripts/setup/install-gazebo-fortress.sh
chmod +x install-gazebo-fortress.sh
./install-gazebo-fortress.sh
```

### Manual Installation

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Fortress
sudo apt update
sudo apt install gz-fortress -y

# Install ROS 2 - Gazebo bridge
sudo apt install ros-humble-ros-gz -y
```

### Verification

```bash
# Launch Gazebo
gz sim shapes.sdf
# GUI should open with 3D shapes scene
```

## Module 3: Isaac Sim & Isaac ROS Installation

:::warning GPU Required
Isaac Sim requires an NVIDIA RTX GPU with 6GB+ VRAM. See [Prerequisites](./prerequisites.md) for details.
:::

### Step 1: NVIDIA Driver Installation

```bash
# Check current driver
nvidia-smi

# If driver missing or outdated, install:
sudo apt install nvidia-driver-535 -y
sudo reboot

# Verify after reboot
nvidia-smi  # Should show GPU info and driver version
```

### Step 2: Isaac Sim Installation (Omniverse)

1. Download [NVIDIA Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/)
2. Install Omniverse Launcher
3. Within Omniverse, install **Isaac Sim 2023.1.1**

**Alternative: Cloud Access**
- Use [NVIDIA Omniverse Cloud](https://www.nvidia.com/en-us/omniverse/cloud/) if no local GPU
- Requires account registration

### Step 3: Isaac ROS Packages

```bash
# Install Isaac ROS dependencies
sudo apt install ros-humble-isaac-ros-visual-slam \
                 ros-humble-isaac-ros-apriltag \
                 ros-humble-isaac-ros-dnn-inference -y

# Install CUDA Toolkit (if not already installed)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda-repo-ubuntu2204-12-2-local_12.2.0-535.54.03-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-2-local_12.2.0-535.54.03-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-2-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt install cuda -y
```

### Verification

```bash
# Check CUDA
nvcc --version

# Check Isaac ROS packages
ros2 pkg list | grep isaac_ros
```

## Module 4: VLA Dependencies

### OpenAI Whisper

```bash
# Install Whisper
pip3 install openai-whisper

# Download base model
python3 -c "import whisper; whisper.load_model('base')"
```

### Claude/GPT-4 API Setup

```bash
# Install Anthropic SDK
pip3 install anthropic

# Set API key (get from https://console.anthropic.com)
echo 'export ANTHROPIC_API_KEY="your-api-key-here"' >> ~/.bashrc
source ~/.bashrc
```

**Alternative: OpenAI GPT-4**

```bash
pip3 install openai
echo 'export OPENAI_API_KEY="your-api-key-here"' >> ~/.bashrc
```

### Verification

```bash
# Test Whisper
python3 -c "import whisper; print('Whisper installed')"

# Test Anthropic
python3 -c "import anthropic; print('Anthropic SDK installed')"
```

## Docker Alternative (Modules 1-2 Only)

For users without native Ubuntu installation:

```bash
# Pull ROS 2 Humble image
docker pull osrf/ros:humble-desktop-full

# Run container
docker run -it --rm \
  -v $(pwd):/workspace \
  osrf/ros:humble-desktop-full \
  bash

# Inside container
source /opt/ros/humble/setup.bash
```

## Course Examples & Materials

```bash
# Clone course repository
git clone https://github.com/asadaligith/AI-Humanoid-Robotics-Book.git
cd AI-Humanoid-Robotics-Book/examples

# Install Python dependencies
pip3 install -r requirements.txt
```

## Workspace Setup

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Troubleshooting

**Common Issues**:

| Issue | Solution |
|-------|----------|
| `ros2: command not found` | Source setup: `source /opt/ros/humble/setup.bash` |
| Gazebo won't launch | Check GPU drivers: `glxinfo \| grep OpenGL` |
| Isaac Sim crashes | Verify GPU VRAM: `nvidia-smi` (need 6GB+) |
| Import errors (Python) | Install in virtual environment: `python3 -m venv venv` |

See [Troubleshooting Guide](../troubleshooting/common-issues.md) for detailed solutions.

## Installation Summary

| Module | Component | Installation Time | Disk Space |
|--------|-----------|-------------------|------------|
| 1 | ROS 2 Humble | 15 min | 3GB |
| 2 | Gazebo Fortress | 10 min | 2GB |
| 3 | Isaac Sim + Isaac ROS | 30-60 min | 25GB |
| 4 | Whisper + LLM SDKs | 5 min | 2GB |
| **Total** | | **60-90 min** | **32GB** |

---

**Next**: Start [Module 1: ROS 2 Fundamentals](../modules/module-01-ros2-fundamentals/index.md)
