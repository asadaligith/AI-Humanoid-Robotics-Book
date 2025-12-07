---
sidebar_position: 1
---

# Environment Setup Guide

This comprehensive guide walks you through installing all required software for the AI Humanoid Robotics Book, from operating system setup to AI frameworks.

## Prerequisites

Before starting, ensure you have:
- Computer meeting [Hardware Requirements](../hardware-options.md#recommended-configuration-from-spec)
- Internet connection for package downloads
- ~2-3 hours for full installation
- Administrator/sudo access

## Step 1: Ubuntu 22.04 LTS Installation

### Native Installation (Recommended)

**Download Ubuntu 22.04 LTS**:
1. Visit: https://ubuntu.com/download/desktop
2. Download Ubuntu 22.04.x LTS Desktop (64-bit)
3. Create bootable USB with Rufus (Windows) or Etcher (macOS/Linux)

**Installation Steps**:
```bash
# Boot from USB and select "Install Ubuntu"
# Choose "Normal installation" + "Download updates during installation"
# Partition options:
#   - Dual boot: "Install Ubuntu alongside [existing OS]"
#   - Dedicated machine: "Erase disk and install Ubuntu"
# Allocate at least 100GB for Ubuntu partition
# Create user account and set password
# Reboot and remove USB drive
```

**Post-Installation**:
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y \
  build-essential \
  curl \
  git \
  vim \
  wget \
  software-properties-common
```

### WSL2 on Windows 11 (Limited Support)

**Enable WSL2**:
```powershell
# Run in PowerShell as Administrator
wsl --install -d Ubuntu-22.04
wsl --set-default-version 2
```

**Configure WSL2**:
```bash
# Launch Ubuntu 22.04 from Start Menu
# Create username and password

# Update packages
sudo apt update && sudo apt upgrade -y
```

**Limitations**:
- ⚠️ Gazebo GUI may have rendering issues
- ⚠️ Isaac Sim not supported
- ⚠️ USB device passthrough complex
- ✅ Good for ROS 2 development and CLI tools

## Step 2: ROS 2 Humble Installation

### Official Debian Packages (Recommended)

**Set Locale**:
```bash
locale  # Check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**Setup Sources**:
```bash
# Add ROS 2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Install ROS 2 Humble**:
```bash
# Update apt cache
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Desktop (includes RViz, demos, tutorials)
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# Install additional ROS 2 packages
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization
```

**Initialize rosdep**:
```bash
sudo rosdep init
rosdep update
```

**Environment Setup**:
```bash
# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Expected output: ros2 cli version: 0.18.x
```

**✅ Checkpoint 2.1**: ROS 2 installation successful
```bash
ros2 run demo_nodes_cpp talker
# Expected: [INFO] [talker]: Publishing: 'Hello World: 1'
# Press Ctrl+C to stop
```

### Alternative: ROS 2 Iron (Ubuntu 24.04)

For Ubuntu 24.04 users:
```bash
# Replace 'humble' with 'iron' in all commands above
sudo apt install -y ros-iron-desktop
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
```

## Step 3: Create ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build empty workspace
colcon build --symlink-install

# Source workspace overlay
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**✅ Checkpoint 3.1**: Workspace builds successfully
```bash
cd ~/ros2_ws
colcon build
# Expected: Summary: X packages finished [0.XXs]
```

## Step 4: Gazebo Fortress Installation

### Install Gazebo Fortress

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Fortress
sudo apt update
sudo apt install -y gz-fortress

# Install ROS 2 Gazebo integration
sudo apt install -y ros-humble-ros-gz
```

**Verify Installation**:
```bash
gz sim --version
# Expected: Gazebo Sim, version 6.16.0

# Test Gazebo GUI
gz sim shapes.sdf
# Should open Gazebo window with basic shapes
# Press Ctrl+C to close
```

**✅ Checkpoint 4.1**: Gazebo Fortress runs successfully

### Configure Gazebo Models

```bash
# Create local models directory
mkdir -p ~/.gazebo/models

# Download common models (optional)
git clone https://github.com/osrf/gazebo_models ~/.gazebo/models/gazebo_models
```

## Step 5: NVIDIA Drivers and CUDA (For Isaac Sim)

### Install NVIDIA Drivers

**Check Current Driver**:
```bash
nvidia-smi
# If this works, drivers are already installed
```

**Install Latest Driver**:
```bash
# Add graphics drivers PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install recommended driver (525+ for RTX 3060)
sudo ubuntu-drivers autoinstall

# Reboot required
sudo reboot
```

**Verify Driver**:
```bash
nvidia-smi
# Expected output showing GPU info and driver version
```

### Install CUDA Toolkit 12.x

```bash
# Download CUDA 12.3 (compatible with Isaac Sim 2023.1.1)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-3

# Add CUDA to PATH
echo 'export PATH=/usr/local/cuda-12.3/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA
nvcc --version
# Expected: Cuda compilation tools, release 12.3
```

**✅ Checkpoint 5.1**: CUDA installation successful
```bash
nvidia-smi
nvcc --version
# Both commands should output version information
```

## Step 6: Isaac Sim 2023.1.1 Installation

### Install Omniverse Launcher

**Download Launcher**:
```bash
# Visit: https://www.nvidia.com/en-us/omniverse/download/
# Download AppImage for Linux

# Make executable and run
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

**Login and Setup**:
1. Create NVIDIA account (free)
2. Login to Omniverse Launcher
3. Go to "Exchange" tab
4. Search for "Isaac Sim"

### Install Isaac Sim

**Via Launcher**:
1. Click "Isaac Sim" in Exchange
2. Select version 2023.1.1
3. Click "Install" (~50GB download)
4. Installation location: `~/.local/share/ov/pkg/isaac_sim-2023.1.1/`

**Verify Installation**:
```bash
# Launch Isaac Sim (GUI)
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh

# Expected: Isaac Sim window opens with stage editor
```

**✅ Checkpoint 6.1**: Isaac Sim launches successfully

### Configure Isaac Sim Python Environment

```bash
# Add Isaac Sim Python to PATH
echo 'export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac_sim-2023.1.1"' >> ~/.bashrc
echo 'export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"' >> ~/.bashrc
source ~/.bashrc

# Test Isaac Sim Python
${ISAACSIM_PYTHON_EXE} --version
# Expected: Python 3.10.x
```

### Install ROS 2 Bridge for Isaac Sim

```bash
# Isaac Sim includes ROS 2 bridge packages
# Verify by running example
cd ${ISAACSIM_PATH}
./python.sh standalone_examples/api/omni.isaac.ros2_bridge/camera.py

# Expected: Isaac Sim publishes camera images to ROS 2 topic
```

## Step 7: Python Development Environment

### Install Python 3.10+ and Pip

```bash
# Python 3.10 included with Ubuntu 22.04
python3 --version
# Expected: Python 3.10.x

# Install pip and venv
sudo apt install -y python3-pip python3-venv

# Upgrade pip
python3 -m pip install --upgrade pip
```

### Create Virtual Environment (Optional)

```bash
# For isolating Python packages
python3 -m venv ~/robotics_env
source ~/robotics_env/bin/activate

# Add to ~/.bashrc for convenience
echo "alias robotics='source ~/robotics_env/bin/activate'" >> ~/.bashrc
```

### Install Python Packages

```bash
# Install from book repository requirements.txt
cd ~/AI-Humanoid-Robotics-Book
pip install -r requirements.txt

# Key packages installed:
# - pytest (testing)
# - black, flake8, mypy (code quality)
# - numpy, opencv-python (core libraries)
```

**✅ Checkpoint 7.1**: Python environment configured
```bash
python3 -c "import cv2; print(cv2.__version__)"
# Expected: 4.8.x or similar
```

## Step 8: AI/ML Frameworks

### Install PyTorch (CUDA 12.x)

```bash
# Install PyTorch with CUDA 12.1 support
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Verify GPU support
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
# Expected: CUDA available: True
```

### Install Whisper (OpenAI)

```bash
# Install OpenAI Whisper for voice transcription
pip3 install openai-whisper

# Download base model (required for Module 4-5)
whisper --model base --language en /dev/null 2>&1 | head -1
# Expected: Model will be downloaded to ~/.cache/whisper/
```

### Install Anthropic Claude SDK

```bash
# Install Anthropic Python SDK
pip3 install anthropic

# Configure API key (required for Modules 4-5)
# Get API key from: https://console.anthropic.com/
echo 'export ANTHROPIC_API_KEY="your-api-key-here"' >> ~/.bashrc
source ~/.bashrc
```

**⚠️ Warning**: Never commit API keys to version control. Use `.env` files (gitignored).

## Step 9: Additional Tools

### Install Visual Studio Code

```bash
# Download and install VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" \
  | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null

sudo apt update
sudo apt install -y code

# Install recommended extensions
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension ms-iot.vscode-ros
```

### Install ROS 2 Development Tools

```bash
# Install rqt (ROS Qt GUI tools)
sudo apt install -y ros-humble-rqt*

# Install RViz2 plugins
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rviz-default-plugins

# Verify rqt
rqt
# Should open Qt GUI window
```

### Install Git Configuration

```bash
# Configure Git identity
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Configure line endings
git config --global core.autocrlf input

# Configure default editor
git config --global core.editor vim
```

## Step 10: Docker (Optional, for Reproducible Environments)

### Install Docker Engine

```bash
# Add Docker GPG key and repository
sudo apt update
sudo apt install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### Install NVIDIA Container Toolkit

```bash
# For running GPU-accelerated containers
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list \
  | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install -y nvidia-container-toolkit

# Restart Docker
sudo systemctl restart docker
```

**Verify Docker + GPU**:
```bash
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
# Expected: nvidia-smi output showing GPU info
```

## Step 11: Clone Book Repository

```bash
# Clone AI Humanoid Robotics Book repository
cd ~
git clone https://github.com/asadaligith/AI-Humanoid-Robotics-Book.git
cd AI-Humanoid-Robotics-Book

# Install Node.js dependencies (for Docusaurus)
npm install

# Start development server
npm start
# Expected: Opens http://localhost:3000/AI-Humanoid-Robotics-Book/
```

## Final Verification Checklist

Run all checkpoints to ensure complete setup:

```bash
# ROS 2 Humble
ros2 --version

# Gazebo Fortress
gz sim --version

# NVIDIA Driver
nvidia-smi

# CUDA Toolkit
nvcc --version

# Isaac Sim
${ISAACSIM_PYTHON_EXE} --version

# Python packages
python3 -c "import torch; import cv2; print('PyTorch + OpenCV OK')"

# Docker (optional)
docker --version

# ROS 2 workspace
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep -i ros2
```

**✅ All Green?** You're ready to start [Module 1: ROS 2 Nervous System](../intro.md)!

## Troubleshooting

### Common Issues

**Issue**: `rosdep init` fails with "permission denied"
```bash
# Solution: Run with sudo
sudo rosdep init
rosdep update  # Without sudo
```

**Issue**: Gazebo crashes with "GLX error"
```bash
# Solution: Update graphics drivers
sudo ubuntu-drivers autoinstall
sudo reboot
```

**Issue**: Isaac Sim fails to launch
```bash
# Solution: Check NVIDIA driver version (525+)
nvidia-smi
# Update drivers if needed
sudo apt install nvidia-driver-535
```

**Issue**: `colcon build` fails with "setuptools not found"
```bash
# Solution: Install setuptools
pip3 install setuptools==58.2.0
```

**Issue**: Docker permission denied
```bash
# Solution: Add user to docker group and re-login
sudo usermod -aG docker $USER
# Logout and login again
```

### Getting Help

- ROS 2 Issues: https://answers.ros.org/
- Gazebo Issues: https://community.gazebosim.org/
- Isaac Sim Issues: https://forums.developer.nvidia.com/c/isaac/
- Book-specific: https://github.com/asadaligith/AI-Humanoid-Robotics-Book/issues

---

**Environment setup complete!** Proceed to [How to Use This Book](../how-to-use.md) or start learning with [Module 1](../intro.md).
