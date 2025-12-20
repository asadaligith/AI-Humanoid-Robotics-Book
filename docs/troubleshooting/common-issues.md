---
title: Common Issues & Solutions
description: Troubleshooting guide for common problems in the AI & Humanoid Robotics course
sidebar_position: 1
---

# Common Issues & Solutions

Comprehensive troubleshooting guide organized by module and error type.

## General ROS 2 Issues

### `ros2: command not found`

**Cause**: ROS 2 environment not sourced

**Solution**:
```bash
# Temporary fix (current terminal only)
source /opt/ros/humble/setup.bash

# Permanent fix
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### `Package 'X' not found`

**Cause**: Package not installed or workspace not built

**Solution**:
```bash
# Check if package exists
ros2 pkg list | grep <package_name>

# If missing, install
sudo apt install ros-humble-<package-name>

# If custom package, rebuild workspace
cd ~/ros2_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

### `colcon build` fails with CMake errors

**Cause**: Missing build dependencies

**Solution**:
```bash
# Install dependencies for all packages
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build --symlink-install
```

## Gazebo Issues

### Gazebo GUI won't launch / Black screen

**Cause**: Graphics driver issues

**Solutions**:

**Check OpenGL support**:
```bash
glxinfo | grep "OpenGL version"
# Should show OpenGL 3.3+
```

**Update graphics drivers**:
```bash
# For NVIDIA
sudo ubuntu-drivers autoinstall
sudo reboot

# For Intel/AMD (integrated graphics)
sudo apt install mesa-utils
```

**Run with software rendering (fallback)**:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
gz sim shapes.sdf
```

### `gz sim` crashes immediately

**Cause**: Incompatible Gazebo version

**Solution**:
```bash
# Verify Gazebo Fortress installed
gz sim --version  # Should show "Gazebo Fortress"

# If wrong version, reinstall
sudo apt remove gz-* -y
sudo apt install gz-fortress -y
```

### Models don't load / "Model not found" error

**Cause**: Model paths not configured

**Solution**:
```bash
# Set Gazebo model path
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

## Isaac Sim Issues

### Isaac Sim won't launch

**Cause**: GPU requirements not met

**Verification**:
```bash
nvidia-smi
# Check: GPU model (need RTX 2060+), VRAM (need 6GB+), Driver version (need 535+)
```

**Solutions**:

**Update NVIDIA driver**:
```bash
sudo apt purge nvidia-* -y
sudo apt install nvidia-driver-535 -y
sudo reboot
```

**Use cloud alternative**:
- Sign up for [NVIDIA Omniverse Cloud](https://www.nvidia.com/en-us/omniverse/cloud/)
- No local GPU required

### Isaac Sim crashes during scene load

**Cause**: Insufficient VRAM

**Solution**:
```bash
# Reduce scene complexity
# - Use smaller environments (warehouse_simple.usd instead of warehouse.usd)
# - Reduce texture resolution in Isaac Sim settings

# Monitor VRAM usage
watch -n 1 nvidia-smi
```

### Isaac ROS packages import errors

**Cause**: CUDA/cuDNN mismatch

**Solution**:
```bash
# Check CUDA version
nvcc --version  # Should match Isaac ROS requirements (12.2+)

# Reinstall Isaac ROS packages
sudo apt remove ros-humble-isaac-ros-* -y
sudo apt install ros-humble-isaac-ros-visual-slam -y
```

## VLA (Vision-Language-Action) Issues

### Whisper import error: `No module named 'whisper'`

**Solution**:
```bash
pip3 install --upgrade openai-whisper
# Or use virtual environment:
python3 -m venv ~/venv
source ~/venv/bin/activate
pip install openai-whisper
```

### Whisper inference too slow (>30s per audio clip)

**Causes & Solutions**:

| Cause | Solution |
|-------|----------|
| Using "large" model | Switch to "base" model: `whisper.load_model("base")` |
| No GPU acceleration | Install PyTorch with CUDA: `pip3 install torch --index-url https://download.pytorch.org/whl/cu118` |
| Long audio files | Trim to &lt;30 seconds before processing |

### Claude/GPT-4 API errors

**`AuthenticationError: Invalid API key`**:
```bash
# Verify API key set correctly
echo $ANTHROPIC_API_KEY  # Should not be empty

# Re-set API key
export ANTHROPIC_API_KEY="sk-ant-..."
```

**`RateLimitError`**:
- Wait 60 seconds and retry
- Check API usage limits in console
- Implement exponential backoff in code

**`InvalidRequestError: Message too long`**:
- Reduce system prompt length
- Summarize conversation history
- Use Claude 3.5 Sonnet (200K context) instead of Claude 3 Haiku

## Build & Dependency Issues

### `pip install` fails with permission errors

**Solution**:
```bash
# Use --user flag
pip3 install --user <package>

# OR use virtual environment (recommended)
python3 -m venv ~/venv
source ~/venv/bin/activate
pip install <package>
```

### Docker container can't access GPU

**Cause**: NVIDIA Container Toolkit not installed

**Solution**:
```bash
# Install nvidia-docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker

# Run container with GPU
docker run --gpus all -it nvidia/cuda:12.2-base nvidia-smi
```

## Network & Download Issues

### `apt update` fails / Package downloads timeout

**Solution**:
```bash
# Change to faster mirror
sudo sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list
sudo apt update

# OR use university mirrors (if applicable)
# Edit /etc/apt/sources.list manually
```

### Git clone fails / Large file download stuck

**Solution**:
```bash
# Install Git LFS for large files
sudo apt install git-lfs
git lfs install

# Clone with shallow history (faster)
git clone --depth 1 <repository-url>
```

## Performance Issues

### RViz/Gazebo UI lag

**Solutions**:
1. Reduce visualization quality:
   - RViz: Decrease point cloud density, disable shadows
   - Gazebo: Lower physics update rate, reduce rendering quality

2. Close unnecessary applications

3. Monitor resource usage:
```bash
htop  # Check CPU/RAM usage
nvidia-smi  # Check GPU usage
```

### Simulation runs slower than real-time

**Cause**: Physics computation exceeds real-time factor

**Solutions**:
```bash
# Option 1: Reduce physics accuracy
# In Gazebo SDF, set max_step_size to 0.01 (from 0.001)

# Option 2: Simplify robot model
# Reduce collision mesh complexity in URDF

# Option 3: Disable rendering during batch runs
gz sim -s shapes.sdf  # Server-only mode (no GUI)
```

## Module-Specific Error Codes

### Module 1: ROS 2 Fundamentals

| Error | Code | Solution |
|-------|------|----------|
| Node fails to start | `rclpy.exceptions.InvalidHandle` | Check node name uniqueness |
| Topic not found | `No publishers/subscribers` | Verify topic name with `ros2 topic list` |
| Service timeout | `Service call timed out` | Check server is running: `ros2 service list` |

### Module 3: Isaac/Nav2

| Error | Code | Solution |
|-------|------|----------|
| VSLAM initialization fails | `Visual odometry lost` | Ensure sufficient visual features in scene |
| Nav2 won't plan path | `No path found` | Check costmap configuration, increase planning timeout |
| DOPE detection fails | `No objects detected` | Verify camera published to correct topic |

### Module 4: VLA Pipeline

| Error | Code | Solution |
|-------|------|----------|
| Speech transcription empty | `Whisper returned ""` | Check microphone input level, reduce background noise |
| LLM plan invalid JSON | `JSONDecodeError` | Add JSON schema validation to system prompt |
| Action execution fails | `Action client timeout` | Verify ROS action server running |

## Getting Additional Help

**If issue persists after trying solutions**:

1. **Search GitHub Issues**: [Repository Issues](https://github.com/asadaligith/AI-Humanoid-Robotics-Book/issues)
2. **ROS Answers**: [answers.ros.org](https://answers.ros.org/questions/)
3. **NVIDIA Forums**: [Isaac Sim Forum](https://forums.developer.nvidia.com/c/omniverse/simulation/69)
4. **Course Discussions**: Create new discussion in repository

**When asking for help, include**:
- Operating system version (`lsb_release -a`)
- ROS 2 version (`ros2 --version`)
- Error message (full traceback)
- Steps to reproduce
- What you've already tried

---

**Last Updated**: 2025-12-20

**Contributors**: Welcome! Submit PRs with additional troubleshooting tips.
