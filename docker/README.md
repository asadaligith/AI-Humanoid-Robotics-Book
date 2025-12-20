# Docker Images for AI & Humanoid Robotics Course

This directory contains Dockerfiles for running course modules in containerized environments. Docker provides a consistent, reproducible development environment across different platforms.

## Available Images

### 1. ROS 2 Base (`Dockerfile.ros2-base`)
- **Size**: ~2GB
- **Use For**: Modules 1 & 4 (ROS 2 Fundamentals, VLA)
- **Includes**: ROS 2 Humble, Python dependencies, development tools

### 2. Gazebo Simulation (`Dockerfile.gazebo`)
- **Size**: ~3GB
- **Use For**: Module 2 (Digital Twin - Gazebo & Unity)
- **Includes**: ROS 2 Humble, Gazebo Fortress, ROS-Gazebo bridge

### 3. Isaac ROS (`Dockerfile.isaac-ros`)
- **Size**: ~10GB
- **Use For**: Module 3 (AI-Robot Brain - Isaac/Nav2)
- **Includes**: CUDA, Isaac ROS, Nav2, perception packages
- **Requires**: NVIDIA GPU with CUDA support

## Prerequisites

### All Images
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
newgrp docker
```

### Isaac ROS Image (GPU Required)
```bash
# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Verify GPU access
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

## Building Images

### Build ROS 2 Base Image
```bash
cd AI-Humanoid-Robotics-Book
docker build -f docker/Dockerfile.ros2-base -t ai-robotics:ros2-base .
```

### Build Gazebo Image
```bash
docker build -f docker/Dockerfile.gazebo -t ai-robotics:gazebo .
```

### Build Isaac ROS Image (requires NVIDIA GPU)
```bash
docker build -f docker/Dockerfile.isaac-ros -t ai-robotics:isaac-ros .
```

### Build All Images
```bash
# Build all images sequentially
docker build -f docker/Dockerfile.ros2-base -t ai-robotics:ros2-base .
docker build -f docker/Dockerfile.gazebo -t ai-robotics:gazebo .
docker build -f docker/Dockerfile.isaac-ros -t ai-robotics:isaac-ros .
```

## Running Containers

### ROS 2 Base Container

**Basic Run:**
```bash
docker run -it --rm ai-robotics:ros2-base
```

**With Workspace Mount:**
```bash
docker run -it --rm \
  -v $(pwd)/examples:/root/ros2_ws/src \
  ai-robotics:ros2-base
```

**Run Example Nodes:**
```bash
# Terminal 1: Start container
docker run -it --rm ai-robotics:ros2-base

# Inside container: Run publisher
cd /root/ros2_ws
python3 src/module-01-ros2-fundamentals/example-01-pubsub/publisher.py
```

### Gazebo Container (GUI Support)

**Linux with X11:**
```bash
# Allow Docker to access X server
xhost +local:docker

# Run container with display
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  ai-robotics:gazebo

# Inside container: Launch Gazebo
gz sim shapes.sdf
```

**With Workspace Mount:**
```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/examples:/root/ros2_ws/src \
  --device /dev/dri \
  ai-robotics:gazebo
```

**NVIDIA GPU Support:**
```bash
docker run -it --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ai-robotics:gazebo
```

### Isaac ROS Container (NVIDIA GPU Required)

**Basic Run with GPU:**
```bash
docker run -it --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ai-robotics:isaac-ros
```

**Test CUDA Availability:**
```bash
docker run -it --rm --gpus all ai-robotics:isaac-ros \
  python3 -c "import torch; print('CUDA Available:', torch.cuda.is_available())"
```

**Run Visual SLAM:**
```bash
# Start container
docker run -it --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ai-robotics:isaac-ros

# Inside container
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

## Multi-Container Development

### Docker Compose (Optional)

Create `docker-compose.yml`:
```yaml
version: '3.8'

services:
  ros2-base:
    image: ai-robotics:ros2-base
    volumes:
      - ./examples:/root/ros2_ws/src
    stdin_open: true
    tty: true

  gazebo:
    image: ai-robotics:gazebo
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ./examples:/root/ros2_ws/src
    devices:
      - /dev/dri
    stdin_open: true
    tty: true

  isaac-ros:
    image: ai-robotics:isaac-ros
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ./examples:/root/ros2_ws/src
    runtime: nvidia
    stdin_open: true
    tty: true
```

Run services:
```bash
# Start all services
docker-compose up -d

# Attach to specific service
docker-compose exec ros2-base bash

# Stop all services
docker-compose down
```

## Common Workflows

### Module 1: ROS 2 Fundamentals
```bash
# Run publisher-subscriber example
docker run -it --rm \
  -v $(pwd)/examples:/root/ros2_ws/src \
  ai-robotics:ros2-base bash

# Inside container
cd src/module-01-ros2-fundamentals/example-01-pubsub
python3 publisher.py &
python3 subscriber.py
```

### Module 2: Gazebo Simulation
```bash
# Run Gazebo with custom world
xhost +local:docker
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/examples:/root/ros2_ws/src \
  --device /dev/dri \
  ai-robotics:gazebo

# Inside container
cd src/module-02-digital-twin
gz sim custom_world.sdf
```

### Module 3: Isaac ROS Navigation
```bash
# Run Nav2 with Isaac perception
docker run -it --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ai-robotics:isaac-ros

# Inside container
ros2 launch nav2_bringup navigation_launch.py
```

## Troubleshooting

### Issue 1: "Cannot connect to X server"
**Linux:**
```bash
xhost +local:docker
```

**Windows (WSL2):**
```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

### Issue 2: "NVIDIA GPU not detected"
```bash
# Verify nvidia-docker2 installation
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# Check Docker daemon configuration
cat /etc/docker/daemon.json
# Should include: "default-runtime": "nvidia"
```

### Issue 3: "Permission denied" for /dev/dri
```bash
# Add user to video group
sudo usermod -aG video $USER
newgrp video
```

### Issue 4: Container out of disk space
```bash
# Clean up unused images and containers
docker system prune -a

# Remove specific image
docker rmi ai-robotics:ros2-base
```

## Development Best Practices

1. **Volume Mounts**: Always mount your workspace for code persistence
2. **Named Containers**: Use `--name` for easier management
3. **Resource Limits**: Set memory/CPU limits for production
4. **Environment Files**: Use `.env` files for configuration
5. **Multi-Stage Builds**: Keep images small and optimized

## Image Sizes

| Image | Compressed | Uncompressed |
|-------|------------|--------------|
| ros2-base | ~800MB | ~2GB |
| gazebo | ~1.2GB | ~3GB |
| isaac-ros | ~4GB | ~10GB |

## Advanced Usage

### Development with Remote Container
```bash
# VS Code Remote Containers configuration
# See .devcontainer/devcontainer.json for IDE integration
```

### CI/CD Integration
```bash
# Build in CI pipeline
docker build --cache-from ai-robotics:ros2-base \
  -t ai-robotics:ros2-base-${CI_COMMIT_SHA} .
```

## References

- [Docker Documentation](https://docs.docker.com/)
- [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker)
- [ROS 2 Docker Tutorial](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)
- [Gazebo Docker Images](https://hub.docker.com/r/osrf/ros)
