# Example 01: Isaac Sim Setup and First Steps

## Learning Objectives

By completing this example, you will:
- Install and configure NVIDIA Isaac Sim
- Navigate the Isaac Sim interface
- Load and manipulate robots in simulation
- Understand Isaac Sim's ROS 2 integration
- Run basic perception and navigation examples

## Overview

NVIDIA Isaac Sim is a photorealistic robot simulation platform built on NVIDIA Omniverse. It provides GPU-accelerated physics, rendering, and sensor simulation for robotics development.

**Key Features:**
- RTX ray-traced rendering
- PhysX 5 physics engine
- Synthetic data generation
- ROS/ROS 2 integration
- Domain randomization for ML

## Prerequisites

- NVIDIA GPU (RTX 2060 or better recommended)
- 6GB+ VRAM
- NVIDIA Driver 525+
- Ubuntu 22.04 or Windows 10/11
- ROS 2 Humble installed
- Completed Modules 01-02

## Installation

### Option 1: Workstation Installation

#### Step 1: Install NVIDIA Omniverse Launcher

```bash
# Download from NVIDIA
# https://www.nvidia.com/en-us/omniverse/download/

# For Ubuntu (AppImage)
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

#### Step 2: Install Isaac Sim via Launcher

1. Open Omniverse Launcher
2. Go to "Exchange" tab
3. Search for "Isaac Sim"
4. Click "Install" (version 2023.1.1 or later)
5. Wait for installation (15-20 GB download)

#### Step 3: Install Isaac ROS packages

```bash
# Install Isaac ROS dependencies
sudo apt install -y \
  ros-humble-isaac-ros-visual-slam \
  ros-humble-isaac-ros-apriltag \
  ros-humble-isaac-ros-dnn-inference \
  ros-humble-isaac-ros-image-proc

# Install Python bindings
pip3 install isaacsim-python
```

### Option 2: Cloud Deployment (No GPU Required)

```bash
# Use NVIDIA Omniverse Cloud
# https://www.nvidia.com/en-us/omniverse/cloud/

# Sign up for cloud instance
# Stream Isaac Sim from browser
# No local GPU required!
```

### Option 3: Docker Container

```bash
# Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run with GPU support
docker run --gpus all -it \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

## Verifying Installation

```bash
# Check NVIDIA driver
nvidia-smi

# Check Isaac Sim installation
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh --help

# Test with minimal scene
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh \
  --/app/window/width=1280 \
  --/app/window/height=720 \
  --/app/fastShutdown=1
```

## Isaac Sim Interface Tour

### Main Components

1. **Viewport**: 3D scene visualization with RTX rendering
2. **Stage**: Hierarchy of scene objects (USD format)
3. **Property Panel**: Object properties and parameters
4. **Content Browser**: Asset library and file browser
5. **Console**: Python scripts and command execution

### Essential Shortcuts

- `F`: Frame selected object
- `G`: Toggle grid
- `L`: Toggle lights
- `Ctrl+S`: Save scene
- `Ctrl+O`: Open scene
- `Space`: Play/pause simulation
- `Ctrl+Shift+P`: Command palette

## Loading Your First Robot

### From Asset Library

```python
# In Isaac Sim Python console
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world
world = World()

# Load Carter robot from assets
carter_path = "/World/Carter"
carter_asset = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1/Isaac/Robots/Carter/carter_v1.usd"
add_reference_to_stage(usd_path=carter_asset, prim_path=carter_path)

# Initialize robot
carter = world.scene.add(Robot(prim_path=carter_path, name="carter"))

# Reset world
world.reset()

# Run simulation
for i in range(100):
    world.step(render=True)
```

### Loading Custom URDF

```python
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.robots import Robot
from omni.isaac.urdf import _urdf

# Enable URDF extension
enable_extension("omni.isaac.urdf")

# Import URDF
urdf_path = "/path/to/your/robot.urdf"
robot_path = "/World/MyRobot"

# Convert URDF to USD
success, robot_prim = _urdf.acquire_urdf_interface().parse_urdf(
    urdf_path,
    robot_path,
    {}
)

if success:
    print(f"Robot loaded at {robot_path}")
else:
    print("Failed to load URDF")
```

## ROS 2 Integration

### Enabling ROS 2 Bridge

```python
# In Isaac Sim Python console
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS2 bridge
enable_extension("omni.isaac.ros2_bridge")

# Verify
import rclpy
rclpy.init()
print("ROS 2 bridge active!")
```

### Publishing Clock

```python
# Enable ROS2 clock publisher
from omni.isaac.core import SimulationContext
simulation_context = SimulationContext()

# This automatically publishes /clock topic
simulation_context.initialize_physics()
```

### Camera Publisher Example

```python
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.ros2_bridge import Camera

# Enable extension
enable_extension("omni.isaac.ros2_bridge")

# Create camera and publish to ROS 2
camera = Camera(
    prim_path="/World/Camera",
    position=[2.0, 2.0, 1.5],
    target=[0, 0, 0],
    resolution=(640, 480),
    frequency=30
)

# Topics published:
# - /rgb (sensor_msgs/Image)
# - /camera_info (sensor_msgs/CameraInfo)
```

## Standalone Python Script

```python
#!/usr/bin/env python3
"""
Standalone Isaac Sim script with ROS 2 integration
Run: ~/.local/share/ov/pkg/isaac_sim-*/python.sh this_script.py
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
import rclpy

# Enable ROS2
enable_extension("omni.isaac.ros2_bridge")
rclpy.init()

# Create world
world = World()

# Load robot
carter_path = "/World/Carter"
carter_asset = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1/Isaac/Robots/Carter/carter_v1.usd"
add_reference_to_stage(usd_path=carter_asset, prim_path=carter_path)

carter = world.scene.add(Robot(prim_path=carter_path, name="carter"))

# Reset
world.reset()

# Simulation loop
print("Running simulation...")
for i in range(1000):
    world.step(render=True)

    if i % 100 == 0:
        print(f"Step {i}/1000")

# Cleanup
simulation_app.close()
rclpy.shutdown()
```

## Running Isaac Sim Examples

```bash
# Run Isaac Sim GUI
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh

# Run standalone script
~/.local/share/ov/pkg/isaac_sim-*/python.sh my_script.py

# Run with ROS 2 workspace sourced
source ~/ros2_ws/install/setup.bash
~/.local/share/ov/pkg/isaac_sim-*/python.sh ros2_script.py

# Headless mode (no GUI, faster)
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh --headless
```

## Exercises

### Exercise 1: Navigate the Interface
Load Isaac Sim, explore the interface, and load 3 different robots from the asset library.

### Exercise 2: Camera Setup
Create a scene with a robot and position a camera to view it from multiple angles.

### Exercise 3: ROS 2 Topics
Enable ROS 2 bridge and use `ros2 topic list` to see available topics.

### Exercise 4: Custom Environment
Build a simple environment with obstacles and lighting using USD primitives.

### Exercise 5: Record Data
Set up a camera and record RGB images to disk for dataset creation.

## Common Issues

### Issue 1: Black screen on startup
**Solution**: Update NVIDIA drivers to 525+ and ensure GPU is RTX series.

```bash
nvidia-smi  # Check driver version
sudo apt install nvidia-driver-535  # Update if needed
```

### Issue 2: "No module named 'omni.isaac'"
**Solution**: Use Isaac Sim's Python interpreter, not system Python.

```bash
# Correct
~/.local/share/ov/pkg/isaac_sim-*/python.sh script.py

# Wrong
python3 script.py  # Missing Isaac Sim modules
```

### Issue 3: ROS 2 bridge not working
**Solution**: Source ROS 2 workspace before launching Isaac Sim.

```bash
source /opt/ros/humble/setup.bash
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
```

### Issue 4: Out of VRAM
**Solution**: Reduce resolution, disable RTX, or use headless mode.

```python
simulation_app = SimulationApp({
    "headless": True,
    "width": 640,
    "height": 480
})
```

## Performance Optimization

```python
# Reduce rendering quality for faster simulation
from omni.isaac.core.utils.carb import set_carb_setting

# Disable RTX
set_carb_setting("/rtx/rendermode", "PathTracing")  # or "RayTracedLighting"

# Reduce physics substeps
world.get_physics_context().set_solver_iterations(4)  # Default: 16

# Lower render FPS
set_carb_setting("/app/runLoops/main/rateLimitEnabled", True)
set_carb_setting("/app/runLoops/main/rateLimitFrequency", 30)
```

## Key Takeaways

1. **GPU Required**: Isaac Sim needs NVIDIA RTX GPU for best performance
2. **USD Format**: Universal Scene Description is the core scene format
3. **Python API**: Extensive Python API for automation
4. **ROS 2 Integration**: Seamless bridge to ROS 2 ecosystem
5. **Photorealistic**: RTX rendering for synthetic data generation

## Next Steps

- **Example 02**: Visual SLAM with Isaac ROS
- **Example 03**: Nav2 navigation in Isaac Sim
- **Module 03 Chapter 03**: Synthetic data generation

## References

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac Sim Python API](https://docs.omniverse.nvidia.com/py/isaacsim/)
- [Isaac ROS](https://nvidia-isaac-ros.github.io/)
- [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
