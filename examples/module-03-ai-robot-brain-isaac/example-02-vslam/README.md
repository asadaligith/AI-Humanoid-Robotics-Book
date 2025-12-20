# Example 02: Visual SLAM with Isaac ROS

## Learning Objectives

By completing this example, you will:
- Understand Visual SLAM (Simultaneous Localization and Mapping)
- Set up Isaac ROS Visual SLAM package
- Process stereo camera data for SLAM
- Generate 3D maps of environments
- Integrate SLAM with Nav2 for navigation

## Overview

Visual SLAM uses camera data to build a map of the environment while simultaneously tracking the robot's position. Isaac ROS provides GPU-accelerated VSLAM using NVIDIA's cuVSLAM algorithm.

**Key Concepts:**
- **Localization**: Determining robot position in a map
- **Mapping**: Building a representation of the environment
- **Loop Closure**: Recognizing previously visited locations
- **Odometry**: Estimating robot motion from sensors
- **Point Cloud**: 3D representation of environment features

## Prerequisites

- NVIDIA GPU with CUDA support
- Isaac Sim installed (Example 01)
- ROS 2 Humble
- Isaac ROS packages installed
- Completed Module 03 Chapter 04

## Installation

```bash
# Install Isaac ROS Visual SLAM
sudo apt install ros-humble-isaac-ros-visual-slam -y

# Install dependencies
sudo apt install -y \
  ros-humble-vision-opencv \
  ros-humble-image-transport \
  ros-humble-cv-bridge

# Install visualization tools
sudo apt install ros-humble-rviz2 ros-humble-rqt-image-view -y
```

## System Architecture

```
┌─────────────┐      ┌──────────────┐      ┌─────────────┐
│ Stereo      │─────→│  Isaac ROS   │─────→│ Map &       │
│ Camera      │      │  cuVSLAM     │      │ Odometry    │
└─────────────┘      └──────────────┘      └─────────────┘
                            │
                            ↓
                     ┌──────────────┐
                     │   RViz2      │
                     │ Visualization│
                     └──────────────┘
```

## Isaac Sim Setup with Stereo Camera

### Create Scene with Carter Robot

```python
#!/usr/bin/env python3
"""
Isaac Sim scene with Carter robot and stereo camera for VSLAM
Run: ~/.local/share/ov/pkg/isaac_sim-*/python.sh vslam_scene.py
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.range_sensor import _range_sensor
import numpy as np

# Enable ROS2
enable_extension("omni.isaac.ros2_bridge")

# Create world with physics
world = World()

# Load Carter robot (has stereo camera)
carter_asset = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1/Isaac/Robots/Carter/carter_v1.usd"
add_reference_to_stage(usd_path=carter_asset, prim_path="/World/Carter")

# Add environment (warehouse)
warehouse_asset = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
add_reference_to_stage(usd_path=warehouse_asset, prim_path="/World/Warehouse")

# Configure stereo camera publishing
# Cameras automatically publish to:
# - /front_stereo_camera/left/image_raw
# - /front_stereo_camera/right/image_raw
# - /front_stereo_camera/left/camera_info
# - /front_stereo_camera/right/camera_info

world.reset()

print("Isaac Sim scene ready for Visual SLAM")
print("Topics publishing:")
print("  - /front_stereo_camera/left/image_raw")
print("  - /front_stereo_camera/right/image_raw")

# Run simulation
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

## Running Visual SLAM

### Launch File (vslam.launch.py)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Visual SLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'denoise_input_images': False,
                'rectified_images': True,
                'enable_debug_mode': False,
                'debug_dump_path': '/tmp/cuvslam',
                'enable_slam_visualization': True,
                'enable_landmarks_view': True,
                'enable_observations_view': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'input_left_camera_frame': 'left_camera',
                'input_right_camera_frame': 'right_camera',
                'enable_localization_n_mapping': True,
                'enable_imu_fusion': False,
            }],
            remappings=[
                ('stereo_camera/left/image', '/front_stereo_camera/left/image_raw'),
                ('stereo_camera/left/camera_info', '/front_stereo_camera/left/camera_info'),
                ('stereo_camera/right/image', '/front_stereo_camera/right/image_raw'),
                ('stereo_camera/right/camera_info', '/front_stereo_camera/right/camera_info'),
            ]
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/vslam_config.rviz']
        ),
    ])
```

### Running the System

```bash
# Terminal 1: Start Isaac Sim with robot
~/.local/share/ov/pkg/isaac_sim-*/python.sh vslam_scene.py

# Terminal 2: Launch Visual SLAM
source /opt/ros/humble/setup.bash
ros2 launch isaac_ros_visual_slam vslam.launch.py

# Terminal 3: Monitor topics
ros2 topic list | grep visual_slam
# Output:
# /visual_slam/tracking/odometry
# /visual_slam/tracking/slam_path
# /visual_slam/tracking/vo_path
# /visual_slam/vis/landmarks_cloud
# /visual_slam/vis/loop_closure_cloud
# /visual_slam/vis/observations_cloud
# /visual_slam/status
```

## RViz Configuration

### Add VSLAM Visualizations

```yaml
# vslam_config.rviz
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree:
      - Class: rviz_default_plugins/Grid
        Name: Grid
        Reference Frame: map

      - Class: rviz_default_plugins/TF
        Name: TF
        Frames:
          All Enabled: true

      - Class: rviz_default_plugins/Path
        Name: SLAM Path
        Topic: /visual_slam/tracking/slam_path
        Color: 255; 0; 0

      - Class: rviz_default_plugins/Path
        Name: VO Path
        Topic: /visual_slam/tracking/vo_path
        Color: 0; 255; 0

      - Class: rviz_default_plugins/PointCloud2
        Name: Landmarks
        Topic: /visual_slam/vis/landmarks_cloud
        Size: 5
        Color: 255; 255; 255

      - Class: rviz_default_plugins/Image
        Name: Left Camera
        Topic: /front_stereo_camera/left/image_raw

Global Options:
  Fixed Frame: map
```

## Monitoring SLAM Performance

```bash
# Check odometry output
ros2 topic echo /visual_slam/tracking/odometry

# Check SLAM status
ros2 topic echo /visual_slam/status

# Monitor processing rate
ros2 topic hz /visual_slam/tracking/odometry

# Check TF transforms
ros2 run tf2_ros tf2_echo map base_link

# View landmark count
ros2 topic echo /visual_slam/vis/landmarks_cloud --field width
```

## Saving and Loading Maps

### Save Map

```bash
# SLAM map is automatically saved internally by cuVSLAM
# To export for Nav2, use:
ros2 service call /visual_slam/save_map \
  isaac_ros_visual_slam_interfaces/srv/SaveMap \
  "{map_url: '/tmp/vslam_map.dat'}"
```

### Load Map for Localization

```python
# In launch file, set parameters:
parameters=[{
    'enable_localization_n_mapping': False,  # Localization only
    'enable_localization_only': True,
    'map_file_path': '/tmp/vslam_map.dat',
}]
```

## Integrating with Nav2

### TF Tree for Navigation

```
map
 └─ odom (from Visual SLAM)
     └─ base_link
         ├─ left_camera
         ├─ right_camera
         └─ laser (if present)
```

### Nav2 Integration Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # Visual SLAM for localization
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_localization_n_mapping': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }]
        ),

        # Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'True',
            }.items()
        ),
    ])
```

## Performance Tuning

```python
# Reduce processing load
parameters=[{
    'override_publishing_stamp': False,
    'image_jitter_threshold_ms': 33.33,  # 30 FPS max
    'num_cameras': 2,  # Stereo
    'min_num_images': 4,
    'max_num_images': 20,
    'enable_verbosity': False,
}]
```

## Exercises

### Exercise 1: Map a Room
Use Visual SLAM to map a room in Isaac Sim, save the map, and reload for localization-only mode.

### Exercise 2: Loop Closure
Create a path that loops back to the start and observe loop closure detection.

### Exercise 3: Compare VSLAM vs Lidar SLAM
Run both visual SLAM and lidar-based SLAM (e.g., SLAM Toolbox) and compare accuracy.

### Exercise 4: IMU Fusion
Enable IMU fusion and observe improved odometry during fast motions.

### Exercise 5: Multi-Floor Mapping
Map a multi-story environment and handle vertical loop closures.

## Common Issues

### Issue 1: Poor tracking in texture-less environments
**Solution**: Add visual features or use combined VSLAM + Lidar.

### Issue 2: High GPU usage
**Solution**: Reduce image resolution or frame rate.

```python
# Lower camera resolution
camera.set_resolution((640, 480))  # Instead of (1280, 720)
```

### Issue 3: Drift over time
**Solution**: Enable loop closure detection and ensure good lighting.

### Issue 4: "No valid camera info"
**Solution**: Verify camera_info topics are publishing calibration data.

```bash
ros2 topic echo /front_stereo_camera/left/camera_info
```

## Key Takeaways

1. **GPU Acceleration**: cuVSLAM runs on GPU for real-time performance
2. **Stereo Vision**: Requires calibrated stereo camera pair
3. **Feature-Rich Environments**: Works best with textured scenes
4. **Loop Closure**: Essential for large-scale mapping
5. **Nav2 Compatible**: Provides map→odom transform for navigation

## Next Steps

- **Example 03**: Nav2 navigation with Visual SLAM
- **Module 03 Chapter 05**: Perception pipeline
- **Module 04**: Vision-Language-Action integration

## References

- [Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/)
- [cuVSLAM Documentation](https://docs.nvidia.com/isaac/packages/visual_slam/doc/cuvslam.html)
- [ROS 2 SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Visual SLAM Algorithms](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
