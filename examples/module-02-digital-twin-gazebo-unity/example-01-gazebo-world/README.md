# Example 01: Creating Gazebo Worlds

## Learning Objectives

By completing this example, you will:
- Create custom Gazebo world files (SDF format)
- Add models, lighting, and physics properties
- Configure camera views and environmental settings
- Launch Gazebo worlds from ROS 2
- Understand the relationship between SDF and URDF

## Overview

Gazebo uses SDF (Simulation Description Format) to define simulation worlds. A world file contains the environment, physics engine settings, lighting, and model placements.

**Key Concepts:**
- **World**: The complete simulation environment
- **Model**: Objects in the world (robots, obstacles, furniture)
- **Plugin**: Extends Gazebo functionality (sensors, controllers)
- **Physics Engine**: Simulates dynamics (ODE, Bullet, Simbody)
- **Lighting**: Ambient, directional, point, and spot lights

## Prerequisites

- ROS 2 Humble installed
- Gazebo Fortress installed
- Completed Module 01
- Completed Module 02 Chapter 02

## Directory Structure

```
example-01-gazebo-world/
├── README.md
├── worlds/
│   ├── empty_world.sdf
│   ├── simple_room.sdf
│   └── warehouse.sdf
├── models/
│   └── custom_box/
│       ├── model.config
│       └── model.sdf
└── launch/
    └── gazebo.launch.py
```

## Basic World File Structure

### Minimal World (empty_world.sdf)

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="empty_world">

    <!-- Physics Engine Configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Plugins for ROS 2 integration -->
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>

    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>

    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Adding Models to the World

### Simple Room World

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="simple_room">

    <!-- Include basic world settings (physics, plugins, sun) -->
    <include>
      <uri>file://empty_world.sdf</uri>
    </include>

    <!-- Ground with grid pattern -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Wall 1 (North) -->
    <model name="wall_north">
      <static>true</static>
      <pose>5 0 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 10 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 10 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
            <diffuse>0.9 0.9 0.9 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Wall 2 (South) -->
    <model name="wall_south">
      <static>true</static>
      <pose>-5 0 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 10 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 10 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle Box -->
    <model name="obstacle_box">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.16</ixx>
            <iyy>0.16</iyy>
            <izz>0.16</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Table -->
    <model name="table">
      <pose>-2 -2 0.4 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="surface">
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
        </collision>
        <visual name="surface">
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Using Gazebo Model Database

```xml
<!-- Include models from Gazebo model database -->
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table</uri>
  <pose>0 0 0 0 0 0</pose>
</include>

<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bookshelf</uri>
  <pose>3 3 0 0 0 1.57</pose>
</include>
```

## Physics Configuration

```xml
<physics name="fast" type="ode">
  <!-- Time step (smaller = more accurate, slower) -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time factor (1.0 = real-time, >1 = faster) -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Solver iterations (higher = more stable) -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>

<!-- Gravity -->
<gravity>0 0 -9.81</gravity>

<!-- Magnetic field -->
<magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>

<!-- Atmosphere -->
<atmosphere type="adiabatic"/>
```

## Launch File Integration

### ROS 2 Launch File (gazebo.launch.py)

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to world file
    world_file = os.path.join(
        get_package_share_directory('my_simulation'),
        'worlds',
        'simple_room.sdf'
    )

    # Launch Gazebo with the world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
    ])
```

## Running Gazebo Worlds

### Command Line

```bash
# Launch empty world
gz sim empty_world.sdf

# Launch with GUI
gz sim simple_room.sdf

# Launch without GUI (headless)
gz sim simple_room.sdf -s

# Launch paused
gz sim simple_room.sdf --pause

# Launch with specific server/client
gz sim simple_room.sdf -v 4  # Verbose logging
```

### From ROS 2

```bash
# Using launch file
ros2 launch my_simulation gazebo.launch.py

# Direct execution
ros2 run ros_gz_sim create -world simple_room
```

## Custom Models

### Model Directory Structure

```
custom_box/
├── model.config    # Metadata
└── model.sdf       # Model definition
```

### model.config

```xml
<?xml version="1.0"?>
<model>
  <name>Custom Box</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>you@example.com</email>
  </author>
  <description>
    A custom colored box model
  </description>
</model>
```

### model.sdf

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="custom_box">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.083</ixx>
          <iyy>0.083</iyy>
          <izz>0.083</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.8 0.2 1</ambient>
          <diffuse>0.2 0.8 0.2 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

## Exercises

### Exercise 1: Warehouse Environment
Create a warehouse world with multiple shelves, pallets, and navigation obstacles.

### Exercise 2: Outdoor Terrain
Build an outdoor environment with uneven terrain, trees, and natural lighting.

### Exercise 3: Dynamic Objects
Add movable objects (balls, boxes) with realistic physics and friction.

### Exercise 4: Multi-Room Environment
Create a multi-room apartment with doors, furniture, and different floor materials.

### Exercise 5: Performance Optimization
Optimize a complex world by adjusting physics parameters and using simplified collision meshes.

## Common Issues

### Issue 1: Models not loading
**Solution**: Set `GZ_SIM_RESOURCE_PATH` environment variable to model directory.

```bash
export GZ_SIM_RESOURCE_PATH=/path/to/models:$GZ_SIM_RESOURCE_PATH
```

### Issue 2: Poor performance
**Solution**: Reduce physics iterations, increase max_step_size, simplify collision geometries.

### Issue 3: Objects falling through ground
**Solution**: Ensure ground plane has collision geometry and proper physics properties.

### Issue 4: Dark scene
**Solution**: Add more lights or increase ambient light intensity.

## Best Practices

1. **Modular Design**: Separate models into reusable components
2. **Performance**: Use static models for non-moving objects
3. **Collision Simplification**: Use simple shapes for collision detection
4. **Naming Convention**: Use descriptive, unique model names
5. **Version Control**: Specify SDF version explicitly
6. **Physics Tuning**: Balance accuracy and performance

## Key Takeaways

1. **SDF Format**: Standard for Gazebo world and model description
2. **Physics Matters**: Configure physics engine for simulation needs
3. **Lighting is Critical**: Proper lighting improves visualization
4. **Model Reusability**: Create custom models for repeated use
5. **ROS 2 Integration**: Use ros_gz_bridge for ROS communication

## Next Steps

- **Example 02**: Spawning robots in Gazebo
- **Example 03**: Adding sensors to models
- **Module 02 Chapter 03**: Advanced Gazebo features

## References

- [Gazebo SDF Specification](https://gazebosim.org/api/sim/7/sdf.html)
- [Gazebo Tutorials](https://gazebosim.org/docs/fortress/tutorials)
- [Gazebo Fuel Models](https://app.gazebosim.org/fuel/models)
- [ROS 2 Gazebo Integration](https://github.com/gazebosim/ros_gz)
