# Example 05: URDF (Unified Robot Description Format)

## Learning Objectives

By completing this example, you will:
- Understand URDF structure and syntax
- Create robot models with links and joints
- Add visual and collision geometries
- Define inertial properties for physics simulation
- Visualize robots in RViz
- Use Xacro for parameterized URDF generation

## Overview

URDF (Unified Robot Description Format) is an XML format for describing robot kinematics, dynamics, visual representation, and collision models. It's the standard format for defining robot structures in ROS.

**Key Concepts:**
- **Link**: A rigid body (chassis, wheel, arm segment)
- **Joint**: Connection between two links (revolute, prismatic, fixed)
- **Visual**: Visual appearance for rendering
- **Collision**: Simplified geometry for collision detection
- **Inertial**: Mass and inertia properties for physics

## Prerequisites

- ROS 2 Humble installed
- Completed Examples 01-04
- Completed Module 01 Chapter 06 (first half)

## Directory Structure

```
example-05-urdf/
├── README.md                # This file
├── urdf/
│   ├── simple_robot.urdf    # Basic URDF example
│   ├── mobile_robot.urdf    # Mobile robot with wheels
│   └── robot.xacro          # Xacro macro version
├── launch/
│   └── display.launch.py    # RViz visualization launch file
└── config/
    └── view.rviz            # RViz configuration
```

## URDF Structure

### Basic URDF Syntax

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Wheel Link -->
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting base to wheel -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0.0 -0.1" rpy="0 1.57 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

## Link Components

### 1. Visual (Appearance)

```xml
<visual>
  <!-- Origin relative to link frame -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Geometry options: box, cylinder, sphere, mesh -->
  <geometry>
    <box size="1.0 0.5 0.2"/>
    <!-- OR -->
    <cylinder radius="0.1" length="0.5"/>
    <!-- OR -->
    <sphere radius="0.1"/>
    <!-- OR -->
    <mesh filename="package://my_robot/meshes/chassis.dae" scale="1.0 1.0 1.0"/>
  </geometry>

  <!-- Material color -->
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### 2. Collision (Physics)

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Usually simplified geometry for performance -->
    <box size="1.0 0.5 0.2"/>
  </geometry>
</collision>
```

### 3. Inertial (Dynamics)

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="10.0"/>

  <!-- Moment of inertia tensor -->
  <inertia ixx="0.1" ixy="0.0" ixz="0.0"
           iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

## Joint Types

### 1. Fixed Joint
```xml
<joint name="sensor_joint" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_link"/>
  <origin xyz="0.25 0 0.15" rpy="0 0 0"/>
</joint>
```

### 2. Revolute Joint (Rotational with limits)
```xml
<joint name="arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
</joint>
```

### 3. Continuous Joint (Rotational without limits)
```xml
<joint name="wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="wheel"/>
  <origin xyz="0.2 0.15 0" rpy="0 1.57 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### 4. Prismatic Joint (Linear)
```xml
<joint name="slider_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1.0"/>
</joint>
```

## Complete Mobile Robot Example

```xml
<?xml version="1.0"?>
<robot name="mobile_robot">

  <!-- Base Link (Chassis) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0.3 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.4"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.15 0.225 -0.05" rpy="0 1.57 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.15 -0.225 -0.05" rpy="0 1.57 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Caster Wheel (Simplified as Sphere) -->
  <link name="caster">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster"/>
    <origin xyz="-0.2 0 -0.15" rpy="0 0 0"/>
  </joint>

</robot>
```

## Visualizing URDF in RViz

### Launch File (`display.launch.py`)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_robot'),
        'urdf',
        'mobile_robot.urdf'
    ])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # Joint State Publisher GUI (for manual joint control)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('my_robot'),
                'config',
                'view.rviz'
            ])]
        )
    ])
```

### Running Visualization

```bash
# Install required packages
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2

# Launch visualization
ros2 launch my_robot display.launch.py

# Or manually
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat mobile_robot.urdf)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run rviz2 rviz2
```

## Using Xacro (XML Macros)

Xacro allows parameterized, reusable URDF generation.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">

  <!-- Parameters -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_length" value="0.05"/>
  <xacro:property name="chassis_width" value="0.4"/>

  <!-- Wheel Macro -->
  <xacro:macro name="wheel" params="name x y">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
        </geometry>
        <material name="black">
          <color rgba="0.2 0.2 0.2 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${name}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${name}"/>
      <origin xyz="${x} ${y} 0" rpy="0 1.57 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Base Link -->
  <link name="base_link">
    <!-- ... -->
  </link>

  <!-- Create wheels using macro -->
  <xacro:wheel name="left_wheel" x="0.15" y="${chassis_width/2 + 0.025}"/>
  <xacro:wheel name="right_wheel" x="0.15" y="${-(chassis_width/2 + 0.025)}"/>

</robot>
```

Convert Xacro to URDF:
```bash
xacro robot.xacro > robot.urdf
```

## Validation and Debugging

```bash
# Check URDF syntax
check_urdf mobile_robot.urdf

# View URDF as graph
urdf_to_graphiz mobile_robot.urdf

# List TF frames from URDF
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat mobile_robot.urdf)"
ros2 run tf2_tools view_frames
```

## Exercises

### Exercise 1: Add a Sensor
Add a lidar sensor link to the top of the robot chassis.

### Exercise 2: Create an Arm
Design a 2-DOF robotic arm with revolute joints and visualize it.

### Exercise 3: Inertia Calculation
Calculate correct inertial properties for a box: https://en.wikipedia.org/wiki/List_of_moments_of_inertia

### Exercise 4: Use Meshes
Replace primitive geometries with STL/DAE mesh files.

### Exercise 5: Xacro Parameterization
Create a Xacro file with configurable dimensions for a customizable robot.

## Common Issues

### Issue 1: "robot_description parameter not set"
**Solution**: Ensure robot_state_publisher receives the URDF via `robot_description` parameter.

### Issue 2: No robot visible in RViz
**Solution**: Add RobotModel display, set Fixed Frame to base_link or world.

### Issue 3: "Link ... has no inertia"
**Solution**: Add inertial properties to all links for Gazebo simulation.

### Issue 4: Xacro not found
**Solution**: Install xacro: `sudo apt install ros-humble-xacro`

## Best Practices

1. **Name Convention**: Use descriptive names (left_wheel, not wheel1)
2. **Collision Simplification**: Use simple shapes for collision (performance)
3. **Correct Inertias**: Calculate or estimate realistic inertial properties
4. **Materials**: Define reusable materials at the top
5. **Origin Frames**: Follow REP-103 (x forward, y left, z up)
6. **Xacro for Reusability**: Use macros for repeated elements

## Key Takeaways

1. **URDF is the Standard**: All ROS robots use URDF for description
2. **Links and Joints**: Understand the kinematic tree structure
3. **Three Representations**: Visual, collision, and inertial are all important
4. **Xacro Reduces Duplication**: Use macros for cleaner robot descriptions
5. **Validation is Critical**: Always validate URDF before simulation

## Next Steps

- **Example 06**: TF2 for coordinate transformations
- **Module 02**: Using URDF in Gazebo simulation
- **Custom Meshes**: Creating CAD models for robots

## References

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [URDF XML Specification](http://wiki.ros.org/urdf/XML)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [REP-103: Standard Units and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
