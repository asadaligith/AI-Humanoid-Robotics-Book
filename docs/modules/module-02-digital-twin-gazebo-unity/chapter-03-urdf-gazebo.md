---
title: 'Chapter 3: Loading URDF in Gazebo'
description: 'Import ROS 2 URDF robot models into Gazebo Fortress simulation'
sidebar_position: 3
---

# Chapter 3: Loading URDF in Gazebo

## Learning Objectives

1. **Convert** URDF to SDF format for Gazebo compatibility
2. **Spawn** robot models in Gazebo using ROS 2
3. **Configure** Gazebo plugins for ROS 2 integration
4. **Test** robot visualization and basic movement

## URDF to Gazebo

URDF models need Gazebo-specific tags for physics and rendering[^1]:

```xml
<robot name="humanoid">
  <link name="base_link">
    <!-- Inertial properties (required for dynamics) -->
    <inertial>
      <mass value="50.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>

    <!-- Collision geometry -->
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>

    <!-- Visual geometry -->
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <!-- Gazebo material plugin -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
</robot>
```

[^1]: Gazebo Documentation. "URDF in Gazebo." http://classic.gazebosim.org/tutorials?tut=ros_urdf

## Spawning Robots with ROS 2

### Launch File Approach

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', 'empty.sdf'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'humanoid',
                '-file', '/path/to/humanoid.urdf',
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        )
    ])
```

### Command-Line Spawn

```bash
# Start Gazebo
gz sim empty.sdf &

# Spawn URDF robot
ros2 run ros_gz_sim create -file humanoid.urdf -name humanoid -x 0 -y 0 -z 0.5
```

## ROS-Gazebo Bridge

Connect ROS 2 topics to Gazebo:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge joint states
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
            ],
            output='screen'
        )
    ])
```

## Exercise

**Task**: Load your Module 1 humanoid URDF into Gazebo:
1. Add inertial properties to all links
2. Add Gazebo material tags
3. Create launch file to spawn robot
4. Verify robot appears in Gazebo and RViz2 simultaneously

**Code Example**: See `examples/module-02-digital-twin/example-01-gazebo-humanoid/`

## Summary

URDF robots load into Gazebo with proper inertial tags and ROS-Gazebo bridging.

**Next**: [Chapter 4: Virtual Sensors Integration](./chapter-04-sensors.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 45 minutes
