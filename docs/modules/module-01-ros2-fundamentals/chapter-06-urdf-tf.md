---
title: 'Chapter 6: Robot Modeling with URDF & TF2'
description: 'Create humanoid robot descriptions using URDF and manage coordinate transformations with TF2'
sidebar_position: 6
---

# Chapter 6: Robot Modeling with URDF & TF2

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Create** URDF (Unified Robot Description Format) models for humanoid robots
2. **Visualize** robot models in RViz2
3. **Implement** TF2 transform broadcasters for coordinate frames
4. **Query** coordinate transformations for spatial reasoning

## Introduction

URDF is the standard format for describing robot geometry, kinematics, and visual properties in ROS[^1]. TF2 (Transform Library 2) manages coordinate frame transformations over time[^2].

[^1]: ROS Documentation. "URDF." http://wiki.ros.org/urdf
[^2]: ROS 2 Documentation. "tf2." https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html

## URDF Basics

### Robot Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint connecting base to head -->
  <joint name="base_to_head" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

### Link Components

- **Visual**: Appearance (geometry, color, textures)
- **Collision**: Simplified geometry for physics
- **Inertial**: Mass, center of mass, inertia tensor

### Joint Types

| Type | Description | Degrees of Freedom |
|------|-------------|--------------------|
| Fixed | Rigid connection | 0 |
| Revolute | Single-axis rotation | 1 (with limits) |
| Continuous | Unlimited rotation | 1 |
| Prismatic | Linear motion | 1 |
| Floating | Full 6-DOF | 6 |

## Visualizing URDF in RViz2

```bash
# Install required packages
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-xacro

# Launch visualization
ros2 launch urdf_tutorial display.launch.py model:=humanoid.urdf
```

## TF2 Transform Broadcasting

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
```

## Querying Transformations

```python
from tf2_ros import Buffer, TransformListener

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'camera_link',
                rclpy.time.Time()
            )
            self.get_logger().info(f'Transform: {transform.transform.translation}')
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')
```

## TF2 Tools

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo world base_link

# Visualize in RViz2
rviz2  # Add TF display
```

## Exercise

**Task**: Create a humanoid URDF with:
- Torso (base_link)
- Head (revolute joint, yaw rotation)
- Left and right arms (2 joints each: shoulder, elbow)
- Visualize in RViz2 with joint_state_publisher_gui

**Code Example**: See `examples/module-01-ros2-fundamentals/example-05-urdf/`

## Summary

URDF models define robot structure, while TF2 manages dynamic coordinate transformations.

**Next**: [Chapter 7: Module Summary](./chapter-07-summary.md)

---

**Estimated Reading Time**: 25 minutes
**Hands-On Time**: 60 minutes
