---
title: 'Chapter 4: Virtual Sensors Integration'
description: 'Add cameras, LiDAR, and IMU sensors to simulated robots'
sidebar_position: 4
---

# Chapter 4: Virtual Sensors Integration

## Learning Objectives

1. **Add** depth cameras, LiDAR, and IMU sensors to URDF models
2. **Configure** sensor properties (resolution, range, noise)
3. **Publish** sensor data to ROS 2 topics
4. **Visualize** sensor outputs in RViz2

## Gazebo Sensor Plugins

### Depth Camera

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth_camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/image_raw:=camera/image_raw</remapping>
        <remapping>~/depth/image_raw:=camera/depth/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR (2D Laser Scanner)

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </lidar>
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### IMU (Inertial Measurement Unit)

```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </x>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Verifying Sensor Data

```bash
# List sensor topics
ros2 topic list | grep -E "camera|scan|imu"

# Echo depth image metadata
ros2 topic echo /robot/camera/depth/image_raw --once

# Measure LiDAR frequency
ros2 topic hz /robot/scan

# Visualize in RViz2
rviz2
# Add DepthCloud, LaserScan, and Imu displays
```

## Exercise

**Task**: Add sensors to your humanoid:
- Depth camera on head (30 Hz, 640x480)
- LiDAR on torso (10 Hz, 360 samples)
- IMU on base (100 Hz)

Verify all three publish data and visualize in RViz2.

**Code Example**: See `examples/module-02-digital-twin/example-02-sensors/`

## Summary

Gazebo sensor plugins publish synthetic data to ROS 2 topics for algorithm testing.

**Next**: [Chapter 5: Unity Visualization with ROS](./chapter-05-unity-ros.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 50 minutes
