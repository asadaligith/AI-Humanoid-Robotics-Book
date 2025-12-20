---
title: 'Chapter 4: Visual SLAM'
description: 'Implement Visual SLAM for robot localization using Isaac ROS'
sidebar_position: 4
---

# Chapter 4: Visual SLAM

## Learning Objectives

1. **Implement** Isaac ROS Visual SLAM
2. **Tune** VSLAM parameters for humanoid robots
3. **Visualize** odometry and map in RViz2

## Isaac ROS VSLAM

GPU-accelerated Visual SLAM using stereo or RGB-D cameras.

**Launch Example**:
```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**Topics**:
- Input: `/camera/image_raw`, `/camera/depth/image_raw`
- Output: `/visual_slam/tracking/odometry`, `/visual_slam/tracking/vo_pose`

**Code Example**: See `examples/module-03-isaac-brain/example-01-vslam/`

**Next**: [Chapter 5: Object Pose Estimation](./chapter-05-perception.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 60 minutes
