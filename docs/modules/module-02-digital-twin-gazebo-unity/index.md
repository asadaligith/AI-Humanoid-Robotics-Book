---
title: 'Module 2: Digital Twin - Gazebo & Unity'
description: 'Build realistic simulation environments for humanoid robots using Gazebo Fortress and Unity'
sidebar_position: 2
---

# Module 2: Digital Twin - Gazebo & Unity

## Module Overview

This module teaches you to create digital twins—virtual replicas of physical robots—using Gazebo Fortress for physics simulation and Unity for photorealistic visualization. You'll simulate humanoid robots in realistic environments before deploying to hardware.

## Learning Objectives

By the end of this module, you will be able to:

1. **Load** URDF robot models into Gazebo Fortress simulation
2. **Configure** physics parameters for realistic robot behavior
3. **Integrate** virtual sensors (cameras, LiDAR, IMU) with ROS 2
4. **Create** simulation worlds for testing robot algorithms
5. **Connect** Unity for high-fidelity visualization using ROS-TCP Connector
6. **Apply** domain randomization for robust perception training

## Prerequisites

- **Module 1**: ROS 2 fundamentals (nodes, topics, URDF)
- **Operating System**: Ubuntu 22.04 LTS
- **Hardware**: 8GB+ RAM, dedicated GPU recommended (for Unity/rendering)
- **Software**: Gazebo Fortress, Unity 2022.3 LTS (optional)

## Module Structure

| Chapter | Topic | Simulation Environment | Estimated Time |
|---------|-------|------------------------|----------------|
| [Chapter 1](./chapter-01-introduction.md) | Introduction to Digital Twins | N/A | 30 min |
| [Chapter 2](./chapter-02-gazebo-basics.md) | Gazebo Fortress Basics | Empty world | 1.5 hours |
| [Chapter 3](./chapter-03-urdf-gazebo.md) | Loading URDF in Gazebo | Humanoid spawn | 1.5 hours |
| [Chapter 4](./chapter-04-sensors.md) | Virtual Sensors Integration | Sensor-equipped robot | 2 hours |
| [Chapter 5](./chapter-05-unity-ros.md) | Unity Visualization with ROS | Unity scene | 2 hours |
| [Chapter 6](./chapter-06-physics-tuning.md) | Physics Parameter Tuning | Locomotion testing | 1.5 hours |
| [Chapter 7](./chapter-07-summary.md) | Module Summary | N/A | 30 min |

**Total Estimated Time**: 9-11 hours

## Real-World Application

Digital twins enable:
- **Algorithm Testing**: Validate navigation and manipulation before hardware deployment
- **Failure Analysis**: Simulate edge cases (slippery floors, sensor failures)
- **Training Data Generation**: Synthetic images for perception model training
- **Human-Robot Interaction**: Visualize robot behavior for stakeholders

## Resources

- [Gazebo Documentation](https://gazebosim.org/docs/fortress)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-Gazebo Integration](https://classic.gazebosim.org/tutorials?tut=ros2_overview)

## Next Steps

Start with [Chapter 1: Introduction to Digital Twins](./chapter-01-introduction.md)

---

**Note**: Gazebo Fortress is required. Other versions (Garden, Harmonic) may have API differences.
