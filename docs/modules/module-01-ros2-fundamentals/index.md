---
title: 'Module 1: ROS 2 Fundamentals'
description: 'Master the foundation of ROS 2 for humanoid robotics - nodes, topics, services, actions, and robot modeling'
sidebar_position: 1
---

# Module 1: ROS 2 Fundamentals

## Module Overview

This module introduces the Robot Operating System 2 (ROS 2) Humble Hawksbill, the foundational middleware for building distributed robotics systems. You'll learn core ROS 2 concepts through hands-on examples, preparing you to build communication layers for humanoid robots.

## Learning Objectives

By the end of this module, you will be able to:

1. **Create** ROS 2 nodes and packages for modular robotics applications
2. **Implement** publisher-subscriber patterns for sensor data communication
3. **Design** service and action servers for synchronous and asynchronous robot commands
4. **Model** humanoid robots using URDF (Unified Robot Description Format)
5. **Manage** coordinate transformations with TF2 for spatial reasoning
6. **Apply** ROS 2 best practices for maintainable robotics code

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **Programming**: Python 3.10+ (intermediate level) OR C++17 (basic level)
- **Concepts**: Basic understanding of distributed systems, message passing
- **Hardware**: x86_64 computer with 8GB+ RAM (no GPU required for this module)

## Module Structure

| Chapter | Topic | Code Example | Estimated Time |
|---------|-------|--------------|----------------|
| [Chapter 1](./chapter-01-introduction.md) | Introduction to ROS 2 | Setup verification | 30 min |
| [Chapter 2](./chapter-02-pubsub.md) | Publisher-Subscriber Pattern | Sensor data logger | 1 hour |
| [Chapter 3](./chapter-03-packages.md) | Creating ROS 2 Packages | Custom robot controller | 1.5 hours |
| [Chapter 4](./chapter-04-services.md) | Services for Synchronous Communication | Request-response pattern | 1 hour |
| [Chapter 5](./chapter-05-actions.md) | Actions for Long-Running Tasks | Goal-oriented execution | 1.5 hours |
| [Chapter 6](./chapter-06-urdf-tf.md) | Robot Modeling with URDF & TF2 | Humanoid robot description | 2 hours |
| [Chapter 7](./chapter-07-summary.md) | Module Summary & Integration | N/A | 30 min |

**Total Estimated Time**: 8-10 hours

## Real-World Application

The concepts in this module directly support the capstone project:
- **Pub/Sub**: Sensor data flows (camera, LiDAR, IMU) to perception nodes
- **Services**: Object detection queries, grasp planning requests
- **Actions**: Navigation goals, manipulation trajectories
- **URDF/TF**: Humanoid robot model for simulation and motion planning

## Resources

### Official Documentation
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Design Decisions](https://design.ros2.org/)

### Recommended Reading
- *Programming Robots with ROS* by Morgan Quigley et al. (O'Reilly, 2015)
- *A Gentle Introduction to ROS* by Jason M. O'Kane (University of South Carolina)

### Community
- [ROS Discourse](https://discourse.ros.org/) - Official forum for questions
- [ROS 2 GitHub](https://github.com/ros2) - Source code and issue tracking
- [r/ROS on Reddit](https://www.reddit.com/r/ROS/) - Community discussions

## Next Steps

Start with [Chapter 1: Introduction to ROS 2](./chapter-01-introduction.md) to set up your development environment and verify your installation.

---

**Note**: This module uses ROS 2 Humble exclusively. Other ROS 2 distributions (Foxy, Galactic, Rolling) are not supported due to API differences.
