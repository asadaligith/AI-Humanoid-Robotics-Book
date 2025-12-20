---
title: 'Chapter 2: Isaac Sim Setup'
description: 'Install and configure NVIDIA Isaac Sim for robot simulation'
sidebar_position: 2
---

# Chapter 2: Isaac Sim Setup

## Installation

1. **Download Omniverse Launcher**: https://www.nvidia.com/en-us/omniverse/download/
2. **Install Isaac Sim 2023.1+** via Launcher
3. **Install Isaac ROS**:

```bash
sudo apt install ros-humble-isaac-ros-visual-slam \
                 ros-humble-isaac-ros-apriltag \
                 ros-humble-isaac-ros-dnn-inference
```

## Verification

Launch Isaac Sim and load a demo scene to verify GPU acceleration is working.

**Code Example**: See `examples/module-03-isaac-brain/example-01-vslam/`

**Next**: [Chapter 3: Synthetic Data Generation](./chapter-03-synthetic-data.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 60 minutes
