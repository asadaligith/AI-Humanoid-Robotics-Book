---
title: 'Chapter 1: Introduction to ROS 2'
description: 'Learn the fundamentals of ROS 2 architecture, install ROS 2 Humble, and verify your development environment'
sidebar_position: 1
---

# Chapter 1: Introduction to ROS 2

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Explain** the core architectural differences between ROS 1 and ROS 2
2. **Install** ROS 2 Humble Hawksbill on Ubuntu 22.04
3. **Verify** your ROS 2 installation by running demo nodes
4. **Navigate** the ROS 2 command-line interface (CLI)

## Introduction

The Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. Despite its name, ROS 2 is not an operating system but a middleware layer that provides hardware abstraction, device drivers, libraries, visualizers, message-passing, and package management for robotics applications[^1].

### Why ROS 2?

ROS 2 addresses critical limitations of ROS 1:
- **Real-time capabilities**: Deterministic communication via DDS (Data Distribution Service)
- **Security**: Built-in authentication and encryption (SROS2)
- **Multi-platform**: Windows, macOS, Linux, and embedded systems
- **Production-ready**: Used in commercial robots (e.g., Boston Dynamics Spot, NASA Mars rovers)

[^1]: ROS 2 Documentation. "About ROS 2." https://docs.ros.org/en/humble/index.html

## ROS 2 Architecture

### Communication Paradigms

ROS 2 supports three primary communication patterns:

1. **Topics** (Publish-Subscribe): Asynchronous, many-to-many data streaming
2. **Services** (Request-Response): Synchronous, one-to-one remote procedure calls
3. **Actions** (Goal-Oriented): Asynchronous, long-running tasks with feedback

### Computation Graph

A ROS 2 system consists of:
- **Nodes**: Independent processes performing computation
- **Topics**: Named buses for message exchange
- **Services**: Named endpoints for synchronous calls
- **Actions**: Named endpoints for goal-based tasks
- **Parameters**: Configuration values stored per node

```mermaid
graph LR
    A[Camera Node] -->|/image_raw| B[Perception Node]
    B -->|/detected_objects| C[Planning Node]
    C -->|/goal_pose| D[Navigation Node]
    D -->|/cmd_vel| E[Robot Driver]
```

## Installation

### Prerequisites

Ensure you have Ubuntu 22.04 LTS (Jammy Jellyfish) installed:

```bash
lsb_release -a
# Expected output: Ubuntu 22.04.x LTS
```

### Install ROS 2 Humble

Follow the official installation guide:

```bash
# Set up sources
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y
```

**Installation time**: ~10 minutes on typical broadband connection.

### Environment Setup

Add ROS 2 to your shell environment:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verification

### Test Installation

Run the talker-listener demo:

```bash
# Terminal 1: Start talker node
ros2 run demo_nodes_cpp talker

# Terminal 2: Start listener node (in a new terminal)
ros2 run demo_nodes_cpp listener
```

**Expected output** (Terminal 1):
```
[INFO] [talker]: Publishing: 'Hello World: 0'
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
```

**Expected output** (Terminal 2):
```
[INFO] [listener]: I heard: [Hello World: 0]
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
```

### Verify Core Tools

Check essential ROS 2 commands:

```bash
# Check ROS 2 version
ros2 --version
# Expected: ros2 cli version X.Y.Z

# List available packages
ros2 pkg list | head -10

# Check node graph
ros2 node list
# Expected: /talker and /listener (if demo running)
```

## ROS 2 CLI Basics

Essential commands for daily use:

| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 run` | Execute a node | `ros2 run demo_nodes_cpp talker` |
| `ros2 launch` | Run multiple nodes | `ros2 launch turtlesim multisim.launch.py` |
| `ros2 topic` | Inspect topics | `ros2 topic list` |
| `ros2 node` | Inspect nodes | `ros2 node info /talker` |
| `ros2 service` | Call services | `ros2 service call /add_two_ints` |
| `ros2 param` | Get/set parameters | `ros2 param get /talker use_sim_time` |

## Exercise

**Task**: Verify your ROS 2 installation by running the turtlesim demo and controlling the turtle.

**Steps**:
1. Install turtlesim: `sudo apt install ros-humble-turtlesim`
2. Run turtlesim node: `ros2 run turtlesim turtlesim_node`
3. In a new terminal, run teleop: `ros2 run turtlesim turtle_teleop_key`
4. Use arrow keys to move the turtle
5. Take a screenshot showing the turtle's path

**Bonus**: Use `ros2 topic echo /turtle1/pose` to see the turtle's position in real-time.

## Summary

In this chapter, you:
- Learned the differences between ROS 1 and ROS 2
- Installed ROS 2 Humble on Ubuntu 22.04
- Verified your installation with demo nodes
- Explored the ROS 2 CLI

**Next**: [Chapter 2: Publisher-Subscriber Pattern](./chapter-02-pubsub.md) - Learn how to create custom nodes that exchange data via topics.

## Further Reading

- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html)
- [ROS 2 vs ROS 1 Comparison](https://design.ros2.org/articles/changes.html)
- [DDS Specification](https://www.omg.org/spec/DDS/)

---

**Estimated Reading Time**: 20 minutes
**Hands-On Time**: 30 minutes
