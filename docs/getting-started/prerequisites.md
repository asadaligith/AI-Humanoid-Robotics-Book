---
title: Prerequisites
description: System requirements and prerequisite knowledge for the AI & Humanoid Robotics course
sidebar_position: 1
---

# Prerequisites

Before starting this course, ensure you have the necessary hardware, software, and foundational knowledge.

## Hardware Requirements

### Minimum Configuration

**For Modules 1-2 (ROS 2 & Gazebo)**:
- CPU: Intel Core i5 (8th gen) or AMD Ryzen 5
- RAM: 8GB DDR4
- Storage: 50GB free SSD space
- GPU: Integrated graphics (Intel UHD 630 or equivalent)
- OS: Ubuntu 22.04 LTS (native or dual-boot recommended)

### Recommended Configuration

**For Modules 3-4 (Isaac Sim & VLA)**:
- CPU: Intel Core i7/i9 or AMD Ryzen 7/9
- RAM: 16GB+ DDR4
- Storage: 100GB free NVMe SSD
- GPU: **NVIDIA RTX 2060 or better** (6GB+ VRAM)
- OS: Ubuntu 22.04 LTS (native installation required for GPU)

### Optional Hardware

**For Capstone Deployment**:
- NVIDIA Jetson Orin Nano (8GB) - $499
- USB Camera (1080p, 30fps) - $30-50
- USB Microphone or headset - $20-40

## Software Requirements

### Operating System

**Primary**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- Native installation recommended (dual-boot or dedicated machine)
- WSL2 supported for Modules 1-2 only
- Virtual machines NOT recommended (GPU passthrough issues)

**Alternative**: Docker containers
- Provided for modules without GPU requirements
- See [Installation Guide](./installation.md) for details

### Required Tools

Install before starting the course:

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Essential build tools
sudo apt install -y build-essential cmake git curl wget

# Python 3.10+
sudo apt install -y python3 python3-pip python3-venv

# Version control
sudo apt install -y git git-lfs

# Text editor (choose one)
sudo apt install -y vim  # or
sudo snap install code --classic  # VS Code
```

## Prerequisite Knowledge

### Required Skills

**Programming**:
- ✅ Python: Functions, classes, basic OOP (intermediate level)
- ✅ Bash: Command line navigation, environment variables
- ⚠️ C++: Not required but helpful for advanced topics

**Mathematics**:
- ✅ Linear Algebra: Vectors, matrices, transformations
- ✅ Geometry: 3D coordinate systems, rotations, translations
- ⚠️ Calculus: Not required for this course

**Robotics Fundamentals**:
- ⚠️ Prior robotics experience: **Not required** (taught from basics)
- ⚠️ ROS 1 experience: **Not required** (ROS 2 differs significantly)

### Recommended Preparation

**If new to Python**:
1. [Python Official Tutorial](https://docs.python.org/3/tutorial/) - 8 hours
2. Practice: Classes, file I/O, command-line scripts

**If new to Linux/Ubuntu**:
1. [Ubuntu Desktop Guide](https://help.ubuntu.com/stable/ubuntu-help/) - 4 hours
2. Practice: Terminal commands, package management, file permissions

**If new to 3D Math**:
1. [3Blue1Brown Linear Algebra Series](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) - 3 hours
2. Focus: Vectors, dot product, matrix multiplication, transformations

## Verification Checklist

Before starting Module 1, verify your setup:

- [ ] Ubuntu 22.04 LTS installed (native or WSL2)
- [ ] Python 3.10+ installed (`python3 --version`)
- [ ] Git installed (`git --version`)
- [ ] 50GB+ free disk space (`df -h`)
- [ ] Stable internet connection (for package downloads)
- [ ] (Optional) NVIDIA GPU with latest drivers (`nvidia-smi`)

## Getting Help

**System Setup Issues**:
- [Ubuntu Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop)
- [NVIDIA Driver Installation](https://ubuntu.com/server/docs/nvidia-drivers-installation)

**Course-Specific Help**:
- [Troubleshooting Guide](../troubleshooting/common-issues.md)
- GitHub Discussions: [Link to repo discussions]

---

**Estimated Setup Time**: 2-4 hours (including OS installation)

**Next**: [Installation Guide](./installation.md) to set up ROS 2 and course dependencies
