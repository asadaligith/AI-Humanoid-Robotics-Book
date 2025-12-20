---
title: 'Chapter 6: Jetson Deployment'
description: 'Deploy VLA pipeline on NVIDIA Jetson for real-time operation'
sidebar_position: 6
---

# Chapter 6: Jetson Deployment

## Learning Objectives

1. **Optimize** Whisper for Jetson
2. **Deploy** VLA pipeline on edge device
3. **Monitor** performance metrics

## Deployment Steps

1. **Install Dependencies** on Jetson Orin
2. **Copy VLA Package** via SCP
3. **Build Workspace**: `colcon build --symlink-install`
4. **Launch**: `ros2 launch vla_pipeline jetson_main.launch.py`

## Performance Optimization

- Use Whisper "tiny" model for faster inference
- Cache LLM responses for common commands
- Enable MAXN power mode: `sudo nvpmodel -m 0`

**Code Example**: See `scripts/jetson/deploy-capstone.sh`

**Next**: [Chapter 7: Module Summary](./chapter-07-summary.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 60 minutes
