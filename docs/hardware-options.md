---
sidebar_position: 5
---

# Hardware Options

This page provides guidance on selecting hardware for running the AI Humanoid Robotics Book examples, from simulation-only setups to real robot deployment.

## Overview

You can complete this entire book using **simulation only** on a standard desktop or laptop. Physical hardware is **optional** and only required if you want to deploy to real robots.

### Hardware Tiers

| Tier | Use Case | Estimated Cost | Performance |
|------|----------|----------------|-------------|
| **Minimum** | Simulation only (Gazebo) | $500-800 | Basic ROS 2, limited AI |
| **Recommended** | Full simulation (Isaac Sim + AI) | $1,200-1,800 | All modules, good performance |
| **Embedded** | Real robot deployment | $400-600 (compute only) | Edge inference, mobile |
| **Cloud** | No local hardware | $0.50-2.00/hour | Scalable, pay-per-use |

## Recommended Configuration (From Spec)

This is the **baseline configuration** used for all book examples and verified through clarifications:

### Desktop/Laptop

**CPU**: 6-core processor or better
- Intel Core i5-12400 / i5-13400
- AMD Ryzen 5 5600X / 5600
- Base clock: 3.0+ GHz

**GPU**: NVIDIA RTX 3060 6GB or equivalent
- CUDA Compute Capability: 8.6+
- VRAM: 6GB minimum for Isaac Sim
- Ray tracing cores (useful for photorealistic rendering)

**Memory**: 16GB DDR4/DDR5
- 3200MHz or faster recommended
- Dual-channel configuration

**Storage**: 100GB+ free SSD space
- NVMe SSD strongly recommended for Isaac Sim
- SATA SSD acceptable for Gazebo-only workflows

**Operating System**: Ubuntu 22.04 LTS
- Native installation (dual-boot or dedicated machine)
- WSL2 on Windows 11 (limited support, simulation only)

### Why This Configuration?

**From Clarification Session 2025-12-07**:
> Q: What are the recommended hardware specifications for running simulations?
> A: **Mid-range desktop with 16GB RAM, RTX 3060, 6-core CPU**

**Rationale**:
- **6-core CPU**: Handles ROS 2 nodes, simulation physics, AI inference concurrently
- **RTX 3060 6GB**: Minimum for Isaac Sim + real-time object detection
- **16GB RAM**: Sufficient for ROS 2 workspace + Gazebo/Isaac Sim + models
- **100GB SSD**: ROS 2 (10GB) + Isaac Sim (50GB) + datasets (20GB) + workspace (20GB)

## Minimum Configuration (Budget)

For students with budget constraints, this setup runs **Gazebo only** (no Isaac Sim):

**CPU**: 4-core processor
- Intel Core i3-12100 / AMD Ryzen 3 4100
- Integrated graphics acceptable

**GPU**: Not required (CPU fallback)
- Integrated Intel/AMD graphics
- NVIDIA GTX 1650 (optional, improves performance)

**Memory**: 8GB RAM
- Expect slower performance with large models

**Storage**: 50GB free SSD space

**Limitations**:
- ⚠️ Cannot run Isaac Sim (requires NVIDIA RTX GPU)
- ⚠️ Limited AI model training (use pre-trained models)
- ⚠️ Slower Gazebo rendering with complex scenes
- ✅ Can complete Modules 1-2 and most of Module 3

## Embedded Deployment (Real Robots)

For deploying to physical humanoid robots, use embedded computing platforms:

### NVIDIA Jetson Orin Nano 8GB

**Specifications**:
- **GPU**: 1024-core NVIDIA Ampere with 32 Tensor Cores
- **CPU**: 6-core ARM Cortex-A78AE (up to 2.0 GHz)
- **Memory**: 8GB LPDDR5 (128-bit)
- **Storage**: microSD (64GB+), NVMe SSD supported
- **Power**: 7W - 15W configurable
- **I/O**: GPIO, I2C, SPI, UART, USB 3.2, Gigabit Ethernet, CSI camera

**Cost**: ~$499 (Developer Kit)

**Use Case**:
- Onboard inference for object detection (COCO 20+ classes)
- VSLAM and Nav2 navigation
- Claude API calls (cloud-based task planning)
- Real-time sensor fusion

**Performance**:
- Object detection: 15-20 FPS (YOLOv8 medium)
- Navigation: Full Nav2 stack supported
- Voice transcription: Whisper base model (edge)

**Limitations**:
- Cannot train models (use pre-trained or cloud training)
- Limited simultaneous AI workloads
- Requires power optimization for battery operation

### Alternative: Raspberry Pi 5 (8GB)

**Specifications**:
- **CPU**: Quad-core ARM Cortex-A76 (2.4 GHz)
- **GPU**: VideoCore VII (no CUDA)
- **Memory**: 8GB LPDDR4X
- **Cost**: ~$80 (board only)

**Use Case**:
- Low-cost ROS 2 experiments
- Sensor data collection
- Basic navigation (no AI perception)

**Limitations**:
- ⚠️ No GPU acceleration for AI
- ⚠️ Cannot run object detection in real-time
- ⚠️ Limited to simple control tasks

## Cloud-Based Options

For students without suitable hardware, use cloud GPU instances:

### AWS EC2 G4dn Instances

**Recommended**: `g4dn.xlarge`
- **GPU**: NVIDIA T4 (16GB VRAM)
- **CPU**: 4 vCPUs (Intel Xeon)
- **Memory**: 16GB
- **Storage**: EBS (provision 100GB+)
- **Cost**: ~$0.526/hour on-demand (~$1.00/hour with full setup)

**Setup**:
```bash
# Launch Ubuntu 22.04 Deep Learning AMI
# Pre-installed: CUDA, cuDNN, Docker, NVIDIA drivers

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install Isaac Sim (via Omniverse)
# Follow NVIDIA Isaac Sim installation guide
```

**Estimated Monthly Cost** (part-time usage):
- 20 hours/month: ~$20
- 40 hours/month: ~$40

### Google Cloud Platform (GCP)

**Recommended**: `n1-standard-4` + NVIDIA T4
- Similar specs to AWS G4dn
- Cost: ~$0.35/hour (compute) + $0.35/hour (GPU) = $0.70/hour
- Preemptible instances: ~$0.25/hour (can be interrupted)

### Paperspace Gradient

**Recommended**: P4000 or RTX 4000 instances
- **Cost**: $0.51/hour (P4000), $0.76/hour (RTX 4000)
- **Benefit**: Jupyter notebook interface, easier for beginners
- **Drawback**: Less control than AWS/GCP

### Cloud Limitations

- ⚠️ No persistent state (save work frequently)
- ⚠️ Network latency for remote desktop
- ⚠️ Data transfer costs for large datasets
- ✅ No upfront hardware investment
- ✅ Access to powerful GPUs on-demand

## Component Comparison

### GPU Options

| GPU Model | VRAM | CUDA Cores | Cost | Isaac Sim | Object Detection | Training |
|-----------|------|------------|------|-----------|------------------|----------|
| **RTX 3060** | 6GB | 3584 | $300 | ✅ Yes | ✅ 25+ FPS | ⚠️ Small models |
| **RTX 3070** | 8GB | 5888 | $500 | ✅ Yes | ✅ 40+ FPS | ✅ Medium models |
| **RTX 4060 Ti** | 8GB/16GB | 4352 | $400/$500 | ✅ Yes | ✅ 50+ FPS | ✅ Medium models |
| **RTX 4070** | 12GB | 5888 | $600 | ✅ Yes | ✅ 60+ FPS | ✅ Large models |
| GTX 1650 | 4GB | 896 | $150 | ❌ No | ⚠️ 8-10 FPS | ❌ No |
| AMD RX 6600 | 8GB | N/A | $250 | ❌ No CUDA | ❌ No | ❌ No |

**Key Takeaway**: NVIDIA RTX series required for Isaac Sim. RTX 3060 is the minimum viable GPU.

### CPU Options

| CPU Model | Cores | Base/Boost Clock | Cost | ROS 2 + Gazebo | ROS 2 + Isaac Sim |
|-----------|-------|------------------|------|----------------|-------------------|
| **Intel i5-12400** | 6 (6P) | 2.5/4.4 GHz | $180 | ✅ Good | ✅ Recommended |
| **AMD Ryzen 5 5600X** | 6 | 3.7/4.6 GHz | $200 | ✅ Good | ✅ Recommended |
| Intel i5-13400 | 10 (6P+4E) | 2.5/4.6 GHz | $220 | ✅ Excellent | ✅ Excellent |
| AMD Ryzen 7 5800X | 8 | 3.8/4.7 GHz | $280 | ✅ Excellent | ✅ Excellent |
| Intel i3-12100 | 4 | 3.3/4.3 GHz | $120 | ⚠️ Adequate | ❌ Insufficient |

**Key Takeaway**: 6+ cores recommended. Intel 12th gen or AMD Ryzen 5000 series for best value.

### Memory Requirements

| Use Case | Minimum | Recommended | Optimal |
|----------|---------|-------------|---------|
| ROS 2 + Gazebo | 8GB | 16GB | 32GB |
| ROS 2 + Isaac Sim | 16GB | 32GB | 64GB |
| Training models locally | 16GB | 32GB | 64GB+ |
| Embedded (Jetson) | 4GB | 8GB | 16GB (AGX) |

**Memory Breakdown** (16GB system running Isaac Sim):
- Ubuntu + desktop: 2GB
- ROS 2 workspace: 1GB
- Isaac Sim: 6-8GB
- AI models (inference): 2-3GB
- Buffers/overhead: 2-3GB

## Storage Considerations

### Disk Space Requirements

| Component | Space Required | Type |
|-----------|----------------|------|
| Ubuntu 22.04 | 10GB | OS |
| ROS 2 Humble | 5-8GB | Framework |
| Gazebo Fortress | 2GB | Simulation |
| Isaac Sim 2023.1 | 50GB | Simulation + assets |
| COCO dataset | 20GB | Training data |
| ROS 2 workspace | 5-10GB | Code + builds |
| Bag files (recordings) | 10-20GB | Data logs |
| **Total** | **100-120GB** | |

### SSD vs HDD

**NVMe SSD** (recommended for Isaac Sim):
- 3000+ MB/s read/write
- Low latency for asset streaming
- Examples: Samsung 980 Pro, WD Black SN850

**SATA SSD** (acceptable for Gazebo):
- 500+ MB/s read/write
- Sufficient for ROS 2 development

**HDD** (not recommended):
- ❌ Slow Isaac Sim loading (minutes vs seconds)
- ❌ Slow ROS 2 bag playback
- ❌ Poor development experience

## Peripherals and Accessories

### For Development

**Recommended**:
- **Webcam**: Logitech C920 or better (testing vision pipelines)
- **Microphone**: USB microphone for Whisper testing
- **Monitor**: 1920x1080 minimum (1440p recommended for multi-window)
- **Ethernet**: Wired connection for stable ROS 2 communication

**Optional**:
- **3D Mouse**: SpaceMouse for Gazebo/Isaac Sim navigation
- **Game Controller**: Xbox/PS controller for teleoperation

### For Physical Robot Deployment

**Sensors**:
- Intel RealSense D435i (~$300): RGB-D camera + IMU
- Hokuyo/SICK LIDAR (~$1000+): 2D laser scanning
- IMU: BNO055 (~$30) or higher-grade

**Actuators**:
- Dynamixel servos (MX/XM/X series): ~$50-200 each
- DC motors + encoders: Budget alternative

**Power**:
- LiPo battery: 14.8V 4S, 5000+ mAh
- Voltage regulators: 5V/12V for sensors and compute

**Communication**:
- Wi-Fi 6 router: Low-latency robot telemetry
- Ethernet switch: Wired sensor connections

## Cost Estimates

### Full Setup Costs

**Simulation-Only (Recommended)**:
- Desktop PC (RTX 3060 build): $1,200-1,500
- Monitor + peripherals: $200-300
- **Total**: ~$1,500-1,800

**Embedded Deployment**:
- Jetson Orin Nano Dev Kit: $500
- RealSense D435i: $300
- Sensors + actuators: $500-1,000
- Mechanical parts: $300-500
- **Total**: ~$1,600-2,300 (plus simulation PC)

**Cloud-Only (No Hardware)**:
- AWS g4dn.xlarge: $0.50/hour
- 40 hours/month for 4 months: ~$160 total
- **Total**: ~$160 (course duration)

## Recommendations by Student Profile

### Undergraduate Student (Budget-Conscious)

**Recommended**: Cloud + Budget Laptop
- Use AWS/Paperspace for Isaac Sim work
- Budget laptop for ROS 2 coding (no simulation)
- **Cost**: ~$500 (laptop) + $100-200 (cloud)

### Graduate Student / Professional

**Recommended**: Mid-range Desktop
- RTX 3060 or 4060 desktop
- Run all modules locally
- **Cost**: ~$1,500-1,800

### Robotics Lab / Team

**Recommended**: High-end Desktop + Embedded
- RTX 4070 or 4080 desktop (shared)
- Jetson Orin Nano for each team member
- **Cost**: ~$2,500-3,500

### Hobbyist / Self-Learner

**Recommended**: Gaming Laptop (if available)
- Repurpose existing gaming laptop with RTX 3060+
- Dual-boot Ubuntu 22.04
- **Cost**: $0 (if already owned)

## Setup Guides

Detailed setup instructions for each hardware option are available in:

- [Environment Setup Appendix](appendices/environment-setup.md): Ubuntu, ROS 2, Gazebo, Isaac Sim installation
- [Docker Configuration](.github/docker/ros2-humble.Dockerfile): Reproducible environments
- Module-specific guides: Hardware integration in Module 5

## Frequently Asked Questions

**Q: Can I use Windows instead of Ubuntu?**
A: Windows 11 with WSL2 works for ROS 2 development and Gazebo, but Isaac Sim requires native Linux. Dual-boot recommended.

**Q: Can I use a MacBook?**
A: ARM-based Macs (M1/M2/M3) cannot run ROS 2 natively or Isaac Sim. Use cloud instances or virtualization (limited support).

**Q: Is AMD GPU supported?**
A: AMD GPUs work for Gazebo but not for Isaac Sim (requires NVIDIA CUDA). NVIDIA GPUs strongly recommended.

**Q: Can I run this on a virtual machine?**
A: VMs can run ROS 2 and Gazebo (with GPU passthrough) but Isaac Sim is not supported in VMs. Use cloud instances instead.

**Q: What if I already have different hardware?**
A: If you have 8+ cores, 16GB+ RAM, and an RTX 2060+, you can likely run everything. Test Isaac Sim compatibility first.

---

**Need help choosing?** Ask in [GitHub Discussions](https://github.com/asadaligith/AI-Humanoid-Robotics-Book/discussions) or consult the [Environment Setup Guide](appendices/environment-setup.md).
