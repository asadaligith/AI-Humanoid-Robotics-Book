---
sidebar_position: 2
---

# Hardware Bill of Materials (BOM)

This document provides detailed component lists and cost estimates for building a physical humanoid robot for Module 5 Capstone deployment.

**Note**: Physical hardware is **optional**. All modules can be completed in simulation only.

## Complete System BOM

### Tier 1: Basic Wheeled Platform (~$800-1,200)

For students wanting to test on real hardware without full humanoid complexity.

| Category | Component | Quantity | Unit Price | Total | Notes |
|----------|-----------|----------|------------|-------|-------|
| **Compute** | NVIDIA Jetson Orin Nano 8GB | 1 | $499 | $499 | Primary onboard computer |
| **Power** | 14.8V 5000mAh LiPo Battery | 1 | $60 | $60 | 4S battery for motors |
| **Power** | 5V/5A Buck Converter | 1 | $15 | $15 | Power for Jetson |
| **Locomotion** | TT Gear Motors (200 RPM) | 2 | $8 | $16 | Wheel drive motors |
| **Locomotion** | Motor Driver (L298N) | 1 | $12 | $12 | H-bridge for motors |
| **Locomotion** | Robot Wheels (80mm) | 2 | $5 | $10 | Wheel assemblies |
| **Locomotion** | Caster Wheel | 1 | $8 | $8 | Balance wheel |
| **Perception** | Intel RealSense D435i | 1 | $299 | $299 | RGB-D camera + IMU |
| **Perception** | USB Microphone | 1 | $25 | $25 | For Whisper voice input |
| **Structure** | Aluminum Chassis Plate | 2 | $15 | $30 | Robot base |
| **Structure** | Standoffs and Hardware | 1 | $20 | $20 | M3/M4 screws, standoffs |
| **Networking** | Wi-Fi 6 USB Adapter | 1 | $30 | $30 | Low-latency telemetry |
| **Safety** | Emergency Stop Button | 1 | $12 | $12 | Physical e-stop |
| **Misc** | Cables and Connectors | 1 | $25 | $25 | USB-C, JST, XT60 |
| **Total** | | | | **$1,061** | |

**Capabilities**:
- ✅ Autonomous navigation
- ✅ Object detection (20+ COCO classes)
- ✅ Voice command via Whisper
- ✅ LLM task planning (Claude API)
- ❌ No manipulation (no arm)

---

### Tier 2: Mobile Manipulator (~$2,500-3,500)

Adds a 6-DOF robotic arm for pick-and-place tasks.

**Includes all Tier 1 components, plus**:

| Category | Component | Quantity | Unit Price | Total | Notes |
|----------|-----------|----------|------------|-------|-------|
| **Manipulation** | Dynamixel XM430-W350-T Servos | 6 | $180 | $1,080 | Shoulder (2), elbow, wrist (3) |
| **Manipulation** | Dynamixel U2D2 USB Adapter | 1 | $65 | $65 | Servo communication |
| **Manipulation** | Parallel Gripper Kit | 1 | $120 | $120 | End effector |
| **Manipulation** | Arm Structure (Aluminum) | 1 | $200 | $200 | Custom or kit (e.g., Interbotix) |
| **Power** | 12V 10A Buck Converter | 1 | $25 | $25 | Additional power for servos |
| **Perception** | Additional RealSense D435i | 1 | $299 | $299 | Wrist-mounted camera |
| **Structure** | Reinforced Chassis | 1 | $80 | $80 | Stronger base for arm |
| **Tier 2 Add-On Total** | | | | **$1,869** | |
| **Tier 2 Complete Total** | | | | **$2,930** | |

**Capabilities**:
- ✅ All Tier 1 features
- ✅ 6-DOF manipulation
- ✅ Pick-and-place tasks
- ✅ Full Capstone project

---

### Tier 3: Bipedal Humanoid (~$8,000-12,000)

Full humanoid with legs, torso, arms, and head. **Advanced project**, requires significant mechanical engineering.

**Includes all Tier 2 components, replaces locomotion with legs**:

| Category | Component | Quantity | Unit Price | Total | Notes |
|----------|-----------|----------|------------|-------|-------|
| **Legs** | Dynamixel MX-106T Servos | 12 | $350 | $4,200 | 6 per leg (hip, knee, ankle) |
| **Torso** | Dynamixel MX-64T Servos | 3 | $230 | $690 | Waist pitch/roll/yaw |
| **Arms** | Dynamixel XM430-W350-T | 8 | $180 | $1,440 | 4 per arm (shoulder, elbow, wrist) |
| **Head** | Dynamixel XL430-W250-T | 2 | $60 | $120 | Pan/tilt for camera |
| **Gripper** | Parallel Grippers | 2 | $120 | $240 | Both hands |
| **IMU** | VectorNav VN-100 | 1 | $600 | $600 | High-accuracy orientation |
| **Perception** | RealSense D435i (Head) | 1 | $299 | $299 | Primary vision |
| **Perception** | RealSense D435i (Wrist) | 2 | $299 | $598 | Hand cameras |
| **Power** | 14.8V 10,000mAh LiPo | 2 | $120 | $240 | Extended runtime |
| **Power** | Power Distribution Board | 1 | $150 | $150 | Multi-voltage regulation |
| **Structure** | Humanoid Frame Kit | 1 | $1,500 | $1,500 | Aluminum/carbon fiber |
| **Compute** | Upgrade to Jetson AGX Orin | 1 | $1,999 | $1,999 | 32GB, higher compute |
| **Tier 3 Add-On Total** | | | | **$10,076** | |
| **Tier 3 Complete Total** | | | | **~$11,137** | Replaces Tier 1 locomotion |

**Capabilities**:
- ✅ Bipedal walking (requires advanced control)
- ✅ Dual-arm manipulation
- ✅ Human-like interaction
- ⚠️ Requires PhD-level robotics knowledge

**Recommendation**: Start with Tier 1 or Tier 2. Bipedal locomotion is beyond scope of this book.

---

## Component Details

### Compute Platforms

#### NVIDIA Jetson Orin Nano 8GB (Recommended)

**Specifications**:
- GPU: 1024-core NVIDIA Ampere, 32 Tensor Cores
- CPU: 6-core ARM Cortex-A78AE @ 2.0 GHz
- Memory: 8GB LPDDR5
- I/O: 4x USB 3.2, Gigabit Ethernet, GPIO, I2C, SPI, UART
- Power: 7W-15W (configurable)

**Vendor**: NVIDIA
**Part Number**: 945-13766-0000-000
**Purchase**: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/

**Alternatives**:
- Jetson Orin NX 16GB ($699): More compute, higher cost
- Jetson AGX Orin 32GB ($1,999): Maximum performance for Tier 3
- Raspberry Pi 5 8GB ($80): Budget option, no GPU acceleration

---

### Sensors

#### Intel RealSense D435i (RGB-D Camera)

**Specifications**:
- RGB: 1920x1080 @ 30 FPS
- Depth: Stereo cameras, 1280x720 @ 90 FPS
- Range: 0.2m - 10m (optimal: 0.3m - 3m)
- IMU: BMI055 (accel + gyro)
- Interface: USB 3.1 Type-C
- Dimensions: 90mm x 25mm x 25mm

**Vendor**: Intel
**Part Number**: 82635AWGDVKPRQ
**Purchase**: https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435i.html

**Alternatives**:
- RealSense D455 ($329): Longer range (up to 20m)
- RealSense L515 ($350): LiDAR-based, indoor only
- Kinect Azure ($399): Higher resolution, more expensive

**ROS 2 Package**: `ros-humble-realsense2-camera`

---

#### USB Microphone (For Whisper)

**Recommended**: Blue Snowball iCE ($50)
- Type: Condenser
- Polar Pattern: Cardioid
- Sample Rate: 44.1 kHz
- Interface: USB 2.0

**Budget Alternative**: Mini USB Microphone ($15)
- Any USB microphone compatible with Linux ALSA

**ROS 2 Package**: `audio_common` (custom nodes for Whisper integration)

---

### Actuators

#### Dynamixel XM430-W350-T (Mid-Range Servo)

**Specifications**:
- Torque: 4.1 Nm @ 12V
- Speed: 46 RPM @ 12V (no load)
- Position Resolution: 0.088° (4096 positions/rev)
- Communication: TTL Half-Duplex UART
- Feedback: Position, velocity, current, temperature
- Dimensions: 28.5mm x 46.5mm x 34mm
- Weight: 82g

**Vendor**: ROBOTIS
**Part Number**: XM430-W350-T
**Purchase**: https://www.robotis.us/dynamixel-xm430-w350-t/

**Use Cases**:
- Robot arm joints (shoulder, elbow, wrist)
- Lightweight manipulation tasks

**Alternatives**:
- XM540-W270-T ($220): Higher torque (6.9 Nm)
- XL430-W250-T ($60): Lower torque (1.5 Nm), budget

**ROS 2 Package**: `dynamixel_sdk`, `dynamixel_workbench`

---

#### Dynamixel MX-106T (High-Torque Servo)

**Specifications**:
- Torque: 10.9 Nm @ 14.8V
- Speed: 41 RPM @ 14.8V
- Position Resolution: 0.088°
- Communication: RS-485
- Dimensions: 40.2mm x 65.1mm x 46mm
- Weight: 165g

**Vendor**: ROBOTIS
**Part Number**: MX-106T
**Purchase**: https://www.robotis.us/dynamixel-mx-106t/

**Use Cases**:
- Leg joints (hip, knee, ankle) for bipedal robots
- High-load manipulation

**Note**: Expensive. Only needed for Tier 3 humanoid.

---

### Power Systems

#### 14.8V 5000mAh LiPo Battery (4S)

**Specifications**:
- Voltage: 14.8V nominal (16.8V fully charged)
- Capacity: 5000mAh (74 Wh)
- Discharge Rate: 30C (150A burst)
- Connector: XT60
- Weight: ~450g

**Vendor**: Turnigy, Tattu, or similar
**Purchase**: HobbyKing, Amazon

**Runtime Estimates**:
- Jetson Orin Nano @ 10W: ~7 hours
- With 2 motors @ 5W each: ~4 hours
- With 6 servos @ 5W average: ~2 hours

**⚠️ Safety**:
- Use LiPo-safe charging bag
- Never discharge below 3.0V per cell
- Use fire-resistant storage

---

#### Buck Converters

**5V/5A Buck Converter** (For Jetson):
- Input: 7V-24V
- Output: 5V @ 5A (25W)
- Efficiency: >90%
- Part: UBEC or Pololu D24V50F5

**12V/10A Buck Converter** (For Servos):
- Input: 14.8V-24V
- Output: 12V @ 10A (120W)
- Part: LM2596-based or similar

---

### Structure and Mechanics

#### Aluminum Chassis Plates

**Material**: 6061-T6 Aluminum
**Thickness**: 3mm or 5mm
**Dimensions**: 200mm x 300mm (typical)
**Purchase**: eBay, McMaster-Carr, custom laser cutting

**Alternatives**:
- Acrylic sheets: Lighter, cheaper, less rigid
- Carbon fiber: Lighter, more expensive
- 3D printed PLA: Prototyping only, low strength

---

#### Robot Arm Structure

**Option 1: Purchase Kit**
- Interbotix WidowX 250 Robot Arm Kit ($1,500)
  - Includes 5x Dynamixel servos, structure, gripper
  - ROS 2 compatible
  - Purchase: https://www.trossenrobotics.com/

**Option 2: Custom Build**
- Aluminum extrusions (8020 T-slot): $50-100
- Laser-cut brackets: $50-100
- 3D printed parts: $20-50
- Total: ~$200 + servos

---

## Connectivity and Networking

### Wi-Fi 6 USB Adapter

**Recommended**: TP-Link Archer T3U Plus ($30)
- Standard: 802.11ax (Wi-Fi 6)
- Speed: Up to 1300 Mbps (5 GHz)
- Interface: USB 3.0
- Linux driver: Built-in kernel support

**Purpose**: Low-latency ROS 2 communication between robot and development PC.

---

### Ethernet Switch (Optional)

**Recommended**: TP-Link TL-SG105 ($20)
- Ports: 5x Gigabit Ethernet
- Use case: Wired connection for multiple sensors

---

## Tools and Accessories

### Required Tools

| Tool | Purpose | Estimated Cost |
|------|---------|----------------|
| Soldering Iron + Solder | Connector assembly | $30 |
| Multimeter | Voltage/current testing | $20 |
| Hex Key Set (Metric) | M2.5, M3, M4 screws | $15 |
| Wire Strippers | Cable preparation | $15 |
| LiPo Charger (Balance) | Battery charging | $50 |
| Heat Shrink Tubing | Insulation | $10 |
| Crimping Tool (JST) | Connector crimping | $25 |
| **Total** | | **~$165** |

---

### Optional Tools

| Tool | Purpose | Estimated Cost |
|------|---------|----------------|
| 3D Printer | Custom part fabrication | $200-500 |
| Oscilloscope | Signal debugging | $100-300 |
| Power Supply (Bench) | Testing without battery | $50-100 |

---

## Assembly Components

### Connectors and Cables

| Component | Quantity | Unit Price | Total | Use Case |
|-----------|----------|------------|-------|----------|
| XT60 Connectors (Male/Female) | 5 pairs | $1.50 | $7.50 | Battery connections |
| JST-XH Balance Connectors | 5 | $0.50 | $2.50 | LiPo balance leads |
| USB-C to USB-A Cable (1m) | 2 | $8 | $16 | Jetson, RealSense |
| USB-A to Micro-USB (0.5m) | 3 | $5 | $15 | Dynamixel U2D2 |
| Ethernet Cable (Cat6, 2m) | 2 | $5 | $10 | Wired networking |
| Jumper Wires (Male-Female) | 40-pack | $8 | $8 | GPIO connections |
| Heat Shrink Tubing (Assorted) | 1 pack | $10 | $10 | Insulation |
| **Total** | | | **$69** | |

---

### Fasteners and Hardware

| Component | Quantity | Unit Price | Total |
|-----------|----------|------------|-------|
| M3 Screws (10mm, 20mm, 30mm) | 100-pack | $8 | $8 |
| M4 Screws (10mm, 20mm, 30mm) | 100-pack | $10 | $10 |
| M3 Standoffs (10mm, 20mm) | 50-pack | $12 | $12 |
| M3/M4 Nuts and Washers | 200-pack | $8 | $8 |
| Lock Washers (M3, M4) | 100-pack | $5 | $5 |
| Cable Ties (Assorted) | 200-pack | $8 | $8 |
| **Total** | | | **$51** | |

---

## Vendor Directory

### Electronics and Robotics

- **ROBOTIS**: Dynamixel servos and accessories
  - https://www.robotis.us/
- **Trossen Robotics**: Interbotix robot kits
  - https://www.trossenrobotics.com/
- **Adafruit**: Sensors, breakout boards, cables
  - https://www.adafruit.com/
- **SparkFun**: Electronics components
  - https://www.sparkfun.com/
- **Pololu**: Motors, motor drivers, power supplies
  - https://www.pololu.com/

### Compute and AI

- **NVIDIA**: Jetson developer kits
  - https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/
- **Intel**: RealSense cameras
  - https://store.intelrealsense.com/

### Power and Batteries

- **HobbyKing**: LiPo batteries, chargers
  - https://hobbyking.com/
- **Amazon**: General batteries and power supplies

### Structure and Mechanics

- **McMaster-Carr**: Fasteners, extrusions, raw materials
  - https://www.mcmaster.com/
- **80/20 Inc**: Aluminum T-slot framing
  - https://8020.net/
- **SendCutSend**: Custom laser cutting service
  - https://sendcutsend.com/

---

## Cost Summary by Tier

| Tier | Description | Hardware Cost | Tools Cost | Total |
|------|-------------|---------------|------------|-------|
| **Tier 1** | Basic wheeled platform | $1,061 | $165 | **$1,226** |
| **Tier 2** | Mobile manipulator | $2,930 | $165 | **$3,095** |
| **Tier 3** | Bipedal humanoid | $11,137 | $165 + $300 | **$11,602** |

**Additional Recurring Costs**:
- LiPo battery replacement: $60/year (with care)
- Spare servos (failures): $100-200/year
- Claude API usage: ~$20-50/month (Module 4-5)

---

## Purchasing Recommendations

### For Individual Learners

**Start with**: Simulation only ($0 hardware)
**Upgrade to**: Tier 1 if budget allows (~$1,200)
**Skip**: Tier 3 (bipedal) unless you have robotics lab access

### For University Labs

**Recommended**: Tier 2 mobile manipulator (~$3,000)
- Sharable among students
- Covers full Capstone project
- Repairable (modular design)

### For Research Groups

**Recommended**: Multiple Tier 2 platforms + 1 Tier 3 (if grant funded)
- Tier 2 for student projects
- Tier 3 for advanced research (walking, balancing)

---

## Maintenance and Spare Parts

### Recommended Spares

| Component | Quantity | Reason |
|-----------|----------|--------|
| Dynamixel Servos | 2 | Burn-out from overload |
| LiPo Batteries | 1 | Degradation over time |
| RealSense D435i | 0-1 | Durable, rarely fails |
| Buck Converters | 1 | Failures from short circuits |
| Jetson Orin Nano | 0 | Reliable, expensive |

**Estimated Spare Parts Cost**: $400-500

---

## Import and Shipping Considerations

**International Shipping**:
- RealSense cameras: Ships worldwide from Intel
- Dynamixel servos: Direct from ROBOTIS Korea or US distributors
- LiPo batteries: ⚠️ Restricted on air freight (ship ground/sea)

**Import Duties**:
- Vary by country (typically 5-25% for electronics)
- Check with local customs before large orders

---

## Safety and Compliance

### Electrical Safety

- Use fuses on battery lines (rated for max current)
- Install emergency stop button (wired to motor enable)
- Never operate near flammable materials
- Store LiPo batteries in fireproof bags

### Mechanical Safety

- Add soft padding to robot edges
- Limit servo speed during testing
- Implement collision detection in software
- Test in isolated area away from people

### Regulatory Compliance

- FCC/CE certification not required for personal/research use
- If commercializing, consult with compliance expert

---

**Ready to build?** Start with [Environment Setup](environment-setup.md) to configure software before ordering hardware.
