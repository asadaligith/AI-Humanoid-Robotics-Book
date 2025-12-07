---
sidebar_position: 4
---

# Conventions and Notation

This page explains the notation systems, formatting standards, and diagram conventions used throughout the AI Humanoid Robotics Book.

## Code Formatting

### Language Identifiers

Code blocks are clearly labeled with their language:

**Python** (primary language for ROS 2 nodes):
```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
```

**Bash** (for terminal commands and scripts):
```bash
source /opt/ros/humble/setup.bash
ros2 run my_package my_node
```

**YAML** (for configuration files):
```yaml
/**:
  ros__parameters:
    use_sim_time: true
```

**XML** (for URDF, launch files, package.xml):
```xml
<robot name="my_robot">
  <link name="base_link"/>
</robot>
```

**C++** (for performance-critical components):
```cpp
#include <rclcpp/rclcpp.hpp>
auto node = std::make_shared<rclcpp::Node>("cpp_node");
```

### File Path References

File paths follow this convention:

**Absolute Path** (from repository root):
```
examples/module_01_ros2/simple_publisher.py
```

**Workspace-Relative Path** (from ROS 2 workspace):
```
~/ros2_ws/src/my_package/my_package/node.py
```

**Line Number References**:
```
src/perception/object_detector.py:45
```
Indicates line 45 in that specific file.

**File Path in Code Blocks**:
```python
# File: examples/module_01_ros2/simple_publisher.py
#!/usr/bin/env python3
```

### Command Prompt Notation

**User Commands** (you type these):
```bash
ros2 run demo_nodes_cpp talker
```

**Expected Output** (system response):
```
[INFO] [1234567890.123] [talker]: Publishing: 'Hello World: 1'
```

**Multi-line Commands** (use backslash continuation):
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  map:=/path/to/map.yaml
```

**Comments in Commands**:
```bash
# This is an explanation
ros2 topic echo /cmd_vel  # Monitor velocity commands
```

### Environment Variables

Environment variables are shown in UPPERCASE:

```bash
export ROS_DOMAIN_ID=42
export GAZEBO_MODEL_PATH=/path/to/models
```

### Package and Node Names

**Package Names**: `snake_case`
```
my_robot_control
object_detection_pkg
```

**Node Names**: `snake_case` with descriptive suffixes
```
camera_publisher_node
task_planner_node
```

**Topic Names**: Forward-slash hierarchy
```
/camera/rgb/image_raw
/robot/cmd_vel
/perception/detections
```

**Frame IDs**: `snake_case`
```
base_link
camera_optical_frame
map
odom
```

## Diagram Conventions

### Architecture Diagrams

**Component Boxes**:
- **Rectangles**: Software components (nodes, packages)
- **Cylinders**: Data storage (databases, bag files)
- **Clouds**: External services (APIs, cloud resources)

**Connection Lines**:
- **Solid Arrow**: Data flow (topic, service, action)
- **Dashed Arrow**: Dependency or optional connection
- **Bidirectional**: Request-response pattern

**Color Coding**:
- **Blue**: ROS 2 components
- **Green**: Simulation/Gazebo components
- **Orange**: AI/ML components (perception, planning)
- **Purple**: Hardware interfaces

**Example**:
```
┌─────────────┐    /cmd_vel    ┌──────────────┐
│ Task        │──────────────>│ Robot        │
│ Planner     │                │ Controller   │
│ (Claude)    │<──────────────│              │
└─────────────┘    /odom       └──────────────┘
     ORANGE          topic           BLUE
```

### Data Flow Diagrams

**Notation**:
- **Circles**: Processes or transformations
- **Rectangles**: Data stores
- **Arrows**: Data movement direction
- **Labels**: Data type or topic name

**Example**:
```
Camera → Image Processing → Object Detection → Task Planner
       (sensor_msgs/Image)  (DetectionArray)   (TaskGoal)
```

### State Machine Diagrams

**States**: Rounded rectangles
**Transitions**: Labeled arrows
**Initial State**: Solid circle
**Final State**: Double circle

**Example**:
```
● → [IDLE] → [NAVIGATING] → [MANIPULATING] → ◉
      ↑           |               |
      └───────────┴───────────────┘
           (task_complete)
```

### Coordinate Frame Diagrams

**Axes Convention** (ROS REP 103):
- **X**: Red (forward)
- **Y**: Green (left)
- **Z**: Blue (up)

**Frame Notation**:
```
       Z (up)
       ↑
       |
       |──→ X (forward)
      /
     /
    ↙ Y (left)
```

**Transform Trees**:
```
map
 └─ odom
     └─ base_link
         ├─ camera_link
         │   └─ camera_optical_frame
         └─ gripper_link
```

## Mathematical Notation

### Vectors and Matrices

**Vectors** (bold lowercase):
- Position: **p** = [x, y, z]ᵀ
- Velocity: **v** = [vₓ, vᵧ, vᵩ]ᵀ

**Matrices** (bold uppercase):
- Rotation: **R** ∈ SO(3)
- Transformation: **T** ∈ SE(3)

**Coordinate Frames** (superscript):
- ᴬ**p**ᴮ: Position of frame B expressed in frame A

### Equations

**Inline Math**: Use parentheses for clarity
- Linear velocity: v = √(vₓ² + vᵧ²)

**Block Equations**: Centered and numbered
```
Transform composition:
ᴬTᶜ = ᴬTᴮ · ᴮTᶜ
```

### Quaternions

**Notation**: q = [qₓ, qᵧ, qᵤ, qᵥ]
- qₓ, qᵧ, qᵤ: Vector component
- qᵥ: Scalar component

## ROS 2 Specific Conventions

### Message Types

**Format**: `package_name/MessageType`

Examples:
- `std_msgs/String`
- `sensor_msgs/Image`
- `geometry_msgs/Twist`
- `nav_msgs/Odometry`

### QoS Profiles

**Notation**:
```python
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,    # or TRANSIENT_LOCAL
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

### Launch File Parameters

**Parameter Notation**:
```python
DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation time'
)
```

**Parameter Usage**:
```python
'use_sim_time': LaunchConfiguration('use_sim_time')
```

## Citation Format

### In-Text Citations

**APA 7th Edition** with Context7 metadata:

**Single Citation**:
> ROS 2 uses DDS for communication **[CTX7-ROS2-001]**.

**Multiple Citations**:
> Several frameworks support robotics simulation **[CTX7-GAZEBO-003]**, **[CTX7-ISAAC-012]**.

**Citation in Code Comments**:
```python
# Implementation based on Nav2 architecture [CTX7-NAV2-008]
class NavigationController:
    pass
```

### Full Citation Format

See [Citations Appendix](appendices/citations.md) for complete references.

**Format**:
```
[CTX7-ID] Author(s). (Year). Title. DOI/URL
```

## Callout Boxes

### Tip (💡)

> 💡 **Tip**: Use `colcon build --symlink-install` for faster development iteration.

**When to use**: Best practices, helpful shortcuts, efficiency tips

### Warning (⚠️)

> ⚠️ **Warning**: Always source your workspace before running nodes: `source install/setup.bash`

**When to use**: Common errors, dangerous operations, easy mistakes

### Deep Dive (🔬)

> 🔬 **Deep Dive**: For advanced users interested in DDS QoS tuning, see **[CTX7-DDS-005]**.

**When to use**: Advanced topics, theoretical background, optional reading

### Checkpoint (✅)

> ✅ **Checkpoint**: Verify your node is running: `ros2 node list` should show `/example_node`

**When to use**: Verification steps, testing instructions, progress markers

### Citation Reference (📚)

> 📚 **Citation**: For the complete ROS 2 design rationale, see **[CTX7-ROS2-DESIGN-001]**

**When to use**: Links to primary sources, documentation, research papers

## Version Notation

### Software Versions

**ROS 2 Distributions**: Codename (release date)
- ROS 2 Humble Hawksbill (May 2022) - LTS
- ROS 2 Iron Irwini (May 2023)

**Gazebo Versions**: Name + Version
- Gazebo Fortress 6.16.0
- Gazebo Garden 7.4.0

**Isaac Sim**: Year.Version.Patch
- Isaac Sim 2023.1.1
- Isaac Sim 2024.1.0

### Python Package Versions

**Format**: `package==version`

```
rclpy==3.3.11
opencv-python==4.8.1.78
numpy>=1.24.0,<2.0.0
```

## Hardware Specifications

### Component Naming

**CPUs**: Manufacturer Model
- Intel Core i5-12400
- AMD Ryzen 5 5600X

**GPUs**: Brand Series Model VRAM
- NVIDIA RTX 3060 6GB
- NVIDIA RTX 4090 24GB

**Memory**: Size Type Speed
- 16GB DDR4-3200
- 32GB DDR5-4800

**Storage**: Size Type Interface
- 500GB NVMe SSD
- 1TB SATA SSD

### Embedded Platforms

**Format**: Platform (RAM/Storage)
- NVIDIA Jetson Orin Nano (8GB)
- Raspberry Pi 4 Model B (4GB)

## File and Directory Structure

### ROS 2 Workspace Layout

```
ros2_ws/
├── src/               # Source packages
│   ├── my_package/
│   │   ├── my_package/    # Python package
│   │   ├── launch/        # Launch files
│   │   ├── config/        # YAML configs
│   │   ├── urdf/          # Robot descriptions
│   │   └── package.xml
├── build/             # Build artifacts (gitignored)
├── install/           # Installed packages
└── log/               # Build/runtime logs
```

### Docusaurus Book Structure

```
AI-Humanoid-Robotics-Book/
├── docs/              # Markdown content
│   ├── intro.md
│   ├── modules/
│   └── appendices/
├── examples/          # Executable code
│   ├── module_01_ros2/
│   └── module_05_capstone/
├── assets/            # Media and models
│   ├── urdf/
│   ├── worlds/
│   └── diagrams/
└── tests/             # Automated tests
```

## Terminology Standards

### Consistent Term Usage

**Preferred Terms**:
- "ROS 2" (not "ROS2" or "ros2" in prose)
- "Gazebo" (not "Gazebo Sim" in general text)
- "Isaac Sim" (not "Isaac Simulator")
- "node" (lowercase, ROS 2 component)
- "topic" (lowercase, ROS 2 communication)

**Capitalization**:
- Proper nouns: ROS 2, Gazebo, Ubuntu, Python
- Generic terms: robotics, simulation, navigation
- Acronyms: SLAM, URDF, DDS, QoS, API

### Glossary References

Unfamiliar terms link to the [Glossary](appendices/glossary.md):

> The **VSLAM** algorithm uses visual odometry for localization.

See glossary for definition of VSLAM.

---

**Questions about notation?** Check the [Glossary](appendices/glossary.md) or ask in [GitHub Discussions](https://github.com/asadaligith/AI-Humanoid-Robotics-Book/discussions).
