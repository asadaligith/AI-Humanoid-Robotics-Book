---
sidebar_position: 3
---

# Glossary

Technical terms and acronyms used throughout the AI Humanoid Robotics Book.

---

## A

**Action** (ROS 2)
: A communication pattern in ROS 2 for long-running tasks with feedback. Consists of goal, feedback, and result messages. Example: Navigation to a waypoint.

**Actuator**
: A component that produces motion or force. Examples: Servo motors, DC motors, pneumatic cylinders.

**APA Citation**
: American Psychological Association citation format (7th edition) used for academic references in this book.

**API (Application Programming Interface)**
: A set of functions and protocols for building software. Example: Claude API for LLM task planning.

**Autonomous Navigation**
: The ability of a robot to move from point A to B without human intervention, using sensors and planning algorithms.

---

## B

**Base Link**
: The primary coordinate frame attached to a robot's main body, typically at the center of rotation. All other frames are defined relative to base_link.

**Bag File** (ROS 2)
: A file format (.db3) for recording and replaying ROS 2 topic data. Used for debugging and testing.

**BOM (Bill of Materials)**
: A comprehensive list of components, quantities, and costs required to build a hardware system.

**Bounding Box**
: A rectangular box around a detected object in an image, defined by (x, y, width, height) or corner coordinates.

**Buck Converter**
: A DC-to-DC power converter that steps down voltage efficiently. Example: 14.8V battery to 5V for Jetson.

---

## C

**Callback**
: A function executed in response to an event, such as receiving a message on a ROS 2 topic.

**Camera Optical Frame**
: A coordinate frame aligned with a camera's optical axis, following the convention: X right, Y down, Z forward (into the scene).

**Capstone Project**
: Module 5 of this book, integrating all previous modules into an autonomous humanoid robot system.

**COCO Dataset**
: Common Objects in Context, a large-scale object detection dataset with 80 classes (20+ used in this book).

**Colcon**
: The build tool for ROS 2 workspaces, replacing catkin from ROS 1.

**Context7**
: An MCP (Model Context Protocol) server for managing citations and documentation links in AI-assisted workflows.

**Coordinate Frame**
: A reference system defining position and orientation, with X, Y, Z axes. Also called "coordinate system" or "frame."

**CUDA (Compute Unified Device Architecture)**
: NVIDIA's parallel computing platform for GPU-accelerated applications.

---

## D

**DDS (Data Distribution Service)**
: The middleware protocol used by ROS 2 for inter-process communication, providing publish-subscribe and request-response patterns.

**Depth Camera**
: A camera that captures distance (depth) information in addition to RGB images. Example: Intel RealSense D435i.

**Digital Twin**
: A virtual replica of a physical system used for simulation and testing before real-world deployment.

**Docker**
: A containerization platform for packaging software with dependencies into isolated, reproducible environments.

**DOF (Degrees of Freedom)**
: The number of independent ways a system can move. A 6-DOF robot arm has 6 joints allowing 6 independent motions.

**Dynamixel**
: A brand of smart servo motors by ROBOTIS, commonly used in robotics for precise position control with feedback.

---

## E

**Embodied AI**
: Artificial intelligence deployed in physical agents (robots) that interact with the real world, as opposed to purely digital AI.

**End Effector**
: The device at the end of a robotic arm (e.g., gripper, suction cup) that interacts with objects.

**Executor** (ROS 2)
: A component that manages callback execution for ROS 2 nodes, handling subscriptions, timers, and services.

---

## F

**Forward Kinematics**
: Computing the position and orientation of a robot's end effector from its joint angles.

**FPS (Frames Per Second)**
: The rate at which images are captured or processed, critical for real-time perception systems.

**Frame ID**
: A string identifier for a coordinate frame in ROS 2 TF2 system. Example: "base_link", "camera_optical_frame".

---

## G

**Gazebo**
: An open-source 3D robotics simulator, successor to Gazebo Classic. Used for testing robots in virtual environments.

**GitHub Actions**
: A CI/CD platform for automating workflows like testing and deployment directly from GitHub repositories.

**Grasp Pose**
: The position and orientation a robot gripper must achieve to successfully grasp an object.

**GPU (Graphics Processing Unit)**
: Specialized hardware for parallel computations, essential for AI inference and simulation rendering.

---

## H

**Homogeneous Transformation**
: A 4x4 matrix representing both rotation and translation in 3D space, used in robotics for coordinate frame transformations.

**Humanoid Robot**
: A robot with a human-like body structure, typically including head, torso, arms, and legs.

---

## I

**IMU (Inertial Measurement Unit)**
: A sensor measuring acceleration and angular velocity, used for odometry and stabilization.

**Inverse Kinematics**
: Computing the joint angles required to position a robot's end effector at a desired location (opposite of forward kinematics).

**Isaac Sim**
: NVIDIA's photorealistic robotics simulator built on Omniverse, supporting GPU-accelerated physics and rendering.

---

## J

**Jetson**
: NVIDIA's embedded computing platform for edge AI, commonly used in autonomous robots. Example: Jetson Orin Nano.

**Joint State**
: The position, velocity, and effort (torque) of a robot's joints, published on `/joint_states` topic in ROS 2.

---

## K

**Kinematics**
: The study of motion without considering forces. In robotics: forward and inverse kinematics for arm movement.

---

## L

**Launch File**
: A Python or XML file in ROS 2 that starts multiple nodes with configuration parameters simultaneously.

**LiDAR (Light Detection and Ranging)**
: A sensor that measures distances using laser beams, creating 2D or 3D maps of the environment.

**LLM (Large Language Model)**
: A neural network trained on text for natural language understanding and generation. Example: Anthropic Claude.

**LTS (Long-Term Support)**
: A software release maintained with updates for extended periods. Example: Ubuntu 22.04 LTS, ROS 2 Humble (5 years).

---

## M

**Manipulation**
: The task of grasping, moving, and placing objects using a robotic arm.

**MCP (Model Context Protocol)**
: A protocol for connecting AI assistants to external data sources, used for Context7 citation management.

**Message** (ROS 2)
: A data structure transmitted over topics. Example: `geometry_msgs/Twist` for velocity commands.

**Model (ML)**
: A trained neural network for tasks like object detection, segmentation, or voice recognition.

---

## N

**Nav2 (Navigation2)**
: The ROS 2 navigation framework for autonomous path planning and obstacle avoidance.

**Node** (ROS 2)
: An executable program that performs computation and communicates via topics, services, or actions.

**NVIDIA Container Toolkit**
: Software enabling GPU access inside Docker containers for CUDA applications.

---

## O

**Odometry**
: Estimating a robot's position and velocity from sensor data (wheel encoders, IMU, visual odometry).

**Omniverse**
: NVIDIA's platform for 3D collaboration and simulation, hosting Isaac Sim.

---

## P

**Package** (ROS 2)
: A directory containing nodes, launch files, configuration, and metadata (package.xml) organized as a unit.

**Perception**
: The robotic capability to interpret sensor data (cameras, LiDAR) to understand the environment.

**Physical AI**
: AI systems that interact with the physical world through robotic embodiment (sensors, actuators).

**Pose**
: Position (x, y, z) and orientation (quaternion or Euler angles) of an object or robot in 3D space.

**Publisher** (ROS 2)
: A node or component that sends messages to a topic.

---

## Q

**QoS (Quality of Service)**
: Configuration for ROS 2 communication reliability, durability, and history. Example: RELIABLE vs BEST_EFFORT.

**Quaternion**
: A mathematical representation of 3D rotation using 4 values (x, y, z, w), avoiding gimbal lock.

---

## R

**RealSense**
: Intel's brand of depth cameras used for 3D perception and SLAM.

**REP (ROS Enhancement Proposal)**
: Standards documents for ROS conventions. Example: REP 103 defines coordinate frame conventions.

**RGB-D**
: RGB (color) + Depth camera, providing both visual and distance information.

**Robot State Publisher**
: A ROS 2 node that publishes the robot's TF transforms based on URDF and joint states.

**ROS 2 (Robot Operating System 2)**
: An open-source framework for robot software development, successor to ROS 1.

**rqt**
: A Qt-based GUI framework for ROS 2, providing tools like topic visualization and parameter tuning.

**RViz2**
: The 3D visualization tool for ROS 2, displaying sensor data, TF frames, and robot models.

---

## S

**Sensor Fusion**
: Combining data from multiple sensors (camera, LiDAR, IMU) to improve perception accuracy.

**Service** (ROS 2)
: A synchronous request-response communication pattern. Example: Trigger a calculation and wait for result.

**SLAM (Simultaneous Localization and Mapping)**
: Building a map while tracking the robot's position within it.

**Subscriber** (ROS 2)
: A node or component that receives messages from a topic.

---

## T

**Task Planning**
: Decomposing high-level goals (e.g., "fetch coffee") into low-level actions (navigate, grasp, return).

**TF2 (Transform Library 2)**
: ROS 2 system for tracking coordinate frame relationships over time.

**Topic** (ROS 2)
: A named communication channel for publishing and subscribing to messages asynchronously.

**Trajectory**
: A time-parameterized path defining position and velocity at each timestep for a robot's motion.

---

## U

**URDF (Unified Robot Description Format)**
: An XML format for describing robot geometry, kinematics, and dynamics in ROS.

**USD (Universal Scene Description)**
: Pixar's file format for 3D scenes, used by Isaac Sim for robot and environment models.

---

## V

**VLA (Vision-Language-Action)**
: An AI architecture combining vision (cameras), language understanding (LLMs), and action (motor commands).

**VRAM (Video RAM)**
: Memory on a GPU for storing textures and computation data. Example: RTX 3060 has 6GB VRAM.

**VSLAM (Visual SLAM)**
: SLAM using camera images instead of LiDAR for mapping and localization.

---

## W

**Whisper**
: OpenAI's neural network for speech-to-text transcription, supporting 99 languages.

**Workspace** (ROS 2)
: A directory structure (src/, build/, install/) containing ROS 2 packages for development.

**WSL2 (Windows Subsystem for Linux 2)**
: A compatibility layer for running Linux on Windows, useful for ROS 2 development on Windows machines.

---

## X

**XML (eXtensible Markup Language)**
: A markup language used for URDF robot descriptions, launch files (ROS 2), and configuration.

---

## Y

**YAML (YAML Ain't Markup Language)**
: A human-readable data serialization format used for ROS 2 configuration files.

**YOLO (You Only Look Once)**
: A family of real-time object detection models. Example: YOLOv8 for detecting COCO classes.

---

## Symbols and Abbreviations

**6-DOF**: 6 Degrees of Freedom
**API**: Application Programming Interface
**CI/CD**: Continuous Integration / Continuous Deployment
**CPU**: Central Processing Unit
**DDS**: Data Distribution Service
**DOF**: Degrees of Freedom
**FPS**: Frames Per Second
**GPU**: Graphics Processing Unit
**GUI**: Graphical User Interface
**I/O**: Input/Output
**IMU**: Inertial Measurement Unit
**LiDAR**: Light Detection and Ranging
**LLM**: Large Language Model
**LTS**: Long-Term Support
**MCP**: Model Context Protocol
**ML**: Machine Learning
**QoS**: Quality of Service
**RGB**: Red Green Blue
**RGB-D**: RGB + Depth
**ROS**: Robot Operating System
**SLAM**: Simultaneous Localization and Mapping
**TF**: Transform (ROS coordinate frame system)
**URDF**: Unified Robot Description Format
**USD**: Universal Scene Description
**VLA**: Vision-Language-Action
**VRAM**: Video RAM
**VSLAM**: Visual SLAM
**WSL**: Windows Subsystem for Linux
**YAML**: YAML Ain't Markup Language
**YOLO**: You Only Look Once

---

## Frame Convention (ROS REP 103)

**Right-Hand Rule**:
- **X**: Forward (red)
- **Y**: Left (green)
- **Z**: Up (blue)

**Rotations**:
- **Roll**: Rotation around X-axis
- **Pitch**: Rotation around Y-axis
- **Yaw**: Rotation around Z-axis

---

## Common ROS 2 Message Types

**geometry_msgs/Twist**
: Linear and angular velocity commands for mobile robot control.

**sensor_msgs/Image**
: Camera image data (RGB, grayscale, or depth).

**sensor_msgs/PointCloud2**
: 3D point cloud from LiDAR or depth camera.

**nav_msgs/Odometry**
: Robot pose and velocity estimates from odometry sources.

**std_msgs/String**
: Simple string message for text data.

**tf2_msgs/TFMessage**
: Transform data for coordinate frame relationships.

---

## Common ROS 2 Topics

**/cmd_vel**: Velocity commands (geometry_msgs/Twist)
**/joint_states**: Robot joint positions and velocities
**/odom**: Odometry data (nav_msgs/Odometry)
**/scan**: 2D laser scan data (sensor_msgs/LaserScan)
**/camera/image_raw**: Raw camera images (sensor_msgs/Image)
**/tf**: Transform tree (tf2_msgs/TFMessage)

---

**Can't find a term?** Ask in [GitHub Discussions](https://github.com/asadaligith/AI-Humanoid-Robotics-Book/discussions) or submit a suggestion via [Issues](https://github.com/asadaligith/AI-Humanoid-Robotics-Book/issues).
