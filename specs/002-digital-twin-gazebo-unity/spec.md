# Feature Specification: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature Branch**: `002-digital-twin-gazebo-unity`
**Created**: 2025-12-07
**Status**: Draft
**Input**: User description: "Module 2: The Digital Twin (Gazebo & Unity) - Target audience: Students learning simulation-based robotics and environment modeling. Focus: Setting up Gazebo (Fortress/Humble integration), Physics simulation (gravity, collisions, joints), Simulating sensors (LiDAR, Depth Cameras, IMU), Introduction to Unity for robot visualization. Success criteria: Students can load a humanoid URDF into Gazebo, spawn sensors and verify data streams, understand SDF vs URDF differences, at least 3 working simulation examples included, Unity section includes a minimal scene + humanoid visualization. Constraints: Chapter length 700–1500 words, all tasks must be reproducible on Ubuntu 22.04, no advanced Unity game design—only robotics visualization."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Loading Humanoid Robot into Gazebo Simulation (Priority: P1)

A student wants to see their humanoid robot from Module 1 come alive in a physics-based simulation environment.

**Why this priority**: This is the foundational step for all simulation work. Students must successfully load and visualize their robot before progressing to physics tuning or sensor integration. It builds directly on Module 1 URDF knowledge.

**Independent Test**: Can be fully tested by launching Gazebo, loading the URDF, and verifying the robot appears with correct geometry and joint structure. Delivers immediate visual feedback and confidence in simulation workflow.

**Acceptance Scenarios**:

1. **Given** student has Gazebo Fortress installed with ROS 2 Humble integration, **When** they launch Gazebo and load their humanoid URDF file, **Then** the robot appears in the simulation world with all links and joints visible
2. **Given** robot is loaded in Gazebo, **When** student inspects joint states using Gazebo GUI, **Then** they can identify all joints and verify joint limits match URDF specifications
3. **Given** robot model is in simulation, **When** student applies gravity, **Then** the robot responds to physics correctly (falls, collides with ground plane)

---

### User Story 2 - Understanding Physics Simulation Parameters (Priority: P2)

A student wants to understand how physics engines work by experimenting with gravity, friction, collision properties, and joint dynamics.

**Why this priority**: Physics simulation is the core value of Gazebo. Understanding how to tune physics parameters is essential for realistic robot behavior and prepares students for advanced control work.

**Independent Test**: Can be fully tested by modifying SDF properties, running simulations, and observing behavioral changes (e.g., changing friction affects sliding). Delivers understanding of simulation realism.

**Acceptance Scenarios**:

1. **Given** robot is in Gazebo world, **When** student modifies gravity parameters (e.g., from 9.81 to 3.71 m/s² for Mars), **Then** robot motion reflects the gravity change
2. **Given** student wants to adjust contact physics, **When** they modify friction coefficients in SDF file, **Then** robot sliding behavior changes observably
3. **Given** student configures joint damping and friction, **When** they command joint motions, **Then** joint response speed and settling behavior reflects the tuning parameters

---

### User Story 3 - Spawning and Configuring Virtual Sensors (Priority: P3)

A student wants to equip their simulated humanoid with sensors (LiDAR, depth camera, IMU) and verify sensor data streams in ROS 2.

**Why this priority**: Sensor simulation enables testing perception algorithms without physical hardware. This is critical for developing vision, mapping, and localization capabilities.

**Independent Test**: Can be fully tested by spawning sensors via SDF, launching sensor plugins, and subscribing to ROS 2 topics to verify data publication. Delivers hands-on sensor integration experience.

**Acceptance Scenarios**:

1. **Given** student adds a depth camera sensor to robot head in SDF, **When** they launch simulation with camera plugin, **Then** depth images are published to ROS 2 topic and viewable in RViz2
2. **Given** student attaches LiDAR sensor to robot torso, **When** they spawn obstacles in Gazebo world, **Then** LiDAR point cloud data reflects obstacle positions accurately
3. **Given** student configures IMU sensor, **When** they tilt or move the robot, **Then** IMU publishes orientation and acceleration data on ROS 2 topics

---

### User Story 4 - Understanding SDF vs URDF Trade-offs (Priority: P4)

A student wants to understand when to use URDF (robot description) versus SDF (world and simulation description) and how they complement each other.

**Why this priority**: Confusion between URDF and SDF is a common beginner mistake. Clear understanding enables students to structure simulation environments correctly.

**Independent Test**: Can be fully tested by comparing equivalent robot descriptions in both formats and identifying key differences. Delivers conceptual clarity for simulation architecture.

**Acceptance Scenarios**:

1. **Given** student has a URDF robot description, **When** they learn SDF structure, **Then** they can identify which robot properties belong in URDF (kinematics, visuals) vs SDF (physics plugins, sensors, world elements)
2. **Given** student wants to create a complete simulation, **When** they design their file structure, **Then** they use URDF for robot definition and SDF for world composition with physics properties
3. **Given** student encounters simulation-specific features (e.g., sensor noise models), **When** they need to add these, **Then** they correctly choose SDF for plugin configuration

---

### User Story 5 - Visualizing Robot in Unity for Cross-Platform Display (Priority: P5)

A student wants to create an alternative visualization of their humanoid robot using Unity for presentations or remote monitoring.

**Why this priority**: Unity provides accessible, cross-platform visualization that can reach non-Linux users. This broadens the audience for robotics demonstrations and introduces Unity ecosystem for future VR/AR applications.

**Independent Test**: Can be fully tested by setting up Unity scene, importing robot model via Unity Robotics Hub, and establishing ROS 2 communication. Delivers basic Unity-ROS integration skills.

**Acceptance Scenarios**:

1. **Given** student has Unity installed with Unity Robotics Hub package, **When** they import URDF using URDF Importer, **Then** humanoid robot appears in Unity scene with correct visual geometry
2. **Given** robot is in Unity scene, **When** student configures ROS-TCP connection, **Then** Unity can subscribe to ROS 2 joint state topics
3. **Given** ROS 2 communication is established, **When** student sends joint commands from ROS 2, **Then** Unity visualization reflects joint movements in real-time

---

### Edge Cases

- What happens when URDF contains meshes not supported by Gazebo? (Should provide clear error messages and fallback to primitive shapes)
- How does system handle sensor data at high frequencies (e.g., 100Hz depth camera)? (Should demonstrate performance considerations and throttling)
- What if Unity loses ROS-TCP connection during visualization? (Should show reconnection handling)
- How do physics solvers handle unstable configurations (e.g., very light robot on very heavy base)? (Should demonstrate solver parameter tuning)
- What happens when SDF world file references missing models? (Should show proper error handling and model path configuration)
- How does Gazebo handle URDF/SDF with conflicting inertial properties? (Should explain precedence rules)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Module MUST provide step-by-step instructions for installing Gazebo Fortress with ROS 2 Humble integration on Ubuntu 22.04
- **FR-002**: Module MUST include at least 3 complete working simulation examples: (1) basic robot loading, (2) sensor integration, (3) physics parameter tuning
- **FR-003**: Module MUST demonstrate loading a humanoid URDF into Gazebo and verifying all joints and links appear correctly
- **FR-004**: Module MUST explain key physics simulation parameters: gravity, friction, damping, collision properties, and demonstrate their effects
- **FR-005**: Module MUST provide examples of spawning and configuring at least 3 sensor types: LiDAR, depth camera, and IMU
- **FR-006**: Module MUST show how to verify sensor data streams using ROS 2 topic inspection commands (ros2 topic list, ros2 topic echo)
- **FR-007**: Module MUST include clear comparison table or explanation of SDF vs URDF differences, use cases, and when to use each format
- **FR-008**: Module MUST provide Unity setup instructions including Unity Robotics Hub installation and URDF Importer package
- **FR-009**: Module MUST demonstrate creating a minimal Unity scene with humanoid robot visualization connected to ROS 2 joint states
- **FR-010**: Module length MUST be between 700-1500 words excluding code snippets and configuration files
- **FR-011**: All examples MUST be reproducible on Ubuntu 22.04 with standard hardware (no GPU requirements for Gazebo section)
- **FR-012**: Module MUST include practical exercises for students to modify physics parameters and observe behavioral changes

### Key Entities

- **Gazebo Simulation World**: Complete simulation environment containing robot models, environmental objects, lighting, and physics configuration
- **SDF (Simulation Description Format)**: XML format for describing complete simulation scenes including worlds, models, physics properties, plugins, and sensors
- **Physics Engine**: Computational system that simulates gravity, collisions, friction, joint dynamics, and other physical forces
- **Virtual Sensor**: Simulated sensor component (LiDAR, camera, IMU) that generates synthetic data streams matching physical sensor characteristics
- **Sensor Plugin**: Gazebo plugin that implements sensor physics, noise models, and publishes data to ROS 2 topics
- **Unity Scene**: Container for game objects including robot models, cameras, lighting, and environmental assets
- **Unity Robotics Hub**: Suite of Unity packages enabling ROS integration including ROS-TCP connector and URDF Importer
- **URDF Importer**: Unity package that converts URDF robot descriptions into Unity GameObjects with articulation hierarchies
- **ROS-TCP Connection**: Network bridge enabling real-time message exchange between ROS 2 and Unity applications

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can successfully launch Gazebo Fortress with ROS 2 Humble integration and load a humanoid URDF within 10 minutes of completing the module
- **SC-002**: Students can spawn at least 2 virtual sensors on their robot and verify data publication to ROS 2 topics using command-line tools
- **SC-003**: Students can modify at least 3 physics parameters (gravity, friction, damping) and observe corresponding behavioral changes in simulation
- **SC-004**: Students can explain the difference between URDF and SDF in 2-3 sentences and correctly identify which format to use for specific simulation needs
- **SC-005**: Students can set up a minimal Unity scene, import their humanoid robot, and establish ROS 2 communication within 30 minutes
- **SC-006**: 85% of students successfully complete all 3 working simulation examples without encountering environment-specific errors
- **SC-007**: Students can visualize live joint state data from ROS 2 in Unity, demonstrating bidirectional communication setup
- **SC-008**: Module content stays within 700-1500 word count limit while maintaining clarity and completeness

### Learning Outcomes

- **LO-001**: Students understand how physics simulation engines approximate real-world dynamics and limitations of simulation accuracy
- **LO-002**: Students can configure virtual sensors to match real sensor specifications (field of view, range, frequency, noise characteristics)
- **LO-003**: Students recognize when to use Gazebo (high-fidelity physics) versus Unity (cross-platform visualization, VR/AR)
- **LO-004**: Students gain confidence in debugging simulation issues using Gazebo logs and ROS 2 introspection tools

## Scope & Constraints *(mandatory)*

### In Scope

- Gazebo Fortress installation and ROS 2 Humble integration setup
- Loading humanoid URDF files into Gazebo simulation
- Core physics simulation concepts: gravity, collisions, friction, joint dynamics
- Spawning and configuring virtual sensors: LiDAR, depth cameras, IMU
- Verifying sensor data streams using ROS 2 topics
- Understanding SDF format and comparing with URDF
- Basic Unity setup with Unity Robotics Hub
- URDF import into Unity using URDF Importer
- Minimal Unity scene creation with robot visualization
- ROS-TCP connection setup for Unity-ROS 2 communication
- Practical physics parameter tuning exercises

### Out of Scope

- Photorealistic rendering in Unity (focus is functional visualization only)
- Advanced Unity game design, shaders, or visual effects
- Large-scale environment creation or terrain modeling
- Multi-robot simulation scenarios
- Advanced Gazebo plugins or custom plugin development
- Real-time performance optimization for complex scenes
- VR/AR integration in Unity (mentioned as future possibility only)
- Custom sensor plugin development beyond configuration
- Distributed simulation across multiple machines
- Hardware-in-the-loop (HIL) simulation

### Assumptions

- Students have completed Module 1 and have a working humanoid URDF
- Students are running Ubuntu 22.04 LTS operating system
- Students have ROS 2 Humble already installed and configured
- Students have basic familiarity with XML file structure (from URDF work)
- For Unity section: Students have access to Windows, macOS, or Linux machine capable of running Unity Editor
- Students have sufficient disk space for Unity installation (~10GB)
- Internet connection available for downloading Gazebo models and Unity packages
- Students understand basic ROS 2 concepts (topics, nodes) from Module 1

### Dependencies

- Gazebo Fortress (gz-sim7) with ROS 2 Humble integration packages
- ros-humble-ros-gz bridge packages for ROS 2-Gazebo communication
- Unity Editor 2020.3 LTS or newer
- Unity Robotics Hub package (ROS-TCP Connector, URDF Importer)
- Standard Gazebo sensor plugins (camera, lidar, imu)
- RViz2 for visualizing sensor data
- Example Gazebo worlds and models from Gazebo Fuel repository
- Python 3.10+ for launch file scripts

## Non-Functional Requirements *(optional)*

### Performance

- Gazebo simulation MUST maintain at least 30 FPS (real-time factor ≥ 0.8) with humanoid robot and 2-3 sensors on mid-range hardware (Intel i5, 8GB RAM, integrated graphics)
- Sensor data publication latency MUST be under 100ms for typical configurations
- Unity visualization MUST achieve at least 30 FPS on recommended hardware when displaying single humanoid robot

### Usability

- All Gazebo launch commands MUST include clear explanations of command-line arguments
- Physics parameter changes MUST produce observable effects within 5 seconds of simulation time
- Error messages for missing models or sensors MUST include helpful hints for resolution
- Unity URDF import process MUST complete in under 2 minutes for typical humanoid models

### Accessibility

- All simulation visualization MUST be describable in text for visually impaired students (e.g., "robot falls to ground plane")
- Configuration files MUST use clear, descriptive naming conventions
- Module MUST include troubleshooting section for common setup issues (missing dependencies, path configuration)

## Open Questions & Assumptions *(optional)*

### Assumptions Made

1. **Gazebo Version**: Assuming Gazebo Fortress (modern gz-sim) rather than legacy Gazebo Classic (gazebo11). Fortress has better ROS 2 integration and is the recommended version for new projects.

2. **Unity Scope**: Assuming "minimal" Unity section means basic visualization only - importing robot, connecting to ROS 2, showing joint movements. Not covering lighting, materials, UI, or game mechanics.

3. **Sensor Complexity**: Assuming basic sensor configuration is sufficient (attaching sensors, setting update rates, verifying data). Not covering sensor noise modeling, custom ray patterns, or advanced plugin parameters.

4. **Physics Depth**: Assuming introductory physics concepts - demonstrating that parameters exist and showing observable effects. Not deriving equations of motion or explaining solver algorithms in detail.

5. **Example Complexity**: Assuming "working examples" means students can copy-paste/run commands successfully. Not requiring students to write simulation code from scratch, just modify existing examples.

6. **Unity Platform**: Assuming Unity examples demonstrate workflow on one platform (e.g., Ubuntu) but acknowledge cross-platform capability. Not providing platform-specific instructions for Windows/macOS Unity setup.
