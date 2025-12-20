# Integration Architecture Specification

**Feature**: Cross-Module Integration Architecture
**Date**: 2025-12-20
**Status**: Draft
**Branch**: `001-project-readiness`

---

## Summary

This specification defines the integration architecture for the "Physical AI & Humanoid Robotics" educational platform, establishing how the 5 modules, RAG chatbot, platform UI, and capstone project integrate to deliver a cohesive learning experience. The architecture addresses data flow between modules, shared interfaces, deployment boundaries, and end-to-end validation strategies.

**Primary Objective**: Define explicit integration points, shared contracts, and data flow patterns to ensure seamless interaction between all platform components (documentation, code examples, RAG chatbot, simulations, and real-world deployment).

**Success Metric**: All components integrate without manual configuration; students can progress from Module 1 → Module 5 → Capstone with zero integration friction.

---

## Problem Statement

### Current State
- 5 modules exist as independent specifications with implied but undefined integration points
- RAG chatbot, documentation platform, and code examples lack explicit shared contracts
- No integration testing framework to validate cross-component interactions
- Unclear data flow between simulation (Gazebo/Isaac Sim) and real-world deployment (Jetson)

### Pain Points
1. **Fragmented Learning Experience**: Students must manually configure integrations between modules
2. **No Shared State**: Code examples cannot reference previous module outputs
3. **RAG Chatbot Isolation**: Chatbot cannot provide context-aware guidance based on student progress
4. **Deployment Ambiguity**: Unclear which components run where (local, cloud, edge)

### Success Criteria
- ✅ All modules share standardized ROS 2 interfaces (topics, services, actions)
- ✅ RAG chatbot has read-only access to student progress data
- ✅ Code examples use shared libraries for common tasks (perception, navigation, manipulation)
- ✅ Deployment architecture clearly defines component boundaries and communication protocols
- ✅ Integration tests validate end-to-end workflows (Module 1 output → Module 2 input)

---

## Technical Context

**Language/Version**: Python 3.10+, JavaScript ES6+, C++, ROS 2 Humble Hawksbill
**Primary Dependencies**: Docusaurus 3.x, FastAPI 0.104+, ROS 2 Humble, Gazebo Fortress, Isaac Sim 2023.1+
**Storage**: Qdrant Cloud (vector DB), Neon Postgres (conversation history), GitHub (code/docs)
**Testing**: pytest, Google Test, Jest, Playwright, integration tests (custom)
**Target Platform**: Ubuntu 22.04 LTS (development), NVIDIA Jetson Orin (edge deployment), Web browsers (documentation)

**Constraints**:
- ROS 2 Humble only (version lock for consistency)
- Free tier hosting (Render/Fly.io for backend, GitHub Pages for frontend)
- Offline-capable code examples (cannot depend on cloud services for core functionality)
- Cross-platform compatibility (x86_64 + ARM64 for Jetson)

---

## Integration Architecture

### 1. Component Diagram

```
┌────────────────────────────────────────────────────────────────────────┐
│                        STUDENT BROWSER                                  │
├────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────┐          ┌──────────────────────────┐         │
│  │  Docusaurus Site    │          │   RAG Chatbot Widget     │         │
│  │  (GitHub Pages)     │◄─────────┤   (React Component)      │         │
│  │                     │          │                          │         │
│  │  - Module Docs      │          │  - Session Management    │         │
│  │  - Learning Paths   │          │  - Query Interface       │         │
│  │  - Progress Tracker │          └──────────┬───────────────┘         │
│  └─────────────────────┘                     │ HTTPS (REST API)        │
│                                               │                          │
└───────────────────────────────────────────────┼──────────────────────────┘
                                                │
                         ┌──────────────────────▼──────────────────────┐
                         │      RAG CHATBOT BACKEND (Cloud)            │
                         │      Hosted on Render/Fly.io/Railway        │
                         ├─────────────────────────────────────────────┤
                         │                                              │
                         │  ┌──────────────┐    ┌─────────────────┐   │
                         │  │ FastAPI App  │───►│ RAG Pipeline    │   │
                         │  │ /api/chat    │    │ (LangChain)     │   │
                         │  └──────────────┘    └────────┬────────┘   │
                         │                               │             │
                         │  ┌────────────────────────────▼──────────┐ │
                         │  │  Qdrant Vector DB (Embeddings)        │ │
                         │  │  - Chapter embeddings                 │ │
                         │  │  - FAQ embeddings                     │ │
                         │  └───────────────────────────────────────┘ │
                         │                                              │
                         │  ┌───────────────────────────────────────┐ │
                         │  │  Neon Postgres (Conversation History) │ │
                         │  │  - Session tracking                   │ │
                         │  │  - Message history                    │ │
                         │  └───────────────────────────────────────┘ │
                         └──────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│                    STUDENT LOCAL WORKSTATION                           │
│                    (Ubuntu 22.04, ROS 2 Humble)                        │
├────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐           │
│  │ Module 1 Code  │  │ Module 2 Code  │  │ Module 3 Code  │           │
│  │ (ROS 2 Nodes)  │  │ (Gazebo Sim)   │  │ (Isaac Sim)    │           │
│  └───────┬────────┘  └───────┬────────┘  └───────┬────────┘           │
│          │                   │                   │                      │
│          └───────────────────┼───────────────────┘                      │
│                              │ ROS 2 DDS                                │
│                              ▼                                           │
│  ┌─────────────────────────────────────────────────────────┐           │
│  │         Shared ROS 2 Workspace (Capstone)               │           │
│  │  - Common interfaces (messages, services, actions)      │           │
│  │  - Shared libraries (perception, navigation, manip)     │           │
│  └─────────────────────────────────────────────────────────┘           │
│                                                                          │
└────────────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────────────┐
│                    NVIDIA JETSON ORIN (Edge Device)                    │
│                    (Optional - Sim-to-Real Transfer)                   │
├────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────┐           │
│  │         Capstone ROS 2 Workspace (Deployed)             │           │
│  │  - Isaac ROS nodes (GPU-accelerated)                    │           │
│  │  - Nav2 stack                                            │           │
│  │  - MoveIt 2                                              │           │
│  │  - VLA controller                                        │           │
│  └─────────────────────────────────────────────────────────┘           │
│                                                                          │
└────────────────────────────────────────────────────────────────────────┘
```

---

### 2. Shared Interfaces (ROS 2 Contracts)

All modules use standardized ROS 2 interfaces defined in a shared package: `robotics_book_interfaces`

**Package Structure**:
```
robotics_book_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── RobotState.msg          # Module 1 → Module 2,3,4,5
│   ├── PerceptionResult.msg    # Module 3 → Module 4,5
│   ├── VLACommand.msg          # Module 4 → Module 5
│   └── TaskStatus.msg          # Module 5 → Student feedback
├── srv/
│   ├── PlanPath.srv            # Module 3 navigation service
│   ├── GraspObject.srv         # Module 3 manipulation service
│   └── ExecuteVLA.srv          # Module 4 VLA execution service
└── action/
    ├── NavigateToGoal.action   # Module 3 navigation action
    └── PickAndPlace.action     # Module 3 manipulation action
```

**RobotState.msg** (Module 1 output, consumed by all):
```
# Standard robot state message
Header header
geometry_msgs/Pose pose          # Robot position/orientation
geometry_msgs/Twist velocity     # Linear/angular velocity
sensor_msgs/JointState joints    # Joint positions/velocities/efforts
uint8 status                     # 0=IDLE, 1=MOVING, 2=ERROR
string status_message            # Human-readable status
```

**PerceptionResult.msg** (Module 3 output):
```
Header header
sensor_msgs/Image image                     # Input image
vision_msgs/Detection2DArray detections    # YOLO detections
sensor_msgs/PointCloud2 depth_cloud        # Depth point cloud
geometry_msgs/PoseArray object_poses       # 3D object poses (from depth)
float32 confidence                         # Overall confidence score
```

**VLACommand.msg** (Module 4 output):
```
Header header
string voice_transcript           # Whisper transcription
string language_plan             # GPT-4 generated plan
string[] action_sequence         # Executable actions ["navigate(x,y)", "grasp(obj)"]
float32 confidence               # VLA confidence score
```

---

### 3. Data Flow Patterns

#### Pattern 1: Module Sequential Pipeline (Learning Path)

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  Module 1   │───►│  Module 2   │───►│  Module 3   │───►│  Module 4   │───►│  Module 5   │
│  ROS 2 Pub  │    │  URDF/      │    │  Perception │    │  VLA        │    │  Capstone   │
│  Sub, Basics│    │  Gazebo Sim │    │  Navigation │    │  Integration│    │  Integration│
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
      │                  │                  │                  │                  │
      └──────────────────┴──────────────────┴──────────────────┴──────────────────┘
                              Shared ROS 2 Workspace
                         (robotics_book_interfaces package)
```

**Example**: Student completes Module 1 → produces `/robot/state` topic → Module 2 URDF uses same state message → Module 3 subscribes to `/robot/state` for perception

#### Pattern 2: RAG Chatbot Context-Aware Guidance

```
Student Progress Tracker (Browser sessionStorage)
       │
       ├─► Current Module: "module-03-ai-robot-brain-isaac"
       ├─► Current Chapter: "chapter-03-vslam"
       └─► Completed Chapters: ["module-01/ch01", "module-01/ch02", ...]
             │
             ▼
    RAG Query: "How do I implement VSLAM?"
             │
             ▼
    Query Augmentation: "module:module-03 chapter:chapter-03 How do I implement VSLAM?"
             │
             ▼
    Qdrant Vector Search (with filters: {module: "module-03", chapter: "chapter-03"})
             │
             ▼
    Retrieval: Top 5 chunks from Chapter 3 (VSLAM) + related examples
             │
             ▼
    GPT-4 Generation: "Based on Chapter 3 of Module 3, here's how to implement VSLAM..."
```

**Implementation**:
```javascript
// Frontend: Send progress context with query
fetch('/api/chat', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    message: userQuery,
    session_id: getSessionId(),
    context: {
      current_module: 'module-03-ai-robot-brain-isaac',
      current_chapter: 'chapter-03-vslam',
      completed_chapters: getCompletedChapters()
    }
  })
});
```

```python
# Backend: Use context for filtered retrieval
from qdrant_client.models import Filter, FieldCondition, MatchValue

def rag_query(query: str, context: dict):
    # Build filter from student context
    filter_conditions = []
    if context.get('current_module'):
        filter_conditions.append(
            FieldCondition(key="module_id", match=MatchValue(value=context['current_module']))
        )

    # Search with filter
    results = qdrant_client.search(
        collection_name="robotics_docs",
        query_vector=get_embedding(query),
        query_filter=Filter(must=filter_conditions),
        limit=5
    )
    return results
```

#### Pattern 3: Sim-to-Real Transfer

```
Gazebo Simulation (Module 2)                  NVIDIA Jetson Orin (Real World)
┌─────────────────────────────┐               ┌─────────────────────────────┐
│  use_sim_time: true         │               │  use_sim_time: false        │
│  /camera/image (simulated)  │───Transfer───►│  /camera/color/image_raw    │
│  /scan (simulated laser)    │    Same ROS   │  /scan (real LiDAR)         │
│  /cmd_vel → Gazebo physics  │    2 Topics   │  /cmd_vel → Motor controllers│
└─────────────────────────────┘               └─────────────────────────────┘
         Shared Launch File: capstone/launch/main.launch.py
              (Parameterized: use_sim_time={true|false})
```

**Launch File Abstraction**:
```python
# capstone/launch/main.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Perception node (works with both sim and real cameras)
        Node(
            package='capstone',
            executable='perception_node',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])
```

**Usage**:
- Simulation: `ros2 launch capstone main.launch.py use_sim_time:=true`
- Real robot: `ros2 launch capstone main.launch.py use_sim_time:=false`

---

### 4. Deployment Boundaries

| Component | Deployment Target | Protocol | Authentication |
|-----------|------------------|----------|----------------|
| Docusaurus Site | GitHub Pages (static CDN) | HTTPS | None (public) |
| RAG Chatbot Frontend | Embedded in Docusaurus (React) | N/A | None (session-based) |
| RAG Chatbot Backend | Render/Fly.io/Railway (cloud) | HTTPS (REST API) | API key (optional for rate limiting) |
| Qdrant Vector DB | Qdrant Cloud (managed service) | gRPC over TLS | API key (from .env) |
| Neon Postgres | Neon Cloud (managed service) | PostgreSQL over TLS | Connection string (from .env) |
| Module Code Examples | Student's local machine | ROS 2 DDS (local network) | None (local only) |
| Capstone Workspace | Student's local machine OR Jetson Orin | ROS 2 DDS | SSH (for Jetson deployment) |

**Network Diagram**:
```
Internet (Public)
    │
    ├──► GitHub Pages CDN (Docusaurus Site) ──────────┐
    │                                                   │
    └──► Render/Fly.io/Railway (RAG Backend) ─────────┤
            │                                          │ HTTPS
            ├──► Qdrant Cloud (Vector DB)             │
            └──► Neon Cloud (Postgres)                │
                                                       │
                                                       ▼
                                               Student Browser
                                                       │
                                                       │ Downloads code
                                                       ▼
Student Local Machine (Ubuntu 22.04)          NVIDIA Jetson Orin
    │                                              │
    └─── ROS 2 DDS (local network only) ──────────┘
         (No internet required for code execution)
```

---

### 5. Integration Testing Strategy

#### Level 1: Unit Tests (Per Module)
- Module 1: Test ROS 2 nodes independently (publisher, subscriber, service)
- Module 2: Test URDF parsing, Gazebo world loading
- Module 3: Test perception algorithms (YOLO, depth estimation, VSLAM)
- Module 4: Test VLA components (Whisper, GPT-4 mocks, action execution)
- Module 5: Test capstone nodes individually

#### Level 2: Integration Tests (Cross-Module)
```python
# tests/integration/test_module_1_to_2.py
import pytest
import rclpy
from robotics_book_interfaces.msg import RobotState

def test_module1_publishes_robot_state():
    """Verify Module 1 publisher produces RobotState messages"""
    rclpy.init()
    node = rclpy.create_node('test_node')

    received_msg = None
    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    subscription = node.create_subscription(RobotState, '/robot/state', callback, 10)

    # Wait for message
    rclpy.spin_once(node, timeout_sec=5.0)

    assert received_msg is not None, "No RobotState message received from Module 1"
    assert received_msg.status in [0, 1, 2], "Invalid status value"

    rclpy.shutdown()

def test_module2_consumes_robot_state():
    """Verify Module 2 Gazebo simulation subscribes to RobotState"""
    # Launch Gazebo with robot model
    # Publish RobotState message
    # Verify robot moves in simulation
    pass
```

#### Level 3: End-to-End Tests (Full Learning Path)
```python
# tests/e2e/test_learning_path.py
import pytest
import subprocess

def test_student_workflow():
    """Simulate complete student learning path: Module 1 → 5 → Capstone"""

    # Step 1: Run Module 1 example (pub/sub)
    proc1 = subprocess.Popen(['ros2', 'run', 'module_01', 'publisher'])
    time.sleep(2)
    proc2 = subprocess.Popen(['ros2', 'run', 'module_01', 'subscriber'])
    time.sleep(5)

    # Verify communication
    result = subprocess.run(['ros2', 'topic', 'echo', '/example_topic', '--once'], capture_output=True, timeout=10)
    assert b'Hello' in result.stdout

    proc1.kill()
    proc2.kill()

    # Step 2: Load Module 2 URDF in Gazebo
    # ... (launch Gazebo, verify model loaded)

    # Step 3: Run Module 3 perception pipeline
    # ... (run YOLO, verify detections)

    # Step 4: Run Module 4 VLA pipeline
    # ... (simulate voice command, verify action execution)

    # Step 5: Run full capstone integration
    # ... (verify all nodes communicate)
```

#### Level 4: RAG Chatbot Integration Tests
```python
# tests/integration/test_rag_chatbot.py
import pytest
import requests

def test_rag_context_aware_query():
    """Verify RAG chatbot uses student progress context for filtering"""

    response = requests.post('http://localhost:8000/api/chat', json={
        'message': 'How do I implement VSLAM?',
        'session_id': 'test-session-123',
        'context': {
            'current_module': 'module-03-ai-robot-brain-isaac',
            'current_chapter': 'chapter-03-vslam'
        }
    })

    assert response.status_code == 200
    data = response.json()

    # Verify response mentions Module 3 content
    assert 'Module 3' in data['response'] or 'VSLAM' in data['response']

    # Verify retrieved chunks are from correct module
    assert all(chunk['metadata']['module_id'] == 'module-03-ai-robot-brain-isaac'
               for chunk in data['retrieved_chunks'])

def test_rag_refuses_off_topic():
    """Verify RAG chatbot refuses off-topic questions"""

    response = requests.post('http://localhost:8000/api/chat', json={
        'message': 'What is the weather today?',
        'session_id': 'test-session-456'
    })

    assert response.status_code == 200
    data = response.json()

    # Verify refusal message
    assert 'robotics' in data['response'].lower() or 'off-topic' in data['response'].lower()
```

---

## Implementation Plan

### Phase 1: Shared Interfaces (Week 2)
- [ ] Create `robotics_book_interfaces` ROS 2 package
- [ ] Define message types: RobotState, PerceptionResult, VLACommand, TaskStatus
- [ ] Define service types: PlanPath, GraspObject, ExecuteVLA
- [ ] Define action types: NavigateToGoal, PickAndPlace
- [ ] Build and test package

### Phase 2: Module Integration (Weeks 3-7)
- [ ] Update Module 1 examples to publish RobotState
- [ ] Update Module 2 URDF to subscribe to RobotState
- [ ] Update Module 3 perception to publish PerceptionResult
- [ ] Update Module 4 VLA to publish VLACommand
- [ ] Update Module 5 capstone to consume all interfaces

### Phase 3: RAG Chatbot Integration (Weeks 8-9)
- [ ] Implement progress tracker in Docusaurus (sessionStorage)
- [ ] Add context parameter to RAG API
- [ ] Implement filtered Qdrant queries
- [ ] Test context-aware responses

### Phase 4: Deployment Setup (Week 10)
- [ ] Deploy RAG backend to Render/Fly.io
- [ ] Deploy Docusaurus site to GitHub Pages
- [ ] Create Jetson deployment scripts
- [ ] Document deployment boundaries

### Phase 5: Integration Testing (Weeks 11-12)
- [ ] Write unit tests for each module
- [ ] Write cross-module integration tests
- [ ] Write RAG chatbot integration tests
- [ ] Write end-to-end learning path tests
- [ ] Run full test suite in CI/CD

---

## Acceptance Criteria

- ✅ All modules use `robotics_book_interfaces` package for communication
- ✅ Module 1 output (RobotState) successfully consumed by Module 2
- ✅ Module 3 output (PerceptionResult) successfully consumed by Module 4
- ✅ Module 4 output (VLACommand) successfully consumed by Module 5 capstone
- ✅ RAG chatbot returns context-aware responses (filtered by current module/chapter)
- ✅ Sim-to-real transfer works with single launch file (`use_sim_time` parameter)
- ✅ Integration tests pass in CI/CD pipeline
- ✅ Deployment diagram documented with clear component boundaries
- ✅ Students can complete full learning path without manual integration configuration

---

## Non-Goals

- ❌ Real-time multi-user collaboration (out of scope for MVP)
- ❌ Cloud-based ROS 2 execution (students run code locally)
- ❌ Advanced authentication/authorization (RAG chatbot is public, session-based only)
- ❌ Mobile app integration (web-only for MVP)
- ❌ Hardware abstraction layer for non-Jetson devices (Jetson-only for sim-to-real)

---

## References

- ROS 2 Interfaces: https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html
- Docusaurus Swizzling: https://docusaurus.io/docs/swizzling
- Qdrant Filtering: https://qdrant.tech/documentation/concepts/filtering/
- Integration Testing Best Practices: https://martinfowler.com/articles/integration-tests.html
