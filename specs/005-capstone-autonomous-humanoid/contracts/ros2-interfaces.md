# ROS 2 Interface Contracts - Capstone Autonomous Humanoid

**Purpose**: Define all ROS 2 communication interfaces (topics, services, actions) for the five-capability integration system.

**Version**: 1.0.0
**Last Updated**: 2025-12-07
**Robot Platform**: Unitree G1 Humanoid

---

## Overview

The capstone system uses **5 primary nodes** communicating via ROS 2 topics, services, and actions. This document provides the canonical reference for all interfaces.

**Communication Patterns**:
- **Topics** (pub/sub): High-frequency data streams, loose coupling
- **Services** (req/rep): Synchronous queries, tight coupling
- **Actions** (goal-oriented): Long-running operations with feedback and cancellation

---

## 1. Voice Input Node

**Node Name**: `/voice_input_node`

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/voice/transcribed_text` | `std_msgs/String` | On-demand | Transcribed voice commands from Whisper |

**Message Format**:
```python
# std_msgs/String
data: "Bring me the red cup from the kitchen table"
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `whisper_model` | string | "base" | Whisper model size (tiny, base, small, medium, large) |
| `language` | string | "en" | Language code (ISO 639-1) |
| `sample_rate` | int | 16000 | Audio sample rate (Hz) |
| `duration` | double | 3.0 | Recording duration per capture (seconds) |
| `silence_threshold` | double | 0.01 | Voice activity detection threshold (0.0-1.0) |
| `device` | string | "cpu" | Compute device ("cpu" or "cuda") |

---

## 2. LLM Planner Node

**Node Name**: `/llm_planner_node`

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/voice/transcribed_text` | `std_msgs/String` | Voice commands to plan |
| `/system/capability_manifest` | `std_msgs/String` | Available robot actions (JSON) |

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/planning/action_sequence` | `std_msgs/String` | On-demand | Generated action plan (JSON) |
| `/planning/clarification_request` | `std_msgs/String` | On-demand | Questions when command is ambiguous |

**Action Sequence Format** (JSON):
```json
{
  "command_understood": true,
  "clarification_needed": null,
  "action_sequence": [
    {"action": "navigate_to", "params": {"location": "kitchen_table"}},
    {"action": "detect_object", "params": {"name": "cup", "color": "red"}},
    {"action": "pick_object", "params": {"object_id": "red_cup_001"}},
    {"action": "navigate_to", "params": {"location": "user_location"}},
    {"action": "place_object", "params": {"location": "user_hand"}}
  ]
}
```

### Services

| Service | Service Type | Description |
|---------|-------------|-------------|
| `/planning/validate_plan` | `std_srvs/Trigger` | Validate plan against capability manifest |

**Service Request/Response**:
```python
# Request (std_srvs/Trigger)
# (empty request)

# Response
bool success        # True if plan is valid
string message      # Validation details or error message
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | string | "gpt-4" | LLM model name |
| `temperature` | double | 0.0 | Sampling temperature (0.0 = deterministic) |
| `max_tokens` | int | 1000 | Maximum response tokens |

---

## 3. Navigation Controller

**Node Name**: `/navigation_controller`

### Action Servers

| Action | Action Type | Description |
|--------|-------------|-------------|
| `/navigation/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Navigate to goal pose |

**Action Goal**:
```python
# nav2_msgs/NavigateToPose.Goal
geometry_msgs/PoseStamped pose
  Header header
    stamp: current time
    frame_id: "map"
  Pose pose
    Point position
      float64 x
      float64 y
      float64 z
    Quaternion orientation
      float64 x, y, z, w
```

**Action Feedback**:
```python
# nav2_msgs/NavigateToPose.Feedback
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float64 distance_remaining
```

**Action Result**:
```python
# nav2_msgs/NavigateToPose.Result
# (empty - success indicated by action status)
```

**Action Status Codes**:
- `STATUS_SUCCEEDED`: Goal reached successfully
- `STATUS_ABORTED`: Navigation failed (obstacle blocking, no valid path)
- `STATUS_CANCELED`: User or system canceled navigation

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/navigation/status` | `std_msgs/String` | 1 Hz | Current navigation state |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_retries` | int | 3 | Maximum navigation retry attempts |
| `timeout` | double | 30.0 | Navigation timeout (seconds) |

---

## 4. Object Detection Node

**Node Name**: `/object_detection_node`

### Subscribed Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | RGB camera feed |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz | Depth image (optional, for 3D localization) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Camera calibration parameters |

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/perception/detected_objects` | `vision_msgs/Detection2DArray` | 10 Hz | Detected objects with bounding boxes |

**Detection Message Format**:
```python
# vision_msgs/Detection2DArray
Header header
  stamp: detection timestamp
  frame_id: "camera_optical_frame"

Detection2D[] detections
  Header header
  string[] results  # Object class names
    string id       # e.g., "cup"
    float64 score   # Confidence (0.0-1.0)

  BoundingBox2D bbox
    Pose2D center
      float64 x, y      # Pixel coordinates
      float64 theta     # Orientation
    float64 size_x      # Width (pixels)
    float64 size_y      # Height (pixels)

  string id             # Unique detection ID (e.g., "red_cup_001")
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | string | "yolov8n" | Detection model (yolov8n, yolov8s, isaac_ros) |
| `confidence_threshold` | double | 0.7 | Minimum confidence to publish detection |
| `nms_threshold` | double | 0.45 | Non-maximum suppression threshold |
| `target_classes` | string[] | ["cup", "mug", "bottle", ...] | Classes to detect |

---

## 5. Manipulation Controller

**Node Name**: `/manipulation_controller`

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/perception/detected_objects` | `vision_msgs/Detection2DArray` | Objects available for grasping |
| `/joint_states` | `sensor_msgs/JointState` | Current robot joint positions |

### Action Servers

| Action | Action Type | Description |
|--------|-------------|-------------|
| `/manipulation/pick_object` | `custom_msgs/PickObject` | Pick detected object |
| `/manipulation/place_object` | `custom_msgs/PlaceObject` | Place held object at location |

**Pick Action Goal** (custom_msgs/PickObject):
```python
# PickObject.Goal
string object_id              # From detection (e.g., "red_cup_001")
geometry_msgs/Pose grasp_pose # Pre-computed grasp pose (optional)
```

**Pick Action Feedback**:
```python
# PickObject.Feedback
string phase                  # "approaching", "grasping", "lifting", "completed"
float64 distance_to_object    # Meters
sensor_msgs/JointState current_joints
```

**Pick Action Result**:
```python
# PickObject.Result
bool success
string message                # "Object grasped successfully" or error description
geometry_msgs/Pose final_pose # Object pose after grasp
```

**Place Action Goal** (custom_msgs/PlaceObject):
```python
# PlaceObject.Goal
string location               # "table", "user_hand", "bin", "shelf"
geometry_msgs/Pose place_pose # Target placement pose (optional)
```

**Place Action Feedback**:
```python
# PlaceObject.Feedback
string phase                  # "moving", "releasing", "retracting", "completed"
float64 distance_to_target    # Meters
```

**Place Action Result**:
```python
# PlaceObject.Result
bool success
string message
geometry_msgs/Pose final_pose
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_grasp_attempts` | int | 3 | Retry limit for failed grasps |
| `grasp_force` | double | 10.0 | Target grip force (Newtons) |
| `approach_distance` | double | 0.15 | Pre-grasp approach distance (meters) |

---

## 6. Integration State Machine

**Node Name**: `/integration_demo`

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/voice/transcribed_text` | `std_msgs/String` | Triggers planning state |
| `/planning/action_sequence` | `std_msgs/String` | Received action plan |
| `/planning/clarification_request` | `std_msgs/String` | Clarification needed |

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|--------------|------|-------------|
| `/system/status` | `std_msgs/String` | 1 Hz | Current state machine state and status |

**Status Message Format**:
```python
# std_msgs/String
data: "[NAVIGATING] Executing action 2/5: navigate_to kitchen_table"
```

---

## Message Type Definitions

### Custom Messages (custom_msgs package)

**ActionPlan.msg**:
```
bool command_understood
string clarification_needed
Action[] action_sequence

# Action.msg
string action
Parameter[] params

# Parameter.msg
string key
string value
```

**PickObject.action**:
```
# Goal
string object_id
geometry_msgs/Pose grasp_pose
---
# Result
bool success
string message
geometry_msgs/Pose final_pose
---
# Feedback
string phase
float64 distance_to_object
sensor_msgs/JointState current_joints
```

**PlaceObject.action**:
```
# Goal
string location
geometry_msgs/Pose place_pose
---
# Result
bool success
string message
geometry_msgs/Pose final_pose
---
# Feedback
string phase
float64 distance_to_target
```

---

## QoS Profiles

### Recommended QoS Settings

**Sensor Topics** (camera, depth):
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

**Command Topics** (voice, planning):
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**State Topics** (status):
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

---

## Interface Testing

### Verify Node Connectivity

```bash
# List all active topics
ros2 topic list

# Check topic info
ros2 topic info /voice/transcribed_text

# Echo topic data
ros2 topic echo /planning/action_sequence

# Check message type
ros2 interface show std_msgs/msg/String
```

### Test Services

```bash
# List services
ros2 service list

# Call service
ros2 service call /planning/validate_plan std_srvs/srv/Trigger
```

### Test Actions

```bash
# List actions
ros2 action list

# Send action goal
ros2 action send_goal /navigation/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}}}}"
```

---

## Compliance Validation

All interfaces must satisfy:

✅ **Type Safety**: All message types defined in standard packages or custom_msgs
✅ **Naming Convention**: Topics use `/subsystem/topic_name` hierarchy
✅ **QoS Correctness**: Sensor topics use BEST_EFFORT, commands use RELIABLE
✅ **Documentation**: Every interface has description and example
✅ **Testing**: Interface tests exist in `tests/module-05-capstone/test_interfaces.py`

---

**See Also**:
- [State Machine Contract](state-machine.md) - Execution flow and state transitions
- [LLM Prompts Contract](llm-prompts.md) - Prompt templates and output formats
- [Chapter 1: System Architecture](../../docs/modules/module-05-capstone/chapter-01-architecture.md)
