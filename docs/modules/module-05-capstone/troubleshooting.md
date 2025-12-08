---
sidebar_position: 9
---

# Troubleshooting Guide

## Overview

This guide addresses common integration issues and edge cases encountered when deploying the autonomous humanoid capstone system. Each section provides symptom descriptions, root cause analysis, and step-by-step resolution procedures.

## General Debugging Strategy

Before diving into specific issues:

1. **Check System Status**: `ros2 topic echo /system/status` (current FSM state)
2. **Verify Node Health**: `ros2 node list` (all expected nodes running?)
3. **Inspect Logs**: `ros2 run rqt_console rqt_console` (error messages)
4. **Monitor Topics**: `ros2 topic hz <topic>` (message publication rates)
5. **Validate Transforms**: `ros2 run tf2_tools view_frames` (coordinate frame tree)

## Systematic Debugging Workflow

When diagnosing complex integration failures, follow this structured methodology to isolate root causes efficiently:

**Phase 1: Identify Symptom Location** (5 minutes). Monitor the `/system/status` topic to determine which FSM state the system is stuck in—IDLE, LISTENING, PLANNING, NAVIGATING, PERCEIVING, or MANIPULATING. This immediately narrows the failure domain to one of five capabilities. For example, if stuck in PLANNING state for >10 seconds, the issue resides in the LLM planner node, not navigation or perception.

**Phase 2: Verify Input Dependencies** (10 minutes). Before debugging the failing component, validate that its inputs are correct. If navigation fails, first confirm the LLM published a valid action sequence (`ros2 topic echo /llm/action_sequence`) and the goal pose is within map bounds. Many "navigation failures" are actually upstream planning failures that produce invalid goals. Checking inputs first prevents debugging the wrong component.

**Phase 3: Enable Mock Modes for Isolation** (5 minutes). Set the failing component's mock flag to `True` while keeping others real. If the system proceeds past the failure point with the mock enabled, the issue is confirmed within that specific capability's implementation—not integration logic. This binary search approach (mock vs real) eliminates 80% of possible failure locations within minutes, compared to hours of manual code inspection.

**Phase 4: Analyze Logs with Timestamps** (10 minutes). Use `rqt_console` to view error messages, but critically, check timestamps to verify message ordering. A "grasp failed" error at T+30s followed by "object not found" at T+25s indicates the detection failure caused the grasp attempt on a null object. Temporal analysis reveals causality chains that symptom-based debugging misses.

##  1. Voice Not Detected

### Symptoms
- Voice input node running but no transcriptions published to `/voice/transcribed_text`
- Console shows "No voice activity detected" repeatedly
- System stuck in `IDLE` or `LISTENING` state

### Root Causes

**A. Microphone Not Accessible**

Check microphone permissions and device availability:

```bash
# List audio devices
arecord -l

# Test microphone recording
arecord -d 3 -f cd test.wav
aplay test.wav  # Should hear playback
```

**Resolution**:
- Verify Docker container has `--device=/dev/snd:/dev/snd` flag
- Check PulseAudio permissions: `usermod -aG audio $USER`
- Use USB headset microphone as alternative

**B. Silence Threshold Too High**

Voice activity detection may be too conservative.

**Resolution**:
```yaml
# config/voice_config.yaml
silence_threshold: 0.01  # Lower to 0.005 for quieter environments
duration: 3.0  # Increase to 4.0 for longer commands
```

**C. Wrong Audio Input Device**

Multiple microphones may confuse device selection.

**Resolution**:
```python
# voice_input_node.py
# Explicitly set device index
import sounddevice as sd
print(sd.query_devices())  # List all devices
sd.default.device = 1  # Set to correct input device index
```

### Verification
```bash
# Monitor transcriptions
ros2 topic echo /voice/transcribed_text

# Expected output after speaking "bring me the mug":
data: "Bring me the mug"
```

---

## 2. LLM Timeout or Invalid Output

### Symptoms
- System stuck in `PLANNING` state for >30 seconds
- Error: "Failed to parse plan JSON"
- LLM returns hallucinated actions not in capability manifest

### Root Causes

**A. API Key Not Configured**

```bash
# Check if OpenAI API key is set
echo $OPENAI_API_KEY

# Expected: sk-proj-...
```

**Resolution**:
```bash
export OPENAI_API_KEY="sk-proj-YOUR_KEY_HERE"
# Or add to ~/.bashrc for persistence
```

**B. Network Connectivity Issues**

```bash
# Test API connectivity
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"

# Should return JSON with model list
```

**Resolution**:
- Check firewall rules (allow HTTPS to api.openai.com)
- Verify internet connection
- Use local LLM as fallback (Llama 2 via Ollama)

**C. LLM Output Schema Mismatch**

LLM may not follow expected JSON format.

**Resolution**:
```python
# llm_planner_node.py - add strict schema validation
import jsonschema

schema = {
    "type": "object",
    "properties": {
        "action_sequence": {"type": "array"},
        "confidence": {"type": "number"}
    },
    "required": ["action_sequence", "confidence"]
}

try:
    jsonschema.validate(instance=llm_output, schema=schema)
except jsonschema.ValidationError as e:
    self.get_logger().error(f"Invalid LLM output: {e.message}")
    # Request clarification or use fallback plan
```

### Verification
```bash
# Publish test command
ros2 topic pub /voice/transcribed_text std_msgs/String \
  "{data: 'Bring me the mug'}"

# Monitor LLM output
ros2 topic echo /llm/action_sequence

# Expected: JSON with action_sequence array
```

---

## 3. Navigation Failures

### Symptoms
- Nav2 goal aborted with "No valid path found"
- Robot gets stuck oscillating in place
- Collision detected in planning scene
- Navigation timeout after 30 seconds

### Root Causes

**A. Costmap Inflation Too Conservative**

Obstacles appear larger than reality, blocking narrow passages.

**Resolution**:
```yaml
# config/nav2_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.35  # Reduce from 0.55m to 0.35m
        cost_scaling_factor: 2.0  # Reduce from 3.0
```

**B. Map-Odometry Mismatch**

SLAM localization may have drifted.

**Resolution**:
```bash
# Check TF tree for map→odom transform
ros2 run tf2_tools view_frames

# Re-localize robot (publish initial pose estimate)
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0, y: 0}}}}"
```

**C. Goal Pose Unreachable**

Target location may be inside obstacle or outside map bounds.

**Resolution**:
```python
# Validate goal before sending to Nav2
def validate_goal(pose: PoseStamped) -> bool:
    # Check if within map bounds
    if not (MAP_MIN_X <= pose.pose.position.x <= MAP_MAX_X):
        return False

    # Check if goal is in free space (query costmap)
    costmap_value = query_costmap(pose.pose.position.x, pose.pose.position.y)
    return costmap_value < LETHAL_OBSTACLE_THRESHOLD
```

**D. Recovery Behaviors Not Configured**

Robot gives up immediately without attempting recovery.

**Resolution**:
```yaml
# config/nav2_params.yaml
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      max_retries: 3  # Retry spin recovery 3 times
```

### Verification
```bash
# Send test navigation goal
ros2 topic pub /navigation/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}"

# Monitor navigation status
ros2 topic echo /navigation/status

# Expected: "SUCCEEDED" within 15 seconds
```

---

## 4. Object Not Detected

### Symptoms
- Perception node running but no detections published
- System stuck in `PERCEIVING` state
- Detection confidence too low (&lt;0.6 threshold)
- Wrong object detected (cup instead of mug)

### Root Causes

**A. Camera Not Publishing Images**

```bash
# Check camera topic
ros2 topic hz /camera/image_raw

# Expected: 10-30 Hz
# If 0 Hz, camera node is down or device not connected
```

**Resolution**:
- Verify camera device: `v4l2-ctl --list-devices`
- Restart camera node: `ros2 run capstone_demo camera_node`
- Check Gazebo simulation camera plugin is active

**B. Object Not in YOLO Training Set**

COCO dataset may not include target object.

**Resolution**:
```yaml
# config/detection_classes.yaml
# Map LLM object names to closest COCO class
llm_to_coco_mapping:
  "coffee mug": "cup"  # YOLO trained on "cup", not "mug"
  "water bottle": "bottle"
```

**C. Confidence Threshold Too High**

Detection exists but confidence &lt;0.6 (rejected).

**Resolution**:
```python
# object_detection_node.py
self.conf_threshold = 0.5  # Reduce from 0.6 to 0.5

# Or use class-specific thresholds
class_thresholds = {
    'mug': 0.7,  # Higher for common objects
    'apple': 0.5  # Lower for small objects
}
```

**D. Lighting Conditions Poor**

Camera image too dark or overexposed.

**Resolution**:
```python
# Adaptive histogram equalization
import cv2
frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
frame_equalized = cv2.equalizeHist(frame_gray)
frame = cv2.cvtColor(frame_equalized, cv2.COLOR_GRAY2BGR)
```

### Verification
```bash
# Visualize detections
ros2 run rqt_image_view rqt_image_view /perception/annotated_image

# Should show bounding boxes around detected objects
```

---

## 5. Grasp Failures

### Symptoms
- Gripper closes but object not grasped (falls immediately)
- MoveIt planning fails with "IK solution not found"
- Manipulation timeout after 20 seconds
- Collision detected during grasp approach

### Root Causes

**A. Object Pose Inaccurate**

2D detection doesn't provide accurate 3D position.

**Resolution**:
```python
# Use depth camera to estimate 3D pose
from sensor_msgs.msg import PointCloud2

def estimate_3d_pose(detection_2d, depth_image):
    # Project 2D bbox center to 3D using depth
    center_x, center_y = detection_2d.bbox.center.x, detection_2d.bbox.center.y
    depth_value = depth_image[int(center_y), int(center_x)]

    # Convert to 3D point in camera frame
    point_3d = project_to_3d(center_x, center_y, depth_value)

    return point_3d
```

**B. Pre-Grasp Offset Insufficient**

Gripper collides with table during approach.

**Resolution**:
```yaml
# config/grasp_poses.yaml
mug:
  pre_grasp_offset: 0.15  # Increase from 0.10m to 0.15m
  approach_vector: [0, 0, -1]  # Ensure vertical approach
```

**C. Gripper Force Too Weak**

Object slips due to insufficient grip force.

**Resolution**:
```python
# manipulation_controller.py
def close_gripper(self, force=150):  # Increase from default 100N
    gripper_msg = JointState()
    gripper_msg.name = ['gripper_joint']
    gripper_msg.effort = [force]  # Higher force
    self.gripper_pub.publish(gripper_msg)
```

**D. IK Solver Failure**

Target pose outside robot's reachable workspace.

**Resolution**:
```python
# Validate pose before IK query
def is_reachable(target_pose: Pose) -> bool:
    distance = sqrt(target_pose.position.x**2 + target_pose.position.y**2)
    return MIN_REACH <= distance <= MAX_REACH  # e.g., 0.2m to 0.8m

if not is_reachable(grasp_pose):
    self.get_logger().error("Object outside reachable workspace")
    # Move robot base closer before grasping
    self.navigate_to_pre_grasp_location(grasp_pose)
```

### Verification
```bash
# Send test pick command
ros2 topic pub /manipulation/pick_target geometry_msgs/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.8}}}"

# Monitor manipulation status
ros2 topic echo /manipulation/status

# Expected: "PICK_SUCCESS" within 10 seconds
```

---

## Edge Case Handling

### Ambiguous Voice Commands

**Scenario**: "Get the object from the table" (which object?)

**Resolution**:
```python
# llm_planner_node.py - detect ambiguity in plan
if plan['confidence'] < 0.7:
    clarification = "Which object would you like? Please specify color or type."
    self.clarification_pub.publish(String(data=clarification))
    self.state = State.AWAITING_CLARIFICATION
```

### No Valid Navigation Path

**Scenario**: Kitchen blocked by obstacles, no path exists

**Resolution**:
```python
# integration_demo.py - retry with replanning
if nav_status == "ABORTED" and self.retry_count < 3:
    self.retry_count += 1
    # Clear costmap and try alternative route
    self.clear_costmap_service.call_async(Empty.Request())
    time.sleep(1.0)
    self.execute_navigation(params)  # Retry
else:
    # Request LLM to replan with different location
    self.request_llm_replan("Navigation to kitchen failed. Try alternative route.")
```

### Wrong Object Grasped

**Scenario**: Grasped cup instead of mug (visually similar)

**Resolution**:
```python
# Validate object after grasp using gripper force sensor
expected_object = "mug"
expected_weight = 0.3  # kg (from object database)

actual_weight = self.get_gripper_force() / 9.81  # Convert force to mass

if abs(actual_weight - expected_weight) > 0.1:
    self.get_logger().warn(f"Grasped wrong object (expected {expected_weight}kg, got {actual_weight}kg)")
    # Drop object and retry with stricter detection
    self.open_gripper()
    self.execute_detection(params, conf_threshold=0.8)  # Increase threshold
```

### LLM API Down

**Scenario**: OpenAI API returns 503 Service Unavailable

**Resolution**:
```python
# Use cached plans for common commands
CACHED_PLANS = {
    "bring me the mug from the kitchen": [
        {"action": "navigate_to", "params": {"location": "kitchen"}},
        {"action": "detect_object", "params": {"name": "mug"}},
        {"action": "pick_object", "params": {"object_id": "mug_0"}},
        {"action": "navigate_to", "params": {"location": "user"}},
        {"action": "place_object", "params": {"location": "table"}}
    ]
}

try:
    plan = self.query_llm(voice_command)
except requests.exceptions.RequestException:
    # Fallback to cached plan
    plan = CACHED_PLANS.get(voice_command.lower())
    if plan:
        self.get_logger().warn("LLM API down, using cached plan")
    else:
        self.transition_to(State.FAILED)
```

---

## Diagnostic Tools

### ROS 2 Introspection

```bash
# List all active nodes
ros2 node list

# Check node info
ros2 node info /integration_demo

# Monitor topic publication rate
ros2 topic hz /system/status

# Echo topic for debugging
ros2 topic echo /voice/transcribed_text

# Visualize TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Gazebo Debugging

```bash
# Enable Gazebo verbose logging
gazebo --verbose

# Inspect model poses
gz model -m unitree_g1 -p

# Check sensor outputs
gz topic -e /camera/image_raw
```

### Log Analysis

```bash
# View all logs
ros2 run rqt_console rqt_console

# Filter by severity
ros2 run rqt_console rqt_console --filter-severity error

# Export logs to file
ros2 run rqt_console rqt_console > capstone_logs.txt
```

---

## Summary

Effective troubleshooting requires systematic diagnosis:

✅ **Monitor system status** continuously (`/system/status` topic) to identify which capability failed

✅ **Verify component isolation** - test each capability independently using methods from [Testing Methodology](testing-methodology.md)

✅ **Inspect logs and metrics** - ROS 2 console, Gazebo verbose output, resource monitoring

✅ **Implement graceful degradation** - retry logic, fallback plans, clarification requests

Most integration failures trace to configuration mismatches (wrong thresholds, missing API keys, incorrect transforms) rather than algorithmic bugs. Systematic validation of each component's inputs and outputs accelerates root cause identification.

**Next Steps**: Return to [System Architecture](chapter-01-architecture.md) for component interaction overview, or proceed to [Benchmarking](benchmarking.md) to quantify system performance after resolving issues.
