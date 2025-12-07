---
sidebar_position: 7
---

# Testing Methodology

## Overview

This guide documents how to test each of the five capstone capabilities **independently** before attempting full integration. Independent testing isolates failures, accelerates debugging, and validates acceptance criteria (voice: 85% accuracy, navigation: <10% failure, detection: correct bounding boxes, manipulation: 70% success).

## Why Test Capabilities in Isolation?

Debugging a failed end-to-end task ("fetch the mug from the kitchen") is difficult when failures could originate from any of five subsystems:

1. **Voice transcription** misheard command ("fetch the bug")
2. **LLM planning** generated invalid action sequence
3. **Navigation** failed to reach kitchen due to obstacle
4. **Perception** detected wrong object (cup instead of mug)
5. **Manipulation** grasp failed due to unreachable pose

**Isolation testing** decouples these dependencies, enabling you to verify each capability meets its acceptance criteria independently.

## Capability 1: Voice Input

### Acceptance Criteria
- **Accuracy**: ≥85% word error rate (WER) on capstone command set
- **Latency**: <5 seconds per 3-second audio clip
- **Coverage**: Handles all 20 test commands correctly

### Test Procedure

**1. Create Test Audio Dataset**

Record 20 voice commands (or use text-to-speech):

```bash
# Test commands (save as test_commands.txt)
Bring me the mug from the kitchen
Navigate to the living room
Pick up the red bottle
Place the cup on the table
Find the apple in the kitchen
Detect objects on the counter
Go to the charging station
Deliver the book to the bedroom
...
```

**2. Run Voice Node in Isolation**

```bash
# Terminal 1: Start voice input node
ros2 run capstone_demo voice_input_node

# Terminal 2: Monitor transcriptions
ros2 topic echo /voice/transcribed_text
```

**3. Play Test Audio and Validate**

For each test command:
1. Play audio clip (or speak into microphone)
2. Record transcribed output
3. Compare to ground truth using WER:

```python
# Calculate Word Error Rate (WER)
from jiwer import wer

ground_truth = "Bring me the mug from the kitchen"
hypothesis = "Bring me the bug from the kitchen"  # Transcription
error_rate = wer(ground_truth, hypothesis)  # 0.167 (1 out of 6 words wrong)
```

**4. Pass/Fail Criteria**

- **PASS**: Average WER ≤ 15% (≥85% accuracy) across 20 commands
- **PASS**: 95th percentile latency < 5 seconds
- **FAIL**: If any critical word misrecognized (e.g., "navigate" → "activate")

### Mock Mode for Integration Testing

```python
# integration_demo.py - bypass voice with hardcoded commands
MOCK_VOICE = True
if MOCK_VOICE:
    transcribed_text = "Bring me the mug from the kitchen"  # Hardcoded
else:
    transcribed_text = self.voice_result  # From real voice node
```

---

## Capability 2: LLM Planning

### Acceptance Criteria
- **Validity**: 100% of generated plans pass JSON schema validation
- **Correctness**: ≥90% of plans semantically match command intent
- **Latency**: <10 seconds per planning request (API call)

### Test Procedure

**1. Create Test Command Set**

```python
test_commands = [
    "Bring me the mug from the kitchen",
    "Navigate to the bedroom",
    "Pick up the red apple and place it on the table",
    "Detect objects in the living room",
    ...
]
```

**2. Run LLM Node in Isolation**

```bash
# Terminal 1: Start LLM planner
ros2 run capstone_demo llm_planner_node

# Terminal 2: Publish test commands
ros2 topic pub /voice/transcribed_text std_msgs/String \
  "{data: 'Bring me the mug from the kitchen'}"

# Terminal 3: Monitor generated plans
ros2 topic echo /llm/action_sequence
```

**3. Validate Output Schema**

```python
import jsonschema

# Expected schema (from llm-prompts.md contract)
schema = {
    "type": "object",
    "properties": {
        "actions": {"type": "array"},
        "confidence": {"type": "number", "minimum": 0, "maximum": 1}
    },
    "required": ["actions", "confidence"]
}

# Validate LLM output
try:
    jsonschema.validate(instance=llm_output, schema=schema)
    print("✓ Schema validation passed")
except jsonschema.ValidationError as e:
    print(f"✗ Schema validation failed: {e.message}")
```

**4. Semantic Correctness Check**

Manually review each generated plan:

| Command | Generated Plan | Correct? |
|---------|----------------|----------|
| "Bring me the mug" | `["navigate_to kitchen", "detect_object mug", "pick_object mug", "navigate_to user", "place_object table"]` | ✓ YES |
| "Go to bedroom" | `["navigate_to bedroom"]` | ✓ YES |
| "Get the apple" | `["navigate_to kitchen", "detect_object banana", ...]` | ✗ NO (wrong object) |

**5. Pass/Fail Criteria**

- **PASS**: 100% schema validation success
- **PASS**: ≥90% semantic correctness (18 out of 20 plans correct)
- **FAIL**: Any hallucinated actions not in capability manifest

### Mock Mode

```python
MOCK_LLM = True
if MOCK_LLM:
    action_sequence = [
        {"action": "navigate_to", "params": {"location": "kitchen"}},
        {"action": "detect_object", "params": {"name": "mug"}},
        ...
    ]
else:
    action_sequence = self.llm_result
```

---

## Capability 3: Navigation

### Acceptance Criteria
- **Success Rate**: ≥90% of navigation goals reached within tolerance
- **Failure Rate**: <10% aborted due to obstacles/planning failures
- **Latency**: <15 seconds for typical 5-meter navigation

### Test Procedure

**1. Define Test Waypoints**

Create 10 test waypoints in Gazebo world:

```yaml
# test_waypoints.yaml
waypoints:
  - name: "kitchen"
    x: 3.5
    y: 2.0
    theta: 0.0
  - name: "living_room"
    x: 0.0
    y: 0.0
    theta: 1.57
  - name: "bedroom"
    x: -2.0
    y: 3.5
    theta: 3.14
  ...
```

**2. Run Navigation Node in Isolation**

```bash
# Terminal 1: Launch Gazebo and Nav2
ros2 launch capstone_demo gazebo_demo.launch.py

# Terminal 2: Start navigation controller
ros2 run capstone_demo navigation_controller

# Terminal 3: Send test goals
ros2 topic pub /navigation/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 3.5, y: 2.0, z: 0.0}}}"
```

**3. Measure Success/Failure**

```python
# Automated test script
import time

results = []
for waypoint in test_waypoints:
    start_time = time.time()
    send_nav_goal(waypoint)
    status = wait_for_result(timeout=30)  # Max 30s per goal
    latency = time.time() - start_time

    results.append({
        'waypoint': waypoint['name'],
        'status': status,  # SUCCEEDED, ABORTED, TIMEOUT
        'latency': latency
    })

success_rate = sum(1 for r in results if r['status'] == 'SUCCEEDED') / len(results)
print(f"Success rate: {success_rate * 100:.1f}%")
```

**4. Pass/Fail Criteria**

- **PASS**: ≥90% success rate (9 out of 10 waypoints reached)
- **PASS**: All successful navigations complete in <15 seconds
- **FAIL**: If robot collides with obstacles (check /scan topic)

### Mock Mode

```python
MOCK_NAV = True
if MOCK_NAV:
    time.sleep(2)  # Simulate navigation delay
    nav_status = "SUCCEEDED"
else:
    nav_status = self.nav_result.status
```

---

## Capability 4: Object Detection

### Acceptance Criteria
- **Precision**: ≥85% correct detections (true positives)
- **Bounding Box Accuracy**: IoU ≥ 0.5 with ground truth
- **Latency**: <100ms per frame (≥10 FPS)

### Test Procedure

**1. Create Annotated Test Dataset**

Capture 50 test images from Gazebo with ground truth labels:

```yaml
# test_detections.yaml
- image: "kitchen_scene_01.png"
  objects:
    - class: "mug"
      bbox: [320, 240, 80, 120]  # [x_center, y_center, width, height]
    - class: "bottle"
      bbox: [450, 260, 60, 150]
- image: "kitchen_scene_02.png"
  objects:
    - class: "apple"
      bbox: [200, 300, 70, 70]
...
```

**2. Run Detection Node in Isolation**

```bash
# Terminal 1: Publish test images
ros2 bag play test_images.bag

# Terminal 2: Run detection node
ros2 run capstone_demo object_detection_node

# Terminal 3: Monitor detections
ros2 topic echo /perception/detections
```

**3. Measure Precision and IoU**

```python
def compute_iou(bbox_pred, bbox_gt):
    """Intersection over Union for bounding boxes."""
    x1 = max(bbox_pred[0], bbox_gt[0])
    y1 = max(bbox_pred[1], bbox_gt[1])
    x2 = min(bbox_pred[2], bbox_gt[2])
    y2 = min(bbox_pred[3], bbox_gt[3])

    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    area_pred = (bbox_pred[2] - bbox_pred[0]) * (bbox_pred[3] - bbox_pred[1])
    area_gt = (bbox_gt[2] - bbox_gt[0]) * (bbox_gt[3] - bbox_gt[1])
    union = area_pred + area_gt - intersection

    return intersection / union if union > 0 else 0

# Evaluate all test images
true_positives = 0
false_positives = 0

for test_case in test_dataset:
    detections = run_detection(test_case['image'])
    ground_truth = test_case['objects']

    for det in detections:
        matched = False
        for gt in ground_truth:
            if det['class'] == gt['class'] and compute_iou(det['bbox'], gt['bbox']) >= 0.5:
                true_positives += 1
                matched = True
                break
        if not matched:
            false_positives += 1

precision = true_positives / (true_positives + false_positives)
print(f"Precision: {precision * 100:.1f}%")
```

**4. Pass/Fail Criteria**

- **PASS**: Precision ≥85%
- **PASS**: Average IoU ≥0.5 for matched detections
- **PASS**: Inference latency <100ms per frame (measure with `/perception/annotated_image` timestamp)

### Mock Mode

```python
MOCK_PERCEPTION = True
if MOCK_PERCEPTION:
    detected_objects = [
        {"class": "mug", "confidence": 0.92, "bbox": [320, 240, 80, 120]}
    ]
else:
    detected_objects = self.perception_result.detections
```

---

## Capability 5: Manipulation

### Acceptance Criteria
- **Success Rate**: ≥70% of pick-and-place tasks complete successfully
- **Latency**: <15 seconds per pick-and-place sequence
- **Failure Recovery**: Retry logic activates and succeeds within 3 attempts

### Test Procedure

**1. Define Test Objects and Poses**

```yaml
# test_manipulation.yaml
test_cases:
  - object: "mug"
    pick_pose: {x: 0.5, y: 0.0, z: 0.8}
    place_pose: {x: 0.3, y: 0.3, z: 0.8}
  - object: "bottle"
    pick_pose: {x: 0.6, y: -0.2, z: 0.75}
    place_pose: {x: 0.4, y: 0.0, z: 0.75}
  ...
```

**2. Run Manipulation Node in Isolation**

```bash
# Terminal 1: Launch MoveIt 2
ros2 launch capstone_demo moveit_demo.launch.py

# Terminal 2: Run manipulation controller
ros2 run capstone_demo manipulation_controller

# Terminal 3: Send pick goals
ros2 topic pub /manipulation/pick_target geometry_msgs/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.8}}}"
```

**3. Measure Success Rate**

```python
results = []
for test_case in test_manipulation_cases:
    # Send pick command
    send_pick_goal(test_case['pick_pose'])
    pick_status = wait_for_result(timeout=20)

    # Send place command (only if pick succeeded)
    if pick_status == "PICK_SUCCESS":
        send_place_goal(test_case['place_pose'])
        place_status = wait_for_result(timeout=20)
        overall_success = (place_status == "PLACE_SUCCESS")
    else:
        overall_success = False

    results.append({
        'object': test_case['object'],
        'success': overall_success
    })

success_rate = sum(1 for r in results if r['success']) / len(results)
print(f"Manipulation success rate: {success_rate * 100:.1f}%")
```

**4. Pass/Fail Criteria**

- **PASS**: Success rate ≥70% (14 out of 20 trials)
- **PASS**: Retry logic activates on first failure and succeeds within 3 attempts
- **FAIL**: If any collision detected (check MoveIt planning scene)

### Mock Mode

```python
MOCK_MANIPULATION = True
if MOCK_MANIPULATION:
    time.sleep(5)  # Simulate manipulation delay
    manip_status = "PICK_SUCCESS"
else:
    manip_status = self.manip_result.status
```

---

## Full Integration Testing

After **all five capabilities pass** independent tests, enable full integration:

```python
# integration_demo.py
MOCK_VOICE = False
MOCK_LLM = False
MOCK_NAV = False
MOCK_PERCEPTION = False
MOCK_MANIPULATION = False
```

Run end-to-end test:

```bash
ros2 launch capstone_demo gazebo_demo.launch.py

# Speak into microphone or publish command
ros2 topic pub /voice/user_command std_msgs/String \
  "{data: 'Bring me the mug from the kitchen'}"
```

**Expected Timeline** (from quickstart.md):
- T+0s: Voice transcription starts
- T+5s: LLM planning completes
- T+10s: Navigation to kitchen
- T+25s: Object detection
- T+30s: Grasp execution
- T+45s: Return navigation
- T+60s: Place and task complete

**Integration Pass Criteria**:
- Task completes successfully in <90 seconds
- No failures in nominal scenario (all objects present, no obstacles)
- Graceful degradation in edge cases (object not found → clarification)

---

## Mock Mode Architecture Benefits

Mock modes provide critical advantages for both development velocity and debugging efficiency in autonomous robotics systems.

**Rapid Iteration Without Dependencies**: Enabling mock modes allows developers to test integration logic without launching the full ROS 2 stack, Gazebo simulator, or external API services. A complete integration test that normally requires 90 seconds (including simulator startup, voice transcription, LLM API calls, navigation, and manipulation) completes in under 5 seconds with all mocks enabled. This 18x speedup dramatically accelerates the debug-test-fix cycle during active development.

**Deterministic Testing for CI/CD**: Mock modes eliminate non-deterministic failures caused by network latency (LLM API timeouts), sensor noise (voice transcription errors), or physics simulation variance (navigation path variations). Automated test suites running in continuous integration pipelines achieve 100% reproducibility when using mocks, compared to 60-70% success rates with real components due to environmental factors. This determinism enables reliable regression detection—if a test fails, the failure indicates a code bug, not environmental flakiness.

**Selective Component Testing**: The granular mock configuration (`mock_voice`, `mock_llm`, `mock_navigation`, `mock_perception`, `mock_manipulation`) enables testing specific integration paths. For example, to validate LLM planning logic without voice dependencies, enable only `mock_voice=True` while keeping all other mocks disabled. This selective isolation pinpoints failures to specific component boundaries, reducing debugging time from hours to minutes when tracking down integration bugs.

**Cost Reduction for Educational Deployments**: Mock modes eliminate external API costs during development and student lab exercises. With 30 students each running 10 test iterations per lab session, full LLM integration would incur 300 API calls ($0.30-$3.00 depending on model). Mock mode reduces this to zero cost for routine testing, reserving API quota for final validation runs where real-world behavior matters most.

---

## Troubleshooting Isolation Tests

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Voice WER >20% | Background noise, poor microphone | Use headset mic, test in quiet room |
| LLM invalid JSON | Prompt template outdated | Update prompt in `llm_prompts.md` contract |
| Navigation aborts | Costmap inflation too conservative | Reduce `inflation_radius` in `nav2_params.yaml` |
| Detection precision <70% | Wrong YOLO model or low confidence | Use YOLOv8m (not nano), lower threshold to 0.5 |
| Manipulation <50% success | IK solver failing | Check joint limits, simplify target poses |

---

## Common Testing Pitfalls and Best Practices

Effective isolation testing requires systematic methodology to avoid common mistakes that mask failures or produce false positives.

**Pitfall 1: Testing Integration Before Capabilities Pass**. Teams frequently skip independent capability validation and jump directly to end-to-end testing, leading to multi-hour debugging sessions where the failure could originate from any of five subsystems. Best practice: Enforce a strict testing sequence—each capability must achieve its acceptance criteria (voice ≥85% WER, navigation ≥90% success, manipulation ≥70% success) before enabling integration tests. Gate integration testing behind capability-level CI checks to prevent premature integration attempts.

**Pitfall 2: Insufficient Test Dataset Diversity**. Testing with only 5-10 similar commands (e.g., "get the mug", "fetch the cup") produces artificially high success rates that collapse in production when users issue diverse commands. Best practice: Create test datasets with 20+ commands covering edge cases—ambiguous objects ("get the red thing"), multi-step tasks ("bring the mug and then navigate to the bedroom"), unknown locations ("find the remote"), and negation ("don't pick up the apple"). Acceptance criteria should hold across this diverse dataset, not just nominal cases.

**Pitfall 3: Ignoring Latency Targets During Development**. Functional correctness ("it eventually works") is insufficient—autonomous systems must meet strict latency requirements to be practically useful. A navigation system that succeeds 100% but takes 45 seconds for a 5-meter path fails the <15 second acceptance criterion. Best practice: Measure and log latency for every test trial using ROS 2 timestamps, not wall-clock time. Track p50, p95, and p99 latencies to detect performance regressions early, before they compound into unacceptable end-to-end delays.

---

## Summary

Independent capability testing is **essential** for validating complex autonomous systems:

✅ **Isolate failures** to specific subsystems (not entire pipeline)

✅ **Validate acceptance criteria** quantitatively (WER, success rate, IoU)

✅ **Enable mock modes** for testing integration logic without hardware dependencies

✅ **Parallelize development** (team members can work on separate capabilities)

Once all five capabilities pass independent tests, full integration has a **>90% probability of success** on the first attempt—compared to <20% without isolation testing.

**Next Steps**: Return to [Chapter 1: System Architecture](chapter-01-architecture.md) to see how these capabilities integrate, or proceed to [Performance Benchmarking](benchmarking.md) to measure end-to-end system performance.
