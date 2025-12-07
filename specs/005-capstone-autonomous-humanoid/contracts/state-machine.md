# State Machine Contract - Capstone Autonomous Humanoid

**Purpose**: Define the 11-state finite state machine (FSM) that orchestrates voice-commanded task execution.

**Version**: 1.0.0
**Last Updated**: 2025-12-07
**Implementation**: `examples/module-05-capstone/integration_demo.py`

---

## State Machine Overview

The integration controller uses a **finite state machine** to coordinate five capabilities (voice, LLM, navigation, perception, manipulation) into a cohesive autonomous system.

**Key Properties**:
- **11 states** covering nominal execution and failure modes
- **Deterministic transitions** based on events
- **Retry logic** with configurable budgets (max 3 attempts per action)
- **Graceful degradation** when unrecoverable failures occur

---

## State Definitions

### 1. IDLE

**Description**: System waiting for voice command

**Entry Conditions**:
- System startup
- Previous task completed (from COMPLETED state)
- Previous task failed (from FAILED state)

**Behaviors**:
- Voice input node actively listening
- All other subsystems idle
- Status: "Ready for command"

**Exit Events**:
- `VOICE_DETECTED` → Transition to LISTENING

**Timeout**: None (indefinite wait)

---

### 2. LISTENING

**Description**: Capturing audio from microphone

**Entry Conditions**:
- Voice activity detected (energy above threshold)

**Behaviors**:
- Recording audio for configured duration (default 3 seconds)
- Voice input node capturing samples

**Exit Events**:
- `AUDIO_CAPTURED` → Transition to TRANSCRIBING
- `TIMEOUT` (>5s) → Transition to FAILED

**Timeout**: 5 seconds

---

### 3. TRANSCRIBING

**Description**: Whisper processing audio to text

**Entry Conditions**:
- Audio capture complete

**Behaviors**:
- Whisper model inference running
- Transcription in progress

**Exit Events**:
- `TEXT_READY` → Transition to PLANNING
- `TRANSCRIPTION_FAILED` → Transition to FAILED

**Timeout**: 10 seconds

---

### 4. PLANNING

**Description**: LLM generating action sequence

**Entry Conditions**:
- Transcribed text available

**Behaviors**:
- LLM API call in progress
- Prompt constructed with capability manifest
- Waiting for structured JSON response

**Exit Events**:
- `PLAN_GENERATED` → Transition to VALIDATING
- `AMBIGUOUS_COMMAND` → Transition to AWAITING_CLARIFICATION
- `TIMEOUT` (>30s) → Transition to FAILED
- `API_ERROR` → Transition to FAILED

**Timeout**: 30 seconds (LLM can be slow)

**Retry Logic**: On timeout, retry once with shorter prompt

---

### 5. VALIDATING

**Description**: Checking plan feasibility

**Entry Conditions**:
- Plan received from LLM

**Behaviors**:
- JSON schema validation
- Action existence check (all actions in capability manifest)
- Parameter type checking
- Dependency validation (pick requires prior detect)

**Exit Events**:
- `PLAN_VALID` → Transition to NAVIGATING (start executing first action)
- `PLAN_INVALID` → Transition to FAILED

**Timeout**: 1 second (validation is fast)

---

### 6. NAVIGATING

**Description**: Robot moving to target location

**Entry Conditions**:
- Current action is `navigate_to`
- Valid navigation goal available

**Behaviors**:
- Nav2 action goal sent
- Costmap updates being processed
- Path planning active
- Obstacle avoidance running

**Exit Events**:
- `NAVIGATION_SUCCEEDED` → Transition to next action (usually PERCEIVING)
- `NAVIGATION_FAILED` → Retry (max 3) or FAILED
- `NAVIGATION_CANCELED` → Transition to FAILED

**Timeout**: 60 seconds (depends on distance)

**Retry Logic**:
- Attempt 1: Replan with current costmap
- Attempt 2: Request costmap update, replan
- Attempt 3: Try alternate path planner
- Failure: Transition to FAILED

---

### 7. PERCEIVING

**Description**: Detecting target object in camera view

**Entry Conditions**:
- Current action is `detect_object`
- Robot positioned at detection location

**Behaviors**:
- Object detection running on camera feed
- Filtering detections by class and attributes (color, size)
- Computing 3D pose from depth image
- Publishing detections to `/perception/detected_objects`

**Exit Events**:
- `OBJECT_DETECTED` → Transition to next action (usually MANIPULATING for pick)
- `OBJECT_NOT_FOUND` (>10s) → Retry or request clarification
- `CAMERA_FAILURE` → Transition to FAILED

**Timeout**: 10 seconds

**Retry Logic**:
- Attempt 1: Continue detection for another 5 seconds
- Attempt 2: Request user to point out object
- Attempt 3: Ask LLM for alternative object
- Failure: Transition to AWAITING_CLARIFICATION or FAILED

---

### 8. MANIPULATING

**Description**: Picking or placing object

**Entry Conditions**:
- Current action is `pick_object` or `place_object`
- Target object localized (for pick) or target location known (for place)

**Behaviors**:
- MoveIt 2 motion planning active
- IK solver computing joint trajectories
- Grasp planning (for pick) or placement planning (for place)
- Force control during grasp/release

**Exit Events**:
- `MANIPULATION_SUCCEEDED` → Transition to next action or COMPLETED
- `GRASP_FAILED` → Retry (max 3) or FAILED
- `IK_FAILURE` → Adjust grasp pose, retry
- `COLLISION_DETECTED` → Abort, transition to FAILED

**Timeout**: 30 seconds (includes motion planning)

**Retry Logic** (for pick):
- Attempt 1: Retry grasp with same pose
- Attempt 2: Adjust grasp orientation ±15°
- Attempt 3: Try alternate grasp from different approach angle
- Failure: Transition to FAILED

**Retry Logic** (for place):
- Attempt 1: Retry placement
- Attempt 2: Adjust placement pose slightly
- Failure: Transition to FAILED (safer to fail than force placement)

---

### 9. COMPLETED

**Description**: Task finished successfully

**Entry Conditions**:
- All actions in plan executed successfully
- Final action completed (usually place_object)

**Behaviors**:
- Log success metrics (latency, success rate)
- Publish completion status
- Reset state machine variables
- Clear action plan

**Exit Events**:
- Automatic transition to IDLE after 2 seconds

**Timeout**: 2 seconds (cleanup delay)

---

### 10. FAILED

**Description**: Unrecoverable error occurred

**Entry Conditions**:
- Retry budget exhausted for any action
- Critical system failure (camera offline, API down)
- Invalid plan that cannot be fixed

**Behaviors**:
- Log failure reason and context
- Publish failure status with diagnostic info
- Reset state machine variables
- Optionally request human intervention

**Exit Events**:
- Automatic transition to IDLE after 5 seconds

**Timeout**: 5 seconds (diagnostic logging delay)

**Failure Codes**:
```python
FAILURE_VOICE_TIMEOUT = 1        # Voice capture timed out
FAILURE_TRANSCRIPTION = 2        # Whisper failed
FAILURE_LLM_TIMEOUT = 3          # LLM API timeout
FAILURE_LLM_API_ERROR = 4        # LLM API error
FAILURE_INVALID_PLAN = 5         # Plan validation failed
FAILURE_NAVIGATION = 6           # Navigation exhausted retries
FAILURE_OBJECT_NOT_FOUND = 7     # Object detection failed
FAILURE_MANIPULATION = 8         # Grasp/place failed
FAILURE_SYSTEM_ERROR = 9         # Unexpected exception
```

---

### 11. AWAITING_CLARIFICATION

**Description**: LLM needs more information from user

**Entry Conditions**:
- LLM set `command_understood=false`
- Clarification question provided

**Behaviors**:
- Display clarification question to user
- Wait for user response (voice or text)
- Timeout after 60 seconds

**Exit Events**:
- `CLARIFICATION_PROVIDED` → Transition to PLANNING (with updated context)
- `TIMEOUT` (>60s) → Transition to FAILED
- `USER_CANCEL` → Transition to IDLE

**Timeout**: 60 seconds

**Example Clarifications**:
- "Which cup? I see red, blue, and white cups."
- "Where should I place the object?"
- "I don't know where the garage is. Can you guide me?"

---

## State Transition Table

| Current State | Event | Next State | Conditions |
|---------------|-------|------------|------------|
| IDLE | VOICE_DETECTED | LISTENING | Voice activity above threshold |
| LISTENING | AUDIO_CAPTURED | TRANSCRIBING | Recording complete |
| LISTENING | TIMEOUT | FAILED | >5 seconds |
| TRANSCRIBING | TEXT_READY | PLANNING | Transcription successful |
| TRANSCRIBING | TRANSCRIPTION_FAILED | FAILED | Whisper error |
| PLANNING | PLAN_GENERATED | VALIDATING | JSON response received |
| PLANNING | AMBIGUOUS_COMMAND | AWAITING_CLARIFICATION | LLM requests info |
| PLANNING | TIMEOUT | FAILED | >30 seconds |
| VALIDATING | PLAN_VALID | NAVIGATING | Schema validation passed |
| VALIDATING | PLAN_INVALID | FAILED | Validation errors |
| NAVIGATING | NAVIGATION_SUCCEEDED | PERCEIVING or MANIPULATING | Next action type |
| NAVIGATING | NAVIGATION_FAILED | NAVIGATING or FAILED | Retry budget |
| PERCEIVING | OBJECT_DETECTED | MANIPULATING | Detection successful |
| PERCEIVING | OBJECT_NOT_FOUND | AWAITING_CLARIFICATION or FAILED | Retry budget |
| MANIPULATING | MANIPULATION_SUCCEEDED | NAVIGATING or COMPLETED | More actions or done |
| MANIPULATING | GRASP_FAILED | MANIPULATING or FAILED | Retry budget |
| AWAITING_CLARIFICATION | CLARIFICATION_PROVIDED | PLANNING | User responded |
| AWAITING_CLARIFICATION | TIMEOUT | FAILED | >60 seconds |
| COMPLETED | AUTO | IDLE | After 2s |
| FAILED | AUTO | IDLE | After 5s |

---

## Execution Flow Examples

### Nominal Flow: "Bring me the red cup"

```
IDLE
  ↓ (user speaks)
LISTENING (3s)
  ↓
TRANSCRIBING (2s)
  ↓
PLANNING (8s)
  ↓
VALIDATING (0.5s)
  ↓
NAVIGATING (12s) ← Action 1: navigate_to kitchen_table
  ↓
PERCEIVING (3s) ← Action 2: detect_object red cup
  ↓
MANIPULATING (5s) ← Action 3: pick_object
  ↓
NAVIGATING (12s) ← Action 4: navigate_to user_location
  ↓
MANIPULATING (4s) ← Action 5: place_object user_hand
  ↓
COMPLETED (2s)
  ↓
IDLE

Total time: ~52 seconds
```

### Failure Flow: Navigation Blocked

```
IDLE → LISTENING → TRANSCRIBING → PLANNING → VALIDATING
  ↓
NAVIGATING (15s, attempt 1)
  ↓ (obstacle blocking path)
NAVIGATING (15s, attempt 2, replan)
  ↓ (still blocked)
NAVIGATING (15s, attempt 3, alternate planner)
  ↓ (no valid path found)
FAILED (5s)
  ↓
IDLE

Total time: ~50 seconds, STATUS: FAILED_NAVIGATION
```

### Clarification Flow: Ambiguous Object

```
IDLE → LISTENING → TRANSCRIBING → PLANNING
  ↓ (LLM: "Which cup?")
AWAITING_CLARIFICATION (10s)
  ↓ (user: "The red one")
PLANNING (6s)
  ↓
VALIDATING → NAVIGATING → ... → COMPLETED

Total time: Normal + 16s clarification overhead
```

---

## Implementation Requirements

### State Machine Class

```python
class State(Enum):
    IDLE = auto()
    LISTENING = auto()
    TRANSCRIBING = auto()
    PLANNING = auto()
    VALIDATING = auto()
    NAVIGATING = auto()
    PERCEIVING = auto()
    MANIPULATING = auto()
    COMPLETED = auto()
    FAILED = auto()
    AWAITING_CLARIFICATION = auto()

class IntegrationDemo(Node):
    def __init__(self):
        self.state = State.IDLE
        self.retry_counts = defaultdict(int)
        self.max_retries = 3
        self.state_entry_time = time.time()

    def transition_to(self, new_state):
        """Transition with logging and timeout reset"""
        logger.info(f"State: {self.state.name} → {new_state.name}")
        self.state = new_state
        self.state_entry_time = time.time()

    def check_timeout(self):
        """Enforce state timeouts"""
        elapsed = time.time() - self.state_entry_time
        if elapsed > STATE_TIMEOUTS[self.state]:
            self.transition_to(State.FAILED)
```

### Retry Budget Management

```python
def execute_action_with_retry(self, action_name, action_fn):
    """Execute action with retry logic"""
    retry_key = f"{action_name}_{self.current_action_index}"

    if self.retry_counts[retry_key] >= self.max_retries:
        logger.error(f"Max retries exhausted for {action_name}")
        self.transition_to(State.FAILED)
        return False

    success = action_fn()
    if not success:
        self.retry_counts[retry_key] += 1
        logger.warn(f"Retry {self.retry_counts[retry_key]}/{self.max_retries}")
        return self.execute_action_with_retry(action_name, action_fn)

    self.retry_counts[retry_key] = 0  # Reset on success
    return True
```

---

## Testing State Machine

### Unit Tests

```bash
# Test state transitions
pytest tests/module-05-capstone/test_state_machine.py::test_nominal_flow

# Test failure recovery
pytest tests/module-05-capstone/test_state_machine.py::test_navigation_retry

# Test timeout enforcement
pytest tests/module-05-capstone/test_state_machine.py::test_llm_timeout
```

### Integration Tests

```bash
# End-to-end test with simulated failures
python scripts/testing/test_failure_injection.py --failures navigation,grasp
```

---

## Monitoring State Machine

```bash
# Watch state transitions in real-time
ros2 topic echo /system/status

# Visualize state machine (custom tool)
python scripts/visualization/state_machine_monitor.py
```

---

**See Also**:
- [ROS 2 Interfaces](ros2-interfaces.md) - Message and action definitions
- [LLM Prompts](llm-prompts.md) - Planning logic
- [Chapter 1: System Architecture](../../docs/modules/module-05-capstone/chapter-01-architecture.md)
