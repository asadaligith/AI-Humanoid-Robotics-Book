# LLM Prompt Engineering Contracts - Capstone Autonomous Humanoid

**Purpose**: Define prompt templates, output formats, and validation schemas for LLM-based task planning.

**Version**: 1.0.0
**Last Updated**: 2025-12-07
**LLM Model**: GPT-4 (gpt-4) or Claude Opus (claude-opus-4)

---

## Overview

The LLM planner transforms natural language commands into structured action sequences executable by the robot. This contract ensures:

- **Consistency**: All prompts follow proven templates
- **Reliability**: Output format is strictly validated
- **Grounding**: LLM constrained to valid robot actions (capability manifest)
- **Error Handling**: Ambiguity and failures handled gracefully

---

## Task Planning Prompt Template

### System Prompt

```
You are a robot task planner for a humanoid robot system. Your job is to convert natural language commands into structured action sequences that the robot can execute.

CRITICAL RULES:
1. Only use actions from the capability manifest provided
2. Provide concrete parameter values (no placeholders like "TBD" or "unknown")
3. If the command is ambiguous, set command_understood=false and ask a clarifying question
4. Actions must follow logical dependencies (navigate before detect, detect before pick, pick before place)
5. Always navigate back to user_location after picking an object for delivery
6. Return ONLY valid JSON with no additional commentary
7. Be precise with location names - only use locations listed in the capability manifest
```

### User Prompt Template

```
Available Actions (from capability manifest):
{capability_manifest_json}

Voice Command: "{transcribed_command}"

Generate a structured action plan in the following JSON format:
{{
  "command_understood": true/false,
  "clarification_needed": null or "your question here",
  "action_sequence": [
    {{"action": "action_name", "params": {{"param1": "value1", "param2": "value2"}}}},
    ...
  ]
}}

Guidelines:
- If command_understood is false, provide a specific clarification question
- Each action must include all required parameters from the capability manifest
- Use descriptive object IDs (e.g., "red_cup_001" not "object_1")
- Navigation goals must be named locations from the manifest
- For multi-object tasks, process one object completely before the next
```

### Complete Prompt (Assembled)

```python
TASK_PLANNER_PROMPT = """
{system_prompt}

{user_prompt}
"""

# Usage
prompt = TASK_PLANNER_PROMPT.format(
    capability_manifest_json=json.dumps(capabilities, indent=2),
    transcribed_command=user_voice_text
)
```

---

## Example Interactions

### Example 1: Simple Fetch Task

**Input**:
```
Voice Command: "Bring me the red cup from the kitchen table"
```

**LLM Output**:
```json
{
  "command_understood": true,
  "clarification_needed": null,
  "action_sequence": [
    {"action": "navigate_to", "params": {"location": "kitchen_table"}},
    {"action": "detect_object", "params": {"name": "cup", "color": "red"}},
    {"action": "pick_object", "params": {"object_id": "red_cup_kitchen"}},
    {"action": "navigate_to", "params": {"location": "user_location"}},
    {"action": "place_object", "params": {"location": "user_hand"}}
  ]
}
```

**Validation**: ✅ PASS
- All actions exist in capability manifest
- Parameters match expected types
- Logical flow: navigate → detect → pick → navigate → place
- Object ID is descriptive

---

### Example 2: Ambiguous Command (Clarification Needed)

**Input**:
```
Voice Command: "Bring me the cup"
```

**LLM Output**:
```json
{
  "command_understood": false,
  "clarification_needed": "I see multiple cups on the kitchen table (red, blue, white). Which cup would you like?",
  "action_sequence": []
}
```

**System Response**:
- Publish clarification to `/planning/clarification_request`
- Transition to AWAITING_CLARIFICATION state
- Wait for user response

**Follow-up Input**:
```
Voice Command: "The red one"
```

**LLM Output** (with context):
```json
{
  "command_understood": true,
  "clarification_needed": null,
  "action_sequence": [
    {"action": "navigate_to", "params": {"location": "kitchen_table"}},
    {"action": "detect_object", "params": {"name": "cup", "color": "red"}},
    {"action": "pick_object", "params": {"object_id": "red_cup_kitchen"}},
    {"action": "navigate_to", "params": {"location": "user_location"}},
    {"action": "place_object", "params": {"location": "user_hand"}}
  ]
}
```

---

### Example 3: Multi-Object Task

**Input**:
```
Voice Command: "Bring me the red cup and the blue plate from the kitchen table"
```

**LLM Output**:
```json
{
  "command_understood": true,
  "clarification_needed": null,
  "action_sequence": [
    {"action": "navigate_to", "params": {"location": "kitchen_table"}},
    {"action": "detect_object", "params": {"name": "cup", "color": "red"}},
    {"action": "pick_object", "params": {"object_id": "red_cup_kitchen"}},
    {"action": "navigate_to", "params": {"location": "user_location"}},
    {"action": "place_object", "params": {"location": "table"}},

    {"action": "navigate_to", "params": {"location": "kitchen_table"}},
    {"action": "detect_object", "params": {"name": "plate", "color": "blue"}},
    {"action": "pick_object", "params": {"object_id": "blue_plate_kitchen"}},
    {"action": "navigate_to", "params": {"location": "user_location"}},
    {"action": "place_object", "params": {"location": "table"}}
  ]
}
```

**Validation**: ✅ PASS
- Processes each object sequentially (fetch cup, return, fetch plate, return)
- Avoids multi-object grasping (robot has two hands but one arm controller)

---

### Example 4: Invalid Location (Error Handling)

**Input**:
```
Voice Command: "Go to the garage"
```

**LLM Output**:
```json
{
  "command_understood": false,
  "clarification_needed": "I don't know where the garage is. My known locations are: kitchen_table, living_room, user_location, charging_station, bedroom, bathroom, dining_table. Can you guide me or choose a different location?",
  "action_sequence": []
}
```

**Alternative LLM Output** (if trained to suggest nearest):
```json
{
  "command_understood": false,
  "clarification_needed": "I don't have a map of the garage. Did you mean 'charging_station' or another location?",
  "action_sequence": []
}
```

---

### Example 5: Tidy-Up Task

**Input**:
```
Voice Command: "Clean up the table and put the cups in the bin"
```

**LLM Output**:
```json
{
  "command_understood": true,
  "clarification_needed": null,
  "action_sequence": [
    {"action": "navigate_to", "params": {"location": "table"}},
    {"action": "detect_object", "params": {"name": "cup"}},
    {"action": "pick_object", "params": {"object_id": "cup_table_1"}},
    {"action": "navigate_to", "params": {"location": "bin"}},
    {"action": "place_object", "params": {"location": "bin"}},

    {"action": "navigate_to", "params": {"location": "table"}},
    {"action": "detect_object", "params": {"name": "cup"}},
    {"action": "pick_object", "params": {"object_id": "cup_table_2"}},
    {"action": "navigate_to", "params": {"location": "bin"}},
    {"action": "place_object", "params": {"location": "bin"}},

    {"action": "wait", "params": {"duration": 1.0}}
  ]
}
```

**Note**: LLM assumes multiple cups exist and generates loop-like behavior. In practice, detection step would return empty if no more cups found, causing task completion.

---

## Output Validation Schema

### JSON Schema (jsonschema library)

```python
PLAN_SCHEMA = {
    "type": "object",
    "required": ["command_understood", "action_sequence"],
    "properties": {
        "command_understood": {
            "type": "boolean",
            "description": "True if command can be executed, false if clarification needed"
        },
        "clarification_needed": {
            "type": ["string", "null"],
            "description": "Question to ask user if command_understood is false"
        },
        "action_sequence": {
            "type": "array",
            "description": "Ordered list of actions to execute",
            "items": {
                "type": "object",
                "required": ["action", "params"],
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["navigate_to", "detect_object", "pick_object", "place_object", "wait", "open_gripper", "close_gripper"]
                    },
                    "params": {
                        "type": "object",
                        "description": "Action-specific parameters"
                    }
                }
            }
        }
    },
    "if": {
        "properties": {"command_understood": {"const": false}}
    },
    "then": {
        "required": ["clarification_needed"],
        "properties": {
            "clarification_needed": {"type": "string", "minLength": 10}
        }
    }
}
```

### Validation Function

```python
from jsonschema import validate, ValidationError

def validate_plan(plan_json, capability_manifest):
    """Validate LLM-generated plan"""
    try:
        # Schema validation
        validate(instance=plan_json, schema=PLAN_SCHEMA)

        # Action existence check
        valid_actions = {cap["action"] for cap in capability_manifest["capabilities"]}
        for action in plan_json["action_sequence"]:
            if action["action"] not in valid_actions:
                raise ValueError(f"Unknown action: {action['action']}")

        # Parameter validation (action-specific)
        for action in plan_json["action_sequence"]:
            validate_action_params(action, capability_manifest)

        return True, "Plan is valid"

    except ValidationError as e:
        return False, f"Schema validation failed: {e.message}"
    except ValueError as e:
        return False, str(e)

def validate_action_params(action, manifest):
    """Validate parameters for specific action"""
    action_name = action["action"]
    params = action["params"]

    # Find action definition in manifest
    action_def = next(
        (cap for cap in manifest["capabilities"] if cap["action"] == action_name),
        None
    )

    if not action_def:
        raise ValueError(f"Action {action_name} not in manifest")

    # Check required parameters
    for param_name, param_spec in action_def["params"].items():
        if param_name not in params and not param_spec.get("optional", False):
            raise ValueError(f"Missing required parameter '{param_name}' for action '{action_name}'")

        # Type checking
        if param_name in params:
            param_value = params[param_name]
            expected_type = param_spec["type"]

            if expected_type == "string" and not isinstance(param_value, str):
                raise ValueError(f"Parameter '{param_name}' must be string, got {type(param_value)}")

            # Enum validation
            if "enum" in param_spec:
                if param_value not in param_spec["enum"]:
                    raise ValueError(f"Parameter '{param_name}' must be one of {param_spec['enum']}, got '{param_value}'")

    return True
```

---

## Prompt Engineering Best Practices

### 1. Temperature Setting

```python
# Use temperature=0.0 for deterministic, reliable output
response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[{"role": "user", "content": prompt}],
    temperature=0.0,  # Deterministic
    max_tokens=1000
)
```

**Rationale**: Robotics requires predictability. Temperature=0.0 ensures consistent action plans for identical commands.

### 2. Few-Shot Examples (Optional Enhancement)

Add examples to prompt for better performance:

```python
FEW_SHOT_EXAMPLES = """
Example 1:
Command: "Bring me the red cup"
Output: {{"command_understood": true, "action_sequence": [...]}}

Example 2:
Command: "Bring me the cup"  (ambiguous)
Output: {{"command_understood": false, "clarification_needed": "Which cup?..."}}
"""

prompt = f"{SYSTEM_PROMPT}\n\n{FEW_SHOT_EXAMPLES}\n\n{USER_PROMPT}"
```

### 3. Chain-of-Thought (Optional)

For complex commands, use intermediate reasoning:

```python
# Add to system prompt
"Before generating the action sequence, think step-by-step:
1. What is the user asking for?
2. Where is the object likely located?
3. What sequence of actions achieves the goal?
Then generate the JSON action plan."
```

### 4. Retry on Malformed JSON

```python
def call_llm_with_retry(prompt, max_retries=3):
    for attempt in range(max_retries):
        response = call_llm(prompt)
        try:
            plan = json.loads(response)
            return plan
        except json.JSONDecodeError:
            logger.warn(f"Malformed JSON (attempt {attempt+1}), retrying...")
            prompt += "\n\nREMINDER: Return ONLY valid JSON, no additional text."

    raise ValueError("Failed to get valid JSON after retries")
```

---

## Error Handling Strategies

### LLM Timeout

```python
try:
    response = openai.ChatCompletion.create(..., timeout=30)
except openai.error.Timeout:
    # Fallback: Use predefined plan for common commands
    if "bring me the cup" in command.lower():
        return PREDEFINED_FETCH_PLAN
    else:
        raise
```

### API Key Invalid

```python
try:
    response = openai.ChatCompletion.create(...)
except openai.error.AuthenticationError:
    logger.error("Invalid OPENAI_API_KEY")
    # Transition to FAILED state with specific error code
    return None
```

### Rate Limiting

```python
try:
    response = openai.ChatCompletion.create(...)
except openai.error.RateLimitError:
    time.sleep(5)  # Backoff
    response = openai.ChatCompletion.create(...)  # Retry once
```

---

## Testing LLM Prompts

### Unit Tests

```python
def test_simple_fetch_command():
    command = "Bring me the red cup"
    plan = generate_plan(command, capability_manifest)

    assert plan["command_understood"] is True
    assert len(plan["action_sequence"]) == 5
    assert plan["action_sequence"][0]["action"] == "navigate_to"
    assert plan["action_sequence"][1]["action"] == "detect_object"

def test_ambiguous_command():
    command = "Bring me the cup"
    plan = generate_plan(command, capability_manifest)

    assert plan["command_understood"] is False
    assert "which cup" in plan["clarification_needed"].lower()

def test_invalid_location():
    command = "Go to the moon"
    plan = generate_plan(command, capability_manifest)

    assert plan["command_understood"] is False
    assert "don't know" in plan["clarification_needed"].lower()
```

### Integration Tests

```bash
# Test with real LLM API
pytest tests/module-05-capstone/test_llm_planner.py --integration

# Test with mock LLM (faster, no API cost)
pytest tests/module-05-capstone/test_llm_planner.py --mock
```

---

## Monitoring LLM Performance

### Metrics to Track

```python
# Log LLM performance metrics
metrics = {
    "latency": response_time,  # Seconds
    "tokens_used": response.usage.total_tokens,
    "cost": tokens_used * COST_PER_1K_TOKENS,
    "command_understood": plan["command_understood"],
    "validation_passed": validate_plan(plan)
}

logger.info(f"LLM metrics: {metrics}")
```

### Cost Monitoring

```python
# Estimate cost (GPT-4 pricing as of 2024)
INPUT_COST_PER_1K = 0.03   # $0.03 per 1K input tokens
OUTPUT_COST_PER_1K = 0.06  # $0.06 per 1K output tokens

cost = (
    (input_tokens / 1000) * INPUT_COST_PER_1K +
    (output_tokens / 1000) * OUTPUT_COST_PER_1K
)

logger.info(f"LLM call cost: ${cost:.4f}")
```

---

## Alternative: Local LLM Models

For cost-sensitive or offline deployments:

```python
# Use local model (Llama 2, Mistral, etc.)
from transformers import AutoTokenizer, AutoModelForCausalLM

model = AutoModelForCausalLM.from_pretrained("meta-llama/Llama-2-13b-chat-hf")
tokenizer = AutoTokenizer.from_pretrained("meta-llama/Llama-2-13b-chat-hf")

# Same prompt template, different inference
inputs = tokenizer(prompt, return_tensors="pt")
outputs = model.generate(**inputs, max_new_tokens=500)
response = tokenizer.decode(outputs[0])
```

**Tradeoffs**:
- **Pros**: No API cost, offline capable, data privacy
- **Cons**: Lower quality than GPT-4, requires GPU, slower inference

---

**See Also**:
- [Capability Manifest](../examples/module-05-capstone/config/capability_manifest.json) - Available robot actions
- [LLM Planner Node](../examples/module-05-capstone/llm_planner_node.py) - Implementation
- [State Machine](state-machine.md) - Planning state and transitions
