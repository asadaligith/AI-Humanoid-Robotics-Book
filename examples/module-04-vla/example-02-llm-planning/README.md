# Example 02: LLM Task Planning with Claude API

## Learning Objectives

By completing this example, you will:
- Integrate Anthropic Claude API for task planning
- Convert natural language to robot actions
- Implement structured output parsing
- Handle API errors and retries
- Optimize for low-latency responses

## Overview

Claude is Anthropic's large language model that converts natural language instructions into structured robot action plans. It's the reasoning engine in the Vision-Language-Action pipeline.

**Key Capabilities:**
- Natural language understanding
- Task decomposition
- Context awareness
- Tool use and function calling
- Safety and alignment

## Prerequisites

- Python 3.8+
- ROS 2 Humble installed
- Anthropic API key
- Completed Example 01 (Whisper)
- Completed Modules 01-03

## Installation

```bash
# Install Anthropic SDK
pip3 install anthropic

# Install additional dependencies
pip3 install pydantic  # For structured outputs
pip3 install tenacity  # For retries
```

## Getting API Key

```bash
# Sign up at https://console.anthropic.com/
# Get API key from dashboard
# Set environment variable
export ANTHROPIC_API_KEY="sk-ant-api03-..."

# Add to .bashrc for persistence
echo 'export ANTHROPIC_API_KEY="sk-ant-api03-..."' >> ~/.bashrc
```

## Claude Models

| Model | Context | Cost (Input/Output) | Use Case |
|-------|---------|---------------------|----------|
| Claude 3.5 Sonnet | 200K | $3/$15 per 1M tokens | Production (balanced) |
| Claude 3 Opus | 200K | $15/$75 per 1M tokens | Complex reasoning |
| Claude 3 Haiku | 200K | $0.25/$1.25 per 1M tokens | Fast, simple tasks |

**Recommendation**: Use **Claude 3.5 Sonnet** for robot task planning.

## Basic Claude Usage

### Simple Completion

```python
#!/usr/bin/env python3
"""
Basic Claude API usage
"""
import anthropic
import os

# Initialize client
client = anthropic.Anthropic(
    api_key=os.environ.get("ANTHROPIC_API_KEY")
)

# Send message
message = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=1024,
    messages=[
        {
            "role": "user",
            "content": "Convert this to robot actions: Go to the kitchen and bring me a cup"
        }
    ]
)

print(message.content[0].text)
```

### Structured Output with Tool Use

```python
#!/usr/bin/env python3
"""
Claude with structured output using tool use
"""
import anthropic
import json
import os
from pydantic import BaseModel
from typing import List, Literal

# Define action schema
class RobotAction(BaseModel):
    action: Literal["navigate", "pick", "place", "wait"]
    target: str
    parameters: dict = {}

class TaskPlan(BaseModel):
    goal: str
    actions: List[RobotAction]

# Tool definition for Claude
navigation_tool = {
    "name": "create_robot_plan",
    "description": "Creates a structured plan of robot actions to accomplish a goal",
    "input_schema": {
        "type": "object",
        "properties": {
            "goal": {
                "type": "string",
                "description": "The high-level goal to accomplish"
            },
            "actions": {
                "type": "array",
                "description": "Ordered list of robot actions",
                "items": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["navigate", "pick", "place", "wait"],
                            "description": "Type of action to perform"
                        },
                        "target": {
                            "type": "string",
                            "description": "Target location or object"
                        },
                        "parameters": {
                            "type": "object",
                            "description": "Additional parameters for the action"
                        }
                    },
                    "required": ["action", "target"]
                }
            }
        },
        "required": ["goal", "actions"]
    }
}

def plan_task(instruction: str) -> TaskPlan:
    """Convert natural language to robot task plan."""
    client = anthropic.Anthropic(api_key=os.environ.get("ANTHROPIC_API_KEY"))

    # System prompt for robot context
    system_prompt = """You are a robot task planner. Convert natural language instructions into structured robot action sequences.

Available actions:
- navigate: Move to a location
- pick: Grasp an object
- place: Put down an object
- wait: Pause for a duration

Available locations: kitchen, living_room, bedroom, office
Available objects: cup, plate, book, remote

Break down complex tasks into simple atomic actions."""

    message = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=2048,
        system=system_prompt,
        tools=[navigation_tool],
        messages=[
            {
                "role": "user",
                "content": instruction
            }
        ]
    )

    # Extract tool use
    for block in message.content:
        if block.type == "tool_use" and block.name == "create_robot_plan":
            plan_data = block.input
            return TaskPlan(**plan_data)

    raise ValueError("No valid plan generated")

# Test
if __name__ == "__main__":
    instruction = "Go to the kitchen, pick up a cup, and bring it to the living room"
    plan = plan_task(instruction)

    print(f"Goal: {plan.goal}")
    print("\nAction Plan:")
    for i, action in enumerate(plan.actions, 1):
        print(f"{i}. {action.action.upper()} -> {action.target}")
        if action.parameters:
            print(f"   Parameters: {action.parameters}")
```

## ROS 2 Integration

### LLM Planner Node

```python
#!/usr/bin/env python3
"""
ROS 2 node for LLM-based task planning
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
import anthropic
import json
import os
from pydantic import BaseModel
from typing import List, Literal
from tenacity import retry, stop_after_attempt, wait_exponential

class RobotAction(BaseModel):
    action: Literal["navigate", "pick", "place", "wait"]
    target: str
    parameters: dict = {}

class TaskPlan(BaseModel):
    goal: str
    actions: List[RobotAction]

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')

        # Parameters
        self.declare_parameter('api_key', '')
        self.declare_parameter('model', 'claude-3-5-sonnet-20241022')
        self.declare_parameter('max_tokens', 2048)
        self.declare_parameter('temperature', 0.0)

        api_key = self.get_parameter('api_key').value
        if not api_key:
            api_key = os.environ.get('ANTHROPIC_API_KEY')

        self.model = self.get_parameter('model').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.temperature = self.get_parameter('temperature').value

        # Initialize Claude client
        self.client = anthropic.Anthropic(api_key=api_key)
        self.get_logger().info('Claude API initialized')

        # Subscriber for voice commands
        self.command_sub = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10
        )

        # Publisher for action plan
        self.plan_pub = self.create_publisher(String, 'action_plan', 10)

        # Service for on-demand planning
        self.plan_service = self.create_service(
            Trigger,
            'plan_task',
            self.plan_service_callback
        )

        self.get_logger().info('LLM Planner ready')

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=2, max=10))
    def generate_plan(self, instruction: str) -> TaskPlan:
        """Generate task plan with retry logic."""
        system_prompt = """You are a robot task planner. Convert natural language instructions into structured robot action sequences.

Available actions:
- navigate: Move to a location (e.g., kitchen, living_room, bedroom)
- pick: Grasp an object (e.g., cup, plate, book)
- place: Put down an object at a location
- wait: Pause for a specified duration in seconds

Break down complex tasks into atomic actions. Be specific and unambiguous."""

        tool_def = {
            "name": "create_robot_plan",
            "description": "Creates a structured plan of robot actions",
            "input_schema": {
                "type": "object",
                "properties": {
                    "goal": {"type": "string"},
                    "actions": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "action": {
                                    "type": "string",
                                    "enum": ["navigate", "pick", "place", "wait"]
                                },
                                "target": {"type": "string"},
                                "parameters": {"type": "object"}
                            },
                            "required": ["action", "target"]
                        }
                    }
                },
                "required": ["goal", "actions"]
            }
        }

        message = self.client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            temperature=self.temperature,
            system=system_prompt,
            tools=[tool_def],
            messages=[{"role": "user", "content": instruction}]
        )

        # Extract tool use
        for block in message.content:
            if block.type == "tool_use" and block.name == "create_robot_plan":
                return TaskPlan(**block.input)

        raise ValueError("No valid plan generated")

    def command_callback(self, msg):
        """Process voice command and generate plan."""
        instruction = msg.data
        self.get_logger().info(f'Received command: {instruction}')

        try:
            plan = self.generate_plan(instruction)

            # Log plan
            self.get_logger().info(f'Generated plan for: {plan.goal}')
            for i, action in enumerate(plan.actions, 1):
                self.get_logger().info(f'  {i}. {action.action} -> {action.target}')

            # Publish plan as JSON
            plan_msg = String()
            plan_msg.data = plan.model_dump_json()
            self.plan_pub.publish(plan_msg)

        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')

    def plan_service_callback(self, request, response):
        """Service callback for on-demand planning."""
        # This would typically get the instruction from somewhere
        response.success = True
        response.message = "Planning service not yet implemented"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Whisper node
        Node(
            package='vla_system',
            executable='whisper_node',
            name='whisper_node',
            parameters=[{'model_name': 'base'}]
        ),

        # LLM Planner node
        Node(
            package='vla_system',
            executable='llm_planner',
            name='llm_planner',
            parameters=[{
                'model': 'claude-3-5-sonnet-20241022',
                'max_tokens': 2048,
                'temperature': 0.0,
            }],
            output='screen'
        ),
    ])
```

## Running the System

```bash
# Terminal 1: Launch VLA pipeline
ros2 launch vla_system vla_pipeline.launch.py

# Terminal 2: Monitor plans
ros2 topic echo /action_plan

# Speak: "Go to the kitchen and bring me a cup"
# Output (JSON):
# {
#   "goal": "Retrieve cup from kitchen",
#   "actions": [
#     {"action": "navigate", "target": "kitchen"},
#     {"action": "pick", "target": "cup"},
#     {"action": "navigate", "target": "living_room"},
#     {"action": "place", "target": "table"}
#   ]
# }
```

## Advanced Features

### Context Management

```python
class LLMPlannerWithMemory(Node):
    def __init__(self):
        super().__init__('llm_planner_memory')
        self.conversation_history = []

    def generate_plan_with_context(self, instruction: str):
        """Generate plan with conversation history."""
        # Add user message
        self.conversation_history.append({
            "role": "user",
            "content": instruction
        })

        message = self.client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            messages=self.conversation_history,
            tools=[self.tool_def]
        )

        # Add assistant response to history
        self.conversation_history.append({
            "role": "assistant",
            "content": message.content
        })

        return self.extract_plan(message)
```

### Vision Integration

```python
import base64

def plan_with_vision(image_path: str, instruction: str):
    """Generate plan based on image and instruction."""
    # Read and encode image
    with open(image_path, 'rb') as f:
        image_data = base64.b64encode(f.read()).decode('utf-8')

    message = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=2048,
        messages=[{
            "role": "user",
            "content": [
                {
                    "type": "image",
                    "source": {
                        "type": "base64",
                        "media_type": "image/jpeg",
                        "data": image_data
                    }
                },
                {
                    "type": "text",
                    "text": f"Based on this image, {instruction}"
                }
            ]
        }]
    )

    return message.content[0].text
```

## Cost Optimization

```python
# Use Haiku for simple tasks
def select_model(instruction: str) -> str:
    """Select model based on complexity."""
    word_count = len(instruction.split())

    if word_count < 10:
        return "claude-3-haiku-20240307"  # Fast and cheap
    elif word_count < 30:
        return "claude-3-5-sonnet-20241022"  # Balanced
    else:
        return "claude-3-opus-20240229"  # Complex reasoning

# Cache system prompts
message = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=1024,
    system=[
        {
            "type": "text",
            "text": system_prompt,
            "cache_control": {"type": "ephemeral"}  # Cache this
        }
    ],
    messages=[{"role": "user", "content": instruction}]
)
```

## Exercises

### Exercise 1: Multi-Step Planning
Handle complex instructions requiring 5+ actions with dependencies.

### Exercise 2: Error Handling
Implement graceful degradation when API is unavailable.

### Exercise 3: Dynamic Tool Definition
Create tools based on robot's current capabilities.

### Exercise 4: Vision-Language Planning
Integrate camera feed for visual grounding of plans.

### Exercise 5: Plan Validation
Add a validation step to check plan feasibility before execution.

## Common Issues

### Issue 1: API rate limits
**Solution**: Implement exponential backoff and caching.

### Issue 2: Ambiguous plans
**Solution**: Refine system prompt, add examples, increase temperature.

### Issue 3: High latency
**Solution**: Use Haiku model, reduce max_tokens, cache prompts.

### Issue 4: Invalid JSON output
**Solution**: Use tool calling for structured outputs instead of parsing.

## Key Takeaways

1. **Tool Use is Essential**: Structured outputs via tool calling
2. **System Prompts Matter**: Provide clear robot context and constraints
3. **Retry Logic**: Handle API failures gracefully
4. **Model Selection**: Balance cost, speed, and capability
5. **Context Management**: Maintain conversation history for complex interactions

## Next Steps

- **Example 03**: ROS action executor
- **Module 04 Chapter 05**: End-to-end VLA integration
- **Module 05**: Autonomous humanoid capstone

## References

- [Anthropic Claude API](https://docs.anthropic.com/en/api/)
- [Claude Tool Use](https://docs.anthropic.com/en/docs/tool-use)
- [Prompt Engineering Guide](https://docs.anthropic.com/en/docs/prompt-engineering)
- [Claude Pricing](https://www.anthropic.com/pricing)
