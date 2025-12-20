---
title: 'Chapter 3: LLM Task Planning'
description: 'Use Claude/GPT-4 to generate robot action plans from natural language'
sidebar_position: 3
---

# Chapter 3: LLM Task Planning

## Learning Objectives

1. **Design** prompts for robot task planning
2. **Parse** LLM outputs into structured action plans
3. **Handle** failure cases and ambiguous commands

## System Prompt Design

```python
SYSTEM_PROMPT = """
You are a robot task planner. Convert user commands into JSON action plans.

Available actions:
- navigate_to(x, y, theta)
- pick_object(object_id)
- place_object(x, y, z)

Example:
User: "Pick up the red cube"
Output: {"actions": [
  {"type": "navigate_to", "target": "red_cube"},
  {"type": "pick_object", "object_id": "red_cube"}
]}
"""
```

## Anthropic Claude Integration

```python
import anthropic

client = anthropic.Anthropic(api_key=os.environ["ANTHROPIC_API_KEY"])
response = client.messages.create(
    model="claude-3-5-sonnet-20241022",
    max_tokens=1024,
    system=SYSTEM_PROMPT,
    messages=[{"role": "user", "content": "Pick up the red cube"}]
)
plan = json.loads(response.content[0].text)
```

**Code Example**: See `examples/module-04-vla/example-01-vla-pipeline/llm_planner.py`

**Next**: [Chapter 4: ROS Action Executor](./chapter-04-ros-executor.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 45 minutes
