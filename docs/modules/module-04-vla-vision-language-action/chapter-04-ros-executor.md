---
title: 'Chapter 4: ROS Action Executor'
description: 'Execute LLM-generated action plans using ROS 2 actions'
sidebar_position: 4
---

# Chapter 4: ROS Action Executor

## Learning Objectives

1. **Parse** JSON action plans
2. **Invoke** ROS 2 actions (navigate, grasp, place)
3. **Handle** action failures and retries

## Executor Node

```python
class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.grasp_client = ActionClient(self, Grasp, '/grasp_object')

    def execute_plan(self, plan_json):
        for action in plan_json['actions']:
            if action['type'] == 'navigate_to':
                self.send_nav_goal(action['target'])
            elif action['type'] == 'pick_object':
                self.send_grasp_goal(action['object_id'])
```

**Code Example**: See `examples/module-04-vla/example-01-vla-pipeline/ros_executor.py`

**Next**: [Chapter 5: End-to-End Integration](./chapter-05-integration.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 50 minutes
