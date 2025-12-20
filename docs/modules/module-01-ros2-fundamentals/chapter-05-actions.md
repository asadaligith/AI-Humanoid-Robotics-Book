---
title: 'Chapter 5: Actions for Long-Running Tasks'
description: 'Implement goal-based asynchronous tasks with feedback using ROS 2 actions'
sidebar_position: 5
---

# Chapter 5: Actions for Long-Running Tasks

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Create** ROS 2 action servers for long-running tasks
2. **Implement** action clients with goal handling and feedback
3. **Design** custom action definitions for robotics workflows
4. **Monitor** action execution with feedback and result callbacks

## Introduction

Actions combine the best of topics and services: asynchronous execution like topics, bidirectional communication like services, plus periodic feedback and cancel ability[^1].

[^1]: ROS 2 Documentation. "Understanding Actions." https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## Action Structure

An action consists of three parts:
1. **Goal**: Request to start the task
2. **Feedback**: Periodic progress updates
3. **Result**: Final outcome when complete

## When to Use Actions

✅ **Use actions for**:
- Navigation (send goal pose, get progress feedback)
- Manipulation (grasp object, report grasping stages)
- Long computations (path planning with incremental results)
- Preemptable tasks (ability to cancel mid-execution)

## Action Server Example

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciServer(Node):
    def __init__(self):
        super().__init__('fibonacci_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciServer()
    rclpy.spin(node)
```

## Action Client Example

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciClient(Node):
    def __init__(self):
        super().__init__('fibonacci_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.sequence}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
```

## Custom Action Definitions

**File**: `my_interfaces/action/NavigateToGoal.action`
```
# Goal
geometry_msgs/PoseStamped target_pose
---
# Result
bool success
float32 total_distance
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
```

## Testing Actions

```bash
# List available actions
ros2 action list

# Send goal from CLI
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}" --feedback

# Show action info
ros2 action info /fibonacci
```

## Exercise

**Task**: Create a "battery charge" action where:
- Goal: Target charge percentage (e.g., 80%)
- Feedback: Current charge percentage every second
- Result: Final charge level and time taken

**Code Example**: See `examples/module-01-ros2-fundamentals/example-04-action/`

## Summary

Actions provide goal-oriented, long-running tasks with feedback and cancellation capabilities.

**Next**: [Chapter 6: Robot Modeling with URDF & TF2](./chapter-06-urdf-tf.md)

---

**Estimated Reading Time**: 20 minutes
**Hands-On Time**: 45 minutes
