---
title: 'Chapter 4: Services for Synchronous Communication'
description: 'Implement request-response patterns using ROS 2 services for on-demand robot queries'
sidebar_position: 4
---

# Chapter 4: Services for Synchronous Communication

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Create** ROS 2 service servers to handle requests
2. **Implement** service clients to make synchronous calls
3. **Design** custom service definitions for robotics tasks
4. **Apply** services for on-demand computations

## Introduction

Services provide synchronous request-response communication in ROS 2. Unlike topics, services guarantee a response and support bidirectional data flow[^1].

[^1]: ROS 2 Documentation. "Understanding Services." https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html

## When to Use Services

✅ **Use services for**:
- Query operations (get robot status, request grasp pose)
- On-demand computations (path planning, object recognition)
- Configuration updates (set parameters, toggle modes)
- Low-frequency requests (&lt;1 Hz)

❌ **Avoid services for**:
- Continuous data streams (use topics)
- Long-running tasks (use actions)
- High-frequency calls (>10 Hz)

## Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AdditionServer(Node):
    def __init__(self):
        super().__init__('addition_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )
        self.get_logger().info('Addition service ready')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AdditionServer()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Service Client Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AdditionClient(Node):
    def __init__(self):
        super().__init__('addition_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        future = self.cli.call_async(req)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = AdditionClient()
    future = node.send_request(10, 20)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    node.get_logger().info(f'Result: {result.sum}')
    rclpy.shutdown()
```

## Custom Service Definitions

**File**: `my_interfaces/srv/GetObjectPose.srv`
```
# Request
string object_id
---
# Response
geometry_msgs/Pose pose
bool success
string message
```

## Testing Services

```bash
# List available services
ros2 service list

# Call service from CLI
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Show service type
ros2 service type /add_two_ints
```

## Exercise

**Task**: Create a "battery status" service where a client requests the current battery percentage, and the server responds with voltage, percentage, and health status.

**Expected**:
- Request: Empty
- Response: voltage (float), percentage (int), is_healthy (bool)

**Code Example**: See `examples/module-01-ros2-fundamentals/example-03-service/`

## Summary

Services enable synchronous request-response communication for on-demand robotics tasks.

**Next**: [Chapter 5: Actions for Long-Running Tasks](./chapter-05-actions.md)

---

**Estimated Reading Time**: 20 minutes
**Hands-On Time**: 35 minutes
