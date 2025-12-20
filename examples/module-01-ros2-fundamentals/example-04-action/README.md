# Example 04: Actions (Long-Running Operations with Feedback)

## Learning Objectives

By completing this example, you will:
- Understand the action communication pattern in ROS 2
- Create an action server for long-running tasks
- Implement an action client that sends goals and receives feedback
- Handle goal cancellation and result retrieval
- Learn when to use actions vs. services

## Overview

Actions are designed for long-running tasks that provide feedback during execution, support cancellation, and return a final result. They combine elements of both topics and services to create a more powerful communication pattern.

**Key Concepts:**
- **Action Server**: Executes long-running tasks and provides feedback
- **Action Client**: Sends goals, receives feedback, and retrieves results
- **Goal**: The task request sent by the client
- **Feedback**: Periodic updates during task execution
- **Result**: Final outcome returned when the task completes

## When to Use Actions

**Use Actions For:**
- Long-running operations (navigation, manipulation, charging)
- Tasks requiring progress feedback (file downloads, computations)
- Operations that may need cancellation (robot movements)
- Asynchronous operations with definitive completion

**Use Services For:**
- Quick request-response operations
- Tasks that complete immediately
- Operations without intermediate feedback

**Use Topics For:**
- Continuous data streams
- Broadcasting state updates
- Fire-and-forget messaging

## Prerequisites

- ROS 2 Humble installed
- Completed Examples 01-03
- Completed Module 01 Chapter 05

## Directory Structure

```
example-04-action/
├── README.md           # This file
├── fibonacci_server.py # Action server implementation
├── fibonacci_client.py # Action client implementation
└── package.xml         # Package dependencies (optional)
```

## Action Type: `example_interfaces/Fibonacci`

This example uses the built-in `Fibonacci` action type:

**Goal:**
```
int32 order    # Number of Fibonacci numbers to compute
```

**Feedback:**
```
int32[] partial_sequence    # Fibonacci numbers computed so far
```

**Result:**
```
int32[] sequence    # Complete Fibonacci sequence
```

## Code Walkthrough

### Action Server (`fibonacci_server.py`)

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Initialize feedback and result
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] +
                feedback_msg.partial_sequence[i-1]
            )

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
```

**Key Components:**
1. **ActionServer Creation**: Registers action with name and callback
2. **Goal Handle**: Manages goal lifecycle (accept, execute, succeed/abort/cancel)
3. **Feedback Publishing**: Sends periodic updates to client
4. **Cancellation Handling**: Checks if client requested cancellation
5. **Result Return**: Provides final outcome

### Action Client (`fibonacci_client.py`)

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()
```

**Key Components:**
1. **ActionClient Creation**: Connects to action server
2. **Goal Sending**: Asynchronous goal submission with callbacks
3. **Feedback Callback**: Processes periodic updates
4. **Result Callback**: Handles final result
5. **Future Handling**: Manages asynchronous operations

## Running the Example

### Terminal 1: Start the Action Server

```bash
source /opt/ros/humble/setup.bash
python3 fibonacci_server.py
```

Expected output:
```
[INFO] [fibonacci_action_server]: Action server ready
```

### Terminal 2: Run the Action Client

```bash
source /opt/ros/humble/setup.bash
python3 fibonacci_client.py 10
```

Expected output (client):
```
[INFO] [fibonacci_action_client]: Goal accepted
[INFO] [fibonacci_action_client]: Received feedback: [0, 1]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1]
[INFO] [fibonacci_action_client]: Received feedback: [0, 1, 1, 2]
...
[INFO] [fibonacci_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

## Command Line Action Calls

```bash
# List all active actions
ros2 action list

# Get information about an action
ros2 action info /fibonacci

# Send a goal from command line
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}"

# Send goal and show feedback
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" --feedback
```

## Introspection Tools

```bash
# List all actions
ros2 action list

# Show action type
ros2 action type /fibonacci

# Show action definition
ros2 interface show example_interfaces/action/Fibonacci

# Monitor action with rqt
rqt_graph
```

## Goal Lifecycle

```
IDLE → ACCEPTED → EXECUTING → SUCCEEDED/ABORTED/CANCELED
         ↓
      REJECTED
```

1. **IDLE**: No goal active
2. **ACCEPTED**: Server accepted the goal
3. **EXECUTING**: Server is processing the goal
4. **SUCCEEDED**: Goal completed successfully
5. **ABORTED**: Server terminated the goal due to an error
6. **CANCELED**: Client or server canceled the goal
7. **REJECTED**: Server rejected the goal

## Exercises

### Exercise 1: Add Cancellation Support
Modify the client to cancel the goal after receiving 5 feedback messages.

```python
def feedback_callback(self, feedback_msg):
    if len(feedback_msg.feedback.partial_sequence) >= 5:
        self._cancel_future = self._goal_handle.cancel_goal_async()
```

### Exercise 2: Timeout Handling
Implement a timeout mechanism that aborts the goal if it takes too long.

### Exercise 3: Multiple Goal Management
Modify the server to handle multiple concurrent goals using threading.

### Exercise 4: Progress Bar
Display a progress bar in the client based on feedback (e.g., showing percentage completion).

### Exercise 5: Custom Action
Create a custom action for a "count down timer" with:
- Goal: target seconds
- Feedback: seconds remaining
- Result: completion status

## Common Issues

### Issue 1: "Action server not available"
**Solution**: Ensure the server is running before starting the client. Use `wait_for_server()`.

### Issue 2: Goal rejected unexpectedly
**Solution**: Check server logs for rejection reason. Verify goal message format.

### Issue 3: Feedback not received
**Solution**: Verify feedback callback is registered in `send_goal_async()`.

### Issue 4: Result never received
**Solution**: Ensure server calls `goal_handle.succeed()` or `goal_handle.abort()`.

## Actions vs. Services Comparison

| Feature | Actions | Services |
|---------|---------|----------|
| Duration | Long-running | Short |
| Feedback | Yes | No |
| Cancellation | Yes | No |
| Result | Yes | Yes |
| Use Case | Navigation, manipulation | Quick queries |

## Advanced Topics

### Preemption (Multiple Goals)

```python
# In server execute_callback
if goal_handle.is_active:
    # Preempt current goal
    current_goal_handle.abort()

# Execute new goal
```

### Goal Queuing

```python
class MultiGoalActionServer(Node):
    def __init__(self):
        self.goal_queue = []
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback,
            goal_callback=self.goal_callback
        )

    def goal_callback(self, goal_request):
        # Accept or reject based on queue size
        if len(self.goal_queue) < 10:
            return GoalResponse.ACCEPT
        return GoalResponse.REJECT
```

## Key Takeaways

1. **Actions for Long Tasks**: Use actions for operations that take time
2. **Feedback is Essential**: Clients need progress updates for user experience
3. **Cancellation Support**: Always handle cancellation gracefully
4. **Asynchronous by Design**: Actions use callbacks and futures
5. **Goal Lifecycle Management**: Properly transition through states

## Next Steps

- **Example 05**: URDF for robot modeling
- **Example 06**: TF2 for coordinate transformations
- **Module 01 Chapter 06**: Advanced action patterns

## References

- [ROS 2 Action Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-an-Action-Server-Client/Py.html)
- [Action Design Guide](https://docs.ros.org/en/humble/Concepts/About-Actions.html)
- [example_interfaces Actions](https://github.com/ros2/example_interfaces/tree/humble/action)
