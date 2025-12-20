# Example 01: Publisher-Subscriber Pattern

## Learning Objectives

By completing this example, you will:
- Understand the publish-subscribe communication pattern in ROS 2
- Create a simple publisher node that sends messages
- Create a subscriber node that receives and processes messages
- Learn about ROS 2 topics and message types
- Understand the decoupled nature of pub-sub communication

## Overview

The Publisher-Subscriber pattern is the most fundamental communication mechanism in ROS 2. It enables asynchronous, many-to-many communication between nodes through named topics.

**Key Concepts:**
- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic
- **Topic**: A named bus over which nodes exchange messages
- **Message Type**: The data structure of messages (e.g., `std_msgs/String`)

## Prerequisites

- ROS 2 Humble installed
- Basic Python knowledge
- Completed Module 01 Chapter 02

## Directory Structure

```
example-01-pubsub/
├── README.md           # This file
├── publisher.py        # Publisher node implementation
├── subscriber.py       # Subscriber node implementation
└── package.xml         # Package dependencies (optional)
```

## Code Walkthrough

### Publisher Node (`publisher.py`)

The publisher node creates a timer that publishes a message every second:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

**Key Components:**
1. **Node Initialization**: Inherits from `Node` class
2. **Publisher Creation**: `create_publisher(msg_type, topic_name, queue_size)`
3. **Timer**: Triggers callback at regular intervals
4. **Message Publishing**: Creates and publishes messages

### Subscriber Node (`subscriber.py`)

The subscriber node listens to the topic and processes incoming messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

**Key Components:**
1. **Subscription Creation**: `create_subscription(msg_type, topic_name, callback, queue_size)`
2. **Callback Function**: Automatically called when messages arrive
3. **Message Processing**: Access message data through `msg` parameter

## Running the Example

### Terminal 1: Start the Publisher

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Run the publisher
python3 publisher.py
```

Expected output:
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

### Terminal 2: Start the Subscriber

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Run the subscriber
python3 subscriber.py
```

Expected output:
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [minimal_subscriber]: I heard: "Hello World: 2"
...
```

## Introspection Tools

While the nodes are running, use these commands to inspect the system:

```bash
# List all active topics
ros2 topic list

# See information about the 'topic' topic
ros2 topic info /topic

# Echo messages being published
ros2 topic echo /topic

# Check the message rate
ros2 topic hz /topic

# View the node graph
ros2 node list
rqt_graph
```

## Exercises

### Exercise 1: Modify the Message
Change the publisher to send custom messages (e.g., your name, timestamps, counter).

### Exercise 2: Multiple Subscribers
Run multiple subscriber instances and observe that all receive the same messages.

### Exercise 3: Change the Topic Name
Modify both publisher and subscriber to use a different topic name like `/chatter`.

### Exercise 4: Adjust Publishing Rate
Change the timer period from 1.0 seconds to 0.5 seconds (2 Hz publishing rate).

### Exercise 5: Different Message Types
Use `std_msgs/Int32` instead of `String` and publish integer values.

```python
from std_msgs.msg import Int32

# In publisher
msg = Int32()
msg.data = self.i
```

## Common Issues

### Issue 1: "Module 'rclpy' has no attribute 'init'"
**Solution**: Ensure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`

### Issue 2: No messages received
**Solution**: Verify both nodes use the exact same topic name (case-sensitive).

### Issue 3: High CPU usage
**Solution**: Avoid tight loops without sleep/timers. Use ROS 2 timers for periodic tasks.

## Key Takeaways

1. **Decoupled Communication**: Publishers and subscribers don't need to know about each other
2. **Many-to-Many**: Multiple publishers and subscribers can use the same topic
3. **Asynchronous**: Publishing doesn't block; subscribers receive messages via callbacks
4. **Type Safety**: Publishers and subscribers must use the same message type
5. **Quality of Service (QoS)**: The queue size parameter affects reliability vs. performance

## Next Steps

- **Example 02**: Creating proper ROS 2 packages
- **Example 03**: Using Services for synchronous communication
- **Module 01 Chapter 03**: Deep dive into ROS 2 package structure

## References

- [ROS 2 Publisher-Subscriber Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [std_msgs Message Types](https://github.com/ros2/common_interfaces/tree/humble/std_msgs)
