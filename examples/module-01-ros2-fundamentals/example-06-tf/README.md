# Example 06: TF2 (Transform Library)

## Learning Objectives

By completing this example, you will:
- Understand coordinate frames and transformations
- Use TF2 to broadcast and listen to transforms
- Create and manage transform trees
- Perform coordinate transformations between frames
- Debug transform issues with TF2 tools

## Overview

TF2 (Transform Library 2) is the ROS 2 system for tracking coordinate frames over time. It maintains the relationship between multiple coordinate frames in a tree structure and allows you to transform data between any two frames at any point in time.

**Key Concepts:**
- **Frame**: A coordinate system (base_link, camera_frame, map, odom)
- **Transform**: Translation + rotation from one frame to another
- **TF Tree**: Hierarchical structure of all frames
- **Static Transform**: Never changes (camera → base_link)
- **Dynamic Transform**: Changes over time (map → robot_position)

## Prerequisites

- ROS 2 Humble installed
- Completed Examples 01-05
- Completed Module 01 Chapter 06

## Directory Structure

```
example-06-tf/
├── README.md                    # This file
├── static_broadcaster.py        # Static transform broadcaster
├── broadcaster.py               # Dynamic transform broadcaster
├── listener.py                  # Transform listener
└── launch/
    └── tf_demo.launch.py        # Launch all TF nodes
```

## Why Use TF2?

Without TF2:
```python
# Manually transform a point from camera to robot base
point_camera = [1.0, 0.0, 0.5]  # Point in camera frame

# You'd need to manually track:
camera_to_base_translation = [0.1, 0, 0.2]
camera_to_base_rotation = [0, 0, 0, 1]  # quaternion

# Manual transformation (error-prone!)
point_base = manual_transform(point_camera, camera_to_base_translation, camera_to_base_rotation)
```

With TF2:
```python
# TF2 handles all transformations automatically
transform = tf_buffer.lookup_transform('base_link', 'camera_frame', rclpy.time.Time())
point_base = do_transform_point(point_camera, transform)
```

## TF2 Architecture

```
      map
       |
      odom
       |
    base_link
    /   |   \
wheel  camera  lidar
```

## Broadcasting Transforms

### Static Transform (Fixed Relationship)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class StaticFramePublisher(Node):
    """
    Publishes a static transform from base_link to camera_frame.

    Static transforms are for relationships that never change,
    like a camera mounted on a robot chassis.
    """

    def __init__(self):
        super().__init__('static_tf_broadcaster')

        # Create static broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transform
        self.make_transforms()

    def make_transforms(self):
        # Create transform message
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Parent frame
        t.child_frame_id = 'camera_frame'  # Child frame

        # Translation (x, y, z) in meters
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2

        # Rotation as quaternion (x, y, z, w)
        # No rotation = (0, 0, 0, 1)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send static transform
        self.tf_static_broadcaster.sendTransform(t)

        self.get_logger().info('Published static transform: base_link -> camera_frame')


def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
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

### Dynamic Transform (Moving Relationship)

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DynamicFramePublisher(Node):
    """
    Publishes a dynamic transform that changes over time.

    Example: Robot's position in the world (odom -> base_link)
    """

    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')

        # Create dynamic broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish transforms periodically
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

        # Simulate robot movement
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.get_logger().info('Dynamic TF broadcaster started')

    def broadcast_timer_callback(self):
        # Simulate circular motion
        self.x = 2.0 * math.cos(self.theta)
        self.y = 2.0 * math.sin(self.theta)
        self.theta += 0.05

        # Create transform message
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame (world)
        t.child_frame_id = 'base_link'  # Child frame (robot)

        # Translation (robot position in world)
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Rotation (robot orientation)
        # Convert yaw angle to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        # Send transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicFramePublisher()
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

## Listening to Transforms

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point


class FrameListener(Node):
    """
    Listens to TF transforms and performs coordinate transformations.
    """

    def __init__(self):
        super().__init__('tf_listener')

        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to perform lookups
        self.timer = self.create_timer(1.0, self.on_timer)

        self.get_logger().info('TF listener started')

    def on_timer(self):
        # Define a point in the camera frame
        point_camera = PointStamped()
        point_camera.header.frame_id = 'camera_frame'
        point_camera.header.stamp = self.get_clock().now().to_msg()
        point_camera.point.x = 1.0
        point_camera.point.y = 0.0
        point_camera.point.z = 0.0

        try:
            # Look up transform from camera_frame to base_link
            # At the current time
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # Target frame
                'camera_frame',  # Source frame
                Time()  # Latest available transform
            )

            # Transform the point
            point_base = do_transform_point(point_camera, transform)

            self.get_logger().info(
                f'Point in camera: ({point_camera.point.x}, {point_camera.point.y}, {point_camera.point.z})'
            )
            self.get_logger().info(
                f'Point in base:   ({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f})'
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
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

## Running the Example

### Terminal 1: Static Transform
```bash
source /opt/ros/humble/setup.bash
python3 static_broadcaster.py
```

### Terminal 2: Dynamic Transform
```bash
source /opt/ros/humble/setup.bash
python3 broadcaster.py
```

### Terminal 3: Transform Listener
```bash
source /opt/ros/humble/setup.bash
python3 listener.py
```

## TF2 Command Line Tools

```bash
# View current TF tree
ros2 run tf2_ros tf2_echo base_link camera_frame

# Monitor transform between two frames
ros2 run tf2_ros tf2_echo odom base_link

# View TF tree as PDF
ros2 run tf2_tools view_frames

# Echo all transforms
ros2 topic echo /tf

# Monitor static transforms
ros2 topic echo /tf_static
```

## Common Frame Names (by Convention)

- **map**: World fixed frame (SLAM/localization origin)
- **odom**: Odometry frame (continuous, may drift)
- **base_link**: Robot's base center
- **base_footprint**: Projection of base_link on ground
- **camera_link**: Camera optical center
- **laser**: Lidar sensor frame
- **left_wheel**, **right_wheel**: Wheel frames

## Transform Lookup Options

### Latest Transform
```python
transform = tf_buffer.lookup_transform(
    target_frame,
    source_frame,
    Time()  # Get latest
)
```

### Transform at Specific Time
```python
transform = tf_buffer.lookup_transform(
    target_frame,
    source_frame,
    Time(seconds=123, nanoseconds=456)
)
```

### Wait for Transform
```python
try:
    transform = tf_buffer.lookup_transform(
        target_frame,
        source_frame,
        Time(),
        timeout=Duration(seconds=1.0)
    )
except (LookupException, ConnectivityException, ExtrapolationException):
    self.get_logger().error('Transform not available')
```

## Debugging TF Issues

### Problem: "Transform not found"
```bash
# Check if frames exist
ros2 run tf2_ros tf2_echo parent_frame child_frame

# Visualize TF tree
ros2 run tf2_tools view_frames
# Opens frames.pdf showing the tree
```

### Problem: "Extrapolation into the future"
- Your transform timestamp is in the future
- **Solution**: Use `Time()` for latest transform, or ensure timestamps are correct

### Problem: "Connectivity exception"
- Frames are not connected in the tree
- **Solution**: Check that all intermediate transforms are being published

## Visualizing TF in RViz

```bash
# Launch RViz
ros2 run rviz2 rviz2

# In RViz:
# 1. Set Fixed Frame to "map" or "odom"
# 2. Add -> TF display
# 3. You'll see all coordinate frames and their relationships
```

## Exercises

### Exercise 1: Multi-Sensor Robot
Create a TF tree with multiple sensors (lidar, cameras) on a robot.

### Exercise 2: Transform Points
Write a node that reads laser scan points and transforms them to the map frame.

### Exercise 3: TF from Odometry
Subscribe to odometry messages and publish odom → base_link transforms.

### Exercise 4: Time Travel
Look up a transform at a specific time in the past using the TF buffer.

### Exercise 5: TF Chain
Create a chain: world → robot → arm → gripper → object

## Common Mistakes

1. **Forgetting to publish**: No broadcaster means no transform
2. **Wrong timestamp**: Use `get_clock().now()` for current time
3. **Reversed frames**: Parent/child order matters
4. **Missing static transforms**: Sensor mounts should be static
5. **TF vs TF Static**: Static transforms use `/tf_static` topic

## Best Practices

1. **Use Static for Fixed Relationships**: Camera mounts, sensor positions
2. **Consistent Frame Names**: Follow REP-105 conventions
3. **Publish at Consistent Rate**: At least 10 Hz for dynamic transforms
4. **Include Timestamp**: Always set header.stamp
5. **Error Handling**: Always catch TF exceptions

## Key Takeaways

1. **TF2 Manages Frames**: Tracks all coordinate systems automatically
2. **Tree Structure**: Parent-child relationships form a tree
3. **Time Support**: Can query transforms at any time (within buffer)
4. **Automatic Chaining**: TF2 computes multi-hop transforms
5. **Essential for Robotics**: Sensors, navigation, manipulation all use TF2

## Next Steps

- **Module 02**: Using TF2 with URDF and Gazebo
- **Module 03**: TF2 in navigation and SLAM
- **Advanced TF2**: Time synchronization, message filters

## References

- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
- [tf2_ros API Documentation](https://docs.ros2.org/latest/api/tf2_ros/)
- [TF2 Design Document](http://wiki.ros.org/tf2/Design)
