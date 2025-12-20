#!/usr/bin/env python3
"""
Minimal Publisher Example for ROS 2
This node publishes String messages to the 'topic' topic at 1 Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal publisher node that sends messages periodically.

    This node demonstrates the basic publisher pattern in ROS 2:
    - Creates a publisher for std_msgs/String messages
    - Uses a timer to publish messages at a fixed rate
    - Logs each published message for visibility
    """

    def __init__(self):
        """Initialize the publisher node."""
        super().__init__('minimal_publisher')

        # Create a publisher
        # Parameters: message_type, topic_name, queue_size
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer that calls timer_callback every 1.0 seconds
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for message numbering
        self.i = 0

        self.get_logger().info('Publisher node started, publishing at 1 Hz')

    def timer_callback(self):
        """
        Timer callback function that publishes a message.

        This function is called every timer_period seconds.
        It creates a new message, populates it with data, and publishes it.
        """
        # Create a new String message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to initialize and run the publisher node.

    Args:
        args: Command line arguments (optional)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the publisher node
    minimal_publisher = MinimalPublisher()

    try:
        # Keep the node running and processing callbacks
        # This will run until interrupted (Ctrl+C)
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Handle graceful shutdown
        minimal_publisher.get_logger().info('Publisher shutting down...')
    finally:
        # Clean up and shutdown
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
