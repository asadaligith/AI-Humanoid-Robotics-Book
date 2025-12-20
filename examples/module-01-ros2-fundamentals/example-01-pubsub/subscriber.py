#!/usr/bin/env python3
"""
Minimal Subscriber Example for ROS 2
This node subscribes to String messages on the 'topic' topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal subscriber node that receives and processes messages.

    This node demonstrates the basic subscriber pattern in ROS 2:
    - Creates a subscription to std_msgs/String messages
    - Defines a callback function to process incoming messages
    - Logs each received message for visibility
    """

    def __init__(self):
        """Initialize the subscriber node."""
        super().__init__('minimal_subscriber')

        # Create a subscription
        # Parameters: message_type, topic_name, callback_function, queue_size
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        # (self.subscription is stored but never explicitly used)
        self.subscription

        self.get_logger().info('Subscriber node started, listening on "topic"')

    def listener_callback(self, msg):
        """
        Callback function that processes received messages.

        This function is automatically called whenever a message arrives
        on the subscribed topic.

        Args:
            msg (String): The received message
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')

        # Additional processing can be added here
        # For example:
        # - Store message data
        # - Trigger actions based on message content
        # - Forward data to other nodes
        # - Update internal state


def main(args=None):
    """
    Main function to initialize and run the subscriber node.

    Args:
        args: Command line arguments (optional)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the subscriber node
    minimal_subscriber = MinimalSubscriber()

    try:
        # Keep the node running and processing callbacks
        # This will run until interrupted (Ctrl+C)
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Handle graceful shutdown
        minimal_subscriber.get_logger().info('Subscriber shutting down...')
    finally:
        # Clean up and shutdown
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
