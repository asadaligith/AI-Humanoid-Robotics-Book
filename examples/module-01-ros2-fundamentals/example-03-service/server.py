#!/usr/bin/env python3
"""
Minimal Service Server Example for ROS 2
This node provides an 'add_two_ints' service that adds two integers.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):
    """
    A minimal service server node that adds two integers.

    This node demonstrates the basic service pattern in ROS 2:
    - Creates a service server
    - Defines a callback to process requests
    - Returns responses to clients
    """

    def __init__(self):
        """Initialize the service server node."""
        super().__init__('minimal_service')

        # Create a service
        # Parameters: service_type, service_name, callback_function
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Service ready: add_two_ints')
        self.get_logger().info('Call with: ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"')

    def add_two_ints_callback(self, request, response):
        """
        Service callback that processes addition requests.

        This function is called whenever a client sends a request.
        It receives the request, performs the operation, and returns the response.

        Args:
            request (AddTwoInts.Request): Contains 'a' and 'b' integers
            response (AddTwoInts.Response): Will be populated with 'sum'

        Returns:
            AddTwoInts.Response: The populated response with the sum
        """
        # Log the incoming request
        self.get_logger().info(f'Incoming request: {request.a} + {request.b}')

        # Perform the addition
        response.sum = request.a + request.b

        # Log the result
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')

        # Return the response to the client
        return response


def main(args=None):
    """
    Main function to initialize and run the service server.

    Args:
        args: Command line arguments (optional)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the service server node
    minimal_service = MinimalService()

    try:
        # Keep the node running and processing service requests
        # This will run until interrupted (Ctrl+C)
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        # Handle graceful shutdown
        minimal_service.get_logger().info('Service server shutting down...')
    finally:
        # Clean up and shutdown
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
