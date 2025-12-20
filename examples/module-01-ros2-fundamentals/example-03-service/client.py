#!/usr/bin/env python3
"""
Minimal Service Client Example for ROS 2
This node calls the 'add_two_ints' service to add two integers.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalClientAsync(Node):
    """
    A minimal service client node that requests addition operations.

    This node demonstrates the basic service client pattern in ROS 2:
    - Creates a service client
    - Waits for service availability
    - Sends requests asynchronously
    - Processes responses
    """

    def __init__(self):
        """Initialize the service client node."""
        super().__init__('minimal_client_async')

        # Create a client for the AddTwoInts service
        # Parameters: service_type, service_name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        self.get_logger().info('Waiting for service "add_two_ints"...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service available!')

        # Create a request object
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send an addition request to the service.

        Args:
            a (int): First integer
            b (int): Second integer

        Returns:
            Future: A future object that will contain the response
        """
        # Populate the request
        self.req.a = a
        self.req.b = b

        # Log the request
        self.get_logger().info(f'Sending request: {a} + {b}')

        # Send the request asynchronously
        # This returns a future immediately without blocking
        self.future = self.cli.call_async(self.req)

        return self.future


def main(args=None):
    """
    Main function to initialize and run the service client.

    Usage:
        python3 client.py <a> <b>

    Args:
        args: Command line arguments (optional)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Parse command line arguments
    if len(sys.argv) != 3:
        print('Usage: client.py <a> <b>')
        print('Example: python3 client.py 5 7')
        return

    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('Error: Arguments must be integers')
        print('Example: python3 client.py 5 7')
        return

    # Create the client node
    minimal_client = MinimalClientAsync()

    # Send the request
    future = minimal_client.send_request(a, b)

    # Wait for the response (blocking)
    # This spins the node until the future is complete
    rclpy.spin_until_future_complete(minimal_client, future)

    try:
        # Get the response
        response = future.result()

        # Log the result
        minimal_client.get_logger().info(f'Result: {a} + {b} = {response.sum}')
        print(f'\nResult: {a} + {b} = {response.sum}')

    except Exception as e:
        minimal_client.get_logger().error(f'Service call failed: {e}')

    finally:
        # Clean up and shutdown
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
