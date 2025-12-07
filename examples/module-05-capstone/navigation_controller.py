#!/usr/bin/env python3
"""
Navigation Controller Node - Nav2 Action Client Wrapper

Provides simplified interface to Nav2 navigation stack for autonomous
goal-directed navigation. Handles action client communication, goal
cancellation, and navigation status reporting.

This node wraps the Nav2 NavigateToPose action server to provide a
simplified interface for the capstone integration demo.

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


class NavigationController(Node):
    """
    ROS 2 node that wraps Nav2's NavigateToPose action client.

    Subscribes to goal poses and publishes navigation status.
    """

    def __init__(self):
        super().__init__('navigation_controller')

        # Declare parameters
        self.declare_parameter('server_timeout', 10.0)
        self.declare_parameter('goal_tolerance', 0.1)

        self.server_timeout = self.get_parameter('server_timeout').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/navigation/goal',
            self.navigate_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/navigation/status',
            10
        )

        # State
        self.current_goal_handle = None
        self.goal_active = False

        self.get_logger().info("Navigation controller initialized")

    def navigate_callback(self, goal_pose: PoseStamped):
        """
        Handle new navigation goal.

        Args:
            goal_pose: Target pose in map frame
        """
        self.get_logger().info(
            f"Received navigation goal: "
            f"x={goal_pose.pose.position.x:.2f}, "
            f"y={goal_pose.pose.position.y:.2f}"
        )

        # Cancel existing goal if active
        if self.goal_active and self.current_goal_handle:
            self.get_logger().warn("Cancelling previous navigation goal")
            self.current_goal_handle.cancel_goal_async()

        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(
            timeout_sec=self.server_timeout
        ):
            self.get_logger().error(
                f"Nav2 action server not available after "
                f"{self.server_timeout}s timeout"
            )
            self.publish_status("FAILED: Nav2 server unavailable")
            return

        # Construct goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send goal
        self.publish_status("NAVIGATING")
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle goal acceptance response from Nav2.

        Args:
            future: Future containing goal handle
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected by Nav2")
            self.publish_status("FAILED: Goal rejected")
            return

        self.get_logger().info("Navigation goal accepted")
        self.current_goal_handle = goal_handle
        self.goal_active = True

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle periodic feedback from Nav2.

        Args:
            feedback_msg: Nav2 feedback (current pose, distance remaining)
        """
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose

        self.get_logger().info(
            f"Navigation feedback - current position: "
            f"x={current_pose.position.x:.2f}, "
            f"y={current_pose.position.y:.2f}",
            throttle_duration_sec=2.0  # Log every 2 seconds
        )

    def result_callback(self, future):
        """
        Handle navigation completion result.

        Args:
            future: Future containing navigation result
        """
        result = future.result()
        status = result.status

        self.goal_active = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded!")
            self.publish_status("SUCCEEDED")

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Navigation aborted (obstacle or planner failure)")
            self.publish_status("ABORTED")

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Navigation cancelled by user")
            self.publish_status("CANCELED")

        else:
            self.get_logger().error(f"Navigation failed with status: {status}")
            self.publish_status(f"FAILED: {status}")

    def publish_status(self, status: str):
        """
        Publish navigation status for integration demo.

        Args:
            status: Status string (NAVIGATING, SUCCEEDED, ABORTED, etc.)
        """
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    navigation_controller = NavigationController()

    try:
        rclpy.spin(navigation_controller)
    except KeyboardInterrupt:
        navigation_controller.get_logger().info("Navigation controller stopped by user")
    finally:
        # Cancel any active goals
        if navigation_controller.goal_active and navigation_controller.current_goal_handle:
            navigation_controller.get_logger().info("Cancelling active navigation goal")
            navigation_controller.current_goal_handle.cancel_goal_async()

        navigation_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
