#!/usr/bin/env python3
"""
Manipulation Controller Node - MoveIt 2 Pick and Place

Interfaces with MoveIt 2 for collision-free motion planning and grasp execution.
Implements pick-and-place actions with pre-grasp, grasp, lift, transport, and place phases.
Includes failure recovery with retry logic and pose perturbation.

This node bridges perception and task completion by physically manipulating
objects detected in the environment.

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import math

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# MoveIt 2 imports (simplified for educational demo)
# In production, use moveit_py or moveit_commander
try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


class ManipulationController(Node):
    """
    ROS 2 node for MoveIt 2-based manipulation.

    Subscribes to pick/place goals and publishes manipulation status.
    """

    def __init__(self):
        super().__init__('manipulation_controller')

        # Declare parameters
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('gripper_group', 'gripper')
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('pre_grasp_offset_z', 0.1)  # 10cm above object
        self.declare_parameter('lift_offset_z', 0.2)  # 20cm lift
        self.declare_parameter('grasp_config_file', 'config/grasp_poses.yaml')

        self.planning_group = self.get_parameter('planning_group').value
        self.gripper_group = self.get_parameter('gripper_group').value
        self.max_retries = self.get_parameter('max_retries').value
        self.pre_grasp_offset = self.get_parameter('pre_grasp_offset_z').value
        self.lift_offset = self.get_parameter('lift_offset_z').value

        # Load grasp library
        self.grasp_library = self.load_grasp_library()

        # MoveIt action client (simplified - in production use MoveGroupInterface)
        if MOVEIT_AVAILABLE:
            self.moveit_client = ActionClient(self, MoveGroup, 'move_action')
            self.get_logger().info("MoveIt 2 action client initialized")
        else:
            self.get_logger().warn(
                "MoveIt 2 not available - running in simulation mode"
            )
            self.moveit_client = None

        # Subscribers
        self.pick_sub = self.create_subscription(
            PoseStamped,
            '/manipulation/pick_target',
            self.pick_callback,
            10
        )

        self.place_sub = self.create_subscription(
            PoseStamped,
            '/manipulation/place_target',
            self.place_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/manipulation/status',
            10
        )

        self.gripper_pub = self.create_publisher(
            JointState,
            '/gripper/command',
            10
        )

        # State
        self.current_object = None
        self.retry_count = 0

        self.get_logger().info(
            f"Manipulation controller initialized "
            f"(group={self.planning_group}, max_retries={self.max_retries})"
        )

    def load_grasp_library(self):
        """
        Load predefined grasp poses from YAML file.

        Returns:
            dict: Grasp configurations keyed by object name
        """
        grasp_file = self.get_parameter('grasp_config_file').value

        try:
            import yaml
            with open(grasp_file, 'r') as f:
                grasp_library = yaml.safe_load(f)
                self.get_logger().info(
                    f"Loaded grasp library with "
                    f"{len(grasp_library.get('grasp_library', {}))} objects"
                )
                return grasp_library.get('grasp_library', {})
        except FileNotFoundError:
            self.get_logger().warn(
                f"Grasp library not found: {grasp_file}. "
                "Using heuristic grasps only."
            )
            return {}
        except Exception as e:
            self.get_logger().error(f"Error loading grasp library: {e}")
            return {}

    def pick_callback(self, target_pose: PoseStamped):
        """
        Execute pick action with retry logic.

        Args:
            target_pose: Object pose to grasp (from perception)
        """
        self.get_logger().info(
            f"Received pick goal at "
            f"({target_pose.pose.position.x:.2f}, "
            f"{target_pose.pose.position.y:.2f}, "
            f"{target_pose.pose.position.z:.2f})"
        )

        self.publish_status("PICKING")

        # Attempt pick with retries
        for attempt in range(self.max_retries):
            success = self.execute_pick_sequence(target_pose)

            if success:
                self.get_logger().info("Pick succeeded!")
                self.publish_status("PICK_SUCCESS")
                self.retry_count = 0
                return
            else:
                self.retry_count = attempt + 1
                self.get_logger().warn(
                    f"Pick attempt {attempt + 1}/{self.max_retries} failed"
                )

                # Perturb pose for next attempt
                if attempt < self.max_retries - 1:
                    target_pose = self.perturb_pose(target_pose, max_offset=0.01)

        # All retries exhausted
        self.get_logger().error(f"Pick failed after {self.max_retries} attempts")
        self.publish_status("PICK_FAILED")

    def execute_pick_sequence(self, target_pose: PoseStamped) -> bool:
        """
        Execute full pick sequence: pre-grasp → grasp → lift.

        Args:
            target_pose: Object pose to grasp

        Returns:
            bool: True if pick succeeded, False otherwise
        """
        # Phase 1: Pre-grasp (approach from above)
        pre_grasp_pose = self.compute_pre_grasp(target_pose)
        if not self.plan_and_execute(pre_grasp_pose):
            self.get_logger().error("Pre-grasp motion failed")
            return False

        # Phase 2: Grasp (move to object and close gripper)
        if not self.plan_and_execute(target_pose):
            self.get_logger().error("Grasp approach failed")
            return False

        self.close_gripper()
        rclpy.spin_once(self, timeout_sec=0.5)  # Wait for gripper to close

        # Phase 3: Lift (retract upward)
        lift_pose = self.compute_lift_pose(target_pose)
        if not self.plan_and_execute(lift_pose):
            self.get_logger().error("Lift motion failed")
            self.open_gripper()  # Release object on failure
            return False

        return True

    def place_callback(self, target_pose: PoseStamped):
        """
        Execute place action.

        Args:
            target_pose: Target pose to place object
        """
        self.get_logger().info(
            f"Received place goal at "
            f"({target_pose.pose.position.x:.2f}, "
            f"{target_pose.pose.position.y:.2f})"
        )

        self.publish_status("PLACING")

        # Phase 1: Pre-place (approach from above)
        pre_place_pose = self.compute_pre_grasp(target_pose)
        if not self.plan_and_execute(pre_place_pose):
            self.get_logger().error("Pre-place motion failed")
            self.publish_status("PLACE_FAILED")
            return

        # Phase 2: Place (lower to target)
        if not self.plan_and_execute(target_pose):
            self.get_logger().error("Place motion failed")
            self.publish_status("PLACE_FAILED")
            return

        # Phase 3: Release gripper
        self.open_gripper()
        rclpy.spin_once(self, timeout_sec=0.5)

        # Phase 4: Retract
        retract_pose = self.compute_lift_pose(target_pose)
        self.plan_and_execute(retract_pose)

        self.get_logger().info("Place succeeded!")
        self.publish_status("PLACE_SUCCESS")

    def compute_pre_grasp(self, target_pose: PoseStamped) -> PoseStamped:
        """
        Compute pre-grasp pose (offset above target).

        Args:
            target_pose: Target grasp pose

        Returns:
            PoseStamped: Pre-grasp pose
        """
        pre_grasp = PoseStamped()
        pre_grasp.header = target_pose.header
        pre_grasp.pose = target_pose.pose

        # Offset upward in z-direction
        pre_grasp.pose.position.z += self.pre_grasp_offset

        return pre_grasp

    def compute_lift_pose(self, target_pose: PoseStamped) -> PoseStamped:
        """
        Compute lift pose (vertical retraction).

        Args:
            target_pose: Current pose

        Returns:
            PoseStamped: Lifted pose
        """
        lift = PoseStamped()
        lift.header = target_pose.header
        lift.pose = target_pose.pose
        lift.pose.position.z += self.lift_offset

        return lift

    def perturb_pose(self, pose: PoseStamped, max_offset: float = 0.01) -> PoseStamped:
        """
        Add small random perturbation to pose for retry attempts.

        Args:
            pose: Original pose
            max_offset: Maximum offset in meters

        Returns:
            PoseStamped: Perturbed pose
        """
        import random

        perturbed = PoseStamped()
        perturbed.header = pose.header
        perturbed.pose = pose.pose

        # Random offset in x and y
        perturbed.pose.position.x += random.uniform(-max_offset, max_offset)
        perturbed.pose.position.y += random.uniform(-max_offset, max_offset)

        self.get_logger().debug(f"Perturbed pose by {max_offset}m")
        return perturbed

    def plan_and_execute(self, target_pose: PoseStamped) -> bool:
        """
        Plan and execute motion to target pose using MoveIt 2.

        Args:
            target_pose: Goal pose

        Returns:
            bool: True if motion succeeded
        """
        if not MOVEIT_AVAILABLE or not self.moveit_client:
            # Simulation mode - assume success
            self.get_logger().info(
                f"[SIM] Moving to ({target_pose.pose.position.x:.2f}, "
                f"{target_pose.pose.position.y:.2f}, "
                f"{target_pose.pose.position.z:.2f})"
            )
            rclpy.spin_once(self, timeout_sec=0.5)
            return True

        # In production: use MoveIt 2 planning and execution
        # goal = MoveGroup.Goal()
        # goal.request.group_name = self.planning_group
        # goal.request.goal_constraints = self.pose_to_constraints(target_pose)
        # future = self.moveit_client.send_goal_async(goal)
        # ... handle result

        return True  # Placeholder

    def close_gripper(self):
        """Send command to close gripper."""
        gripper_msg = JointState()
        gripper_msg.name = ['gripper_joint']
        gripper_msg.position = [0.0]  # 0.0 = closed
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info("Gripper closed")

    def open_gripper(self):
        """Send command to open gripper."""
        gripper_msg = JointState()
        gripper_msg.name = ['gripper_joint']
        gripper_msg.position = [0.08]  # 0.08 = fully open (8cm)
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info("Gripper opened")

    def publish_status(self, status: str):
        """
        Publish manipulation status for integration demo.

        Args:
            status: Status string (PICKING, PICK_SUCCESS, etc.)
        """
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    manipulation_controller = ManipulationController()

    try:
        rclpy.spin(manipulation_controller)
    except KeyboardInterrupt:
        manipulation_controller.get_logger().info(
            "Manipulation controller stopped by user"
        )
    finally:
        manipulation_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
