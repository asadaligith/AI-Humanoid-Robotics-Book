#!/usr/bin/env python3
"""
Integration Demo - Capstone Autonomous Humanoid

Central state machine that orchestrates all five capabilities:
- Voice Input (Whisper)
- LLM Planning
- Navigation (Nav2)
- Object Detection (Computer Vision)
- Manipulation (MoveIt 2)

This node implements the 11-state finite state machine for voice-commanded
fetch-and-deliver tasks.

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
import json
from enum import Enum, auto
import time


class State(Enum):
    """11-state finite state machine for task execution"""
    IDLE = auto()
    LISTENING = auto()
    TRANSCRIBING = auto()
    PLANNING = auto()
    VALIDATING = auto()
    NAVIGATING = auto()
    PERCEIVING = auto()
    MANIPULATING = auto()
    COMPLETED = auto()
    FAILED = auto()
    AWAITING_CLARIFICATION = auto()


class IntegrationDemo(Node):
    """State machine for integrated autonomous system"""

    def __init__(self):
        super().__init__('integration_demo')

        # Mock mode configuration (for independent capability testing)
        self.declare_parameter('mock_voice', False)
        self.declare_parameter('mock_llm', False)
        self.declare_parameter('mock_navigation', False)
        self.declare_parameter('mock_perception', False)
        self.declare_parameter('mock_manipulation', False)
        self.declare_parameter('test_mode', False)  # Enable all mocks for unit testing

        # Read mock parameters
        test_mode = self.get_parameter('test_mode').value
        self.mock_voice = self.get_parameter('mock_voice').value or test_mode
        self.mock_llm = self.get_parameter('mock_llm').value or test_mode
        self.mock_navigation = self.get_parameter('mock_navigation').value or test_mode
        self.mock_perception = self.get_parameter('mock_perception').value or test_mode
        self.mock_manipulation = self.get_parameter('mock_manipulation').value or test_mode

        # State machine
        self.state = State.IDLE
        self.current_plan = None
        self.current_action_index = 0
        self.retry_count = 0
        self.max_retries = 3

        # Log mock mode status
        if test_mode:
            self.get_logger().warn('TEST MODE ENABLED - All capabilities mocked')
        else:
            mock_status = []
            if self.mock_voice:
                mock_status.append('voice')
            if self.mock_llm:
                mock_status.append('llm')
            if self.mock_navigation:
                mock_status.append('navigation')
            if self.mock_perception:
                mock_status.append('perception')
            if self.mock_manipulation:
                mock_status.append('manipulation')

            if mock_status:
                self.get_logger().warn(f'Mock modes active: {", ".join(mock_status)}')

        # Subscribers
        self.create_subscription(
            String,
            '/voice/transcribed_text',
            self.voice_callback,
            10
        )

        self.create_subscription(
            String,
            '/planning/action_sequence',
            self.plan_callback,
            10
        )

        self.create_subscription(
            String,
            '/planning/clarification_request',
            self.clarification_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(String, '/system/status', 10)

        # Action clients (would use actual action types in production)
        # self.nav_client = ActionClient(self, NavigateToPose, '/navigation/navigate_to_pose')
        # self.pick_client = ActionClient(self, PickObject, '/manipulation/pick_object')
        # self.place_client = ActionClient(self, PlaceObject, '/manipulation/place_object')

        # State machine timer (check state every 100ms)
        self.create_timer(0.1, self.state_machine_callback)

        self.get_logger().info('Integration Demo initialized - State: IDLE')
        self.publish_status('IDLE - Waiting for voice command')

    def state_machine_callback(self):
        """Main state machine logic"""
        # This is a simplified state machine - full implementation would
        # include all state transitions and action execution

        if self.state == State.IDLE:
            # Waiting for voice input
            pass

        elif self.state == State.LISTENING:
            # Voice node is capturing audio
            pass

        elif self.state == State.TRANSCRIBING:
            # Whisper is processing audio
            pass

        elif self.state == State.PLANNING:
            # LLM is generating plan
            # Timeout after 30 seconds
            pass

        elif self.state == State.VALIDATING:
            # Check plan feasibility
            if self.current_plan:
                if self.validate_current_plan():
                    self.transition_to(State.NAVIGATING)
                    self.execute_current_action()
                else:
                    self.transition_to(State.FAILED)

        elif self.state == State.NAVIGATING:
            # Navigation action in progress
            # Handled by action client callbacks
            pass

        elif self.state == State.PERCEIVING:
            # Object detection in progress
            pass

        elif self.state == State.MANIPULATING:
            # Pick/place action in progress
            pass

        elif self.state == State.COMPLETED:
            # Task finished successfully
            self.get_logger().info('Task completed successfully!')
            self.reset_state_machine()

        elif self.state == State.FAILED:
            # Error occurred
            self.get_logger().error(f'Task failed in state: {self.state.name}')
            self.reset_state_machine()

        elif self.state == State.AWAITING_CLARIFICATION:
            # Waiting for user to provide more info
            pass

    def voice_callback(self, msg):
        """Handle transcribed voice commands"""
        if self.state != State.IDLE:
            self.get_logger().warn(f'Received voice command but not in IDLE state (current: {self.state.name})')
            return

        transcription = msg.data
        self.get_logger().info(f'Voice command received: "{transcription}"')

        # Mock LLM mode: bypass LLM planning with hardcoded plan
        if self.mock_llm:
            self.get_logger().info('[MOCK] Bypassing LLM with predefined plan')
            mock_plan = {
                'action_sequence': [
                    {'action': 'navigate_to', 'params': {'location': 'kitchen'}},
                    {'action': 'detect_object', 'params': {'name': 'mug', 'color': 'any'}},
                    {'action': 'pick_object', 'params': {'object_id': 'mug_0'}},
                    {'action': 'navigate_to', 'params': {'location': 'user'}},
                    {'action': 'place_object', 'params': {'location': 'table'}}
                ],
                'confidence': 0.95
            }
            self.current_plan = mock_plan
            self.current_action_index = 0
            self.transition_to(State.VALIDATING)
        else:
            self.transition_to(State.PLANNING)

    def plan_callback(self, msg):
        """Handle LLM-generated action plans"""
        if self.state != State.PLANNING:
            self.get_logger().warn(f'Received plan but not in PLANNING state (current: {self.state.name})')
            return

        try:
            plan = json.loads(msg.data)
            self.current_plan = plan
            self.current_action_index = 0

            num_actions = len(plan.get('action_sequence', []))
            self.get_logger().info(f'Plan received with {num_actions} actions')

            self.transition_to(State.VALIDATING)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse plan JSON: {str(e)}')
            self.transition_to(State.FAILED)

    def clarification_callback(self, msg):
        """Handle clarification requests from LLM"""
        clarification = msg.data
        self.get_logger().warn(f'Clarification needed: {clarification}')

        self.transition_to(State.AWAITING_CLARIFICATION)
        self.publish_status(f'AWAITING_CLARIFICATION: {clarification}')

    def validate_current_plan(self):
        """Validate plan before execution"""
        if not self.current_plan:
            return False

        actions = self.current_plan.get('action_sequence', [])
        if not actions:
            self.get_logger().error('Plan has no actions')
            return False

        # Check that all actions have required fields
        for i, action in enumerate(actions):
            if 'action' not in action or 'params' not in action:
                self.get_logger().error(f'Action {i} missing required fields')
                return False

        self.get_logger().info('Plan validation passed')
        return True

    def execute_current_action(self):
        """Execute the current action in the plan"""
        if not self.current_plan:
            self.transition_to(State.FAILED)
            return

        actions = self.current_plan.get('action_sequence', [])
        if self.current_action_index >= len(actions):
            # All actions completed
            self.transition_to(State.COMPLETED)
            return

        action = actions[self.current_action_index]
        action_name = action['action']
        params = action['params']

        self.get_logger().info(f'Executing action {self.current_action_index + 1}/{len(actions)}: {action_name}')

        # Dispatch to appropriate controller
        if action_name == 'navigate_to':
            self.execute_navigation(params)
        elif action_name == 'detect_object':
            self.execute_detection(params)
        elif action_name == 'pick_object':
            self.execute_pick(params)
        elif action_name == 'place_object':
            self.execute_place(params)
        elif action_name == 'wait':
            self.execute_wait(params)
        else:
            self.get_logger().error(f'Unknown action: {action_name}')
            self.transition_to(State.FAILED)

    def execute_navigation(self, params):
        """Execute navigation action"""
        location = params.get('location')
        self.get_logger().info(f'Navigating to: {location}')

        self.transition_to(State.NAVIGATING)

        if self.mock_navigation:
            # Mock mode: simulate navigation without Nav2
            self.get_logger().info(f'[MOCK] Simulating navigation to {location}')
            time.sleep(0.5)  # Quick simulation
            self.on_navigation_success()
        else:
            # Real implementation: send goal to Nav2 action server
            # goal = NavigateToPose.Goal()
            # goal.pose = self.get_pose_for_location(location)
            # self.nav_client.send_goal_async(goal, feedback_callback=self.nav_feedback_callback)

            # For now, simulate success after 2 seconds
            time.sleep(2)
            self.on_navigation_success()

    def execute_detection(self, params):
        """Execute object detection action"""
        object_name = params.get('name')
        object_color = params.get('color', 'any')

        self.get_logger().info(f'Detecting object: {object_color} {object_name}')

        self.transition_to(State.PERCEIVING)

        if self.mock_perception:
            # Mock mode: simulate object detection
            self.get_logger().info(f'[MOCK] Simulating detection of {object_color} {object_name}')
            time.sleep(0.2)  # Quick simulation
            self.on_detection_success()
        else:
            # Real implementation: trigger object detection and wait for result
            # For now, simulate detection
            time.sleep(1)
            self.on_detection_success()

    def execute_pick(self, params):
        """Execute pick action"""
        object_id = params.get('object_id')

        self.get_logger().info(f'Picking object: {object_id}')

        self.transition_to(State.MANIPULATING)

        if self.mock_manipulation:
            # Mock mode: simulate grasp execution
            self.get_logger().info(f'[MOCK] Simulating pick of {object_id}')
            time.sleep(0.3)  # Quick simulation
            self.on_manipulation_success()
        else:
            # Real implementation: send goal to MoveIt 2 pick action
            # For now, simulate grasp
            time.sleep(2)
            self.on_manipulation_success()

    def execute_place(self, params):
        """Execute place action"""
        location = params.get('location')

        self.get_logger().info(f'Placing object at: {location}')

        self.transition_to(State.MANIPULATING)

        if self.mock_manipulation:
            # Mock mode: simulate place execution
            self.get_logger().info(f'[MOCK] Simulating place at {location}')
            time.sleep(0.3)  # Quick simulation
            self.on_manipulation_success()
        else:
            # Real implementation: send goal to MoveIt 2 place action
            time.sleep(2)
            self.on_manipulation_success()

    def execute_wait(self, params):
        """Execute wait action"""
        duration = params.get('duration', 1.0)
        self.get_logger().info(f'Waiting for {duration}s')
        time.sleep(duration)
        self.on_action_complete()

    def on_navigation_success(self):
        """Callback when navigation completes successfully"""
        self.get_logger().info('Navigation successful')
        self.retry_count = 0  # Reset retry counter on success
        self.on_action_complete()

    def on_navigation_failure(self, error_msg: str = "Unknown"):
        """
        Callback when navigation fails.

        Edge case handling:
        - Retry up to 3 times with replanning
        - If all retries fail, request LLM to generate alternative approach
        """
        self.get_logger().error(f'Navigation failed: {error_msg}')

        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().warn(f'Navigation retry {self.retry_count}/{self.max_retries}')

            # Clear costmap for fresh planning attempt
            # In production: self.clear_costmap_service.call_async(Empty.Request())
            time.sleep(0.5)

            # Retry navigation
            actions = self.current_plan.get('action_sequence', [])
            current_action = actions[self.current_action_index]
            self.execute_navigation(current_action['params'])
        else:
            # All retries exhausted - request LLM replanning
            self.get_logger().error('Navigation failed after max retries')
            self.request_llm_replan(f"Navigation failed: {error_msg}. Suggest alternative route or location.")

    def on_detection_success(self):
        """Callback when object detection completes"""
        self.get_logger().info('Object detected successfully')
        self.retry_count = 0
        self.on_action_complete()

    def on_detection_failure(self, error_msg: str = "Object not found"):
        """
        Callback when object detection fails.

        Edge case handling:
        - Lower confidence threshold and retry
        - If still fails, request LLM clarification (ambiguous command?)
        """
        self.get_logger().error(f'Detection failed: {error_msg}')

        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().warn(f'Detection retry {self.retry_count}/{self.max_retries} (lowering threshold)')

            # Retry with lower confidence threshold
            actions = self.current_plan.get('action_sequence', [])
            current_action = actions[self.current_action_index]

            # In production: adjust detection node's confidence parameter
            time.sleep(0.5)
            self.execute_detection(current_action['params'])
        else:
            # Request clarification from user
            self.get_logger().error('Object detection failed after max retries')
            self.request_clarification(
                f"Could not detect {current_action['params'].get('name', 'object')}. "
                "Please specify color, size, or different location."
            )

    def on_manipulation_success(self):
        """Callback when manipulation completes"""
        self.get_logger().info('Manipulation successful')
        self.retry_count = 0
        self.on_action_complete()

    def on_manipulation_failure(self, error_msg: str = "Grasp failed"):
        """
        Callback when manipulation fails.

        Edge case handling:
        - Perturb grasp pose slightly and retry
        - After 3 failures, return to perception step to re-detect object
        """
        self.get_logger().error(f'Manipulation failed: {error_msg}')

        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().warn(f'Manipulation retry {self.retry_count}/{self.max_retries} (perturbing grasp)')

            # Retry with perturbed grasp pose
            actions = self.current_plan.get('action_sequence', [])
            current_action = actions[self.current_action_index]

            time.sleep(0.5)

            # In production: adjust grasp pose by ±1cm random offset
            if current_action['action'] == 'pick_object':
                self.execute_pick(current_action['params'])
            else:
                self.execute_place(current_action['params'])
        else:
            # All retries failed - go back to perception to re-detect object
            self.get_logger().error('Manipulation failed after max retries, re-detecting object')

            # Move action index back to perception step
            actions = self.current_plan.get('action_sequence', [])
            for i in range(self.current_action_index - 1, -1, -1):
                if actions[i]['action'] == 'detect_object':
                    self.current_action_index = i
                    self.retry_count = 0
                    self.execute_current_action()
                    return

            # If no perception step found, fail task
            self.transition_to(State.FAILED)

    def request_llm_replan(self, reason: str):
        """
        Request LLM to generate alternative plan due to failure.

        Args:
            reason: Explanation of why replanning is needed
        """
        self.get_logger().warn(f'Requesting LLM replan: {reason}')

        # In production: send reason back to LLM node with original command
        # For now, transition to clarification state
        self.transition_to(State.AWAITING_CLARIFICATION)
        self.publish_status(f'REPLAN_REQUESTED: {reason}')

    def request_clarification(self, question: str):
        """
        Request clarification from user.

        Args:
            question: Clarification question to ask user
        """
        self.get_logger().warn(f'Requesting clarification: {question}')

        self.transition_to(State.AWAITING_CLARIFICATION)
        self.publish_status(f'CLARIFICATION: {question}')

        # In production: publish to /llm/clarification_request
        # User provides answer via voice or text input

    def on_action_complete(self):
        """Move to next action in sequence"""
        self.current_action_index += 1
        self.retry_count = 0

        # Execute next action or complete
        if self.current_action_index < len(self.current_plan['action_sequence']):
            self.execute_current_action()
        else:
            self.transition_to(State.COMPLETED)

    def transition_to(self, new_state):
        """Transition to new state with logging"""
        old_state = self.state
        self.state = new_state

        self.get_logger().info(f'State transition: {old_state.name} → {new_state.name}')
        self.publish_status(f'{new_state.name}')

    def reset_state_machine(self):
        """Reset to IDLE state"""
        self.current_plan = None
        self.current_action_index = 0
        self.retry_count = 0
        self.transition_to(State.IDLE)

    def publish_status(self, status_text):
        """Publish current status"""
        msg = String()
        msg.data = f'[{self.state.name}] {status_text}'
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = IntegrationDemo()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
