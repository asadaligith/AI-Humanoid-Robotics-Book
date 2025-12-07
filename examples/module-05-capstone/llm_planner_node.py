#!/usr/bin/env python3
"""
LLM Planner Node - Capstone Autonomous Humanoid

Receives voice commands and generates structured action sequences using LLM.
Validates plans against capability manifest and publishes to /planning/action_sequence.

Dependencies:
- openai
- rclpy
- custom_msgs (ActionPlan message)

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json
import os
from pathlib import Path


# LLM Prompt Template
TASK_PLANNER_PROMPT = """You are a robot task planner. Generate a structured action sequence for the following voice command.

Available Actions (from capability manifest):
{capability_manifest}

Voice Command: "{voice_command}"

Output Format (strict JSON):
{{
  "command_understood": true/false,
  "clarification_needed": null or "question text",
  "action_sequence": [
    {{"action": "navigate_to", "params": {{"location": "kitchen_table"}}}},
    {{"action": "detect_object", "params": {{"name": "cup", "color": "red"}}}},
    {{"action": "pick_object", "params": {{"object_id": "detected_id"}}}},
    {{"action": "navigate_to", "params": {{"location": "user_location"}}}},
    {{"action": "place_object", "params": {{"location": "user_hand"}}}}
  ]
}}

Rules:
1. Only use actions from the capability manifest
2. Provide concrete parameters (no placeholders like "TBD" or "detected_id" - use descriptive names)
3. If command is ambiguous, set command_understood=false and provide clarification question
4. Navigation must precede detection/manipulation at each location
5. Pick must precede place for the same object
6. Always navigate back to user_location after picking object
7. Return ONLY valid JSON, no additional text
"""


class LLMPlannerNode(Node):
    """ROS 2 node for LLM-based task planning"""

    def __init__(self):
        super().__init__('llm_planner_node')

        # Load capability manifest
        manifest_path = Path(__file__).parent / 'config' / 'capability_manifest.json'
        with open(manifest_path, 'r') as f:
            self.capabilities = json.load(f)

        self.get_logger().info(f'Loaded {len(self.capabilities["capabilities"])} capabilities')

        # OpenAI API setup
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set!')
            raise ValueError('Missing OPENAI_API_KEY')

        openai.api_key = api_key

        # Declare parameters
        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('temperature', 0.0)  # Deterministic output
        self.declare_parameter('max_tokens', 1000)

        self.model = self.get_parameter('model').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value

        # Subscribers
        self.create_subscription(
            String,
            '/voice/transcribed_text',
            self.voice_callback,
            10
        )

        # Publishers
        self.plan_pub = self.create_publisher(
            String,  # Using String for now (would use custom_msgs/ActionPlan in production)
            '/planning/action_sequence',
            10
        )

        self.clarification_pub = self.create_publisher(
            String,
            '/planning/clarification_request',
            10
        )

        self.get_logger().info('LLM Planner Node initialized')

    def voice_callback(self, msg):
        """Generate action plan from voice command"""
        voice_command = msg.data
        self.get_logger().info(f'Received command: "{voice_command}"')

        try:
            # Construct prompt
            prompt = TASK_PLANNER_PROMPT.format(
                capability_manifest=json.dumps(self.capabilities, indent=2),
                voice_command=voice_command
            )

            # Call LLM
            self.get_logger().info('Calling LLM for plan generation...')
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=self.temperature,
                max_tokens=self.max_tokens
            )

            # Parse JSON response
            response_text = response.choices[0].message.content
            self.get_logger().debug(f'LLM response: {response_text}')

            plan_json = json.loads(response_text)

            # Validate plan structure
            if not self.validate_plan(plan_json):
                self.get_logger().error('Invalid plan structure from LLM')
                return

            # Check if clarification needed
            if not plan_json.get('command_understood', True):
                clarification = plan_json.get('clarification_needed', 'Command not understood')
                self.get_logger().warn(f'Clarification needed: {clarification}')

                # Publish clarification request
                msg = String()
                msg.data = clarification
                self.clarification_pub.publish(msg)
                return

            # Publish valid plan
            self.get_logger().info(f'Plan generated with {len(plan_json["action_sequence"])} actions')
            plan_msg = String()
            plan_msg.data = json.dumps(plan_json)
            self.plan_pub.publish(plan_msg)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse LLM response as JSON: {str(e)}')
        except openai.error.OpenAIError as e:
            self.get_logger().error(f'OpenAI API error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in planning: {str(e)}')

    def validate_plan(self, plan_json):
        """Validate plan structure against expected schema"""
        try:
            # Check required fields
            if 'command_understood' not in plan_json:
                self.get_logger().error('Missing field: command_understood')
                return False

            if 'action_sequence' not in plan_json:
                self.get_logger().error('Missing field: action_sequence')
                return False

            # Validate action sequence
            actions = plan_json['action_sequence']
            if not isinstance(actions, list):
                self.get_logger().error('action_sequence must be a list')
                return False

            # Validate each action
            valid_actions = {cap['action'] for cap in self.capabilities['capabilities']}

            for i, action in enumerate(actions):
                if not isinstance(action, dict):
                    self.get_logger().error(f'Action {i} is not a dict')
                    return False

                if 'action' not in action or 'params' not in action:
                    self.get_logger().error(f'Action {i} missing action or params field')
                    return False

                if action['action'] not in valid_actions:
                    self.get_logger().error(f'Invalid action: {action["action"]}')
                    return False

            self.get_logger().info('Plan validation passed')
            return True

        except Exception as e:
            self.get_logger().error(f'Validation error: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LLMPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
