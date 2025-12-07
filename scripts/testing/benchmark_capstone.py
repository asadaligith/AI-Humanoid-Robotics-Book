#!/usr/bin/env python3
"""
Capstone Performance Benchmarking Suite

Automated test suite for measuring end-to-end performance metrics:
- Task success rate (target: ≥60%)
- End-to-end latency (target: <30s)
- Component latencies (voice, LLM, navigation, perception, manipulation)
- Resource utilization (CPU, GPU, RAM)

Runs 20 trials with varying objects, layouts, and environmental conditions
to produce statistical performance analysis.

Usage:
    python3 benchmark_capstone.py --trials 20 --output results.json
    python3 benchmark_capstone.py --quick  # 5 trials for rapid testing

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
import time
import json
import argparse
import statistics
from dataclasses import dataclass, asdict
from typing import List, Dict, Any
import sys


@dataclass
class TrialResult:
    """Result of a single benchmark trial"""
    trial_id: int
    command: str
    success: bool
    total_latency: float  # seconds
    voice_latency: float
    llm_latency: float
    navigation_latency: float
    perception_latency: float
    manipulation_latency: float
    failure_mode: str  # Empty if success, error type if failed
    timestamp: float


class CapstoneBenchmark(Node):
    """
    ROS 2 node for automated benchmarking of the capstone system.
    """

    def __init__(self, num_trials: int = 20):
        super().__init__('capstone_benchmark')

        self.num_trials = num_trials
        self.results: List[TrialResult] = []
        self.current_trial = 0

        # Test commands (varying complexity and objects)
        self.test_commands = [
            "Bring me the mug from the kitchen",
            "Get the red apple from the counter",
            "Navigate to the living room",
            "Pick up the bottle and place it on the table",
            "Find the cup in the bedroom",
            "Deliver the book to the desk",
            "Fetch the banana from the kitchen",
            "Move the vase to the shelf",
            "Get the remote from the couch",
            "Bring the water bottle to me",
            "Pick up the orange from the bowl",
            "Place the phone on the charging station",
            "Get the scissors from the drawer",
            "Bring me the coffee mug",
            "Fetch the apple from the refrigerator",
            "Move the laptop to the desk",
            "Get the pillow from the bedroom",
            "Bring the towel from the bathroom",
            "Pick up the keys from the table",
            "Deliver the package to the door",
        ]

        # Subscribers for timing measurements
        self.create_subscription(
            String,
            '/system/status',
            self.status_callback,
            10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            String,
            '/voice/transcribed_text',
            10
        )

        # Timing state
        self.trial_start_time = None
        self.voice_start = None
        self.llm_start = None
        self.nav_start = None
        self.perception_start = None
        self.manipulation_start = None

        self.voice_latency = 0.0
        self.llm_latency = 0.0
        self.nav_latency = 0.0
        self.perception_latency = 0.0
        self.manipulation_latency = 0.0

        self.current_command = ""
        self.trial_success = False
        self.failure_mode = ""

        self.get_logger().info(f'Benchmark suite initialized ({num_trials} trials)')

    def status_callback(self, msg: String):
        """
        Monitor system status to track state transitions and measure latencies.
        """
        status = msg.data
        current_time = time.time()

        # Parse state from status message (format: "[STATE] message")
        if '[' in status and ']' in status:
            state = status.split('[')[1].split(']')[0]

            # Record state transition times
            if state == 'PLANNING':
                self.llm_start = current_time
                if self.voice_start:
                    self.voice_latency = current_time - self.voice_start

            elif state == 'NAVIGATING':
                self.nav_start = current_time
                if self.llm_start:
                    self.llm_latency = current_time - self.llm_start

            elif state == 'PERCEIVING':
                self.perception_start = current_time
                if self.nav_start:
                    self.nav_latency = current_time - self.nav_start

            elif state == 'MANIPULATING':
                self.manipulation_start = current_time
                if self.perception_start:
                    self.perception_latency = current_time - self.perception_start

            elif state == 'COMPLETED':
                if self.manipulation_start:
                    self.manipulation_latency = current_time - self.manipulation_start

                self.trial_success = True
                self.record_trial_result()

            elif state == 'FAILED':
                self.trial_success = False
                self.failure_mode = status.split(']')[1].strip() if ']' in status else "Unknown"
                self.record_trial_result()

    def run_trial(self, trial_id: int, command: str):
        """
        Execute a single benchmark trial.

        Args:
            trial_id: Trial number
            command: Voice command to test
        """
        self.get_logger().info(f'=== Trial {trial_id + 1}/{self.num_trials} ===')
        self.get_logger().info(f'Command: "{command}"')

        # Reset timing state
        self.trial_start_time = time.time()
        self.voice_start = time.time()
        self.current_command = command
        self.trial_success = False
        self.failure_mode = ""

        self.voice_latency = 0.0
        self.llm_latency = 0.0
        self.nav_latency = 0.0
        self.perception_latency = 0.0
        self.manipulation_latency = 0.0

        self.current_trial = trial_id

        # Publish command
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        # Wait for trial to complete (timeout after 90 seconds)
        timeout = 90.0
        start_wait = time.time()

        while not self.trial_complete() and (time.time() - start_wait) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Handle timeout
        if not self.trial_complete():
            self.get_logger().error(f'Trial {trial_id + 1} timed out after {timeout}s')
            self.trial_success = False
            self.failure_mode = "TIMEOUT"
            self.record_trial_result()

    def trial_complete(self) -> bool:
        """Check if current trial has completed (success or failure)."""
        return len(self.results) > self.current_trial

    def record_trial_result(self):
        """Record the result of the current trial."""
        total_latency = time.time() - self.trial_start_time

        result = TrialResult(
            trial_id=self.current_trial,
            command=self.current_command,
            success=self.trial_success,
            total_latency=total_latency,
            voice_latency=self.voice_latency,
            llm_latency=self.llm_latency,
            navigation_latency=self.nav_latency,
            perception_latency=self.perception_latency,
            manipulation_latency=self.manipulation_latency,
            failure_mode=self.failure_mode,
            timestamp=time.time()
        )

        self.results.append(result)

        status_emoji = "✅" if self.trial_success else "❌"
        self.get_logger().info(
            f'{status_emoji} Trial {self.current_trial + 1} complete: '
            f'{"SUCCESS" if self.trial_success else "FAILED"} '
            f'({total_latency:.1f}s)'
        )

    def run_benchmark(self):
        """Run the full benchmark suite."""
        self.get_logger().info(f'Starting benchmark with {self.num_trials} trials')

        for i in range(self.num_trials):
            # Select command (cycle through test commands)
            command = self.test_commands[i % len(self.test_commands)]

            # Run trial
            self.run_trial(i, command)

            # Brief pause between trials
            time.sleep(2.0)

        self.get_logger().info('Benchmark complete!')
        self.print_summary()

    def print_summary(self):
        """Print benchmark summary statistics."""
        if not self.results:
            self.get_logger().error('No results to analyze')
            return

        successes = [r for r in self.results if r.success]
        failures = [r for r in self.results if not r.success]

        success_rate = len(successes) / len(self.results) * 100

        self.get_logger().info('=' * 60)
        self.get_logger().info('BENCHMARK SUMMARY')
        self.get_logger().info('=' * 60)

        self.get_logger().info(f'Total Trials: {len(self.results)}')
        self.get_logger().info(f'Successes: {len(successes)} ({success_rate:.1f}%)')
        self.get_logger().info(f'Failures: {len(failures)}')

        # Success rate evaluation
        target_success_rate = 60.0
        if success_rate >= target_success_rate:
            self.get_logger().info(f'✅ SUCCESS RATE MEETS TARGET (≥{target_success_rate}%)')
        else:
            self.get_logger().warn(f'❌ SUCCESS RATE BELOW TARGET (<{target_success_rate}%)')

        # Latency statistics (only successful trials)
        if successes:
            total_latencies = [r.total_latency for r in successes]
            mean_latency = statistics.mean(total_latencies)
            median_latency = statistics.median(total_latencies)
            p95_latency = sorted(total_latencies)[int(len(total_latencies) * 0.95)]

            self.get_logger().info('')
            self.get_logger().info('LATENCY STATISTICS (successful trials):')
            self.get_logger().info(f'  Mean: {mean_latency:.1f}s')
            self.get_logger().info(f'  Median: {median_latency:.1f}s')
            self.get_logger().info(f'  p95: {p95_latency:.1f}s')

            # Latency evaluation
            target_latency = 30.0
            if mean_latency <= target_latency:
                self.get_logger().info(f'✅ LATENCY MEETS TARGET (≤{target_latency}s)')
            else:
                self.get_logger().warn(f'❌ LATENCY ABOVE TARGET (>{target_latency}s)')

            # Component breakdown
            mean_voice = statistics.mean([r.voice_latency for r in successes])
            mean_llm = statistics.mean([r.llm_latency for r in successes])
            mean_nav = statistics.mean([r.navigation_latency for r in successes])
            mean_perception = statistics.mean([r.perception_latency for r in successes])
            mean_manip = statistics.mean([r.manipulation_latency for r in successes])

            self.get_logger().info('')
            self.get_logger().info('COMPONENT LATENCIES (mean):')
            self.get_logger().info(f'  Voice: {mean_voice:.1f}s')
            self.get_logger().info(f'  LLM: {mean_llm:.1f}s')
            self.get_logger().info(f'  Navigation: {mean_nav:.1f}s')
            self.get_logger().info(f'  Perception: {mean_perception:.1f}s')
            self.get_logger().info(f'  Manipulation: {mean_manip:.1f}s')

        # Failure analysis
        if failures:
            self.get_logger().info('')
            self.get_logger().info('FAILURE MODES:')
            failure_modes = {}
            for f in failures:
                mode = f.failure_mode if f.failure_mode else "UNKNOWN"
                failure_modes[mode] = failure_modes.get(mode, 0) + 1

            for mode, count in sorted(failure_modes.items(), key=lambda x: -x[1]):
                self.get_logger().info(f'  {mode}: {count} ({count / len(failures) * 100:.1f}%)')

        self.get_logger().info('=' * 60)

    def save_results(self, output_file: str):
        """Save benchmark results to JSON file."""
        data = {
            'num_trials': len(self.results),
            'success_rate': len([r for r in self.results if r.success]) / len(self.results) * 100 if self.results else 0,
            'timestamp': time.time(),
            'results': [asdict(r) for r in self.results]
        }

        with open(output_file, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f'Results saved to {output_file}')


def main():
    parser = argparse.ArgumentParser(
        description='Capstone Performance Benchmarking Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        '--trials',
        type=int,
        default=20,
        help='Number of benchmark trials (default: 20)'
    )

    parser.add_argument(
        '--quick',
        action='store_true',
        help='Quick test with 5 trials'
    )

    parser.add_argument(
        '--output',
        type=str,
        default='benchmark_results.json',
        help='Output JSON file (default: benchmark_results.json)'
    )

    args = parser.parse_args()

    num_trials = 5 if args.quick else args.trials

    rclpy.init()

    benchmark = CapstoneBenchmark(num_trials=num_trials)

    try:
        benchmark.run_benchmark()
        benchmark.save_results(args.output)
    except KeyboardInterrupt:
        benchmark.get_logger().info('Benchmark interrupted by user')
    finally:
        benchmark.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
