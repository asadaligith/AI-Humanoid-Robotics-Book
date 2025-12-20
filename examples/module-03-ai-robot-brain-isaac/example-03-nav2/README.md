# Example 03: Autonomous Navigation with Nav2

## Learning Objectives

By completing this example, you will:
- Set up Nav2 navigation stack
- Configure robot footprint and kinematic constraints
- Create and load costmaps for navigation
- Send navigation goals programmatically
- Handle dynamic obstacles and path planning

## Overview

Nav2 (Navigation 2) is the ROS 2 navigation framework providing path planning, obstacle avoidance, and goal execution for mobile robots.

**Key Components:**
- **Global Planner**: Finds path from start to goal (NavFn, Smac)
- **Local Planner**: Executes path with obstacle avoidance (DWB, TEB)
- **Costmap**: Represents obstacles and free space
- **Recovery Behaviors**: Handles stuck situations
- **Behavior Tree**: Coordinates navigation behaviors

## Prerequisites

- ROS 2 Humble with Nav2 installed
- Isaac Sim or Gazebo simulation
- Visual SLAM or Lidar SLAM running
- Completed Examples 01-02
- Completed Module 03 Chapter 06

## Installation

```bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y

# Install dependencies
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-tf2-tools

# Install visualization
sudo apt install ros-humble-nav2-rviz-plugins -y
```

## System Architecture

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   SLAM      │────→│  Nav2 Stack  │────→│   Robot     │
│ (map→odom)  │     │  (Planning)  │     │  Controller │
└─────────────┘     └──────────────┘     └─────────────┘
                           │
                           ↓
                    ┌──────────────┐
                    │   Sensors    │
                    │ (Lidar, Cam) │
                    └──────────────┘
```

## Configuration Files

### nav2_params.yaml

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_updated_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25

    # DWB parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

## Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(get_package_share_directory('my_robot'), 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': params_file,
            }.items()
        ),

        # RViz with Nav2 plugins
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')]
        ),
    ])
```

## Sending Navigation Goals

### Command Line

```bash
# Send 2D Nav Goal
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### Python Script

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.navigator = BasicNavigator()

        # Wait for Nav2 to activate
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active!')

    def go_to_pose(self, x, y, theta):
        """Navigate to a specific pose."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # Convert theta to quaternion
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)

        self.navigator.goToPose(goal_pose)

        # Monitor progress
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'Distance remaining: {feedback.distance_remaining:.2f}m'
                )
            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed!')

def main(args=None):
    rclpy.init(args=args)
    nav_controller = NavigationController()

    # Navigate to waypoints
    waypoints = [
        (2.0, 1.0, 0.0),
        (4.0, 3.0, 1.57),
        (1.0, 2.0, 3.14),
    ]

    for x, y, theta in waypoints:
        nav_controller.get_logger().info(f'Navigating to ({x}, {y})')
        nav_controller.go_to_pose(x, y, theta)

    nav_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running Navigation

```bash
# Terminal 1: Start simulation (Isaac Sim or Gazebo)
~/.local/share/ov/pkg/isaac_sim-*/python.sh robot_scene.py

# Terminal 2: Start SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Terminal 3: Start Nav2
ros2 launch my_robot nav2.launch.py

# Terminal 4: Send goal
python3 navigate_waypoints.py
```

## Exercises

### Exercise 1: Waypoint Following
Create a list of waypoints and navigate through them sequentially.

### Exercise 2: Dynamic Obstacles
Add moving obstacles and observe Nav2's replanning behavior.

### Exercise 3: Behavior Trees
Customize the behavior tree to add retry logic or alternative behaviors.

### Exercise 4: Multi-Robot Navigation
Set up Nav2 for two robots with namespace separation.

### Exercise 5: Performance Tuning
Optimize parameters for faster/smoother navigation in your environment.

## Common Issues

### Issue 1: Robot oscillates near goal
**Solution**: Increase `xy_goal_tolerance` and tune DWB critics.

### Issue 2: Path goes through obstacles
**Solution**: Increase `inflation_radius` in costmap configuration.

### Issue 3: Recovery behaviors stuck in loop
**Solution**: Adjust `failure_tolerance` and recovery behavior parameters.

### Issue 4: Transform timeout errors
**Solution**: Verify all TF frames are being published at sufficient rate.

## Key Takeaways

1. **Costmaps are Critical**: Proper obstacle representation is essential
2. **Behavior Trees**: Flexible architecture for navigation behaviors
3. **Local vs Global**: Two-layer planning for efficiency
4. **Recovery Behaviors**: Handle stuck situations gracefully
5. **Tuning Required**: Every robot/environment needs parameter adjustment

## Next Steps

- **Module 04**: Vision-Language-Action integration
- **Module 05**: Full autonomous humanoid capstone
- **Advanced Nav2**: Custom planners and controllers

## References

- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)
- [Nav2 Plugins](https://navigation.ros.org/plugins/index.html)
- [Behavior Trees in Nav2](https://navigation.ros.org/behavior_trees/index.html)
