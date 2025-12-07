#!/usr/bin/env python3
"""
Isaac Sim Launch File for Capstone Demo

Launches NVIDIA Isaac Sim with USD scene, robot, and ROS 2 bridge.
Provides photorealistic simulation with GPU-accelerated physics and rendering.

Prerequisites:
- NVIDIA Isaac Sim 2023.1+ installed
- Omniverse Launcher configured
- USD assets in assets/usd/

Usage:
    ros2 launch capstone_demo isaac_sim_demo.launch.py

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for Isaac Sim demo"""

    # Launch arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Isaac Sim in headless mode (no GUI)'
    )

    isaac_sim_path_arg = DeclareLaunchArgument(
        'isaac_sim_path',
        default_value=os.path.expanduser('~/.local/share/ov/pkg/isaac_sim-2023.1.1'),
        description='Path to Isaac Sim installation directory'
    )

    usd_scene_arg = DeclareLaunchArgument(
        'usd_scene',
        default_value=PathJoinSubstitution([
            FindPackageShare('capstone_demo'),
            'assets', 'usd', 'kitchen_env.usd'
        ]),
        description='USD scene file to load'
    )

    robot_usd_arg = DeclareLaunchArgument(
        'robot_usd',
        default_value=PathJoinSubstitution([
            FindPackageShare('capstone_demo'),
            'assets', 'usd', 'unitree_g1.usd'
        ]),
        description='USD robot model file'
    )

    # Get launch configurations
    headless = LaunchConfiguration('headless')
    isaac_sim_path = LaunchConfiguration('isaac_sim_path')
    usd_scene = LaunchConfiguration('usd_scene')
    robot_usd = LaunchConfiguration('robot_usd')

    # Isaac Sim launcher script
    # Note: This requires Isaac Sim's Python environment
    isaac_sim_cmd = [
        PathJoinSubstitution([isaac_sim_path, 'python.sh']),
        PathJoinSubstitution([
            FindPackageShare('capstone_demo'),
            'scripts', 'isaac_sim_runner.py'
        ]),
        '--scene', usd_scene,
        '--robot', robot_usd,
        '--headless', headless
    ]

    isaac_sim_process = ExecuteProcess(
        cmd=isaac_sim_cmd,
        name='isaac_sim',
        output='screen',
        emulate_tty=True
    )

    # ROS 2 Bridge (connects Isaac Sim to ROS 2 topics)
    # Isaac Sim provides ros2_bridge that translates USD messages to ROS 2
    ros2_bridge_node = Node(
        package='isaac_ros_bridge',
        executable='bridge_node',
        name='isaac_ros_bridge',
        output='screen',
        parameters=[{
            'publish_tf': True,
            'publish_joint_states': True,
            'publish_camera': True,
            'camera_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'pointcloud_topic': '/camera/pointcloud'
        }],
        # Only launch if Isaac ROS bridge is installed
        condition=lambda context: os.path.exists('/opt/nvidia/isaac_ros')
    )

    # Voice Input Node
    voice_node = Node(
        package='capstone_demo',
        executable='voice_input_node',
        name='voice_input_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('capstone_demo'),
                'config', 'voice_config.yaml'
            ])
        ]
    )

    # LLM Planner Node
    llm_node = Node(
        package='capstone_demo',
        executable='llm_planner_node',
        name='llm_planner_node',
        output='screen',
        parameters=[{
            'capability_manifest': PathJoinSubstitution([
                FindPackageShare('capstone_demo'),
                'config', 'capability_manifest.json'
            ])
        }]
    )

    # Navigation Controller
    nav_controller_node = Node(
        package='capstone_demo',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen'
    )

    # Object Detection Node (uses Isaac ROS DOPE for photorealistic detection)
    detection_node = Node(
        package='capstone_demo',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('capstone_demo'),
                'config', 'detection_classes.yaml'
            ]),
            {
                'model_path': 'yolov8n.pt',  # Or use Isaac ROS DOPE
                'confidence_threshold': 0.6,
                'device': 'cuda'  # GPU acceleration in Isaac Sim
            }
        ]
    )

    # Manipulation Controller
    manip_controller_node = Node(
        package='capstone_demo',
        executable='manipulation_controller',
        name='manipulation_controller',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('capstone_demo'),
                'config', 'grasp_poses.yaml'
            ])
        ]
    )

    # Integration Demo (State Machine)
    integration_node = Node(
        package='capstone_demo',
        executable='integration_demo',
        name='integration_demo',
        output='screen',
        parameters=[{
            'mock_voice': False,
            'mock_llm': False,
            'mock_navigation': False,
            'mock_perception': False,
            'mock_manipulation': False,
            'test_mode': False
        }]
    )

    # RViz2 for visualization (optional)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('capstone_demo'),
        'config', 'isaac_sim.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=lambda context: LaunchConfiguration('headless').perform(context) == 'false'
    )

    return LaunchDescription([
        # Launch arguments
        headless_arg,
        isaac_sim_path_arg,
        usd_scene_arg,
        robot_usd_arg,

        # Processes and nodes
        isaac_sim_process,
        ros2_bridge_node,
        voice_node,
        llm_node,
        nav_controller_node,
        detection_node,
        manip_controller_node,
        integration_node,
        rviz_node
    ])
