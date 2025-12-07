#!/usr/bin/env python3
"""
Gazebo Demo Launch File - Capstone Autonomous Humanoid

Orchestrates all ROS 2 nodes for the voice-commanded fetch-and-deliver system:
- Gazebo simulation
- Robot state publisher (TF tree)
- Voice input node
- LLM planner node
- Navigation controller (Nav2)
- Object detection node
- Manipulation controller (MoveIt 2)
- Integration state machine

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from pathlib import Path


def generate_launch_description():
    # Get paths
    pkg_share = FindPackageShare('capstone_demo').find('capstone_demo')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'kitchen_env.world'])
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'unitree_g1.urdf'])

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )

    gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )

    # Launch Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # Launch Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=LaunchConfiguration('gui')
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'unitree_g1',
            '-file', urdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Robot state publisher (publishes TF tree from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Joint state publisher (for simulation)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Voice input node
    voice_input = Node(
        package='capstone_demo',
        executable='voice_input_node',
        name='voice_input_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'voice_config.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # LLM planner node
    llm_planner = Node(
        package='capstone_demo',
        executable='llm_planner_node',
        name='llm_planner_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model': 'gpt-4',
            'temperature': 0.0
        }],
        output='screen'
    )

    # Navigation controller (simplified, full version would include Nav2)
    navigation_controller = Node(
        package='capstone_demo',
        executable='navigation_controller',
        name='navigation_controller',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Object detection node
    object_detection = Node(
        package='capstone_demo',
        executable='object_detection_node',
        name='object_detection_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model': 'yolov8n',  # or 'isaac_ros_detection'
            'confidence_threshold': 0.7
        }],
        output='screen'
    )

    # Manipulation controller (MoveIt 2)
    manipulation_controller = Node(
        package='capstone_demo',
        executable='manipulation_controller',
        name='manipulation_controller',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'max_grasp_attempts': 3
        }],
        output='screen'
    )

    # Integration state machine
    integration_demo = Node(
        package='capstone_demo',
        executable='integration_demo',
        name='integration_demo',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # RViz for visualization (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'capstone.rviz'])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        gui,

        # Gazebo
        gazebo_server,
        gazebo_client,

        # Robot
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,

        # Capability nodes
        voice_input,
        llm_planner,
        navigation_controller,
        object_detection,
        manipulation_controller,

        # Integration
        integration_demo,

        # Visualization
        rviz,
    ])
