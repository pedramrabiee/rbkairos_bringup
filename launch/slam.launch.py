#!/usr/bin/env python3
"""
SLAM Toolbox launch file for RB-Kairos+ mobile manipulator.

Standalone launch for SLAM mapping. Use this to create maps of new environments.

Usage:
    # First launch Gazebo
    ros2 launch rbkairos_bringup rbkairos_complete.launch.py world:=demo

    # Then launch SLAM
    ros2 launch rbkairos_bringup slam.launch.py

    # Drive around with teleop
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/rbkairos/cmd_vel

    # Save map when done
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -r __ns:=/rbkairos

Author: Pedram Rabiee (prabiee@3laws.io)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Get package directory
    bringup_pkg = get_package_share_directory('rbkairos_bringup')

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='rbkairos',
        description='Robot namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml'),
        description='SLAM Toolbox parameters file'
    )

    # Remappings
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # SLAM Toolbox node
    slam_node = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ],
                remappings=remappings
            ),
        ]
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        slam_node,
    ])
