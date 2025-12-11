#!/usr/bin/env python3
"""
Nav2 launch file for RB-Kairos+ mobile manipulator.

Launches the Nav2 navigation stack with support for:
- AMCL localization (with existing map)
- SLAM Toolbox (for mapping)

Usage:
    # Navigation with existing map
    ros2 launch rbkairos_bringup nav2.launch.py map:=/path/to/map.yaml

    # SLAM mapping mode
    ros2 launch rbkairos_bringup nav2.launch.py use_slam:=true

Author: Pedram Rabiee (prabiee@3laws.io)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    bringup_pkg = get_package_share_directory('rbkairos_bringup')

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')

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

    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Use SLAM instead of AMCL for localization'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file (required if use_slam is false)'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_pkg, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 stack'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Respawn nodes if they crash'
    )

    # Configure parameters with namespace substitution
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    # IMPORTANT: TF must be global (/tf and /tf_static), NOT namespaced!
    # All Nav2 nodes must read from and write to global /tf.
    # We do NOT remap /tf or /tf_static.
    # Only remap topics that should be namespaced (scan, cmd_vel, etc.)

    # Nav2 nodes
    nav2_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),

            # Map server (only when not using SLAM)
            Node(
                condition=UnlessCondition(use_slam),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
            ),

            # AMCL (only when not using SLAM)
            Node(
                condition=UnlessCondition(use_slam),
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
            ),

            # Controller server
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('local_costmap/front_laser/scan', 'front_laser/scan'),
                    ('local_costmap/rear_laser/scan', 'rear_laser/scan'),
                ]
            ),

            # Planner server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[
                    ('global_costmap/map', 'map'),
                    ('global_costmap/front_laser/scan', 'front_laser/scan'),
                    ('global_costmap/rear_laser/scan', 'rear_laser/scan'),
                ]
            ),

            # Behavior server
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
            ),

            # BT navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
            ),

            # Waypoint follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
            ),

            # Velocity smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')
                ]
            ),

            # Lifecycle manager for Nav2 nodes
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'bond_timeout': 0.0,
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ]
                }]
            ),

            # Lifecycle manager for localization (map_server + amcl)
            Node(
                condition=UnlessCondition(use_slam),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'bond_timeout': 0.0,
                    'node_names': ['map_server', 'amcl']
                }]
            ),
        ]
    )

    # SLAM Toolbox (when use_slam is true)
    slam_toolbox_node = GroupAction(
        condition=IfCondition(use_slam),
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
            ),
        ]
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_use_slam_cmd,
        declare_map_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        nav2_nodes,
        slam_toolbox_node,
    ])
