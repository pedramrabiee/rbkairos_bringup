#!/usr/bin/env python3

"""
Test launch file for RB-Kairos+ using lll_moveit package.

This is a copy of rbkairos_complete.launch.py modified to use lll_moveit
for MoveIt integration instead of the local gazebo_moveit.launch.py.

Purpose: Incremental testing of lll_moveit migration.

Author: Pedram Rabiee (prabiee@3laws.io)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """
    Function to set up the launch based on configuration file.
    This allows us to read YAML and dynamically configure the launch.
    """
    # Get package paths
    bringup_pkg = get_package_share_directory('rbkairos_bringup')
    gazebo_pkg = get_package_share_directory('robotnik_gazebo_ignition')
    moveit_config_pkg = get_package_share_directory('rbkairos_moveit_config')

    # Load main configuration
    config_file = LaunchConfiguration('config_file').perform(context)
    config_path = os.path.join(bringup_pkg, 'config', config_file)

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Load controller configuration
    controller_config_path = os.path.join(bringup_pkg, 'config', 'controllers.yaml')

    # Load task configuration
    task_config_path = os.path.join(bringup_pkg, 'config', 'tasks.yaml')

    # Extract configuration values
    robot_config = config['robot']
    world_config = config['world']
    sim_config = config['simulation']
    modules_config = config['modules']

    robot_name = robot_config['name']
    namespace = robot_config['namespace']
    use_gripper = str(robot_config['gripper']['enabled']).lower()
    gripper_type = robot_config['gripper']['type']

    # Build launch components
    launch_components = []

    # 1. Launch Gazebo world
    world_override = LaunchConfiguration('world').perform(context)
    world_name = world_override if world_override else world_config['name']
    gui = LaunchConfiguration('gui')

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_pkg, 'launch', 'spawn_world.launch.py'])
        ]),
        launch_arguments={
            'world': world_name,
            'gui': gui,
        }.items()
    )
    launch_components.append(gazebo_world)

    # 2. Get URDF via xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([bringup_pkg, 'urdf', 'rbkairos_plus_with_robotiq.urdf.xacro']),
        f' namespace:={namespace}',
        f' prefix:={namespace}_' if namespace else '',
        f' use_gripper:={use_gripper}',
        f' gripper_type:={gripper_type}',
        ' gazebo_ignition:=true',
        f' ur_type:={robot_config["model"]["ur_type"]}',
    ])

    robot_description = {'robot_description': robot_description_content}

    # 3. Spawn robot in Gazebo with custom URDF
    spawn_position = robot_config['spawn']['position']
    custom_urdf_path = PathJoinSubstitution([bringup_pkg, 'urdf', 'rbkairos_plus_with_robotiq.urdf.xacro'])

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([gazebo_pkg, 'launch', 'spawn_robot.launch.py'])
                ]),
                launch_arguments={
                    'robot_id': robot_name,
                    'robot': 'rbkairos',
                    'robot_model': robot_config['model']['type'],
                    'robot_xacro': custom_urdf_path,
                    'x': str(spawn_position['x']),
                    'y': str(spawn_position['y']),
                    'z': str(spawn_position['z']),
                    'has_arm': 'true',
                    'run_rviz': 'false',
                }.items()
            )
        ]
    )
    launch_components.append(spawn_robot)

    # 4. Launch MoveIt using lll_moveit (robot-agnostic template)
    use_moveit = modules_config['manipulation']['enabled']
    launch_nav2_arg = LaunchConfiguration('launch_nav2').perform(context).lower() == 'true'

    if use_moveit:
        # Determine prefixes for MoveIt (matches Gazebo joint names)
        arm_prefix = f'{namespace}_arm_' if namespace else ''
        gripper_prefix = f'{namespace}_gripper_' if namespace else 'gripper_'
        base_prefix = f'{namespace}_' if namespace else ''

        # Paths to robot-specific configs (with prefixed joint names for Gazebo)
        gazebo_controllers_yaml = os.path.join(bringup_pkg, 'config', 'moveit_gazebo_controllers.yaml')
        joint_limits_yaml = os.path.join(bringup_pkg, 'config', 'joint_limits_gazebo.yaml')

        # Determine which RViz config to use
        if launch_nav2_arg or modules_config['navigation']['enabled']:
            rviz_config = os.path.join(bringup_pkg, 'config', 'rviz', 'rbkairos_full.rviz')
        else:
            rviz_config = os.path.join(bringup_pkg, 'config', 'rviz', 'moveit_gazebo.rviz')

        # Launch MoveIt using lll_moveit's generic move_group_gazebo.launch.py
        # This is the key difference from rbkairos_complete.launch.py
        moveit_launch = TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('lll_moveit'),
                            'launch',
                            'move_group_gazebo.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        # Required parameters for lll_moveit
                        'moveit_config_package': 'rbkairos_moveit_config',
                        'robot_name': 'ur5e_robotiq',
                        'srdf_file': 'config/ur5e_robotiq.srdf.xacro',
                        'controllers_file': gazebo_controllers_yaml,
                        'joint_limits_file': joint_limits_yaml,
                        # Robot-specific parameters
                        'namespace': namespace,
                        'arm_prefix': arm_prefix,
                        'gripper_prefix': gripper_prefix,
                        'base_prefix': base_prefix,
                        'use_rviz': 'true',
                        'rviz_config': rviz_config,
                        'publish_map_odom_tf': 'false' if (launch_nav2_arg or modules_config['navigation']['enabled']) else 'true',
                    }.items()
                )
            ]
        )
        launch_components.append(moveit_launch)

    # 5. Spawn gripper controller if gripper is enabled
    if use_gripper == 'true':
        gripper_controller_config = os.path.join(bringup_pkg, 'config', 'gripper_controllers.yaml')
        gripper_controller_spawner = TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    namespace=namespace,
                    arguments=[
                        'gripper_controller',
                        '--param-file', gripper_controller_config,
                        '--controller-manager', f'/{namespace}/controller_manager'
                    ],
                    output='screen',
                )
            ]
        )
        launch_components.append(gripper_controller_spawner)

    # 6. Navigation stack (if enabled) - using lll_nav2 (robot-agnostic template)
    use_slam_arg = LaunchConfiguration('use_slam').perform(context).lower() == 'true'
    map_file_arg = LaunchConfiguration('map').perform(context)

    if modules_config['navigation']['enabled'] or launch_nav2_arg:
        # Paths to robot-specific Nav2 configs
        nav2_params_yaml = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
        slam_params_yaml = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')

        nav2_launch = TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('lll_nav2'),
                            'launch',
                            'navigation_gazebo.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'params_file': nav2_params_yaml,
                        'namespace': namespace,
                        'use_slam': str(use_slam_arg).lower(),
                        'map': map_file_arg,
                        'slam_params_file': slam_params_yaml,
                    }.items()
                )
            ]
        )
        launch_components.append(nav2_launch)

    # 7. MTC Server (if MoveIt is enabled) - using lll_mtc
    launch_mtc_arg = LaunchConfiguration('launch_mtc').perform(context).lower() == 'true'
    if use_moveit and launch_mtc_arg:
        mtc_server_launch = TimerAction(
            period=12.0,  # After MoveIt is ready
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('lll_mtc'),
                            'launch',
                            'mtc_server.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        # MoveIt configuration (same as move_group)
                        'moveit_config_package': 'rbkairos_moveit_config',
                        'robot_name': 'ur5e_robotiq',
                        'srdf_file': 'config/ur5e_robotiq.srdf.xacro',
                        'arm_prefix': arm_prefix,
                        'gripper_prefix': gripper_prefix,
                        'base_prefix': base_prefix,
                        'joint_limits_file': joint_limits_yaml,  # Same as move_group
                        # Runtime parameters
                        'namespace': namespace,
                        'use_sim_time': 'true',
                        'arm_group': 'ur5e_arm',
                        'gripper_group': 'gripper',
                        'end_effector_link': f'{arm_prefix}tool0',
                        'world_frame': 'map',
                    }.items()
                )
            ]
        )
        launch_components.append(mtc_server_launch)

    # 8. Task executor node (optional)
    if LaunchConfiguration('launch_task_executor').perform(context) == 'true':
        task_executor = Node(
            package='rbkairos_bringup',
            executable='task_executor',
            namespace=namespace,
            parameters=[task_config_path],
            output='screen'
        )
        launch_components.append(task_executor)

    return launch_components


def generate_launch_description():
    """Generate the launch description."""

    bringup_pkg = get_package_share_directory('rbkairos_bringup')

    declare_args = [
        DeclareLaunchArgument(
            'config_file',
            default_value='robot_config.yaml',
            description='Main configuration file name in config/ directory'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'launch_task_executor',
            default_value='false',
            description='Launch task executor node'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'launch_nav2',
            default_value='false',
            description='Launch Nav2 navigation stack'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Use SLAM instead of AMCL (for mapping)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path to map yaml file (required if launch_nav2=true and use_slam=false)'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Override world name from config (e.g., demo for obstacles)'
        ),
        DeclareLaunchArgument(
            'launch_mtc',
            default_value='false',
            description='Launch MTC server for Pick/Place services'
        ),
    ]

    ld = LaunchDescription()

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
