#!/usr/bin/env python3

"""
Unified launch file for RB-Kairos+ mobile manipulator
Reads configuration from YAML files in config/ directory

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
    # Allow world override via launch argument
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

    # 3. Robot state publisher is created by spawn_robot, so we don't need it here

    # 4. Spawn robot in Gazebo with custom URDF
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
                    'robot_xacro': custom_urdf_path,  # This gets passed as robot_xacro_file to robot_description
                    'x': str(spawn_position['x']),
                    'y': str(spawn_position['y']),
                    'z': str(spawn_position['z']),
                    'has_arm': 'true',
                    'run_rviz': 'false',  # We launch RViz separately with MoveIt config
                }.items()
            )
        ]
    )
    launch_components.append(spawn_robot)

    # 4b. Launch MoveIt for Gazebo (move_group + RViz) if enabled
    use_moveit = modules_config['manipulation']['enabled']
    launch_nav2_arg = LaunchConfiguration('launch_nav2').perform(context).lower() == 'true'

    if use_moveit:
        # Determine prefixes for MoveIt (matches Gazebo joint names)
        # In Gazebo, joints/links are prefixed:
        #   - Arm: rbkairos_arm_shoulder_pan_joint
        #   - Gripper: rbkairos_gripper_robotiq_85_left_knuckle_joint
        #   - Base: rbkairos_base_link, rbkairos_top_cover, etc.
        arm_prefix = f'{namespace}_arm_' if namespace else ''
        gripper_prefix = f'{namespace}_gripper_' if namespace else 'gripper_'
        base_prefix = f'{namespace}_' if namespace else ''

        # Determine which RViz config to use:
        # - If Nav2 is enabled, use combined rbkairos_full.rviz (Nav2 + MoveIt)
        # - Otherwise, use moveit_gazebo.rviz (MoveIt only)
        if launch_nav2_arg or modules_config['navigation']['enabled']:
            rviz_config = os.path.join(bringup_pkg, 'config', 'rviz', 'rbkairos_full.rviz')
        else:
            rviz_config = os.path.join(bringup_pkg, 'config', 'rviz', 'moveit_gazebo.rviz')

        # Launch MoveIt using gazebo_moveit.launch.py which is designed for Gazebo integration
        # - Uses Gazebo's robot_description topic
        # - Uses Gazebo's controller names (joint_trajectory_controller)
        # - Runs in the correct namespace
        moveit_launch = TimerAction(
            period=8.0,  # Wait for robot to spawn and controllers to load
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([bringup_pkg, 'launch', 'gazebo_moveit.launch.py'])
                    ]),
                    launch_arguments={
                        'namespace': namespace,
                        'arm_prefix': arm_prefix,
                        'gripper_prefix': gripper_prefix,
                        'base_prefix': base_prefix,
                        'use_rviz': 'true',
                        'rviz_config': rviz_config,
                        # Disable static map->odom TF when Nav2 is enabled (SLAM/AMCL publishes it)
                        'publish_map_odom_tf': 'false' if (launch_nav2_arg or modules_config['navigation']['enabled']) else 'true',
                    }.items()
                )
            ]
        )
        launch_components.append(moveit_launch)

    # 5. Controllers are handled by spawn_robot.launch.py
    # It loads controllers from robotnik_gazebo_ignition/config/profile/rbkairos/ros2_control.yaml

    # 5b. Spawn gripper controller if gripper is enabled
    if use_gripper == 'true':
        gripper_controller_config = os.path.join(bringup_pkg, 'config', 'gripper_controllers.yaml')
        gripper_controller_spawner = TimerAction(
            period=8.0,  # Wait for robot to spawn and other controllers to load
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

    # 6. Navigation stack (if enabled via config or launch argument)
    # Note: launch_nav2_arg already defined above in section 4b
    use_slam_arg = LaunchConfiguration('use_slam').perform(context).lower() == 'true'
    map_file_arg = LaunchConfiguration('map').perform(context)

    if modules_config['navigation']['enabled'] or launch_nav2_arg:
        nav2_launch = TimerAction(
            period=10.0,  # Wait for robot, controllers, and TF to be ready
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([bringup_pkg, 'launch', 'nav2.launch.py'])
                    ]),
                    launch_arguments={
                        'namespace': namespace,
                        'use_sim_time': 'true',
                        'use_slam': str(use_slam_arg).lower(),
                        'map': map_file_arg,
                        'params_file': os.path.join(bringup_pkg, 'config', 'nav2_params.yaml'),
                    }.items()
                )
            ]
        )
        launch_components.append(nav2_launch)

    # 7. MoveIt2 is launched above (4c) if manipulation is enabled

    # 8. Task executor node (optional - reads from tasks.yaml)
    if LaunchConfiguration('launch_task_executor').perform(context) == 'true':
        task_executor = Node(
            package='rbkairos_bringup',  # Will need to create this node
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

    # Declare launch arguments
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
        # Nav2 arguments
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
    ]

    # RViz is handled by spawn_robot.launch.py when run_rviz is true

    # Create launch description
    ld = LaunchDescription()

    # Add arguments
    for arg in declare_args:
        ld.add_action(arg)

    # Add the OpaqueFunction to set up launch based on config
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld