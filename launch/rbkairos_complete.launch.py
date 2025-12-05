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
    world_name = world_config['name']
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

    # 4b. Launch RViz with MoveIt plugin (after robot spawns)
    use_moveit = modules_config['manipulation']['enabled']
    rviz_config_file = os.path.join(bringup_pkg, 'config', 'rviz', 'rbkairos.rviz')

    # Load SRDF for MoveIt MotionPlanning plugin in RViz
    srdf_path = os.path.join(moveit_config_pkg, 'srdf', 'rbkairos.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # Load kinematics config for MoveIt (must be loaded as YAML, not path)
    kinematics_yaml_path = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_yaml_path, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    rviz_node = TimerAction(
        period=5.0,  # Wait for robot to spawn
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace=namespace,  # Run in robot namespace so topics resolve correctly
                arguments=['-d', rviz_config_file],
                parameters=[
                    robot_description_semantic,
                    {'robot_description_kinematics': kinematics_config},
                    {'use_sim_time': True},
                ],
                output='screen',
            )
        ]
    )
    launch_components.append(rviz_node)

    # 4c. Launch MoveIt if enabled
    if use_moveit:
        moveit_launch = TimerAction(
            period=6.0,  # Wait for RViz to start
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([moveit_config_pkg, 'launch', 'moveit.launch.py'])
                    ]),
                    launch_arguments={
                        'use_rviz': 'false',  # RViz already launched above
                        'use_sim_time': 'true',
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

    # 6. Navigation stack (if enabled)
    if modules_config['navigation']['enabled']:
        nav2_config = os.path.join(bringup_pkg, 'config', modules_config['navigation']['config_file'])
        # TODO: Add Nav2 launch
        pass

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