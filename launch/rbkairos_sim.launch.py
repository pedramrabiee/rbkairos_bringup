#!/usr/bin/env python3
"""
Simulation launcher for RB-Kairos+.

This launch file ONLY handles simulation-specific setup:
    - Gazebo world launch
    - Robot spawn
    - Gripper controller spawner

The autonomy stack (MoveIt, Nav2, MTC, Autonomy node) should be launched
separately via:
    ros2 launch lll_mm_autonomy autonomy.launch.py

Usage:
    ros2 launch rbkairos_bringup rbkairos_sim.launch.py

Author: Pedram Rabiee (prabiee@3laws.io)
Copyright: 2025 3Laws Robotics Inc.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup simulation components."""
    launch_components = []

    # Get package paths
    bringup_pkg = get_package_share_directory("rbkairos_bringup")
    gazebo_pkg = get_package_share_directory("robotnik_gazebo_ignition")

    # Load robot configuration
    config_file = LaunchConfiguration("config_file").perform(context)
    config_path = os.path.join(bringup_pkg, "config", config_file)

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # Extract configuration values
    robot_config = config["robot"]
    world_config = config["world"]

    robot_name = robot_config["name"]
    namespace = robot_config["namespace"]
    use_gripper = str(robot_config["gripper"]["enabled"]).lower()

    # Get launch arguments
    world_override = LaunchConfiguration("world").perform(context)
    world_name = world_override if world_override else world_config["name"]
    gui = LaunchConfiguration("gui")

    # 1. Launch Gazebo world
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([gazebo_pkg, "launch", "spawn_world.launch.py"])]
        ),
        launch_arguments={
            "world": world_name,
            "gui": gui,
        }.items(),
    )
    launch_components.append(gazebo_world)

    # 2. Spawn robot in Gazebo
    spawn_position = robot_config["spawn"]["position"]
    custom_urdf_path = PathJoinSubstitution(
        [bringup_pkg, "urdf", "rbkairos_plus_with_robotiq.urdf.xacro"]
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([gazebo_pkg, "launch", "spawn_robot.launch.py"])]
                ),
                launch_arguments={
                    "robot_id": robot_name,
                    "robot": "rbkairos",
                    "robot_model": robot_config["model"]["type"],
                    "robot_xacro": custom_urdf_path,
                    "x": str(spawn_position["x"]),
                    "y": str(spawn_position["y"]),
                    "z": str(spawn_position["z"]),
                    "has_arm": "true",
                    "run_rviz": "false",
                }.items(),
            )
        ],
    )
    launch_components.append(spawn_robot)

    # 3. Spawn gripper controller (simulation-specific)
    # Note: forward_velocity_controller and joint_state_broadcaster are spawned
    # automatically by spawn_robot.launch.py for all controllers in rbkairos_controller_params.yaml
    # Gripper params are defined in rbkairos_controller_params.yaml (robotnik_description)
    if use_gripper == "true":
        gripper_controller_spawner = TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    namespace=namespace,
                    arguments=[
                        "gripper_controller",
                        "--controller-manager",
                        f"/{namespace}/controller_manager",
                    ],
                    output="screen",
                )
            ],
        )
        launch_components.append(gripper_controller_spawner)

    return launch_components


def generate_launch_description():
    """Generate the launch description."""
    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value="robot_config.yaml",
            description="Robot configuration file name in config/ directory",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch Gazebo GUI",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="",
            description="Override world name from config",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
