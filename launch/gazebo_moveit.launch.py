#!/usr/bin/env python3
"""
MoveIt launch file for Gazebo integration with RB-Kairos+

Uses MoveItConfigsBuilder (same approach as demo.launch.py) but configured for Gazebo:
- Uses Gazebo's controller names (joint_trajectory_controller)
- Uses prefixed joint names (rbkairos_arm_* for arm, rbkairos_gripper_* for gripper)
- Gets robot_description from Gazebo's topic (not loading its own URDF)
- Runs move_group and optionally RViz

Usage:
    ros2 launch rbkairos_bringup gazebo_moveit.launch.py

Author: Pedram Rabiee (prabiee@3laws.io)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """Setup MoveIt nodes for Gazebo integration using MoveItConfigsBuilder."""

    # Get parameters at launch time
    namespace = LaunchConfiguration("namespace").perform(context)
    arm_prefix = LaunchConfiguration("arm_prefix").perform(context)
    gripper_prefix = LaunchConfiguration("gripper_prefix").perform(context)
    base_prefix = LaunchConfiguration("base_prefix").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() == 'true'
    publish_map_odom_tf = LaunchConfiguration("publish_map_odom_tf")

    # Get package paths
    bringup_pkg = get_package_share_directory('rbkairos_bringup')

    # Build MoveIt config using MoveItConfigsBuilder
    # Key differences for Gazebo:
    # 1. Do NOT load robot_description - we get it from Gazebo's topic
    # 2. Use gazebo_controllers.yaml which has prefixed joint names and Gazebo controller names
    # 3. Pass arm_prefix, gripper_prefix, and base_prefix to SRDF xacro
    # Path to Gazebo-specific controller config (in bringup package)
    gazebo_controllers_yaml = os.path.join(bringup_pkg, 'config', 'moveit_gazebo_controllers.yaml')

    moveit_config = (
        MoveItConfigsBuilder("ur5e_robotiq", package_name="rbkairos_moveit_config")
        # Note: We intentionally skip .robot_description() - Gazebo provides it
        .robot_description_semantic(
            file_path="config/ur5e_robotiq.srdf.xacro",
            mappings={
                "arm_prefix": arm_prefix,
                "gripper_prefix": gripper_prefix,
                "base_prefix": base_prefix,
            }
        )
        .trajectory_execution(file_path=gazebo_controllers_yaml)
        .to_moveit_configs()
    )

    launch_nodes = []

    # Publish static transform from map to odom (required for MoveIt planning frame)
    # The virtual joint in SRDF connects map -> rbkairos_base_footprint
    # But Gazebo's TF tree starts at rbkairos_odom, so we need map -> rbkairos_odom
    # NOTE: When Nav2 is running, SLAM/AMCL publishes this transform dynamically
    # This static publisher is only used when Nav2 is NOT running
    static_tf_map_to_odom = Node(
        condition=IfCondition(publish_map_odom_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", f"{base_prefix}odom"],
        parameters=[{"use_sim_time": True}],
    )
    launch_nodes.append(static_tf_map_to_odom)

    # Build parameters for move_group, excluding robot_description (comes from topic)
    # We need to manually construct the dict without robot_description
    move_group_params = [
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.trajectory_execution,
        moveit_config.planning_scene_monitor,
        moveit_config.joint_limits,
        {
            "use_sim_time": True,
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        namespace=namespace,
        output="screen",
        parameters=move_group_params,
        # Remap to get robot_description from Gazebo's robot_state_publisher
        remappings=[
            ("robot_description", f"/{namespace}/robot_description"),
        ],
    )
    launch_nodes.append(move_group_node)

    # RViz with MoveIt plugin
    if use_rviz:
        rviz_params = [
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ]

        # Use custom RViz config if provided, otherwise default to moveit_gazebo.rviz
        rviz_config_arg = LaunchConfiguration("rviz_config").perform(context)
        if rviz_config_arg:
            rviz_config = rviz_config_arg
        else:
            rviz_config = os.path.join(bringup_pkg, 'config', 'rviz', 'moveit_gazebo.rviz')

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=namespace,
            output="screen",
            arguments=["-d", rviz_config],
            parameters=rviz_params,
            remappings=[
                ("robot_description", f"/{namespace}/robot_description"),
            ],
        )
        launch_nodes.append(rviz_node)

    return launch_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="rbkairos",
            description="Robot namespace (must match Gazebo)"
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="rbkairos_arm_",
            description="Prefix for arm joints (must match Gazebo URDF)"
        ),
        DeclareLaunchArgument(
            "gripper_prefix",
            default_value="rbkairos_gripper_",
            description="Prefix for gripper joints (must match Gazebo URDF)"
        ),
        DeclareLaunchArgument(
            "base_prefix",
            default_value="rbkairos_",
            description="Prefix for mobile base links (must match Gazebo URDF)"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with MoveIt plugin"
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="Custom RViz config file path (optional, defaults to moveit_gazebo.rviz)"
        ),
        DeclareLaunchArgument(
            "publish_map_odom_tf",
            default_value="true",
            description="Publish static map->odom transform (set to false when using Nav2)"
        ),
        OpaqueFunction(function=launch_setup),
    ])
