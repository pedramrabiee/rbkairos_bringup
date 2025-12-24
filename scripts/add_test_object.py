#!/usr/bin/env python3
"""Add test object and table to MoveIt planning scene for MTC testing."""

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def create_collision_object(frame_id, object_id, primitive_type, dimensions, position, orientation_w=1.0):
    """Create a collision object."""
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = object_id
    obj.operation = CollisionObject.ADD

    prim = SolidPrimitive()
    prim.type = primitive_type
    prim.dimensions = dimensions
    obj.primitives.append(prim)

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.w = orientation_w
    obj.primitive_poses.append(pose)

    return obj


def main():
    rclpy.init()
    node = Node('add_test_object')

    # Parse arguments
    object_id = sys.argv[1] if len(sys.argv) > 1 else 'test_object'
    x = float(sys.argv[2]) if len(sys.argv) > 2 else 0.55
    y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    z = float(sys.argv[4]) if len(sys.argv) > 4 else 1.05

    node.get_logger().info(f'Adding object "{object_id}" at ({x}, {y}, {z})')

    client = node.create_client(ApplyPlanningScene, '/rbkairos/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('ApplyPlanningScene service not available')
        return 1

    scene = PlanningScene()
    scene.is_diff = True

    # Add support surface (pick table) collision object
    # From Gazebo Fuel Table model: surface 1.5m x 0.8m x 0.03m at z=1.0
    # pick_table pose in mtc_test.world: (1.2, 0, 0)
    support_surface = create_collision_object(
        frame_id='map',
        object_id='support_surface',
        primitive_type=SolidPrimitive.BOX,
        dimensions=[1.5, 0.8, 0.03],
        position=[1.2, 0.0, 1.0]
    )
    scene.world.collision_objects.append(support_surface)
    node.get_logger().info('Adding support_surface collision object')

    # Add place table collision object
    # place_table pose in mtc_test.world: (1.2, 1.2, 0)
    place_table = create_collision_object(
        frame_id='map',
        object_id='place_table',
        primitive_type=SolidPrimitive.BOX,
        dimensions=[1.5, 0.8, 0.03],
        position=[1.2, 1.2, 1.0]
    )
    scene.world.collision_objects.append(place_table)
    node.get_logger().info('Adding place_table collision object')

    # Add target object
    target = create_collision_object(
        frame_id='map',
        object_id=object_id,
        primitive_type=SolidPrimitive.BOX,
        dimensions=[0.04, 0.04, 0.08],
        position=[x, y, z]
    )
    scene.world.collision_objects.append(target)

    req = ApplyPlanningScene.Request()
    req.scene = scene
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result().success:
        node.get_logger().info(f'Successfully added support_surface, place_table, and "{object_id}" to planning scene')
    else:
        node.get_logger().error('Failed to add objects to planning scene')
        return 1

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
