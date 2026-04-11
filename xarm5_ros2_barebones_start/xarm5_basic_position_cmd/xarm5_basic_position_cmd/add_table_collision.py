#!/usr/bin/env python3
"""
add_table_collision.py
----------------------
Adds the mounting table as a collision object to the MoveIt2 planning scene.

Run this AFTER launching MoveIt:
  ros2 launch xarm_moveit_config xarm5_moveit_realmove.launch.py robot_ip:=192.168.1.234
  ros2 run xarm5_basic_position_cmd add_table_collision

Table geometry (measured 2026-04-10):
  - Flange plate thickness:       0.35 in  = 8.89 mm
  - Table extends left/right:     15 in each side = 0.381 m (total width 0.762 m)
  - Table extends behind arm:     69.5 in  = 1.7653 m
  - Arm is mounted at the FRONT EDGE of the table, so the table occupies
    the region behind the robot (negative X in the base frame).
  - Table surface is 0.00889 m below the link_base origin (flange thickness).

We model the table as a 0.050 m (5 cm) thick box:
  - X range: -1.7653 m to 0.0 m    (behind robot, up to front edge)
  - Y range: -0.381 m to +0.381 m  (381 mm each side of centerline)
  - Z range: -0.05889 m to -0.00889 m (top of box = bottom of flange)

The box center in the link_base frame:
  x = (-1.7653 + 0.0) / 2        = -0.88265 m
  y = 0.0 m
  z = (-0.05889 + -0.00889) / 2  = -0.03389 m
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


# ------------------------------------------------------------------
# Table dimensions and pose (all in meters, in the link_base frame)
# ------------------------------------------------------------------
TABLE_SIZE_X = 1.7653    # length behind robot
TABLE_SIZE_Y = 0.762     # full width (0.381 each side)
TABLE_SIZE_Z = 0.050     # slab thickness

TABLE_CENTER_X = -TABLE_SIZE_X / 2.0      # centered behind the robot
TABLE_CENTER_Y = 0.0                       # centered left/right
TABLE_CENTER_Z = -0.00889 - TABLE_SIZE_Z / 2.0  # top of box = bottom of flange


class TableAdder(Node):

    def __init__(self):
        super().__init__('add_table_collision')

        self.cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('Waiting for /apply_planning_scene service...')
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Service /apply_planning_scene not available. '
                'Is MoveIt (xarm5_moveit_realmove.launch.py) running?')
            raise RuntimeError('MoveIt not running')

    def _build_table_collision_object(self) -> CollisionObject:
        """Construct a CollisionObject representing the table slab."""
        obj = CollisionObject()
        obj.header.frame_id = 'link_base'  # xArm5 base link
        obj.id = 'mounting_table'
        obj.operation = CollisionObject.ADD

        # Box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_SIZE_Z]

        # Pose of the box center in link_base
        pose = Pose()
        pose.position.x = TABLE_CENTER_X
        pose.position.y = TABLE_CENTER_Y
        pose.position.z = TABLE_CENTER_Z
        pose.orientation.w = 1.0  # no rotation

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)

        return obj

    def add_table(self):
        """Build and publish the table collision object to MoveIt."""
        table = self._build_table_collision_object()

        scene = PlanningScene()
        scene.is_diff = True
        scene.world = PlanningSceneWorld()
        scene.world.collision_objects.append(table)

        req = ApplyPlanningScene.Request()
        req.scene = scene

        self.get_logger().info(
            f"Adding table: size=[{TABLE_SIZE_X:.4f}, {TABLE_SIZE_Y:.4f}, {TABLE_SIZE_Z:.4f}] m, "
            f"center=[{TABLE_CENTER_X:.4f}, {TABLE_CENTER_Y:.4f}, {TABLE_CENTER_Z:.4f}] m")

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Service call timed out or failed.')
            return False

        resp = future.result()
        if resp.success:
            self.get_logger().info('Table added to planning scene successfully.')
            return True
        else:
            self.get_logger().error('Service returned success=false.')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = TableAdder()
    try:
        node.add_table()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
