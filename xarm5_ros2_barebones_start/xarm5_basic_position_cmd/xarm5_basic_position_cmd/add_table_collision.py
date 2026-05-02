#!/usr/bin/env python3
"""
add_table_collision.py
----------------------
Adds collision objects to the MoveIt2 planning scene:
  - Mounting table (behind robot)
  - Pickup cart (left of robot, +y)
  - Dropoff cart (right of robot, -y)

Run this AFTER launching MoveIt (auto-run by launch file).

Table geometry (measured 2026-04-10):
  - Flange plate thickness:       0.35 in  = 8.89 mm
  - Table extends left/right:     15 in each side = 0.381 m (total width 0.762 m)
  - Table extends behind arm:     69.5 in  = 1.7653 m
  - Table surface is 0.00889 m below the link_base origin (flange thickness).

Cart geometry (measured 2026-04-13):
  Both cart surfaces are 4.35 in (0.11049 m) below robot base.
  Carts are 31 in tall. Cart tops at z = -0.11049 m in link_base.

  Pickup cart (LEFT, +y): 24 in long (x) × 18 in wide (y)
    - Tag center x = (26+41)/2 = 33.5 in, tag center y = (4+18)/2 = 11 in
    - Cart center: x = 33.5 in, y = +11 in

  Dropoff cart (RIGHT, -y): 34 in long (x) × 17.5 in wide (y)
    - Tag center x = (26+41)/2 = 33.5 in, tag center y = (4+18)/2 = 11 in
    - Cart center: x = 33.5 in, y = -11 in
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


# ------------------------------------------------------------------
# Dimensions and poses (all in meters, in the link_base frame)
# ------------------------------------------------------------------
IN = 0.0254  # inches to meters

# Mounting table
TABLE_SIZE_X = 1.7653
TABLE_SIZE_Y = 0.762
TABLE_SIZE_Z = 0.050
TABLE_CENTER_X = -TABLE_SIZE_X / 2.0
TABLE_CENTER_Y = 0.0
TABLE_CENTER_Z = -0.00889 - TABLE_SIZE_Z / 2.0

# Per-cart top surface z below link_base. Pickup was measured directly
# (closed gripper tip touched to cart surface, link_tcp z read off Studio
# = -3.07 in). Dropoff is left at the previously-working -5.00 in — the
# drop sequence removes the dropoff_cart collision before the per-tool
# drop pose, so the modeled height there only affects RViz visualization
# and the pre-drop hover-transit clearance, not the drop pose itself.
PICKUP_TOP_Z = -3.07 * IN + 0.002  # -0.07598 m  (user-measured -3.07in,
                                    # then raised 2 mm so the gripper
                                    # geometry at grab pose clears the
                                    # modeled cart slab.)
DROPOFF_TOP_Z = -5.00 * IN  # -0.12700 m

CART_HEIGHT = 31.0 * IN   # 0.7874 m
CART_THICKNESS = 0.025     # model cart top as 2.5cm thick slab

# Both carts: near edge at 18in from base, touching at y=0
CART_NEAR_EDGE_X = 18.0 * IN  # 0.4572 m

# Pickup cart (LEFT of robot, +y, counterclockwise J1): 24in long × 18in
# wide. Flat slab at PICKUP_TOP_Z — no rim model. The 4 outer corners
# of the cart surface are flush with that z.
PICKUP_SIZE_X = 24.0 * IN
PICKUP_SIZE_Y = 18.0 * IN
PICKUP_CENTER_X = CART_NEAR_EDGE_X + PICKUP_SIZE_X / 2.0
PICKUP_CENTER_Y = PICKUP_SIZE_Y / 2.0
PICKUP_CENTER_Z = PICKUP_TOP_Z - CART_THICKNESS / 2.0

# Dropoff cart (RIGHT of robot, -y, clockwise J1): 34in long × 17.5in
# wide. Flat slab at DROPOFF_TOP_Z, no rim.
DROPOFF_SIZE_X = 34.0 * IN
DROPOFF_SIZE_Y = 17.5 * IN
DROPOFF_CENTER_X = CART_NEAR_EDGE_X + DROPOFF_SIZE_X / 2.0
DROPOFF_CENTER_Y = -DROPOFF_SIZE_Y / 2.0
DROPOFF_CENTER_Z = DROPOFF_TOP_Z - CART_THICKNESS / 2.0


def _make_box(obj_id, size_x, size_y, size_z, cx, cy, cz):
    """Create a CollisionObject box in link_base frame."""
    obj = CollisionObject()
    obj.header.frame_id = 'link_base'
    obj.id = obj_id
    obj.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [size_x, size_y, size_z]

    pose = Pose()
    pose.position.x = cx
    pose.position.y = cy
    pose.position.z = cz
    pose.orientation.w = 1.0

    obj.primitives.append(box)
    obj.primitive_poses.append(pose)
    return obj


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

    def add_all(self):
        """Add table + both carts to the planning scene."""
        table = _make_box('mounting_table',
                          TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_SIZE_Z,
                          TABLE_CENTER_X, TABLE_CENTER_Y, TABLE_CENTER_Z)

        pickup = _make_box('pickup_cart',
                           PICKUP_SIZE_X, PICKUP_SIZE_Y, CART_THICKNESS,
                           PICKUP_CENTER_X, PICKUP_CENTER_Y, PICKUP_CENTER_Z)

        dropoff = _make_box('dropoff_cart',
                            DROPOFF_SIZE_X, DROPOFF_SIZE_Y, CART_THICKNESS,
                            DROPOFF_CENTER_X, DROPOFF_CENTER_Y, DROPOFF_CENTER_Z)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world = PlanningSceneWorld()
        scene.world.collision_objects.extend([table, pickup, dropoff])

        req = ApplyPlanningScene.Request()
        req.scene = scene

        self.get_logger().info('Adding collision objects: mounting_table, pickup_cart, dropoff_cart')

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Service call timed out or failed.')
            return False

        resp = future.result()
        if resp.success:
            self.get_logger().info(
                f'All collision objects added:\n'
                f'  table: center=({TABLE_CENTER_X:.3f}, {TABLE_CENTER_Y:.3f}, {TABLE_CENTER_Z:.3f})\n'
                f'  pickup_cart: center=({PICKUP_CENTER_X:.3f}, {PICKUP_CENTER_Y:.3f}, {PICKUP_CENTER_Z:.3f}), '
                f'size=({PICKUP_SIZE_X:.3f}, {PICKUP_SIZE_Y:.3f})\n'
                f'  dropoff_cart: center=({DROPOFF_CENTER_X:.3f}, {DROPOFF_CENTER_Y:.3f}, {DROPOFF_CENTER_Z:.3f}), '
                f'size=({DROPOFF_SIZE_X:.3f}, {DROPOFF_SIZE_Y:.3f})')
            return True
        else:
            self.get_logger().error('Service returned success=false.')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = TableAdder()
    try:
        node.add_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
