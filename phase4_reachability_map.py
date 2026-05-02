#!/usr/bin/env python3
"""
Phase 4: xArm5 Reachable Workspace Characterization

Probes a 3D grid of Cartesian poses via MoveIt's /compute_ik service
(TRAC-IK solver) to map which (x, y, z) positions are reachable with
tool-down orientation (roll=180, pitch=0, yaw=0).

Does NOT move the robot — purely computational IK queries.

Prerequisites:
  MoveIt must be running:
    ros2 launch xarm5_basic_position_cmd xarm5_moveit_with_table.launch.py \
      robot_ip:=192.168.1.234

Usage:
  python3 phase4_reachability_map.py
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import math
import time
import sys

# ── Grid parameters (millimeters, converted to meters for MoveIt) ──────
X_RANGE = (100, 500, 50)     # min_mm, max_mm, step_mm
Y_RANGE = (-400, 400, 50)
Z_RANGE = (0, 500, 50)

# Fixed orientation: tool pointing down
# roll=180° pitch=0° yaw=0° → quaternion: w=0, x=1, y=0, z=0
# (180° rotation about X axis)
ORIENTATION_W = 0.0
ORIENTATION_X = 1.0
ORIENTATION_Y = 0.0
ORIENTATION_Z = 0.0

PLANNING_GROUP = "xarm5"
FRAME_ID = "link_base"
IK_TIMEOUT = 0.1  # seconds per IK query


class ReachabilityMapper(Node):

    def __init__(self):
        super().__init__('reachability_mapper')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info('Waiting for /compute_ik service...')
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/compute_ik not available. Is MoveIt running?')
            raise RuntimeError('MoveIt not running')
        self.get_logger().info('Connected to /compute_ik')

    def check_ik(self, x_m, y_m, z_m):
        """Query IK for a single pose. Returns True if solvable."""
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = int(IK_TIMEOUT * 1e9)

        pose = PoseStamped()
        pose.header.frame_id = FRAME_ID
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x_m
        pose.pose.position.y = y_m
        pose.pose.position.z = z_m
        pose.pose.orientation.w = ORIENTATION_W
        pose.pose.orientation.x = ORIENTATION_X
        pose.pose.orientation.y = ORIENTATION_Y
        pose.pose.orientation.z = ORIENTATION_Z
        req.ik_request.pose_stamped = pose

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            return False

        # error_code.val == 1 means SUCCESS in MoveIt
        return future.result().error_code.val == 1


def main():
    rclpy.init()
    node = ReachabilityMapper()

    x_min, x_max, x_step = X_RANGE
    y_min, y_max, y_step = Y_RANGE
    z_min, z_max, z_step = Z_RANGE

    xs = list(range(x_min, x_max + 1, x_step))
    ys = list(range(y_min, y_max + 1, y_step))
    zs = list(range(z_min, z_max + 1, z_step))

    total = len(xs) * len(ys) * len(zs)
    print(f"Reachability grid: X=[{x_min},{x_max}] Y=[{y_min},{y_max}] Z=[{z_min},{z_max}] step={x_step}mm")
    print(f"Total poses to check: {total}")
    print(f"Orientation: tool-down (roll=180, pitch=0, yaw=0)")
    print(f"Collision checking: enabled (table in scene)")
    print()

    results = []
    reachable_count = 0
    checked = 0

    for x_mm in xs:
        for y_mm in ys:
            for z_mm in zs:
                x_m = x_mm / 1000.0
                y_m = y_mm / 1000.0
                z_m = z_mm / 1000.0

                ok = node.check_ik(x_m, y_m, z_m)
                results.append((x_mm, y_mm, z_mm, ok))
                if ok:
                    reachable_count += 1
                checked += 1

                if checked % 50 == 0 or checked == total:
                    pct = 100.0 * checked / total
                    print(f"  [{checked}/{total}] ({pct:.0f}%) — {reachable_count} reachable so far",
                          flush=True)

    # ── Summary ──────────────────────────────────────────────────────
    print()
    print("=" * 65)
    print("REACHABILITY SUMMARY")
    print("=" * 65)
    print(f"Total poses checked: {total}")
    print(f"Reachable (IK success + collision-free): {reachable_count}")
    print(f"Unreachable: {total - reachable_count}")
    print(f"Reachability rate: {100.0 * reachable_count / total:.1f}%")
    print()

    # ── Find boundaries per axis ─────────────────────────────────────
    reachable_pts = [(x, y, z) for x, y, z, ok in results if ok]

    if reachable_pts:
        rx = [p[0] for p in reachable_pts]
        ry = [p[1] for p in reachable_pts]
        rz = [p[2] for p in reachable_pts]
        print("REACHABLE ENVELOPE (tool-down, collision-free):")
        print(f"  X range: {min(rx)} to {max(rx)} mm")
        print(f"  Y range: {min(ry)} to {max(ry)} mm")
        print(f"  Z range: {min(rz)} to {max(rz)} mm")
        print()

        # Per-Z-layer reachability
        print("REACHABLE AREA PER Z-LAYER:")
        for z_mm in zs:
            layer = [(x, y) for x, y, z, ok in results if ok and z == z_mm]
            if layer:
                lx = [p[0] for p in layer]
                ly = [p[1] for p in layer]
                print(f"  z={z_mm:>4d}mm: {len(layer):>4d} points, "
                      f"x=[{min(lx)},{max(lx)}] y=[{min(ly)},{max(ly)}]")
            else:
                print(f"  z={z_mm:>4d}mm:    0 points (unreachable at this height)")
    else:
        print("WARNING: No reachable poses found! Check orientation and grid bounds.")

    # ── Write results to file ────────────────────────────────────────
    outfile = "/home/julian/ros2_ws/src/tool_bot_ros2/REACHABILITY_MAP.txt"
    with open(outfile, 'w') as f:
        f.write("═" * 65 + "\n")
        f.write("PHASE 4: xArm5 REACHABILITY MAP\n")
        f.write("═" * 65 + "\n")
        f.write(f"Date: 2026-04-12\n")
        f.write(f"Solver: TRAC-IK (solve_type: Distance)\n")
        f.write(f"Orientation: tool-down (roll=180, pitch=0, yaw=0)\n")
        f.write(f"Collision checking: enabled (mounting table in scene)\n")
        f.write(f"Grid: X=[{x_min},{x_max}] Y=[{y_min},{y_max}] Z=[{z_min},{z_max}] step={x_step}mm\n")
        f.write(f"Total: {total} poses, {reachable_count} reachable "
                f"({100.0 * reachable_count / total:.1f}%)\n")
        f.write("\n")

        if reachable_pts:
            f.write("REACHABLE ENVELOPE:\n")
            f.write(f"  X: {min(rx)} to {max(rx)} mm\n")
            f.write(f"  Y: {min(ry)} to {max(ry)} mm\n")
            f.write(f"  Z: {min(rz)} to {max(rz)} mm\n")
            f.write("\n")

            f.write("PER Z-LAYER:\n")
            for z_mm in zs:
                layer = [(x, y) for x, y, z, ok in results if ok and z == z_mm]
                if layer:
                    lx = [p[0] for p in layer]
                    ly = [p[1] for p in layer]
                    f.write(f"  z={z_mm:>4d}mm: {len(layer):>4d} pts, "
                            f"x=[{min(lx)},{max(lx)}] y=[{min(ly)},{max(ly)}]\n")
                else:
                    f.write(f"  z={z_mm:>4d}mm:    0 pts\n")

        f.write("\n")
        f.write("RAW DATA (x_mm, y_mm, z_mm, reachable):\n")
        for x_mm, y_mm, z_mm, ok in results:
            f.write(f"  {x_mm:>4d}, {y_mm:>4d}, {z_mm:>4d}, {'YES' if ok else 'NO'}\n")

    print(f"\nResults written to: {outfile}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(1)
