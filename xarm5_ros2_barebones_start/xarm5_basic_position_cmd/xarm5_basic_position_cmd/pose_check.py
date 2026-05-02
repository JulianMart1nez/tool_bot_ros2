#!/usr/bin/env python3
"""
pose_check.py
-------------
Static diagnostic for the AprilTag → link_base TF chain. Subscribes to
every /fine_loc/tag_<id> topic and, once per second, prints a side-by-
side comparison of the most recent tag pose against the current TCP
pose (`link_base → link_tcp`). Useful for answering the single most
important question at hover:

    "Is the tag XY the camera reports close to the TCP XY?"

At hover the camera is roughly above the tag, so the expected delta is
just the camera-to-TCP mount offset projected onto link_base — a few
centimetres. A delta of tens of centimetres means fine_localization is
producing bogus poses (static TF misconfigured, or a sign flip in the
transform chain).

Usage:
    ros2 run xarm5_basic_position_cmd pose_check

Run with T1 (robot) and T2 (camera + fine_localization) up, the arm at
HOVER_PICKUP_JOINTS, and a tool AprilTag under the camera. The output
points at the exact fix to make.
"""

import math

import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import PoseStamped


TOOL_TAGS = {2: 'phillips_screwdriver', 3: 'hammer', 4: 'flathead_screwdriver'}
SANITY_XY_RADIUS_M = 0.35
REPORT_HZ = 1.0


class PoseCheck(Node):

    def __init__(self):
        super().__init__('pose_check')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._latest = {}  # tag_id -> (stamp_sec, x, y, z)
        for tag_id in TOOL_TAGS:
            self.create_subscription(
                PoseStamped, f'/fine_loc/tag_{tag_id}',
                lambda msg, tid=tag_id: self._on_pose(tid, msg), 10)

        self.create_timer(1.0 / REPORT_HZ, self._tick)

        self.get_logger().info(
            'pose_check running. Listening on /fine_loc/tag_{2,3,4}. '
            f'Will flag any tag whose link_base XY is >'
            f'{SANITY_XY_RADIUS_M*1000:.0f}mm from the current TCP XY.')

    def _on_pose(self, tag_id, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest[tag_id] = (
            t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _tcp_xyz(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'link_base', 'link_tcp', rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except Exception as e:
            self.get_logger().warn(
                f'link_base <- link_tcp TF failed: {e}',
                throttle_duration_sec=2.0)
            return None

    def _tick(self):
        tcp = self._tcp_xyz()
        if tcp is None:
            return
        if not self._latest:
            self.get_logger().info(
                f'tcp=({tcp[0]*1000:+.0f},{tcp[1]*1000:+.0f},'
                f'{tcp[2]*1000:+.0f})mm — no tag poses yet on /fine_loc/tag_*',
                throttle_duration_sec=3.0)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        lines = [
            f'tcp=({tcp[0]*1000:+.0f},{tcp[1]*1000:+.0f},'
            f'{tcp[2]*1000:+.0f})mm']
        for tag_id, (stamp, x, y, z) in sorted(self._latest.items()):
            age = now - stamp
            if age > 2.0:
                lines.append(
                    f'  tag_{tag_id} ({TOOL_TAGS[tag_id]}): stale '
                    f'({age:.1f}s old)')
                continue
            dx = x - tcp[0]
            dy = y - tcp[1]
            dz = z - tcp[2]
            d_xy = math.hypot(dx, dy)
            verdict = 'SANE' if d_xy <= SANITY_XY_RADIUS_M else 'BOGUS'
            lines.append(
                f'  tag_{tag_id} ({TOOL_TAGS[tag_id]}) [{verdict}]  '
                f'link_base=({x*1000:+.0f},{y*1000:+.0f},{z*1000:+.0f})mm  '
                f'delta=({dx*1000:+.0f},{dy*1000:+.0f},{dz*1000:+.0f})mm  '
                f'|Δxy|={d_xy*1000:.0f}mm')
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = PoseCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
