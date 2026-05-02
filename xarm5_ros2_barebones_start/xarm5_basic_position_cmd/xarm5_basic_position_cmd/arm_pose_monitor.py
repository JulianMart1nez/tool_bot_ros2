#!/usr/bin/env python3
"""
arm_pose_monitor.py
-------------------
Prints live TCP position (link_base → link_tcp) and current joint angles at
a fixed rate. Useful for driving the arm by hand/teach and capturing the
resulting pose as a "safe" hover/grasp setpoint.

Default rate: 2 Hz. Override with --ros-args -p rate_hz:=5.0

Output format (one line per tick):
  TCP  x=0.498 y=0.215 z=0.422 m | RPY deg (r=-179.4 p=-1.3 y=-0.2)
  J(deg) 0.0 0.0 0.0 0.0 0.0
  J(rad) 0.000 0.000 0.000 0.000 0.000
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros


FRAME_ID = 'link_base'
TCP_FRAME = 'link_tcp'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


def quat_to_rpy(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class ArmPoseMonitor(Node):

    def __init__(self):
        super().__init__('arm_pose_monitor')
        self.declare_parameter('rate_hz', 2.0)
        rate = float(self.get_parameter('rate_hz').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'arm_pose_monitor running at {rate:.1f} Hz. '
            'Source: TF link_base→link_tcp + /joint_states.')

    def _js_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                FRAME_ID, TCP_FRAME, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(
                f'TF {FRAME_ID}->{TCP_FRAME} unavailable: {e}',
                throttle_duration_sec=2.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)

        line1 = (
            f'TCP  x={t.x:+.3f} y={t.y:+.3f} z={t.z:+.3f} m | '
            f'RPY deg (r={math.degrees(roll):+.1f} '
            f'p={math.degrees(pitch):+.1f} y={math.degrees(yaw):+.1f})')

        j_deg = []
        j_rad = []
        missing = False
        for name in JOINT_NAMES:
            val = self._joint_positions.get(name)
            if val is None:
                j_deg.append('  n/a')
                j_rad.append('  n/a')
                missing = True
            else:
                j_deg.append(f'{math.degrees(val):+6.1f}')
                j_rad.append(f'{val:+6.3f}')
        line2 = 'J(deg) ' + ' '.join(j_deg)
        line3 = 'J(rad) ' + ' '.join(j_rad)

        self.get_logger().info(f'{line1} | {line2} | {line3}')
        if missing:
            self.get_logger().debug('Some joints missing from /joint_states.')


def main(args=None):
    rclpy.init(args=args)
    node = ArmPoseMonitor()
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
