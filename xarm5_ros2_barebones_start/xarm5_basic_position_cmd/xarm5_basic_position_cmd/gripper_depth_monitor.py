#!/usr/bin/env python3
"""
gripper_depth_monitor.py
Monitors distance from the gripper-mounted depth camera to surfaces below.

Publishes:
  /gripper/tool_distance   (Float32) — distance to closest object in center ROI (meters)
  /gripper/cart_distance   (Float32) — distance to cart surface in outer ROI (meters)
  /gripper/tool_height_measured (Float32) — cart_distance - tool_distance (meters)

Subscribes:
  depth_image  (sensor_msgs/Image) — raw depth in mm (Z16 format)
  camera_info  (sensor_msgs/CameraInfo)
  /gripper/monitor_enabled (std_msgs/Bool) — gate on/off

Only processes frames when monitor_enabled is True.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32


class GripperDepthMonitor(Node):

    def __init__(self):
        super().__init__('gripper_depth_monitor')

        self.declare_parameter('center_roi_size', 40)
        self.declare_parameter('outer_roi_inner', 80)
        self.declare_parameter('outer_roi_outer', 160)
        self.declare_parameter('min_valid_pixels', 50)
        self.declare_parameter('publish_rate', 15.0)

        self.center_roi_size = self.get_parameter('center_roi_size').value
        self.outer_roi_inner = self.get_parameter('outer_roi_inner').value
        self.outer_roi_outer = self.get_parameter('outer_roi_outer').value
        self.min_valid_pixels = self.get_parameter('min_valid_pixels').value

        self.enabled = False
        self.camera_info = None
        self.latest_depth = None

        # Publishers
        self.pub_tool_dist = self.create_publisher(Float32, '/gripper/tool_distance', 10)
        self.pub_cart_dist = self.create_publisher(Float32, '/gripper/cart_distance', 10)
        self.pub_tool_height = self.create_publisher(Float32, '/gripper/tool_height_measured', 10)

        # Subscribers
        self.create_subscription(Image, 'depth_image', self._depth_cb, 10)
        self.create_subscription(CameraInfo, 'camera_info', self._info_cb, 10)
        self.create_subscription(Bool, '/gripper/monitor_enabled', self._enable_cb, 10)

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self._process)

        self.get_logger().info(
            f'Depth monitor ready (center_roi={self.center_roi_size}px, '
            f'outer={self.outer_roi_inner}-{self.outer_roi_outer}px). '
            f'Waiting for /gripper/monitor_enabled=True...')

    def _enable_cb(self, msg):
        if msg.data != self.enabled:
            self.enabled = msg.data
            state = 'ENABLED' if self.enabled else 'DISABLED'
            self.get_logger().info(f'Depth monitor {state}')

    def _info_cb(self, msg):
        self.camera_info = msg

    def _depth_cb(self, msg):
        if not self.enabled:
            return
        if msg.encoding != '16UC1':
            self.get_logger().warn(f'Unexpected encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return
        self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    def _extract_roi(self, depth, cx, cy, half_size):
        h, w = depth.shape
        y0 = max(0, cy - half_size)
        y1 = min(h, cy + half_size)
        x0 = max(0, cx - half_size)
        x1 = min(w, cx + half_size)
        return depth[y0:y1, x0:x1]

    def _process(self):
        if not self.enabled or self.latest_depth is None:
            return

        depth = self.latest_depth
        h, w = depth.shape
        cx, cy = w // 2, h // 2

        # Center ROI — tool distance (min of valid pixels)
        center = self._extract_roi(depth, cx, cy, self.center_roi_size // 2)
        center_valid = center[center > 0]

        # Outer annular ROI — cart distance (median of valid pixels)
        outer_full = self._extract_roi(depth, cx, cy, self.outer_roi_outer // 2)
        inner_mask = np.ones_like(outer_full, dtype=bool)
        oh, ow = outer_full.shape
        ocx, ocy = ow // 2, oh // 2
        inner_half = self.outer_roi_inner // 2
        y0 = max(0, ocy - inner_half)
        y1 = min(oh, ocy + inner_half)
        x0 = max(0, ocx - inner_half)
        x1 = min(ow, ocx + inner_half)
        inner_mask[y0:y1, x0:x1] = False
        annular = outer_full[inner_mask]
        annular_valid = annular[annular > 0]

        tool_dist_mm = None
        cart_dist_mm = None

        if len(center_valid) >= self.min_valid_pixels:
            tool_dist_mm = float(np.min(center_valid))
        else:
            self.get_logger().debug('Center ROI: insufficient valid pixels', throttle_duration_sec=2.0)

        if len(annular_valid) >= self.min_valid_pixels:
            cart_dist_mm = float(np.median(annular_valid))
        else:
            self.get_logger().debug('Outer ROI: insufficient valid pixels', throttle_duration_sec=2.0)

        if tool_dist_mm is not None:
            msg = Float32()
            msg.data = tool_dist_mm / 1000.0
            self.pub_tool_dist.publish(msg)

        if cart_dist_mm is not None:
            msg = Float32()
            msg.data = cart_dist_mm / 1000.0
            self.pub_cart_dist.publish(msg)

        if tool_dist_mm is not None and cart_dist_mm is not None:
            msg = Float32()
            msg.data = (cart_dist_mm - tool_dist_mm) / 1000.0
            self.pub_tool_height.publish(msg)
            self.get_logger().info(
                f'Tool: {tool_dist_mm:.0f}mm | Cart: {cart_dist_mm:.0f}mm | '
                f'Height: {cart_dist_mm - tool_dist_mm:.0f}mm',
                throttle_duration_sec=0.5)
        elif cart_dist_mm is not None:
            self.get_logger().info(
                f'Cart only: {cart_dist_mm:.0f}mm',
                throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = GripperDepthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
