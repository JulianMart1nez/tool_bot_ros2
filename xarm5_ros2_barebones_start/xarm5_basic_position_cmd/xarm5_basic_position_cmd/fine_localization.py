#!/usr/bin/env python3
"""
fine_localization.py
--------------------
Continuously detects AprilTags in the gripper-mounted depth camera's RGB
stream and publishes each detected tool tag's absolute 3D position in the
robot's `link_base` frame, computed via TF using the published
camera optical frame transform chain.

This means the consumer (e.g. test_descent / pick-and-place) can simply
read the latest published position and command the fingertips (link_tcp)
to that XY directly — the camera-to-fingertip horizontal offset is
already accounted for through the TF chain.

Subscribes:
  color_image  (sensor_msgs/Image) — RGB from gripper camera
  color_info   (sensor_msgs/CameraInfo) — intrinsics

Publishes (continuously while a tag is in view):
  /fine_loc/result        (geometry_msgs/PoseStamped) — tag position in link_base
  /fine_loc/tag_detected  (std_msgs/Bool) — true while a tool tag is visible
"""

import numpy as np
import dt_apriltags
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import tf2_ros
from tf2_geometry_msgs import do_transform_point


TOOL_TAGS = {2: 'phillips_screwdriver', 3: 'hammer', 4: 'flathead_screwdriver'}
TAG_SIZE = 0.025  # 25mm physical tag
TARGET_FRAME = 'link_base'


class FineLocalization(Node):

    def __init__(self):
        super().__init__('fine_localization')

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_image = None

        self.detector = dt_apriltags.Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0)

        self.declare_parameter('detect_rate_hz', 10.0)
        self.declare_parameter('log_every_n', 10)  # log 1 of every N detections

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_result = self.create_publisher(PoseStamped, '/fine_loc/result', 10)
        self.pub_detected = self.create_publisher(Bool, '/fine_loc/tag_detected', 10)

        self.create_subscription(Image, 'color_image', self._image_cb, 10)
        self.create_subscription(CameraInfo, 'color_info', self._info_cb, 10)

        rate = self.get_parameter('detect_rate_hz').value
        self.create_timer(1.0 / rate, self._tick)

        self._tick_count = 0
        self._last_detected = False

        self.get_logger().info(
            f'Fine localization running continuously at {rate:.1f} Hz. '
            f'Publishing tag pose in {TARGET_FRAME} on /fine_loc/result.')

    def _info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _image_cb(self, msg):
        self.latest_image = msg

    def _detect_tags(self):
        if self.latest_image is None or self.fx is None:
            return None, []
        frame = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=TAG_SIZE)
        tool_tags = [t for t in tags if t.tag_id in TOOL_TAGS]
        return self.latest_image.header, tool_tags

    def _tick(self):
        self._tick_count += 1
        log_n = self.get_parameter('log_every_n').value

        header, tool_tags = self._detect_tags()

        detected_msg = Bool()
        if not tool_tags:
            detected_msg.data = False
            self.pub_detected.publish(detected_msg)
            if self._last_detected:
                self.get_logger().info('No tool tag in view.')
            self._last_detected = False
            return

        # Use the closest tag (smallest depth) to the camera
        tag = min(tool_tags, key=lambda t: float(np.array(t.pose_t).flatten()[2]))
        pose_t = np.array(tag.pose_t).flatten()

        # Build a PointStamped in the camera optical frame and transform to link_base.
        pt = PointStamped()
        pt.header = header
        pt.point.x = float(pose_t[0])
        pt.point.y = float(pose_t[1])
        pt.point.z = float(pose_t[2])

        try:
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME, header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup {TARGET_FRAME} <- {header.frame_id} failed: {e}',
                throttle_duration_sec=2.0)
            detected_msg.data = False
            self.pub_detected.publish(detected_msg)
            return

        pt_base = do_transform_point(pt, tf)

        result = PoseStamped()
        result.header.frame_id = TARGET_FRAME
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose.position.x = pt_base.point.x
        result.pose.position.y = pt_base.point.y
        result.pose.position.z = pt_base.point.z
        result.pose.orientation.w = 1.0
        self.pub_result.publish(result)

        detected_msg.data = True
        self.pub_detected.publish(detected_msg)

        if not self._last_detected or (self._tick_count % log_n == 0):
            self.get_logger().info(
                f'{TOOL_TAGS[tag.tag_id]} (id={tag.tag_id}) @ link_base: '
                f'x={pt_base.point.x*1000:.1f}mm, y={pt_base.point.y*1000:.1f}mm, '
                f'z={pt_base.point.z*1000:.1f}mm  (cam depth={pose_t[2]*1000:.1f}mm)')
        self._last_detected = True


def main(args=None):
    rclpy.init(args=args)
    node = FineLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
