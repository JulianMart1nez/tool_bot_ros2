#!/usr/bin/env python3
"""
realsense_v4l2_pub.py
---------------------
Publishes RealSense D435i RGB frames via V4L2 as ROS2 Image messages.
Workaround for when realsense2_camera ROS package is not installed.

Publishes:
  /gripper_cam/depth_camera/color/image_raw  (sensor_msgs/Image)
  /gripper_cam/depth_camera/color/camera_info (sensor_msgs/CameraInfo)
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class RealSenseV4L2Publisher(Node):

    def __init__(self):
        super().__init__('realsense_v4l2_pub')

        self.declare_parameter('camera_index', 10)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)

        idx = self.get_parameter('camera_index').value
        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        self.cap = cv2.VideoCapture(idx)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'Cannot open camera {idx}')
            raise RuntimeError(f'Cannot open camera {idx}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 166)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(
            Image, '/gripper_cam/depth_camera/color/image_raw', 10)
        self.info_pub = self.create_publisher(
            CameraInfo, '/gripper_cam/depth_camera/color/camera_info', 10)

        self.camera_info = CameraInfo()
        self.camera_info.width = actual_w
        self.camera_info.height = actual_h
        # D435i RGB intrinsics at 640x480 (from RealSense factory calibration)
        self.camera_info.k = [
            615.0, 0.0, 320.0,
            0.0, 615.0, 240.0,
            0.0, 0.0, 1.0
        ]
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.distortion_model = 'plumb_bob'

        self.timer = self.create_timer(1.0 / fps, self._tick)

        self.get_logger().info(
            f'RealSense V4L2 publisher: camera {idx} @ {actual_w}x{actual_h}')

    def _tick(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        stamp = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = 'camera_color_optical_frame'
        self.image_pub.publish(img_msg)

        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = 'camera_color_optical_frame'
        self.info_pub.publish(self.camera_info)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseV4L2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
