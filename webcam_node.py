# webcam_pub.py
# Author: M. Tyrrell
# Date: 04.09.2026
# Purpose: ROS2 (jazzy) node to read in webcam and publish as sensor_msg/Image


# Python specific
import cv2

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class webcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')

        # Camera parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        cam_idx = int(self.get_parameter('camera_index').value)
        fps = float(self.get_parameter('fps').value)
        
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        self.camera = cv2.VideoCapture(cam_idx)
        
        # error for cam connection
        if not self.camera.isOpened():
            self.get_logger().error(f"Could not open camera index {cam_idx}.")
            raise RuntimeError("Camera open failed")

        # call method 'tick()' to repeateded publish image once per frame
        self.timer = self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(f"Publishing /image_raw from webcam index {cam_idx} at ~{fps} FPS")

    def tick(self):
        # capture current frame
        ok, frame = self.camera.read()      
        # check for valid frame
        if not ok or frame is None:
            self.get_logger().warn("Failed to read frame from camera")
            return

        # convert to standard size as defined by parameters
        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)
        #frame = cv2.resize(frame, (w,h), interpolation=cv2.INTER_LINEAR)

        msg_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg_image);

    def destroy_node(self):
        if hasattr(self, 'camera') and self.camera:
            self.camera.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = webcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()