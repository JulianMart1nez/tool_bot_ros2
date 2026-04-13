# april_tag_ids_node.py
# Author: M. Tyrrell
# Date: 04.09.2026
# Purpose: ROS2 (jazzy) node to read in sensor_msg/Image and determine count, ids, and (x,y) position of April Tags
# will also publish a preview of image

# Python specific
import cv2

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16


class selectTagNode(Node):
    def __init__(self):
        super().__init__('select_tag_node')

        # create publisher object 
        self.publisher_ = self.create_publisher(Int16, '/tag_id', 10)

        # TESTING ONLY. id_num SHOULD BE ASSIGNED BY VOICE CONTROL and only published when request is complete
        self.id_num = Int16()
        self.id_num.data = 0 
        # start timer
        timer_period = 5 #second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):                             
        self.get_logger().info("Requesting tag id: " + str(self.id_num.data))
        self.publisher_.publish(self.id_num)


def main():
    rclpy.init()
    node = selectTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()