# april_tag_ids_node.py
# Author: M. Tyrrell
# Date: 04.09.2026
# Purpose: ROS2 (jazzy) node to read in sensor_msg/Image and determine count, ids, and (x,y) position of April Tags
# will also publish a preview of image

# Python specific
import cv2
import dt_apriltags

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
#from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge



class AprilTagIDsNode(Node):
    def __init__(self):
        super().__init__('april_tag_ids_node')

        # object for converting between ROS2 and openCV
        self.bridge = CvBridge()

        # create subscription object
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener,10)
        
        # create publisher object (this is triggered during the listener callback)
        self.publisher_ = self.create_publisher(Int16MultiArray, '/fiducial_markers', 10)

        # Apriltag ID storage container
        self.at_detector = dt_apriltags.Detector(families='tag36h11',
                                                 nthreads=1,
                                                 quad_decimate=1.0,
                                                 quad_sigma=0.0, 
                                                 refine_edges=1, 
                                                 decode_sharpening=0.25, 
                                                 debug=0)

        self.tags = Int16MultiArray()


    def listener(self, imageMessage):
        # print on console for recieved message
        #self.get_logger().info('Image received')

        # convert image to openCV
        received_img = self.bridge.imgmsg_to_cv2(imageMessage)

        # show image
        cv2.imshow("Camera Video", received_img)
        cv2.waitKey(1)

        # id april tags
        tags = self.at_detector.detect(cv2.cvtColor(received_img    , cv2.COLOR_BGR2GRAY), estimate_tag_pose=False, camera_params=None, tag_size=None)
        self.get_logger().info(str(tags))


def main():
    rclpy.init()
    node = AprilTagIDsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()