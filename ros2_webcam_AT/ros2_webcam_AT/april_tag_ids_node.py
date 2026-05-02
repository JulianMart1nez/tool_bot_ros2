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
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


class AprilTagIDsNode(Node):
    def __init__(self):
        super().__init__('april_tag_ids_node')

        # object for converting between ROS2 and openCV
        self.bridge = CvBridge()

        # create subscription object to image
        self.subscription_img = self.create_subscription(Image, '/image_raw', self.listener_img,10)
        
        # create subscription to desired tag ID
        self.subscription_id = self.create_subscription(Int16, '/tag_id', self.listener_id,10)
        
        # create publisher object (this is triggered during the listener callback)
        self.publisher_ = self.create_publisher(PoseStamped, '/tag_location', 10)

        # Apriltag ID storage container
        self.at_detector = dt_apriltags.Detector(families='tag36h11',
                                                 nthreads=1,
                                                 quad_decimate=1.0,
                                                 quad_sigma=0.0, 
                                                 refine_edges=1, 
                                                 decode_sharpening=0.25, 
                                                 debug=0)
        
        # Webcam camera matrix (intrinsic props): 
        #   [fx 0  cx]      [969.20915798   0.         588.80142754]
        #   [0  fy cy]  =   [  0.         966.76406593 283.34318455]
        #   [0  0  1 ]      [  0.           0.           1.        ]
        self.fx = 969.20915798
        self.fy = 966.76406593
        self.cx = 588.80142754
        self.cy = 283.34318455

        # Tag size
        self.tag_size = 0.05 #meters


    def listener_img(self, imageMessage):
        # print on console for recieved message
        #self.get_logger().info('Image received')

        # convert image to openCV
        received_img = self.bridge.imgmsg_to_cv2(imageMessage)

        # id april tags
        tags = self.at_detector.detect(cv2.cvtColor(received_img, cv2.COLOR_BGR2GRAY), estimate_tag_pose=True, camera_params=[self.fx, self.fy, self.cx, self.cy], tag_size=self.tag_size)

        self.ids = []
        self.center_xy = []
        self.pose_t = []
        for i in range(len(tags)):
            current_tag = tags[i]
            self.ids.append(current_tag.tag_id)
            self.center_xy.append(current_tag.center)
            self.pose_t.append(current_tag.pose_t)

            # Draw tag outline and ID on the preview
            corners = current_tag.corners.astype(int)
            for j in range(4):
                pt1 = tuple(corners[j])
                pt2 = tuple(corners[(j + 1) % 4])
                cv2.line(received_img, pt1, pt2, (0, 255, 0), 2)
            cx, cy = int(current_tag.center[0]), int(current_tag.center[1])
            cv2.putText(received_img, f"ID:{current_tag.tag_id}", (cx - 20, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # show annotated image
        cv2.imshow("Camera Video", received_img)
        cv2.waitKey(1)


    def listener_id(self, at_id):       
        self.get_logger().info('Requested April ID ' + str(at_id.data) + ' pose.')
        
        try: 
            # get requested id
            list_id = self.ids.index(at_id.data)
            self.get_logger().info('April ID ' + str(at_id.data) + ' found.')
            
            # build pose message to publish
            #msg = PoseStamped()
            #msg.header.frame_id = 'global_camera_frame'
            #msg.pose.position = self.pose_t[list_id]                   
            #self.publisher_.publish(msg)
        except:
            self.get_logger().info('April ID ' + str(at_id.data) + ' not present.')



def main():
    rclpy.init()
    node = AprilTagIDsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()