#!/bin/bash
gnome-terminal --title="T7 RGB + Depth Feeds + Center Distance" -- bash -c '
source /opt/ros/jazzy/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/tool_bot_ros2/xarm5_ros2_barebones_start/install/setup.bash

RGB=/gripper_cam/depth_camera/color/image_raw
DEPTH=/gripper_cam/depth_camera/aligned_depth_to_color/image_raw

ros2 run rqt_image_view rqt_image_view "$RGB" &
ros2 run rqt_image_view rqt_image_view "$DEPTH" &

# Center-pixel distance in mm/m, ~5 Hz, no install needed.
python3 - <<PY
import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
class C(Node):
    def __init__(self):
        super().__init__("center_depth_probe")
        self.create_subscription(Image,
            "/gripper_cam/depth_camera/aligned_depth_to_color/image_raw",
            self.cb, 5)
    def cb(self, m):
        a = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
        cy, cx = m.height // 2, m.width // 2
        roi = a[cy-5:cy+6, cx-5:cx+6]
        roi = roi[roi > 0]
        if roi.size < 5:
            print("center depth: --- (no return)"); return
        mm = float(np.median(roi))
        print(f"center depth: {mm:7.1f} mm  =  {mm/1000.0:5.3f} m")
rclpy.init(); n = C()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
finally: n.destroy_node(); rclpy.shutdown()
PY

exec bash'
