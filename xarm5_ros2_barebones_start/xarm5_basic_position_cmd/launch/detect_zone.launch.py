"""
Detect-zone launch — single-camera (gripper-mounted RealSense D435i via V4L2).

Starts:
  - realsense_v4l2_pub: publishes RGB frames from /dev/videoN as
    /gripper_cam/depth_camera/color/image_raw + camera_info
  - detect_zone: subscribes to /voice_command/tool_request, runs the
    bird's-eye → scan → hover → center sequence

Use this when the `realsense2_camera` ROS driver is not installed
(depth streams are not available via V4L2 — only RGB).

Usage:
  ros2 launch xarm5_basic_position_cmd detect_zone.launch.py
  ros2 launch xarm5_basic_position_cmd detect_zone.launch.py camera_index:=10
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_index = LaunchConfiguration('camera_index')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')

    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='10'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='30.0'),

        Node(
            package='xarm5_basic_position_cmd',
            executable='realsense_v4l2_pub',
            name='realsense_v4l2_pub',
            parameters=[{
                'camera_index': camera_index,
                'width': width,
                'height': height,
                'fps': fps,
            }],
            output='screen',
        ),

        Node(
            package='xarm5_basic_position_cmd',
            executable='detect_zone',
            name='detect_zone',
            output='screen',
        ),
    ])
