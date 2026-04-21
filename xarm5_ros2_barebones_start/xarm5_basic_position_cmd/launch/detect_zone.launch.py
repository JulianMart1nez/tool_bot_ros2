"""
Detect-zone launch — single-camera (gripper-mounted RealSense D435i via V4L2).

Starts:
  - realsense_v4l2_pub: publishes RGB frames from /dev/videoN as
    /gripper_cam/depth_camera/color/image_raw + camera_info
  - detect_zone: subscribes to /voice_command/tool_request, runs the
    bird's-eye → scan → hover → center sequence, publishes
    /detect_zone/complete on success.
  - fine_localization: continuously detects tool AprilTags and publishes
    per-tag 6-DOF pose in link_base (/fine_loc/result, /fine_loc/tag_<id>).
    Needed by tool_approach (Phase 10a).

tool_approach is NOT launched here — run it in its own terminal so the
keyboard ↑/↓ trace-down interface has a live TTY:

    ros2 run xarm5_basic_position_cmd tool_approach

Use this launch when the `realsense2_camera` ROS driver is not installed
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

        # Static TF: link_tcp → camera_link (mirrors depth_camera.launch.py).
        # Without the realsense2_camera driver we must publish this ourselves.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gripper_camera_tf',
            arguments=[
                '--x', '-0.06985',
                '--y', '0.0',
                '--z', '0.127',
                '--roll', '0.0',
                '--pitch', '-1.5707963',
                # yaw=π matches depth_camera.launch.py default: earlier test
                # runs with yaw=0 showed the classic "chase/flee" pattern
                # (reported_tag ≈ 2·camera − real_tag), fingerprint of a
                # 180° rotation about the camera's optical axis.
                '--yaw', '3.14159',
                '--frame-id', 'link_tcp',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # Static TF: camera_link → camera_color_optical_frame.
        # REP-103 body frame → REP-104 optical frame; normally published by
        # the realsense2_camera driver. Origin coincident (approximation —
        # the real sensor offset is a few mm and irrelevant here).
        # quat (x,y,z,w) = (-0.5, 0.5, -0.5, 0.5) ≡ RPY (-π/2, 0, -π/2).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_color_optical_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_color_optical_frame',
            ],
        ),

        Node(
            package='xarm5_basic_position_cmd',
            executable='detect_zone',
            name='detect_zone',
            output='screen',
        ),

        Node(
            package='xarm5_basic_position_cmd',
            executable='fine_localization',
            name='fine_localization',
            remappings=[
                ('color_image', '/gripper_cam/depth_camera/color/image_raw'),
                ('color_info', '/gripper_cam/depth_camera/color/camera_info'),
            ],
            output='screen',
        ),
    ])
