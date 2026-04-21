"""
depth_camera.launch.py — primary camera + perception launch (realsense2_camera flavor).

Starts:
  - realsense2_camera_node: D435i depth + color streams, with
    align_depth enabled so depth is resampled into the color image grid
    (needed by fine_localization to read depth at a tag's pixel location).
  - gripper_camera_tf: static TF link_tcp → camera_link. The rest of the
    camera TF tree (camera_link → camera_color_optical_frame, etc.) is
    published by the driver.
  - detect_zone: voice-driven bird's-eye → scan → hover → center sequence.
  - fine_localization: per-tag 6-DOF pose + depth sampled at the tag
    center (published on /fine_loc/tag_<id>/depth).
  - gripper_depth_monitor: legacy image-center depth monitor used by
    test_descent. Still handy for closed-loop depth experiments.

tool_approach is NOT launched here — run it in its own terminal so the
keyboard ↑/↓ trace-down interface has a live TTY:

    ros2 run xarm5_basic_position_cmd tool_approach

Usage:
  ros2 launch xarm5_basic_position_cmd depth_camera.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # link_tcp → camera_link mount calibration, exposed as launch args
    # so the values can be tweaked at runtime (no rebuild) while chasing
    # the "arm wanders" bug. Defaults match the in-situ measurement
    # described in the block comment on the static TF below.
    tcp_cam_x = DeclareLaunchArgument('tcp_cam_x', default_value='-0.06985')
    tcp_cam_y = DeclareLaunchArgument('tcp_cam_y', default_value='0.0')
    tcp_cam_z = DeclareLaunchArgument('tcp_cam_z', default_value='0.127')
    tcp_cam_roll = DeclareLaunchArgument('tcp_cam_roll', default_value='0.0')
    tcp_cam_pitch = DeclareLaunchArgument(
        'tcp_cam_pitch', default_value='-1.5707963')
    # Default yaw is π (180°). Earlier test runs with yaw=0 showed the
    # classic "chase/flee" pattern: reported_tag_xy ≈ 2·camera_xy − real_tag_xy,
    # which is the fingerprint of a 180° rotation about the camera's optical
    # axis in the link_tcp → camera_link mount transform. Override at launch
    # with tcp_cam_yaw:=0.0 if you rebuild the mount.
    tcp_cam_yaw = DeclareLaunchArgument('tcp_cam_yaw', default_value='3.14159')

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='depth_camera',
        namespace='gripper_cam',
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'depth_module.depth_profile': '640,480,30',
            'rgb_camera.color_profile': '640,480,30',
            # align_depth republishes depth resampled onto the color image
            # grid, so we can sample depth at a tag's (u,v) pixel from the
            # RGB detection directly. Published as
            # /gripper_cam/depth_camera/aligned_depth_to_color/image_raw
            'align_depth.enable': True,
            'decimation_filter.enable': True,
            'decimation_filter.filter_magnitude': 2,
            'temporal_filter.enable': True,
            'temporal_filter.filter_smooth_alpha': 0.4,
            'temporal_filter.filter_smooth_delta': 20.0,
        }],
        output='screen',
    )

    # Static transform: link_tcp → camera_link (RealSense's default root frame).
    # Translation: camera body is 2.75in forward, centered, 5in above TCP midpoint.
    # Rotation: RealSense `camera_link` convention is +x forward (out the lens),
    # +y left, +z up. The camera is mounted looking *down* relative to the TCP,
    # so camera_link's +x must point along link_tcp +z. Pitch = -π/2 about
    # link_tcp's +y axis achieves that:
    #   camera_link +x →  link_tcp +z   (lens points along TCP +z)
    #   camera_link +y →  link_tcp +y   (camera "left" = TCP +y)
    #   camera_link +z → -link_tcp +x   (camera "up"   = TCP -x)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_camera_tf',
        arguments=[
            '--x', LaunchConfiguration('tcp_cam_x'),
            '--y', LaunchConfiguration('tcp_cam_y'),
            '--z', LaunchConfiguration('tcp_cam_z'),
            '--roll', LaunchConfiguration('tcp_cam_roll'),
            '--pitch', LaunchConfiguration('tcp_cam_pitch'),
            '--yaw', LaunchConfiguration('tcp_cam_yaw'),
            '--frame-id', 'link_tcp',
            '--child-frame-id', 'camera_link',
        ],
    )

    depth_monitor = Node(
        package='xarm5_basic_position_cmd',
        executable='gripper_depth_monitor',
        name='gripper_depth_monitor',
        parameters=[{
            'center_roi_size': 40,
            'outer_roi_inner': 80,
            'outer_roi_outer': 160,
            'min_valid_pixels': 50,
            'publish_rate': 15.0,
        }],
        remappings=[
            ('depth_image', '/gripper_cam/depth_camera/depth/image_rect_raw'),
            ('camera_info', '/gripper_cam/depth_camera/depth/camera_info'),
        ],
        output='screen',
    )

    fine_loc = Node(
        package='xarm5_basic_position_cmd',
        executable='fine_localization',
        name='fine_localization',
        parameters=[{
            'enable_depth': True,
        }],
        remappings=[
            ('color_image', '/gripper_cam/depth_camera/color/image_raw'),
            ('color_info', '/gripper_cam/depth_camera/color/camera_info'),
            ('aligned_depth',
             '/gripper_cam/depth_camera/aligned_depth_to_color/image_raw'),
        ],
        output='screen',
    )

    detect_zone = Node(
        package='xarm5_basic_position_cmd',
        executable='detect_zone',
        name='detect_zone',
        output='screen',
    )

    return LaunchDescription([
        tcp_cam_x,
        tcp_cam_y,
        tcp_cam_z,
        tcp_cam_roll,
        tcp_cam_pitch,
        tcp_cam_yaw,
        realsense_node,
        camera_tf,
        depth_monitor,
        fine_loc,
        detect_zone,
    ])
