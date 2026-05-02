"""
depth_camera.launch.py — primary camera + perception launch (realsense2_camera flavor).

Starts:
  - realsense2_camera_node: D435i depth + color streams, with
    align_depth enabled so depth is resampled into the color image grid
    (needed by fine_localization to read depth at a tag's pixel location).
  - gripper_camera_tf: static TF link_tcp → camera_link. The rest of the
    camera TF tree (camera_link → camera_color_optical_frame, etc.) is
    published by the driver.

    Note: tried switching to UFactory's official Camera Stand
    calibration (link_eef parent + EULER_EEF_TO_COLOR_OPT translation
    from xArm-Developer/ufactory_vision) on 2026-04-29. The composition
    is mathematically correct (same Rz·Ry·Rx convention) but the
    reported tag XY landed past the xArm5's reach envelope — empirical
    evidence that this rig's mount geometry diverges from UFactory's
    standard Camera Stand spec (likely camera-body orientation in the
    bracket or an in-stand calibration drift). Reverted to the
    empirically-tuned values that previously delivered successful
    descent at z=-69 mm. If the rig is later re-calibrated via
    easy_handeye2, replace these values with the calibrated EULER.
  - detect_zone: voice-driven bird's-eye → scan → hover → center sequence.
  - fine_localization: per-tag 6-DOF pose + depth sampled at the tag
    center (published on /fine_loc/tag_<id>/depth).
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
    # so the values can be tweaked at runtime (no rebuild).
    tcp_cam_x = DeclareLaunchArgument('tcp_cam_x', default_value='-0.06985')
    tcp_cam_y = DeclareLaunchArgument('tcp_cam_y', default_value='0.0')
    tcp_cam_z = DeclareLaunchArgument('tcp_cam_z', default_value='0.127')
    tcp_cam_roll = DeclareLaunchArgument('tcp_cam_roll', default_value='0.0')
    tcp_cam_pitch = DeclareLaunchArgument(
        'tcp_cam_pitch', default_value='-1.5707963')
    # Yaw=π was empirically selected to fix a "chase/flee" pattern
    # observed with yaw=0. Don't change without re-validating the
    # full TF chain on hardware.
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

    # Static transform: link_tcp → camera_link.
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
        fine_loc,
        detect_zone,
    ])
