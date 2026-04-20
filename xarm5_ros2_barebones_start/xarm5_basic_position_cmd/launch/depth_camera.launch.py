"""
Launch RealSense D435i depth camera + gripper depth monitor node.

Starts the RealSense ROS 2 node with depth and color streams,
applies decimation + temporal filters, and publishes a static
transform from the gripper TCP to the camera frame.

Usage:
  ros2 launch xarm5_basic_position_cmd depth_camera.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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
            '--x', '-0.06985',    # camera body 2.75in along link_tcp -x
                                  # (when tool-down this maps to link_base +x,
                                  # i.e. away from the base — matches the
                                  # physical mounting where the lens faces
                                  # away from the base).
            '--y', '0.0',         # centered
            '--z', '0.127',       # 5in above TCP midpoint
            '--roll', '0.0',
            '--pitch', '-1.5707963',  # -π/2 → camera looks along TCP +z
            '--yaw', '0.0',
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
        remappings=[
            ('color_image', '/gripper_cam/depth_camera/color/image_raw'),
            ('color_info', '/gripper_cam/depth_camera/color/camera_info'),
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
        realsense_node,
        camera_tf,
        depth_monitor,
        fine_loc,
        detect_zone,
    ])
