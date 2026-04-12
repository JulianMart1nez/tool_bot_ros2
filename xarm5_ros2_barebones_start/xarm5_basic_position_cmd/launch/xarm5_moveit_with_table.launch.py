#!/usr/bin/env python3
"""
xarm5_moveit_with_table.launch.py
---------------------------------
Single-command launch for the full xArm5 stack:
  - Driver + ros2_control + MoveIt2 (via xarm_moveit_config)
  - RViz2 with MoveIt plugin
  - Mounting table collision object auto-published to MoveIt
  - Auto-home: robot moves to home pose (all joints zero) on startup

Usage:
  ros2 launch xarm5_basic_position_cmd xarm5_moveit_with_table.launch.py \
    robot_ip:=192.168.1.234 add_gripper:=true

Startup sequence:
  t=0s:   MoveIt stack starts (driver, ros2_control, move_group, RViz)
  t=10s:  Table collision object added to planning scene
  t=15s:  Robot auto-homes to all-joints-zero pose
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        description='IP address of the xArm5 control box',
    )
    add_gripper_arg = DeclareLaunchArgument(
        'add_gripper',
        default_value='false',
        description='Whether to attach the xArm gripper in the model',
    )

    robot_ip = LaunchConfiguration('robot_ip')
    add_gripper = LaunchConfiguration('add_gripper')

    # 1. Bring up the full MoveIt + driver + RViz stack
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                'xarm5_moveit_realmove.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'add_gripper': add_gripper,
        }.items(),
    )

    # 2. Add table collision object (after MoveIt is ready)
    add_table = Node(
        package='xarm5_basic_position_cmd',
        executable='add_table_collision',
        name='add_table_collision',
        output='screen',
    )
    delayed_add_table = TimerAction(period=10.0, actions=[add_table])

    # 3. Auto-home the robot (after table is added, so collision is active)
    auto_home = Node(
        package='xarm5_basic_position_cmd',
        executable='auto_home',
        name='auto_home',
        output='screen',
    )
    delayed_auto_home = TimerAction(period=15.0, actions=[auto_home])

    return LaunchDescription([
        robot_ip_arg,
        add_gripper_arg,
        moveit_stack,
        delayed_add_table,
        delayed_auto_home,
    ])
