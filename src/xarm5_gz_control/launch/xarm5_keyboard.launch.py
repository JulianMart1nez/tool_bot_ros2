#!/usr/bin/env python3
"""
xarm5_keyboard.launch.py
=========================
Launches the keyboard teleoperation node for joint-space control.
Publishes JointTrajectory directly — no MoveIt Servo required.

Run this in Terminal 2 AFTER xarm5_sim.launch.py is fully up
(wait until RViz2 appears and controllers are active).

Usage:
  ros2 launch xarm5_gz_control xarm5_keyboard.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    use_sim_time = (mode == 'sim')

    keyboard_node = Node(
        package='xarm5_gz_control',
        executable='keyboard_controller.py',
        name='xarm5_keyboard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'dof': 5}],
    )

    return [
        LogInfo(msg='\n[keyboard] Starting joint keyboard controller...'),
        LogInfo(msg='[keyboard] Make sure Terminal 1 is fully up first!\n'),
        keyboard_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            choices=['sim', 'real'],
            description='sim or real hardware mode',
        ),
        OpaqueFunction(function=launch_setup),
    ])
