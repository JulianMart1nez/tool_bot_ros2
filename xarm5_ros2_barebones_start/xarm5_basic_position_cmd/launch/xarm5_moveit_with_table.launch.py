#!/usr/bin/env python3
"""
xarm5_moveit_with_table.launch.py
---------------------------------
Single-command launch for the full xArm5 stack:
  - Driver + ros2_control + MoveIt2 (via xarm_moveit_config)
  - RViz2 with MoveIt plugin
  - Mounting table collision object auto-published to MoveIt

This wraps the official `xarm5_moveit_realmove.launch.py` and chains in
our `add_table_collision` node so the table appears in the planning scene
without any manual step.

Usage:
  ros2 launch xarm5_basic_position_cmd xarm5_moveit_with_table.launch.py \
    robot_ip:=192.168.1.234

Arguments:
  robot_ip       (required) - IP of the xArm5 control box
  add_gripper    (default: false)

How it works:
  1. IncludeLaunchDescription brings up the full MoveIt stack
  2. TimerAction waits 10s for move_group to come up and expose
     /apply_planning_scene
  3. Then launches the add_table_collision node (one-shot: runs, adds
     the table, exits)

If /apply_planning_scene isn't ready in time, just bump the delay.
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

    # 2. After MoveIt is up, run the table adder (one-shot node)
    add_table = Node(
        package='xarm5_basic_position_cmd',
        executable='add_table_collision',
        name='add_table_collision',
        output='screen',
    )

    # 3. Delay the table adder so move_group is ready to serve
    #    /apply_planning_scene. 10 seconds is conservative.
    delayed_add_table = TimerAction(
        period=10.0,
        actions=[add_table],
    )

    return LaunchDescription([
        robot_ip_arg,
        add_gripper_arg,
        moveit_stack,
        delayed_add_table,
    ])
