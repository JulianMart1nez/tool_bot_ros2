#!/bin/bash
gnome-terminal --title="T1 Robot Stack" -- bash -c '
source /opt/ros/jazzy/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/tool_bot_ros2/xarm5_ros2_barebones_start/install/setup.bash
ros2 launch xarm5_basic_position_cmd xarm5_moveit_with_table.launch.py robot_ip:=192.168.1.234 add_gripper:=true
exec bash'
