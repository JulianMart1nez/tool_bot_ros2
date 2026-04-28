#!/bin/bash
gnome-terminal --title="T2 Camera + detect_zone" -- bash -c '
source /opt/ros/jazzy/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/tool_bot_ros2/xarm5_ros2_barebones_start/install/setup.bash
ros2 launch xarm5_basic_position_cmd depth_camera.launch.py
exec bash'
