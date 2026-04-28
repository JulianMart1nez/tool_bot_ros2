#!/bin/bash
gnome-terminal --title="T4 Go To (keyboard)" -- bash -c '
source /opt/ros/jazzy/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/tool_bot_ros2/xarm5_ros2_barebones_start/install/setup.bash
ros2 run xarm5_basic_position_cmd go_to
exec bash'
