#!/bin/bash
gnome-terminal --title="T5 Perception View (annotated)" -- bash -c '
source /opt/ros/jazzy/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/tool_bot_ros2/xarm5_ros2_barebones_start/install/setup.bash
ros2 run rqt_image_view rqt_image_view /debug/overlay &
ros2 run xarm5_basic_position_cmd debug_overlay
exec bash'
