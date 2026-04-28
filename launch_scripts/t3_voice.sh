#!/bin/bash
gnome-terminal --title="T3 Voice Command" -- bash -c '
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run voice_command voice_command_node
exec bash'
