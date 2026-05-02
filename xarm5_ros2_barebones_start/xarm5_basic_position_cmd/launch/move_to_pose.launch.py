"""
move_to_pose.launch.py
----------------------
Launches the move_to_pose node with configurable target parameters.

Usage:
  ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py

Override any parameter on the command line:
  ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=250.0 z:=350.0

IMPORTANT: The xarm5 driver must already be running before launching this.
  ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=192.168.1.234

Units: x/y/z in mm, roll/pitch/yaw in degrees, speed in mm/s, acc in mm/s^2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -----------------------------------------------------------------------
    # Declare launch arguments so they can be overridden from the command line.
    # These map directly to the node parameters.
    # -----------------------------------------------------------------------
    args = [
        DeclareLaunchArgument('x',     default_value='300.0',
                              description='Target X position in mm'),
        DeclareLaunchArgument('y',     default_value='0.0',
                              description='Target Y position in mm'),
        DeclareLaunchArgument('z',     default_value='300.0',
                              description='Target Z position in mm'),
        DeclareLaunchArgument('roll',  default_value='180.0',
                              description='Target roll in degrees'),
        DeclareLaunchArgument('pitch', default_value='0.0',
                              description='Target pitch in degrees'),
        DeclareLaunchArgument('yaw',   default_value='0.0',
                              description='Target yaw in degrees'),
        DeclareLaunchArgument('speed', default_value='50.0',
                              description='Move speed in mm/s'),
        DeclareLaunchArgument('acc',   default_value='500.0',
                              description='Move acceleration in mm/s^2'),
    ]

    # -----------------------------------------------------------------------
    # The move_to_pose node.
    # Parameters are passed in from launch arguments above.
    # -----------------------------------------------------------------------
    move_node = Node(
        package='xarm5_basic_position_cmd',
        executable='move_to_pose',
        name='move_to_pose',
        output='screen',
        parameters=[{
            'x':     LaunchConfiguration('x'),
            'y':     LaunchConfiguration('y'),
            'z':     LaunchConfiguration('z'),
            'roll':  LaunchConfiguration('roll'),
            'pitch': LaunchConfiguration('pitch'),
            'yaw':   LaunchConfiguration('yaw'),
            'speed': LaunchConfiguration('speed'),
            'acc':   LaunchConfiguration('acc'),
        }],
    )

    return LaunchDescription(args + [move_node])
