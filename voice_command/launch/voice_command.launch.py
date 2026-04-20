"""
voice_command.launch.py
=======================
Launches the voice_command_node with configurable parameters.
Run with:
    ros2 launch voice_command voice_command.launch.py

Override parameters on the command line, e.g.:
    ros2 launch voice_command voice_command.launch.py language:=fr-FR energy_threshold:=400
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('voice_command')

    # ── Declare overridable arguments ──────────────────────────────────────
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'triggers.yaml'),
        description='Absolute path to the triggers YAML config file',
    )
    language_arg = DeclareLaunchArgument(
        'language',
        default_value='en-US',
        description='Speech recognition language tag (e.g. en-US, fr-FR)',
    )
    energy_arg = DeclareLaunchArgument(
        'energy_threshold',
        default_value='300',
        description='Microphone energy threshold (higher = less sensitive)',
    )

    # ── Node ───────────────────────────────────────────────────────────────
    voice_node = Node(
        package='voice_command',
        executable='voice_command_node',
        name='voice_command_node',
        output='screen',
        parameters=[{
            'config_file':      LaunchConfiguration('config_file'),
            'language':         LaunchConfiguration('language'),
            'energy_threshold': LaunchConfiguration('energy_threshold'),
        }],
    )

    return LaunchDescription([
        config_file_arg,
        language_arg,
        energy_arg,
        voice_node,
    ])
