#!/usr/bin/env python3
"""
xarm5_sim.launch.py
====================
Master launch file for the xArm5 Gazebo Harmonic + MoveIt2 simulation.

What it starts:
  1. robot_state_publisher  (with xacro-processed URDF)
  2. gz sim                 (Gazebo Harmonic, custom demo world)
  3. gz_ros_bridge          (clock + joint states bridge)
  4. spawn_entity           (drops the robot into Gazebo)
  5. joint_state_broadcaster spawner
  6. xarm5_traj_controller spawner
  7. xarm_gripper_traj_controller spawner
  8. move_group             (MoveIt2)
  9. rviz2                  (with MoveIt2 plugin)

Usage:
  # Simulation (default):
  ros2 launch xarm5_gz_control xarm5_sim.launch.py

  # Real hardware (requires xarm_controller to be built):
  ros2 launch xarm5_gz_control xarm5_sim.launch.py mode:=real robot_ip:=192.168.1.xxx
"""

import os
import tempfile
import yaml
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def run_xacro(xacro_file: str, mappings: dict) -> str:
    """Run xacro and return the resulting XML string."""
    cmd = ['xacro', xacro_file]
    for key, value in mappings.items():
        cmd.append(f'{key}:={value}')
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f'xacro failed on {xacro_file}:\n{result.stderr}'
        )
    return result.stdout


def write_params_file(params: dict) -> str:
    """Write a dict to a temp ROS2 parameter YAML file and return its path.
    ROS2 Jazzy requires the ros__parameters wrapper — flat format no longer works."""
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump({'/**': {'ros__parameters': params}}, tmp)
    tmp.close()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    # ── Resolve arguments ────────────────────────────────────────────
    mode     = LaunchConfiguration('mode').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)

    pkg_this       = get_package_share_directory('xarm5_gz_control')
    pkg_moveit_cfg = get_package_share_directory('xarm_moveit_config')

    use_sim_time = (mode == 'sim')

    # ── 1. Generate URDF + SRDF via xacro ───────────────────────────
    ros2_control_plugin = (
        'gz_ros2_control/GazeboSimSystem' if mode == 'sim'
        else 'uf_robot_hardware/UFRobotSystemHardware'
    )

    urdf_string = run_xacro(
        os.path.join(pkg_this, 'urdf', 'xarm5_gz.urdf.xacro'),
        {
            'ros2_control_plugin': ros2_control_plugin,
            'robot_ip': robot_ip if mode == 'real' else '',
        },
    )

    srdf_string = run_xacro(
        os.path.join(pkg_moveit_cfg, 'srdf', 'xarm.srdf.xacro'),
        {
            'prefix': '',
            'dof': '5',
            'robot_type': 'xarm',
            'add_gripper': 'true',
            'add_vacuum_gripper': 'false',
            'add_bio_gripper': 'false',
            'add_other_geometry': 'false',
        },
    )

    # ── 2. Write dynamic params to temp file ─────────────────────────
    # Passing large strings (URDF/SRDF) as Python dicts causes ROS2
    # parameter serialisation issues with nested structures.  Writing
    # them to a YAML file and passing the path is reliable.
    desc_params_file = write_params_file({
        'robot_description': urdf_string,
        'robot_description_semantic': srdf_string,
    })

    moveit_cfg_file  = os.path.join(pkg_this, 'config', 'moveit_config.yaml')
    controllers_file = os.path.join(pkg_this, 'config', 'ros2_controllers.yaml')

    # ── 3. robot_state_publisher ─────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': urdf_string},
            {'use_sim_time': use_sim_time},
        ],
    )

    nodes = [rsp_node]

    # ── SIM-SPECIFIC: Gazebo + controllers ───────────────────────────
    if mode == 'sim':
        world_file = os.path.join(pkg_this, 'worlds', 'xarm5_demo.sdf')

        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py',
                ])
            ),
            launch_arguments={
                'gz_args': (
                    f'-r -v 3 {world_file} '
                    '--physics-engine gz-physics-bullet-featherstone-plugin'
                ),
            }.items(),
        )

        gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{'use_sim_time': True}],
        )

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_xarm5',
            output='screen',
            arguments=[
                '-topic', '/robot_description',
                '-name',  'xarm5',
                '-x', '0.0', '-y', '0.0', '-z', '1.021',
                '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            ],
            parameters=[{'use_sim_time': True}],
        )

        jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='jsb_spawner',
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}],
        )

        arm_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='arm_spawner',
            output='screen',
            arguments=['xarm5_traj_controller', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}],
        )

        gripper_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='gripper_spawner',
            output='screen',
            arguments=['xarm_gripper_traj_controller', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}],
        )

        nodes += [
            RegisterEventHandler(
                OnProcessStart(target_action=rsp_node, on_start=[gz_sim, gz_bridge])
            ),
            RegisterEventHandler(
                OnProcessStart(target_action=rsp_node, on_start=[spawn_node])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_node, on_exit=[jsb_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner, gripper_spawner])
            ),
        ]

    # ── 4. Static TF: world → link_base ──────────────────────────────
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_base',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'link_base'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 5. MoveIt2 move_group ─────────────────────────────────────────
    # Parameters are passed as files — avoids nested-dict serialisation
    # failures that occur when merging YAML into a single Python dict.
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            desc_params_file,       # robot_description + robot_description_semantic
            {'use_sim_time': use_sim_time},
            moveit_cfg_file,        # all MoveIt config (kinematics, planners, controllers…)
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # ── 6. RViz2 with MoveIt plugin ───────────────────────────────────
    rviz_cfg = os.path.join(pkg_moveit_cfg, 'rviz', 'moveit.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[
            desc_params_file,       # robot_description + robot_description_semantic
            {'use_sim_time': use_sim_time},
            moveit_cfg_file,        # kinematics needed for MoveIt display plugin
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    nodes += [
        static_tf,
        TimerAction(period=5.0, actions=[move_group]),
        TimerAction(period=6.0, actions=[rviz2]),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            choices=['sim', 'real'],
            description='sim = Gazebo Harmonic | real = physical xArm5',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='',
            description='IP address of real xArm5 (only needed when mode:=real)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
