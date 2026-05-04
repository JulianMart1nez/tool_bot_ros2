# START — bring the system up

Autonomous tool pick-and-place on a UFactory xArm5 (5-DOF) + UFACTORY G2 gripper, ROS 2 Jazzy on Ubuntu 24.04.

- Robot IP: `192.168.1.234`
- Driver workspace: `~/xarm_ws` (official `xarm_ros2`, NOT in this repo)
- App workspace (this repo): `~/tool_bot_ros2`
- Voice-command package: `~/tool_bot_ros2/voice_command` (symlinked into `~/ros2_ws/src/voice_command`)
- Active branch: `ROS2_toolbot_master` / `main`. Historical work is preserved in the `legacy` branch.

## Build

```
cd ~/tool_bot_ros2/xarm5_ros2_barebones_start
colcon build --packages-select xarm5_basic_position_cmd --symlink-install
```

## Launch (one terminal each — desktop shortcuts in `~/Desktop/t*.desktop`)

| # | Script | What it brings up |
|---|---|---|
| T1 | `launch_scripts/t1_robot.sh` | MoveIt 2, controllers, collision scene (`xarm5_moveit_with_table.launch.py`) |
| T2 | `launch_scripts/t2_camera.sh` | RealSense, `fine_localization`, `detect_zone` (`depth_camera.launch.py`) |
| T3 | `launch_scripts/t3_voice.sh` | `voice_command_node` (laptop mic) |
| T4 | `launch_scripts/t4_auto.sh` | Fully autonomous orchestrator (`auto_go_to`) — voice→grab→dropoff→home |
| — | `launch_scripts/t4_approach.sh` | **Or** keyboard variant (`go_to`) — manual ↓/↑/g/o control |
| T5 | `launch_scripts/t5_camera_view.sh` | `debug_overlay` + `rqt_image_view /debug/overlay` |
| T6 | `launch_scripts/t6_arm_monitor.sh` | Joint-state stream |
| T7 | `launch_scripts/t7_camera_feeds.sh` | RGB + aligned-depth viewer + center-pixel depth probe |

Use **either** T4 form, not both. Bring up T1 → T2 → T3 → T4 in that order.

## Operate

Speak `"give me a hammer"` (or `phillips` / `flathead`) into the laptop mic. With T4 = `t4_auto.sh`, the arm runs the entire pickup → transit → dropoff → home cycle on its own. With T4 = `t4_approach.sh`, drive the descent yourself with `↓`/`↑`/`s` (cycle step) and `g` to close the gripper.

Voice `"go home"` aborts whatever the arm is doing and moves to the all-zero joint pose.

## Recovery cheats

- xArm dropped to `state:2` (red LED, no error code) — collision-sensitivity false-trip:
  ```
  ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
  ros2 service call /xarm/set_mode  xarm_msgs/srv/SetInt16 "{data: 1}"
  ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
  ```
- Test home without voice:
  ```
  ros2 topic pub --once /voice_command/home_request std_msgs/String "{data: cli}"
  ```
- Between robot-stack relaunches, kill stale processes:
  ```
  pkill -f 'move_group|controller_manager|robot_state_publisher|realsense2_camera|fine_localization|detect_zone'
  ```

## Where to look next

- `docs/notes/CLAUDE.md` — full project context, architectural conventions, current phase status.
- `docs/system_snapshot/` — captured ROS graph + TF tree from a live run.
