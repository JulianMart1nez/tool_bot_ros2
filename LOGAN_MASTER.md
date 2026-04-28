# LOGAN_MASTER branch — Claude context

This branch is Logan's working snapshot of the xArm5 tool-bot. It carries
everything from `julian_tracik_integration` plus the local launch helper
scripts that previously lived only on `~/Desktop`.

Authoritative project context lives in `CLAUDE.md`. Read that first.
This file is just the branch-specific orientation.

## What's new on this branch

- `launch_scripts/t1_robot.sh` … `t7_camera_feeds.sh` — the gnome-terminal
  launchers that bring up the canonical six/seven-terminal stack. They
  used to be tracked only on `~/Desktop`; copying them here means a
  fresh clone gets the full launch sequence with no manual setup.
- `xarm5_ros2_barebones_start/xarm5_basic_position_cmd/xarm5_basic_position_cmd/go_to.py`
  — closed-loop visualization (publishes `/go_to/target_pose` +
  `/go_to/viz` MarkerArray instead of auto-moving) plus the manual-grab
  keyboard additions described below.

## Launching the stack

The scripts assume `~/xarm_ws` and `~/tool_bot_ros2/xarm5_ros2_barebones_start`
are built. Run each in its own gnome-terminal — the order matters:

```
launch_scripts/t1_robot.sh        # MoveIt 2 + xArm driver + table collision
launch_scripts/t2_camera.sh       # realsense2_camera + detect_zone + fine_loc
launch_scripts/t3_voice.sh        # voice_command_node (laptop mic)
launch_scripts/t4_approach.sh     # go_to (keyboard-driven)  ← TTY required
launch_scripts/t5_camera_view.sh  # rqt_image_view of /debug/overlay
launch_scripts/t6_arm_monitor.sh  # arm_pose_monitor logging
launch_scripts/t7_camera_feeds.sh # RGB + aligned_depth viewers + center-pixel depth probe
```

T4 must be its own terminal so the `go_to` keyboard reader has a live
TTY. T1's MoveIt launch auto-starts RViz.

## Manual-grab keyboard workflow (Phase 10b → 10c bridge)

`go_to.py` is now closed-loop on visualization but still operator-driven
on motion. After `detect_zone` hands off, the keyboard controls inside
T4 are:

| Key | Action |
|-----|--------|
| ↓ / Enter | descend by current step size, commit move |
| ↑ | raise by current step size, commit move |
| **s** | cycle approach step (10 mm → 5 mm → 1 mm → wraps) |
| **g** | CLOSE gripper via `/xarm_gripper/gripper_action` |
| **o** | OPEN gripper (release / reset between attempts) |
| m | re-commit move to currently displayed target (no z change) |
| x | toggle target mode (camera ↔ tcp) |
| space | pause/resume target publishing |
| h | send arm home (all-zero joints) |
| q | quit |

Suggested approach for safe, repeatable tool-pickup calibration:

1. ↓ at 10 mm until visibly close to the tool.
2. `s` → 5 mm, refine.
3. `s` → 1 mm, dial in final placement.
4. `g` to close the gripper.
5. `o` to release before the next attempt.

Once a manual sequence reliably grabs each tool, the same step-down
pattern can be promoted into autonomous logic for Phase 10c.

## RViz hookup

To see the live target while keyboard-positioning:

- Add a **Pose** display, topic `/go_to/target_pose`.
- Add a **MarkerArray** display, topic `/go_to/viz`.
  - id 0 (green sphere) = projected target TCP
  - id 1 (yellow cube)  = detected AprilTag
  - id 2 (blue arrow)   = approach axis (target TCP → tag center)

The arrow is purely visual — do not infer scale from it; if the tag
depth from `fine_localization` is wrong (a known open issue per
`CLAUDE.md` Phase 10b notes), the arrow will look exaggerated.
