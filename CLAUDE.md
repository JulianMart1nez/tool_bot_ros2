# CLAUDE.md — xArm5 tool_bot_ros2 project

This file orients Claude on how to work in this repo. Read it first.

## What this project is

Autonomous tool pick-and-place on a UFactory xArm5 (5-DOF) + UFACTORY G2 gripper, ROS 2 Jazzy on Ubuntu 24.04. Tools are detected via AprilTags; trajectories are planned with TRAC-IK + MoveIt 2 / OMPL.

- Robot IP: `192.168.1.234`
- Driver workspace: `~/xarm_ws` (official `xarm_ros2`, NOT in this git)
- App workspace (this repo): `~/ros2_ws/src/tool_bot_ros2`
- Active branch: `julian_tracik_integration`

## The two canonical launches

**Real-robot stack (always this — never `*_fake.launch.py`):**
```
ros2 launch xarm5_basic_position_cmd xarm5_moveit_with_table.launch.py robot_ip:=192.168.1.234 add_gripper:=true
```
Brings up: `ufactory_driver`, `ros2_control` + `xarm5_traj_controller`, MoveIt 2, RViz, table+carts collision objects, auto-home.

**Gripper-mounted depth camera + fine localization (separate terminal):**
```
ros2 launch xarm5_basic_position_cmd depth_camera.launch.py
```
Brings up: RealSense D435i, static TF (`link_tcp → camera_link`), `gripper_depth_monitor`, `fine_localization`.

## Phase status (2026-04-15)

| Phase | Status | Notes |
|-------|--------|-------|
| 1 — Cartesian validation | done | 8/8 poses |
| 2 — MoveIt2 + table collision | done | |
| 3 — TRAC-IK + combined launch | done | |
| 4 — Reachability map | done | 97.3% reachable |
| 5 — Grasp pose generator | done | |
| 6 — Gripper integration | done | UFACTORY G2, drive_joint 0=open, 0.85=closed |
| 7 — Closed-loop descent in grasp_pose_generator | done | |
| 8 — AprilTag perception (overhead webcam) | done | Procrustes-calibrated |
| 8b — Gripper depth-camera fine localization + descent test | **in progress, broken** | Direction logic wrong; bookmarked, rework with user |

## Phase 8b — what exists and what's broken

Files (all in `xarm5_ros2_barebones_start/xarm5_basic_position_cmd/`):
- `launch/depth_camera.launch.py` — RealSense + static TF + depth monitor + fine_localization
- `xarm5_basic_position_cmd/gripper_depth_monitor.py` — center-ROI tool distance, outer-ROI cart distance from `depth/image_rect_raw`
- `xarm5_basic_position_cmd/fine_localization.py` — continuous tag36h11 detection, transformed via TF chain to `link_base`, published on `/fine_loc/result` (PoseStamped) and `/fine_loc/tag_detected` (Bool). `TAG_SIZE = 0.025` (25mm physical).
- `xarm5_basic_position_cmd/test_descent.py` — manual: open gripper, fine-loc, staged refinement to tag XY, closed-loop descent with depth feedback + open-loop fallback below D435i min range.

Static TF (depth_camera.launch.py:50): `link_tcp → camera_link`, x=−0.06985, y=0, z=0.127, pitch=−π/2.

**Known broken:** Live descent test drives the gripper in the *opposite* direction of the AprilTag, then drifts away during staged 20mm hops. The static-TF sign and rotation conventions need a fresh derivation with the user. **Do not iterate sign-by-sign on live runs.** See `.claude/skills/descent-test-debugging.md` for the full lessons learned.

## Architectural conventions worth remembering

- **IK-first planning.** OMPL cannot sample valid goal states from raw Cartesian constraints on a 5-DOF arm. Always call `/compute_ik` (TRAC-IK) first, then plan in joint space against the returned joint values. This matches RViz's internal flow.
- **5-DOF wrist yaw constraint.** At any (x, y), wrist yaw must equal `atan2(y, x)`. Use `_tool_aligned_quat(x, y)` (in `test_descent.py:152`) for every Cartesian goal — RPY = (π, 0, atan2(y, x)). Reusing the current TCP quat for a pose at a different XY produces sporadic NO_IK_SOLUTION (error_code −31) failures.
- **End-effector link.** With gripper attached: use `link_tcp` (172mm below `link_eef`). Without: use `link_eef`.
- **Disabled `avoid_collisions` in IK requests.** Necessary on this 5-DOF arm; MoveGroup planner still does collision checking. There is an inline comment — preserve it.
- **TF chain for depth perception:** `link_base → … → link_tcp → camera_link → camera_color_optical_frame → tag pose`. The static TF link is `camera_link` (RealSense's default root) — anything else (e.g. `gripper_camera_link`) leaves the trees disconnected and `do_transform_point` silently fails.
- **RealSense optical frame:** `+x right (image), +y down, +z forward (depth axis)`. Don't conflate with `camera_link` (`+x forward, +y left, +z up`).

## Hardware / physical constants

- D435i depth min range ~190mm. Below `tcp_above_tool < 60mm`, switch to open-loop descent.
- Safety z floor: −0.085m in `link_base`.
- Tool tag IDs (tag36h11, **25mm** — different from the 2in workspace tags!): hammer=3, phillips=2, flathead=4. Pickup corners: 10/12/16/24. Dropoff corners: 15/17/18/19. Gripper tag: 23.
- Pickup cart: LEFT (+y), 24in × 18in, near edge 18in from base. Dropoff cart: RIGHT (−y), 34in × 17.5in, near edge 18in from base, 1.5in rim. Cart surfaces at z ≈ −2.35in below base. Cart height: 31in.

## Working style with Julian

- Execute commands directly, don't hand the user instructions to run manually.
- Commit and push to GitHub as work completes (Julian's explicit preference).
- Confirm physical assumptions (tag size, mounting orientation) **before** live runs, not after.
- Live-robot tests are the most expensive feedback loop in this project. Use static checks (TF echo, topic echo) first.
- Between robot-stack relaunches, kill stale `robot_state_publisher` / `move_group` / `controller_manager` / `gripper_camera_tf` processes — leftovers conflict on TFs and topics.

## Pointers to deeper context

- `.claude/skills/descent-test-debugging.md` — full lessons-learned for the gripper depth camera + descent test (mistakes made, what worked, what to do next time).
- `REACHABILITY_MAP.txt` — Phase 4 workspace coverage results.
- The auto-memory at `~/.claude/projects/-home-julian-ros2-ws-src-tool-bot-ros2/memory/` carries cross-session context indexed in `MEMORY.md`.
