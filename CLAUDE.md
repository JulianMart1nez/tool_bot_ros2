# CLAUDE.md — xArm5 tool_bot_ros2 project

This file orients Claude on how to work in this repo. Read it first.

## What this project is

Autonomous tool pick-and-place on a UFactory xArm5 (5-DOF) + UFACTORY G2 gripper, ROS 2 Jazzy on Ubuntu 24.04. Tools are detected via AprilTags; trajectories are planned with TRAC-IK + MoveIt 2 / OMPL.

- Robot IP: `192.168.1.234`
- Driver workspace: `~/xarm_ws` (official `xarm_ros2`, NOT in this repo)
- App workspace (this repo): `~/tool_bot_ros2`
- Voice-command package: `~/tool_bot_ros2/voice_command` (symlinked into `~/ros2_ws/src/voice_command` so `~/ros2_ws` still builds it)
- Active branch: `julian_tracik_integration`

## Architecture: single-camera, voice-driven

The project has moved to a **single-camera architecture**. The overhead/scene USB webcam is no longer used for anything. All perception happens through the Intel RealSense D435i mounted on the gripper. Flow:

1. User speaks a phrase like `"give me a hammer"` into the laptop microphone.
2. `voice_command_node` transcribes (Google STT) and, when both a command phrase ("give me", "hand me", "grab", …) and a tool name appear in-order, publishes `std_msgs/String` on `/voice_command/tool_request` formatted as `"phrase=…|tool=…|fiducial=…"`.
3. `detect_zone` receives the request and runs a scripted sequence:
   - **Bird's-eye pose** — tall joint-space pose so both pickup and dropoff zones are visible.
   - **Scan** — detect black-square-on-white-paper zone markers via morphological top-hat + ring/center contrast validation; group them into zones (k-means k=2 when ≥6 markers, single zone otherwise); label the wider one PICKUP.
   - **Hover pose** — joint-space move to hover over the pickup zone.
   - **Frame-center adjustment** — iteratively tweak J1 so the pickup centroid lands within 40 px of the image center.
4. Fine AprilTag localization and descent happen next (still in progress — see "Next big step").

### The four canonical launches

**Robot stack (MoveIt 2 + controllers + collision scene):**
```
ros2 launch xarm5_basic_position_cmd xarm5_moveit_with_table.launch.py robot_ip:=192.168.1.234 add_gripper:=true
```

**Camera + detect_zone + fine_localization (PRIMARY — uses installed `realsense2_camera` driver, depth enabled):**
```
ros2 launch xarm5_basic_position_cmd depth_camera.launch.py
```
Brings up `realsense2_camera` (RGB + depth + aligned_depth_to_color @ 640×480), static TF `link_tcp → camera_link`, `gripper_depth_monitor`, `fine_localization` (depth sampling ON) and `detect_zone`.

**Camera + detect_zone (V4L2 fallback — RGB only, if realsense driver misbehaves):**
```
ros2 launch xarm5_basic_position_cmd detect_zone.launch.py
```
Brings up `realsense_v4l2_pub` (RGB only via `/dev/video10`) + the two static TFs + `detect_zone` + `fine_localization` (depth sampling OFF). Use this if the realsense2_camera node fails to start.

**Voice commands (laptop microphone):**
```
ros2 run voice_command voice_command_node
```

**Tool approach (Phase 10a — align + trace-down, keyboard-driven):**
```
ros2 run xarm5_basic_position_cmd tool_approach
```
Run in its own terminal so the ↑/↓ keys can drive the descent. Auto-starts on `/detect_zone/complete`.

## Hardware / physical constants

- **Camera:** Intel RealSense D435i, mounted on gripper. Without `realsense2_camera`, accessed as V4L2 device `/dev/video10` (RGB only, no depth). D435i depth min range ≈ 190 mm when depth is available.
- **Gripper:** UFACTORY G2, `drive_joint` 0 = open, 0.85 = closed.
- **End-effector link:** `link_tcp` (172 mm below `link_eef`) with gripper attached.
- **Safety z floor:** −0.085 m in `link_base`.
- **Tool AprilTags** (tag36h11, **25.4 mm (1 in)**): hammer=3, phillips=2, flathead=4. Gripper tag: **22**.
- **Zone markers:** black squares on white paper (1in square, on white letter-size sheets). No AprilTags on zone corners. Detected by morphological top-hat + ring/center contrast (see `detect_zone.py:detect_zone_markers`).
- **Zone dimensions:** Pickup 14.5 in × 15.5 in (wider). Dropoff 9.0 in × 15.5 in (narrower). In camera frame at bird's-eye: pickup = RIGHT, dropoff = LEFT.
- **Carts:** Pickup cart LEFT (+y), 24 in × 18 in, near edge 18 in from base. Dropoff cart RIGHT (−y), 34 in × 17.5 in, near edge 18 in from base, 1.5 in rim. Cart surfaces at z ≈ −2.35 in below base. Cart height: 31 in. *(Cart surface z will be re-measured by user.)*

## Key joint poses (degrees → radians in code)

| Pose | J1 | J2 | J3 | J4 | J5 |
|------|----|----|----|----|----|
| Detect Zone (bird's-eye) | 0 | 0 | −160 | 145 | 0 |
| Hover over Pickup | 23 | 23 | −130 | 107 | 23 |

Defined in `detect_zone.py:DETECT_ZONE_JOINTS` / `HOVER_PICKUP_JOINTS`.

## Phase status (2026-04-19)

| Phase | Status | Notes |
|-------|--------|-------|
| 1 — Cartesian validation | done | 8/8 poses |
| 2 — MoveIt2 + table collision | done | |
| 3 — TRAC-IK + combined launch | done | |
| 4 — Reachability map | done | 97.3% reachable |
| 5 — Grasp pose generator | done | |
| 6 — Gripper integration | done | UFACTORY G2 |
| 7 — Closed-loop descent | partial | rework bookmarked — see Phase 8b |
| 8 — Overhead webcam AprilTag perception | **retired** | replaced by single-camera detect_zone |
| 9 — Voice → detect_zone → hover → center | done | end-to-end tested on real hardware |
| 10a — AprilTag align + keyboard trace-down | **done (this merge)** | `tool_approach.py`; no autonomous stop, no grasp yet |
| 10b — Autonomous stop (depth + pixel-size) + grasp | next | needs `ros-jazzy-realsense2-camera` |
| 11 — Transfer to dropoff zone | future | |

## Phase 9 — what exists now

Files (under `xarm5_ros2_barebones_start/xarm5_basic_position_cmd/xarm5_basic_position_cmd/`):

- `detect_zone.py` — the orchestrator. Subscribes to `/voice_command/tool_request` and `/gripper_cam/depth_camera/color/image_raw`, drives the bird's-eye → scan → hover → center sequence via MoveGroup joint-space planning. Saves per-step debug JPEGs (raw + annotated) under `/tmp/detect_zone_debug/<timestamp>_<tool>/`.
- `realsense_v4l2_pub.py` — V4L2 fallback publisher. Opens the RealSense RGB stream via OpenCV, publishes `/gripper_cam/depth_camera/color/image_raw` and `/gripper_cam/depth_camera/color/camera_info` (with D435i factory intrinsics). Locks exposure + white balance to kill RGB flicker.
- `apriltag_perception.py` — retained, but its overhead-camera Procrustes calibration was removed. The tool-tag detection pipeline still runs from the gripper camera; it feeds downstream localization.
- `launch/detect_zone.launch.py` — brings up `realsense_v4l2_pub` + `detect_zone` together.

Voice-command package (now in this repo):

- `voice_command/voice_command/voice_command_node.py` — uses `get_package_share_directory('voice_command')` to find `config/triggers.yaml` (previously used a broken `../..` path).
- `voice_command/config/triggers.yaml` — command phrases + `{hammer:3, phillips:2, flathead:4}` mapping.

Standalone debug tools (not nodes):

- `realsense_preview.py` (repo root) — live viewer with the same zone-marker detection + AprilTag overlay. Useful for tuning without the full ROS stack.
- `live_tag_viewer.py` (repo root) — older AprilTag-only viewer.

### Where debug images land

Every run of `detect_zone` creates `/tmp/detect_zone_debug/<YYYYMMDD_HHMMSS>_<tool>/` and saves step-indexed `*_raw.jpg` + `*_annotated.jpg` pairs:

- `01_birdseye_arrived` — after reaching detect pose
- `02_birdseye_scan_hit` / `_timeout` — during the initial scan
- `03_hover_arrived` — after reaching hover pose
- `04…_center_iterN_scan_…` / `_adjust` / `_DONE` / `_LOST` — per centering iteration
- `*_FAIL_*` — any failure path

Annotations: white crosshair at image center, yellow outlines on detected markers, green hull for PICKUP / orange for DROPOFF, offset in px for centering iterations. First place to look when a run misbehaves.

## Phase 10a — what just landed

- `fine_localization.py` now publishes **full 6-DOF pose** in `link_base` (not just position). Orientation comes from dt_apriltags' solvePnP, so `Rotate(quat) @ [0,0,1] = tag surface normal in link_base`. Tag size updated to 25.4 mm (1 in).
- New per-tag topics `/fine_loc/tag_2`, `/fine_loc/tag_3`, `/fine_loc/tag_4` (PoseStamped). `/fine_loc/result` still carries the closest tag for backward compatibility.
- `fine_localization.py` also publishes two distance-proxy topics per tag:
  - `/fine_loc/tag_<id>/pixel_size` (`std_msgs/Float32`) — mean edge length in pixels from `tag.corners`. RGB-only, always available.
  - `/fine_loc/tag_<id>/depth` (`std_msgs/Float32`) — depth at the tag center sampled from `aligned_depth_to_color` (3-px median, handles both 16UC1 mm and 32FC1 m). Only published when `enable_depth:=true` (depth_camera.launch.py sets this; V4L2 fallback does not).
- `detect_zone.py` now publishes `/detect_zone/complete` (`std_msgs/String`, payload `tool=…|fiducial=…|status=ok|degraded`) on center success.
- New node `tool_approach.py` (`ros2 run xarm5_basic_position_cmd tool_approach`):
  - Auto-starts align mode when it sees `/detect_zone/complete` matching a known fiducial; can also be driven manually via `/tool_approach/cmd` (`start <fid>`, `stop`, `descend [mm]`, `raise [mm]`, `set <mm>`) or keyboard (↑/↓ = raise/descend by 10 mm, space = pause, `q` = quit).
  - Each tick: reads `/fine_loc/tag_<id>`, computes the tag normal, enforces a flat-tag sanity check (>20° tilt → skip with warning), places `link_tcp` along the normal at `standoff_m` above the tag center, plans IK + joint-space move. Safety floor `z = -0.085 m` clamped before every move.
  - Keyboard is the trace-down interface: each ↓ lowers `standoff_m` by 10 mm so the next align tick commands the arm closer to the tag. Standoff is clamped to 50–300 mm.
  - **Two-stage distance authority**: `_distance_mode()` picks between `FAR[pixel]` (RGB pinhole heuristic `d ≈ fx * tag_size / pixel_size`, fx=615) and `CLOSE[depth]` (D435i aligned depth). The switch fires when pixel size ≥ `depth_activation_pixel_size_px` (default 70 px ≈ 225 mm for a 25.4 mm tag) AND a depth sample arrived in the last 2 s. Below threshold we stay on the RGB heuristic — depth is unreliable outside the D435i's ≈190 mm min range. If depth drops out after activation, mode logs as `CLOSE[pixel-fallback]`. Status tick (1 Hz) prints all three numbers on one line for live tuning.
  - Params on `tool_approach`: `depth_activation_pixel_size_px` (70.0), `pixel_dist_fx` (615.0), `tag_size_m` (0.0254), `standoff_m` (0.15), `step_m` (0.010).
- `depth_camera.launch.py` is the PRIMARY launch now that `ros-jazzy-realsense2-camera` is installed: brings up the realsense driver with `align_depth.enable:=true`, the static TF, `gripper_depth_monitor`, `fine_localization` (depth ON) and `detect_zone`. `detect_zone.launch.py` is a V4L2 fallback (RGB only; depth topic absent → `tool_approach` runs pixel-size-only). `tool_approach` is intentionally NOT in either launch so its keyboard thread has a live TTY when run from its own terminal.

## Next big step — Phase 10b: autonomous stop + grasp

Once `detect_zone` has the gripper hovering over the pickup zone and the requested tool's AprilTag (ID 2/3/4, 25.4 mm) is in view, the arm needs to approach, align, and grasp it. The design the user specified:

1. **Scan for the requested tag** (`dt_apriltags` on the gripper RGB stream, already wired up in `apriltag_perception.py` / `fine_localization.py`).
2. **Build a perpendicular axis** — the world-frame normal vector emanating from the physical AprilTag plane. For a flat tool lying on the cart, this is just `+z` in `link_base` passing through the tag center XY. (When the tag isn't horizontal, derive from the tag's pose quaternion.)
3. **Trace the TCP onto that axis** — move the gripper in XY so the camera's optical axis coincides with the tag-normal axis. This is a pure XY refinement above the current hover height: compute the tag's (x, y) in `link_base`, plan an IK-first joint-space move that brings `link_tcp` directly above it with the gripper's tool-down orientation (`_tool_aligned_quat(x, y)` — RPY = (π, 0, atan2(y, x))).
4. **Descend along the axis** — vertical-only motion (z decreasing) while holding (x, y) constant. Use the observed AprilTag *pixel size* as a depth proxy: as the camera gets closer, the tag grows; stop descending once the tag reaches a pre-calibrated pixel size that corresponds to "gripper fingers are at grasp depth." This sidesteps the D435i 190 mm min depth range, which is why the earlier depth-based descent was broken.
5. **Close gripper**, then **lift** straight up on the same axis to a safe transit height.
6. **Transfer to dropoff** — plan a joint-space move to a Hover-over-Dropoff pose (to be added, mirror of `HOVER_PICKUP_JOINTS`), descend to release height, open gripper, retract.

Design notes for whoever implements Phase 10:
- IK-first planning is mandatory on this 5-DOF arm (see "Architectural conventions" below).
- Use the 25.4 mm physical tag size and the D435i color intrinsics (fx=fy≈615, cx=320, cy=240 @ 640×480 — `realsense_v4l2_pub.py:60`) to convert pixel size → distance analytically: `distance ≈ fx * tag_size / pixel_size`.
- Don't rely on depth-image feedback until `realsense2_camera` is installed. Until then the AprilTag pixel-size heuristic is the depth sensor.
- The static TF `link_tcp → camera_link` (translation −0.06985, 0, 0.127; pitch −π/2) derived in `depth_camera.launch.py:50` is still the right mount geometry. Reuse it verbatim.
- Live-robot iteration is expensive. Build the descent math *off-robot* first (static tag pose → planned waypoints → check with TF echo), then do a single dry run, then live.

## Architectural conventions worth remembering

- **IK-first planning.** OMPL cannot sample valid goal states from raw Cartesian constraints on a 5-DOF arm. Always call `/compute_ik` (TRAC-IK) first, then plan in joint space against the returned joint values. This matches RViz's internal flow.
- **5-DOF wrist yaw constraint.** At any (x, y), wrist yaw must equal `atan2(y, x)`. Use `_tool_aligned_quat(x, y)` (`test_descent.py:152`) for every Cartesian goal — RPY = (π, 0, atan2(y, x)). Reusing the current TCP quat for a pose at a different XY produces sporadic NO_IK_SOLUTION (error_code −31) failures.
- **End-effector link.** With gripper attached: `link_tcp` (172 mm below `link_eef`). Without: `link_eef`.
- **Disabled `avoid_collisions` in IK requests.** Necessary on this 5-DOF arm; MoveGroup still does collision checking during planning. Preserve the inline comment explaining why.
- **TF chain for depth perception:** `link_base → … → link_tcp → camera_link → camera_color_optical_frame → tag pose`. The static TF child link is `camera_link` (RealSense's default root) — anything else (e.g. `gripper_camera_link`) leaves the trees disconnected and `do_transform_point` silently fails.
- **RealSense optical frame:** `+x right (image), +y down, +z forward (depth axis)`. Don't conflate with `camera_link` (`+x forward, +y left, +z up`).

## Working style with the user

- Execute commands directly; don't hand the user scripts to run manually when they can be run here.
- Commit and push to GitHub as work lands (explicit preference).
- Confirm physical assumptions (tag size, mounting orientation, cart height) **before** live runs, not after.
- Live-robot tests are the most expensive feedback loop in this project. Use static checks (TF echo, topic echo, the `/tmp/detect_zone_debug` JPEGs) first.
- Between robot-stack relaunches, kill stale `robot_state_publisher` / `move_group` / `controller_manager` / `gripper_camera_tf` / `realsense_v4l2_pub` / `detect_zone` processes — leftovers conflict on TFs and topics, and old camera processes hold `/dev/video10`.
- The user's laptop mic is the voice input (not the RealSense's array mic).

## Pointers to deeper context

- `.claude/skills/descent-test-debugging.md` — lessons learned during the broken depth-based descent experiments (Phase 8b). Read before attempting Phase 10's descent math.
- `REACHABILITY_MAP.txt` — Phase 4 workspace coverage.
- `docs/detect_zone_example.png` — example bird's-eye capture with zones labeled.
- Auto-memory at `~/.claude/projects/-home-logang/memory/` — cross-session context, indexed in `MEMORY.md`.
