# CLAUDE.md — xArm5 tool_bot_ros2 project

This file orients Claude on how to work in this repo. Read it first.

## What this project is

Autonomous tool pick-and-place on a UFactory xArm5 (5-DOF) + UFACTORY G2 gripper, ROS 2 Jazzy on Ubuntu 24.04. Tools are detected via AprilTags; trajectories are planned with TRAC-IK + MoveIt 2 / OMPL.

- Robot IP: `192.168.1.234`
- Driver workspace: `~/xarm_ws` (official `xarm_ros2`, NOT in this repo)
- App workspace (this repo): `~/tool_bot_ros2`
- Voice-command package: `~/tool_bot_ros2/voice_command` (symlinked into `~/ros2_ws/src/voice_command` so `~/ros2_ws` still builds it)
- Active branch: `logan_master`. Historical work archived under `legacy` (a single branch tip that captures `main` + `julian_tracik_integration` + `matthew_cameras` + `xarm5-ros2-starter`).

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
- **Safety z floor:** context-aware. `SAFETY_Z_FLOOR = PICKUP_TOP_Z = -3.07 in + 2 mm = -0.0760 m` for the pickup zone; `DROPOFF_TOP_Z = -5.00 in = -0.1270 m` once `_dropoff_sequence` has installed the synthetic target (lower because the dropoff cart sits below the pickup cart). `_current_safety_floor()` in `go_to.py` switches between the two based on `_in_dropoff_zone`.
- **Tool AprilTags** (tag36h11, **25.4 mm (1 in)**): hammer=3, phillips=2, flathead=4. Gripper tag: **22**.
- **Per-tool sweet pixel sizes (grab-arming bands)** — calibrated against on-tool tag photos at the grasp-ready depth:
  - phillips (fid 2): **96–100 px** (sweet=98.0 ± 2.0)
  - hammer (fid 3): **105–110 px** (sweet=107.5 ± 2.5, strict)
  - flathead (fid 4): **96–100 px** (sweet=98.0 ± 2.0)
  Defined in `go_to.py:SWEET_PX` + `SWEET_TOL_PX_PER_TID`; mirrored in `debug_overlay.py` so the IN BAND visual badge matches the grab-arming check exactly. Update both tables together.
- **Zone markers:** black squares on white paper (1in square, on white letter-size sheets). No AprilTags on zone corners. Detected by morphological top-hat + ring/center contrast (see `detect_zone.py:detect_zone_markers`).
- **Zone dimensions:** Pickup 14.5 in × 15.5 in (wider). Dropoff 9.0 in × 15.5 in (narrower). In camera frame at bird's-eye: pickup = RIGHT, dropoff = LEFT.
- **Carts:** Pickup cart LEFT (+y), 24 in × 18 in, near edge 18 in from base, surface at −3.07 in (user-measured). Dropoff cart RIGHT (−y), 34 in × 17.5 in, near edge 18 in from base, surface at −5.00 in. Cart height: 31 in. The dropoff cart's collision box is suppressed during the descend → release → retreat span (see `_set_dropoff_cart_collision`) so it only constrains pre-drop transit clearance, not the drop pose itself.

## Key joint poses (degrees → radians in code)

| Pose | J1 | J2 | J3 | J4 | J5 |
|------|----|----|----|----|----|
| Detect Zone (bird's-eye) | 0 | 0 | −160 | 145 | 0 |
| Hover over Pickup | 23 | 23 | −130 | 107 | 23 |

Defined in `detect_zone.py:DETECT_ZONE_JOINTS` / `HOVER_PICKUP_JOINTS`.

## Phase status (2026-05-02)

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
| 10a — AprilTag align + keyboard trace-down | done | `tool_approach.py` (legacy); replaced by `go_to.py` for the voice path |
| 10b — go_to continuous tracking + tag-acquire + grab-arming | **done** | TARGET_MODE_CAMERA centers tag in image; per-tool sweet sizes drive grab-arming; tag-yaw IK candidate aligns gripper across the tool axis with radial yaw as fallback |
| 10c — Autonomous stop + gripper close + dropoff | **done** | `auto_go_to.py` orchestrator state-machines voice → descent → grab → dropoff → home with no keyboard input. `_dropoff_sequence` transits via HOVER_DROPOFF, drops at per-tool DROP_POS, auto-opens the gripper, retreats. |
| 11 — Per-tool dropoff calibration | in progress | DROP_POS_BY_FID_DEG defined for known tools; flathead pose calibration ongoing |

## Phase 10b/10c — current state (2026-05-02)

### `go_to.py` — keyboard variant (T4 `ros2 run xarm5_basic_position_cmd go_to`)
- Subscribes: `/voice_command/tool_request`, `/voice_command/home_request`, `/detect_zone/complete`, and dynamically subscribes to `/fine_loc/tag_<fid>` + `/fine_loc/tag_<fid>/pixel_size` for the requested fiducial.
- **TARGET_MODE_CAMERA is the default** — `_tcp_for_mode` pushes the TCP target by `+CAM_OFFSET_X_IN_TCP` radially outward so the camera (mounted −70 mm along link_tcp +X) lands directly over the AprilTag. Image stays centered on the fiducial through descent. Auto-failover to TCP mode in `_do_move` if camera-mode IK fails for every yaw candidate (preserves reachability for tools at the workspace edge, e.g. the hammer). The keyboard `x` toggle still flips modes manually.
- **Tag-yaw IK alignment** (primary candidate): `_quat_to_yaw_base(tag_q)` extracts the printed-up direction of the AprilTag in `link_base`; `_do_move` tries that first so J5 rotates to put the gripper fingers across the tool shaft. Radial yaw `atan2(tag_y, tag_x)` is the IK fallback when tag_yaw is unreachable.
- **Continuous XY tracking** (1.5 Hz): replans IK + joint-space move whenever the smoothed tag XY shifts past `XY_DEADBAND_M = 0.015 m`. Keyboard ↓ lowers `target_z` by the current step (10/5/1 mm cycled with `s`); the next tick re-plans at the new z.
- **Pose smoothing + outlier reject**: running mean over `POSE_SMOOTH_N = 5` samples; any single pose more than `XY_JUMP_REJECT_M = 0.10 m` from the mean is dropped with a WARN log.
- **Grab arming**: when `/fine_loc/tag_<fid>/pixel_size` lands within `SWEET_PX[tid] ± SWEET_TOL_PX_PER_TID[tid]` for `GRAB_HOLD_TICKS = 3` consecutive samples, fires the gripper close action and emits `/tool_approach/grab`.
- **Dropoff sequence (`d`)**: HOVER_DROPOFF → per-tool DROP_POS → auto-open → HOVER_DROPOFF retreat. While there, `_in_dropoff_zone = True` lowers the safety floor to `DROPOFF_TOP_Z` and a synthetic target is installed at the current TCP so ↑/↓ still work for manual fine-tune. `_target_mode` is forced to TCP for the synthetic target and **restored to `DEFAULT_TARGET_MODE` by `_reset_lock_state`** on the next request — without that restore, the second tool request after a dropoff would inherit TCP mode and the camera would land 70 mm off-tag.
- **Home via voice**: `/voice_command/home_request` triggers all-zero joint goal with WARN-level logging at every step. CLI-bypass test: `ros2 topic pub --once /voice_command/home_request std_msgs/String "{data: cli}"`.
- `ReentrantCallbackGroup` + `MultiThreadedExecutor(num_threads=4)` so home/abort callbacks can't be starved by tracking threads. `_autonomous_abort` and `_dropoff_done` events let the orchestrator preempt safely.

### `auto_go_to.py` — fully autonomous variant (T4 `ros2 run xarm5_basic_position_cmd auto_go_to`)
Subclasses `GoTo` and replaces the keyboard fallback with a 5 Hz state machine. Reuses every helper in the parent (IK, MoveGroup, gripper action, dropoff sequence, planning-scene toggles); none of that logic is duplicated.

States: `IDLE → WAIT_DETECT_ZONE → WAIT_TAG_LOCK → SETTLE_INITIAL → DESCEND → GRAB → DROPOFF → HOME → IDLE`. A generation counter (`_auto_gen`) bumps on every fresh tool request; in-flight worker threads (grab, dropoff, home) carry the gen they started with and refuse to mutate state when stale, so a slow worker tearing down can't stomp the new run's `_target_tag_id`.

Behaviour:
- `_tick_descend` steps `target_z` down by `DESCEND_STEP_M = 0.010 m` every `DESCEND_PERIOD_S = 1.0 s` once the previous step's move has settled.
- Tracks `_max_pixel_seen` during the descent. At floor-touchdown without grab-arm, **force-grabs** if `max_pixel_seen ≥ FORCE_GRAB_PIXEL_FRACTION (0.55) × SWEET_PX[tid]` — handles the expected geometric fall-out where the tag exits FOV at very close range because of the 70 mm camera offset; aborts to home only if alignment never approached sweet.
- DROPOFF runs the parent's `_dropoff_sequence` thread and waits on `_dropoff_done` (timeout 60 s). HOME runs `_go_home` inline.

### `detect_zone.py` — new post-hover "tag acquire" phase
- After `_center_zone_in_frame('PICKUP')` succeeds, runs `_acquire_tag(fiducial_id)`:
  1. Quick scan (10 frames) — is tag_<fid> visible?
  2. If not, sweep J1 by −5°, +5°, −10°, +10°, −15°, +15° until the tag is found.
  3. **No pixel-based joint nudging** — earlier attempt (`_center_tag_in_frame`) used a hand-tuned J1/J5 pixel nudge; user reported the arm moved "in nonsensical random directions", so we replaced it with visibility-gate-only. All XY alignment now happens in `go_to.py` using fine_localization's 3D link_base pose (which IS trustworthy through the TF chain).
- Handoff payload `/detect_zone/complete` = `tool=…|fiducial=…|status={ok|zone_only|degraded}`.
- Debug JPEGs: `acquire_tag<fid>_FOUND*.jpg` / `NOT_FOUND.jpg` in `/tmp/detect_zone_debug/<ts>_<tool>/`.

### `debug_overlay.py` — image-only safety signals (no depth)
- Published on `/debug/overlay`. Two signals: **SIZE** (measured tag edge px vs per-tool sweet size) and **ALIGN** (signed delta deg between tag's most-vertical edge pair and image vertical axis; rotate J5 by −ALIGN to correct).
- Per-tool sweet sizes mirrored from `go_to.SWEET_PX` (see "Per-tool sweet pixel sizes" above). Per-tool tolerance via `SWEET_TOL_PX_PER_TID`. Keep the two tables in sync.
- IN BAND badge green iff SIZE in band AND |ALIGN| ≤ 3°.

### xArm firmware collision sensitivity
- Enabled in `xarm5_moveit_with_table.launch.py` via `/xarm/set_collision_sensitivity data:3` at t=8 s. Scale 0–5 (0=off, 3=middle).
- **Observed issue this session**: arm dropped to `state:2` (stopped/paused) with `err:0, warn:0` after a HOME, red LED on. May be collision-sensitivity false-trip. Recovery:
  ```
  ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
  ros2 service call /xarm/set_mode  xarm_msgs/srv/SetInt16 "{data: 1}"
  ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
  ```
  If this keeps happening, drop sensitivity 3 → 2 in `xarm5_moveit_with_table.launch.py`.

### Resolved bugs (kept for future-Claude pattern-matching)

- **"Aligning but not grabbable" / IK reach-edge stall** (root cause 2026-04-20, resolved 2026-04-29). CAMERA mode pushes TCP +70 mm radially → workspace edge → TRAC-IK -31 storm. Fixed by adding a CAMERA→TCP fallback inside `_do_move` and adding the tag-yaw IK candidate in front of radial-yaw.
- **Post-dropoff alignment regression** (resolved 2026-05-02). `_dropoff_sequence_inner` forces `_target_mode = TARGET_MODE_TCP` for the synthetic dropoff target; `_reset_lock_state` was not restoring it, so the second tool request after a dropoff inherited TCP mode and the camera landed 70 mm off the tag. Fix: `_reset_lock_state` now sets `self._target_mode = DEFAULT_TARGET_MODE`.
- **Pixel-silence abort during descent** (resolved 2026-04-29). The original `auto_go_to.py` aborted to home if `pixel_size` went silent for 4 s — but the silence is the EXPECTED geometric fall-out at close range when the camera's +70 mm offset puts the tag near the FOV edge. Replaced with `_max_pixel_seen` tracking + `FORCE_GRAB_PIXEL_FRACTION = 0.55`: at floor touchdown without grab-arm, force-grab if max-seen ≥ 55 % of sweet, otherwise home.

### Canonical terminal launch (`~/tool_bot_ros2/launch_scripts/t*.sh`, mirrored as `~/Desktop/t*.desktop`)
`t1_robot.sh` (MoveIt + controllers + collision scene) → `t2_camera.sh` (`depth_camera.launch.py`) → `t3_voice.sh` (`voice_command_node`) → choose ONE of `t4_approach.sh` (`go_to`, keyboard) or `t4_auto.sh` (`auto_go_to`, fully autonomous). `t5_camera_view.sh` opens `rqt_image_view /debug/overlay` + the debug_overlay node. `t6_arm_monitor.sh` streams robot_states. `t7_camera_feeds.sh` = RGB + aligned_depth viewer + center-pixel depth probe.

### Live system snapshot
`docs/system_snapshot/` contains a captured run of the full stack (`nodes.txt`, `topics.txt`, `services.txt`, `actions.txt`, per-node `node_info/*.txt`, the rendered ROS graph as `ros_graph.{gv,png,pdf}`, and the TF tree as `tf_frames.{gv,png,pdf}`). `build_rqt_graph.py` re-renders the graph from a fresh capture; `README.md` in that directory has the refresh recipe.

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

## Next big step — Phase 11: per-tool dropoff calibration + flathead alignment hardening

`auto_go_to.py` lands the autonomous voice → grab → dropoff → home loop. Remaining work:

1. **Per-tool DROP_POS calibration** — `DROP_POS_BY_FID_DEG` in `go_to.py` needs entries for every fiducial. When a tool's entry is missing, the dropoff sequence stops at HOVER_DROPOFF and waits for manual ↓/o; calibrating the joint pose makes it fully autonomous.
2. **Flathead screwdriver alignment** — separate Claude session noted misalignment for fid=4. Pattern is well understood (PnP yaw instability when the tag is far off-axis); fix candidates include tightening detect_zone's `_acquire_tag` sweep range or post-filtering tag_yaw outliers in `_tag_pose_cb`.
3. **Camera mount calibration drift** — vertical offset of the tag in the gripper view is not zero at "centered" alignment. Likely a small Y or pitch error in `depth_camera.launch.py`'s static TF `link_tcp → camera_link`. Tooling option: expose Y/pitch as launch args so the offset can be dialed out without rebuild.

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
