# Phase 10 — AprilTag Normal-Vector Descent & Grasp

Design doc for the next implementation phase. Read `CLAUDE.md` first.

## Prerequisites (in place)

- Voice-driven `/voice_command/tool_request` publishing `tool=<name>|fiducial=<id>`.
- `detect_zone` drives the arm through bird's-eye → scan → hover → frame-center, ending with the gripper camera roughly centered on the pickup zone.
- Gripper-mounted RealSense D435i publishing RGB on `/gripper_cam/depth_camera/color/image_raw` (via `realsense_v4l2_pub` until `realsense2_camera` is installed).
- `apriltag_perception.py` / `fine_localization.py` detect tool tags (tag36h11, 25 mm, IDs 2/3/4) and publish pose.
- Static TF `link_tcp → camera_link` already correct: translation (−0.06985, 0, 0.127), pitch −π/2.

## Goal

Once `detect_zone` completes and the requested tool's AprilTag is visible, the arm must:
1. Align the gripper along the tag's normal (vertical / perpendicular-to-plane) axis.
2. Descend along that axis until the gripper is at grasp depth (measured via apparent tag size).
3. Close, lift, transit to dropoff, release.

## Step-by-step

### 1. Get the tag pose in `link_base`

`fine_localization.py` already does this: `dt_apriltags` → `solvePnP` → transform through the TF chain `camera_color_optical_frame → camera_link → link_tcp → … → link_base`. Output: `geometry_msgs/PoseStamped` on `/fine_loc/result`. No change needed — just subscribe.

### 2. Compute the tag normal in `link_base`

For tools lying flat on the cart, the normal is `+z` in `link_base` through the tag center XY. That's the common case.

For tags that aren't horizontal (tool on its side), compute the normal from the pose quaternion: rotate the tag's local `+z` axis by the tag's orientation quaternion. `tf_transformations.quaternion_matrix(q)[:3, 2]` gives the `+z` column of the rotation matrix — that is the normal vector.

Decision rule: if the tag's normal is within ~20° of world `+z`, treat it as horizontal (common case). Otherwise log a warning and skip the tool — non-horizontal tools need special handling (out of scope for Phase 10).

### 3. Align XY above the tag

Plan an IK-first joint-space move that places `link_tcp` directly above the tag at the current hover height:

```python
from geometry_msgs.msg import PoseStamped
target = PoseStamped()
target.header.frame_id = 'link_base'
target.pose.position.x = tag_x
target.pose.position.y = tag_y
target.pose.position.z = hover_z   # hold current z
q = tool_aligned_quat(tag_x, tag_y)  # RPY = (π, 0, atan2(y, x))
target.pose.orientation = q
```

Call `/compute_ik` with `group_name='xarm5'`, `avoid_collisions=False`, starting from current joint state. Plan joint-space against the returned joint targets (MoveGroup still does collision checking during planning). Reference pattern in `test_descent.py:152`.

### 4. Descend along the tag normal (pixel-size depth loop)

The D435i can't give depth under ~190 mm, so Phase 8b's depth-feedback descent broke near the grasp. Replace it with a pixel-size heuristic:

**Analytic relation.** Pinhole model gives `pixel_size = fx * tag_physical_size / distance`. With `fx ≈ 615 px` (from `realsense_v4l2_pub.py:60`) and `tag_physical_size = 0.025 m`:

```
distance_m = 615 * 0.025 / pixel_size_px = 15.375 / pixel_size_px
```

So `pixel_size = 150 px` → distance ≈ 102 mm; `pixel_size = 300 px` → ≈ 51 mm.

**Calibration constant.** Before going live, measure the target pixel size at grasp depth experimentally: hand-position the arm at the correct grasp height for each tool, log `pixel_size` from `dt_apriltags` (side length of detected corners). Save per-tool into a config. `GRASP_PIXEL_SIZE = {hammer: X, phillips: Y, flathead: Z}`.

**Descent loop.** Move in 10 mm z-decrements from hover_z, re-detecting the tag after each step. Stop when `pixel_size >= GRASP_PIXEL_SIZE[tool]`. Safety floor: `z_min = −0.085 m` in `link_base` (from `CLAUDE.md`). Never descend past that regardless of tag reading.

Each iteration holds `(tag_x, tag_y)` constant from the most recent fine-loc update — this also corrects for small XY drift during the descent.

### 5. Grasp

```
open_width → close (drive_joint = 0.85).
```

Gripper command interface: UFACTORY G2 via the arm's joint controller — see existing gripper usage in `grasp_pose_generator.py`.

### 6. Lift + transfer

- Lift 100 mm straight up on the same axis (joint-space, hold X/Y, increase z by 0.10 m).
- Define `HOVER_DROPOFF_JOINTS` (mirror of `HOVER_PICKUP_JOINTS` — probably J1 negated, or re-measured once the physical dropoff geometry is locked in). Plan joint-space move.
- Optional: re-detect markers at dropoff hover to confirm a clear drop area. Not required for v1.
- Descend 80 mm (hard-coded for v1 — tools will land on the cart surface from ~80 mm above, which is safe given the safety floor).
- Open gripper (`drive_joint = 0`).
- Lift 100 mm.

### 7. Return to home

`auto_home` is already wired — trigger it as the final step, or leave the arm at the dropoff hover pose depending on UX preference.

## Files to touch

- `xarm5_basic_position_cmd/xarm5_basic_position_cmd/detect_zone.py` — after the `center_success` debug save, trigger Phase 10's approach node (or inline the sequence — TBD by implementer).
- New: `xarm5_basic_position_cmd/xarm5_basic_position_cmd/tool_approach.py` — implements steps 3–7 as a standalone node subscribed to `/fine_loc/result`.
- New: `config/grasp_pixel_sizes.yaml` — per-tool pixel-size calibration.
- `CLAUDE.md` — update Phase status table when each sub-step completes.

## Risks & mitigations

| Risk | Mitigation |
|------|------------|
| IK fails for a reachable-looking tag pose | Always use `tool_aligned_quat(x, y)`. If still failing, log the pose + joint state and skip — don't iterate sign-by-sign live (Phase 8b lesson). |
| Tag detection flickers during descent | Temporal smoothing over last N detections (reuse `deque`-based smoothing from `detect_zone.py`). |
| Tag occluded by gripper at close range | Fall back to last-known pose + open-loop descent below a pixel-size threshold (e.g. `pixel_size > 0.6 * GRASP_PIXEL_SIZE`). |
| Wrist-yaw singularity at (x≈0, y≈0) | Reject tag poses within 5 cm of `link_base` origin — that's outside the physical workspace anyway. |
| Camera mount flex introduces XY drift | Pixel-size calibration already absorbs small drift since detection is continuous. |

## Test plan

1. **Off-robot:** feed a canned tag pose into `tool_approach` and inspect the planned waypoints (no execution) — verify XY, yaw, z decrements are correct.
2. **Static-arm:** with arm parked at hover, step through the descent plan using `plan_only=True` MoveGroup requests. Inspect each plan in RViz.
3. **Dry descent:** execute the first 2–3 descent steps only (stop at 50 mm above surface). Confirm tag pixel size monotonically increases and XY drift is bounded.
4. **Full grasp:** one tool, one zone, once. Only proceed when 1–3 all look clean.
