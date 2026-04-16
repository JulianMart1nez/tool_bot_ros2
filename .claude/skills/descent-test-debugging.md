---
name: descent-test-debugging
description: How to approach (and not approach) the gripper-mounted depth camera + AprilTag closed-loop descent test on this xArm5 project. Use this skill any time the user asks about test_descent, fine_localization, the gripper RealSense camera, or "the gripper went the wrong way." Read it before editing depth_camera.launch.py, fine_localization.py, or test_descent.py.
---

# Descent test debugging — what worked, what didn't, what to do differently

Bookmark of session 2026-04-15. Final state on branch `julian_tracik_integration` at commit `48c0761` is **broken**: test_descent drives the gripper in the opposite direction of the AprilTag, and the staged 20mm-hop refinement drifts further away. The user explicitly asked to bookmark the session and rework the descent logic from scratch next time.

## Setup recap (so you don't re-discover it)

- Gripper-mounted RealSense D435i, mounted with the lens looking *down* when the TCP is in tool-down orientation.
- Camera body: 2.75in along link_tcp −x, centered (y=0), 5in above TCP midpoint.
- Static TF child frame must be `camera_link` (RealSense's default root). Anything else (`gripper_camera_link`, etc.) leaves the TF trees disconnected and `do_transform_point` silently fails.
- `camera_link` convention: +x forward (lens), +y left, +z up.
- `camera_color_optical_frame` convention: +x right (image), +y down, +z forward (depth axis).
- Tool AprilTags are **25mm physical** (tag36h11). The workspace tags are 2in (0.0508m). Don't conflate them.
- xArm5 is 5-DOF. IK only solves reliably when wrist yaw = atan2(y, x). Use `_tool_aligned_quat(x, y)` for every Cartesian goal.
- D435i depth min range ~190mm. Once `tcp_above_tool < 60mm`, fall back to open-loop descent.
- Safety z floor: −0.085m in link_base. Never command below this.

## What did NOT work (mistakes to avoid repeating)

1. **Iterating sign by sign on a live test.** I flipped the camera-x sign from +0.06985 to −0.06985 based on a single observation that the arm went the wrong way. The next live run also went the wrong way (just in a different way). Live-robot runs are the most expensive feedback loop in this project — never use them to brute-force a sign convention.

2. **Adding "staged 20mm hops" to mask a wrong direction.** When the underlying camera→TCP transform is wrong, staging just makes the gripper drift away slowly instead of obviously. The user immediately spotted that hops were "moving randomly away from the tool." Don't soften a broken signal with rate limits.

3. **Defaulting `tag_size = 0.0508` because the workspace tags are 2 inches.** The tool tags are 25mm. This silently 2× scaled depth and made every downstream calculation wrong. Always confirm physical tag size with the user before setting `TAG_SIZE`.

4. **Static TF with translation but no rotation.** First iteration of `depth_camera.launch.py` had the camera in identity orientation. fine_loc reported the tag at (1.22, 0.39, 0.016) — clearly wrong, but only obvious in retrospect. Always derive rotation from the physical mounting (here: pitch=−π/2 about link_tcp +y so the lens points along link_tcp +z).

5. **Using `cur_quat` from the current TCP for the next pose.** Seems safe ("we're not changing orientation"), but on a 5-DOF arm the wrist yaw must match the new target's atan2(y, x), not the old one. IK returns `error_code=-31 (NO_IK_SOLUTION)` with no obvious reason. Always rebuild the quaternion from the *target* XY using `_tool_aligned_quat`.

6. **Disabling `avoid_collisions` in the IK request without a comment.** It's necessary on this 5-DOF arm because collision-aware IK rejects feasible poses near the workspace edge, but a future reader will assume IK is collision-aware. The MoveGroup planner still does collision checking, so this is safe — just keep the inline comment.

7. **Launching `*_fake.launch.py` "to test the launch graph quickly."** Memorialised in `feedback_real_robot_launch.md`: the fake hardware launch does not command the real arm, doesn't expose `drive_joint`, and silently breaks the gripper controller. This project always uses the real-robot launch.

8. **Forgetting to kill stale `robot_state_publisher` / `move_group` / `controller_manager` / `gripper_camera_tf` processes between launches.** Leftovers conflict on TFs and topics. The user reminded me mid-session.

9. **Not running a static sanity check before motion.** Every Phase 8 issue this session would have been caught by a 30-second static check (echo `/fine_loc/result`, echo TF, manually slide the tag and watch the published delta).

## What DID work (keep doing this)

- **TF-based fine localization.** Subscribing to `/fine_loc/result` (PoseStamped in link_base) in test_descent and refreshing `target_x, target_y` each descent iteration — when the underlying transform is correct — is the right architecture. Don't throw it away.
- **Closed-loop depth descent with open-loop fallback below D435i min range.** This pattern is sound; preserve it.
- **`_tool_aligned_quat(x, y)` helper** for every Cartesian goal. After adopting this, IK stopped failing.
- **Killing stale processes then relaunching `xarm5_moveit_with_table.launch.py`** as the canonical reset.
- **Asking the user about physical orientation** ("which way is the screwdriver handle facing?") before changing a sign. This is the right instinct — but do it *before* the live run, not after.

## What to do next time the user wants to rework this

1. **Do not start by editing code.** Walk through a one-page derivation with the user of the full transform chain: `link_base → link_tcp → camera_link → camera_color_optical_frame → tag pose`. Write down expected sign of each axis at each step. Identify which sign you're uncertain about and ask the user — once.

2. **Static sanity check before any motion.** With the arm parked in pre-grasp, the user manually moves the tag in a known direction (e.g. "I'm sliding it toward the base ~5cm"). Echo `/fine_loc/result` and confirm the published delta has the expected sign. Iterate on the static TF *only* by re-running this check, never with a live descent.

3. **Address AprilTag pose ambiguity.** dt_apriltags returns the closer of two valid solutions for oblique views. Currently `fine_localization.py` picks the closest tag by `pose_t[2]` but does nothing about the per-tag flip ambiguity. A spurious flip could explain the iter-#5 +106mm XY jump in an earlier test. Consider: average over N detections, reject if `pose_err > threshold`, or constrain expected XY range from prior knowledge of the tool's location.

4. **Resist the urge to add stages, rate limits, or smoothing while the underlying signal is wrong.** Get one descent working with raw direct steering before adding any safety/UX layer.

5. **Verify physical assumptions before each test:**
   - Tag size in `fine_localization.py` matches the tag in the user's hand.
   - `ros2 run tf2_ros tf2_echo link_base camera_color_optical_frame` returns a sensible transform.
   - `ros2 topic echo /fine_loc/result --once` returns coordinates inside the workspace.
   - `ros2 topic echo /gripper/tool_distance --once` returns a value within D435i range.

## Files of interest (locations as of commit 48c0761)

- `xarm5_ros2_barebones_start/xarm5_basic_position_cmd/launch/depth_camera.launch.py:50` — static TF (suspect)
- `xarm5_ros2_barebones_start/xarm5_basic_position_cmd/xarm5_basic_position_cmd/fine_localization.py:39` — TAG_SIZE = 0.025
- `xarm5_ros2_barebones_start/xarm5_basic_position_cmd/xarm5_basic_position_cmd/test_descent.py:152` — `_tool_aligned_quat`
- `xarm5_ros2_barebones_start/xarm5_basic_position_cmd/xarm5_basic_position_cmd/test_descent.py:284` — Step 3 staged refinement (added this session, currently masking the bug)

## Project-wide guardrails to honor

- Use the canonical real-robot launch (see `feedback_real_robot_launch.md`).
- Don't push to main; this project's working branch is `julian_tracik_integration`.
- Commit and push after notable progress (the user explicitly prefers this).
