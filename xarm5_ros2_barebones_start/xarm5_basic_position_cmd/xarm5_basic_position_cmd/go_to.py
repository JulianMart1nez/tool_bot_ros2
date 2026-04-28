#!/usr/bin/env python3
"""
go_to.py — AprilTag closed-loop target visualization + keyboard-commit moves.

Behaviour after detect_zone hovers over the pickup zone:

  /detect_zone/complete fiducial=N  → go_to subscribes to:
    /fine_loc/tag_N            (PoseStamped, in link_base)
    /fine_loc/tag_N/pixel_size (Float32)

  Tracking timer (TRACK_RATE_HZ): every tick recomputes the projected
  target TCP pose from the LATEST tag pose (closed-loop — no saved
  snapshot) and publishes it for visualization:

    /go_to/target_pose   (geometry_msgs/PoseStamped, link_base)
    /go_to/viz           (visualization_msgs/MarkerArray)
       id=0 SPHERE       — target TCP position (green)
       id=1 CUBE         — detected AprilTag (yellow)
       id=2 ARROW        — tool approach axis (blue)

  The track tick does NOT auto-move the arm. Motion only happens when
  the operator presses ↓/↑/m on the keyboard — each key press rebuilds
  the IK target from the LIVE tag pose, calls /compute_ik fresh, and
  plans a joint-space move to it. Nothing is "saved": every commit is a
  new IK against the newest tag sample.

  Keyboard:
    ↓ / Enter : descend  by current step size and commit move
    ↑         : raise    by current step size and commit move
    s         : cycle approach step size (10mm → 5mm → 1mm → wrap)
    m         : commit move to current displayed target (no z change)
    g         : CLOSE gripper (manual grab) via /xarm_gripper/gripper_action
    o         : OPEN  gripper (release / reset between manual attempts)
    x         : toggle target mode (camera ↔ tcp)
    space     : pause/resume target publishing
    h         : send arm home (all joints zero)
    q         : quit

  Manual grab workflow: descend with ↓ at 10mm until close, press 's' to
  switch to 5mm, refine, press 's' again for 1mm, refine, then 'g' to
  close the gripper. 'o' opens for the next attempt.

  Voice:
    /voice_command/home_request → arm home (high-priority, with verbose log)
    /voice_command/tool_request → reset state, wait for new detect_zone

Grab arming: when /fine_loc/tag_N/pixel_size lands within sweet_px ± tol
for GRAB_HOLD_TICKS consecutive samples, publishes /tool_approach/grab
and pauses tracking until reset.

To visualize in RViz: add a "Pose" display on /go_to/target_pose and a
"MarkerArray" display on /go_to/viz.

To test home WITHOUT voice (sanity check the go_to side):
  ros2 topic pub --once /voice_command/home_request std_msgs/String "{data: 'cli'}"
"""

import math
import sys
import select
import termios
import threading
import time
import tty
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest, PositionIKRequest,
)
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray


PLANNING_GROUP = 'xarm5'
FRAME_ID = 'link_base'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
SAFETY_Z_FLOOR = -0.110
TOOL_TAG_IDS = (2, 3, 4)
TOOL_NAMES = {2: 'phillips', 3: 'hammer', 4: 'flathead'}
HOME_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0)

STEP_M = 0.010
# Cycled by the 's' key during final approach. Order = the rotation order
# the user sees: coarse → medium → fine → wraps. Default index 0 keeps
# the historical 10 mm behaviour for ↑/↓.
STEP_SIZES_M = (0.010, 0.005, 0.001)

# UFACTORY G2 gripper drive_joint range: 0 = open, 0.85 = closed.
# Effort 0.0 means "no caller-imposed limit" (the action server uses its
# own default), which is what test_descent.py has used reliably.
GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSE_POS = 0.85
GRIPPER_EFFORT = 0.0
GRIPPER_ACTION_TIMEOUT_S = 10.0

MAX_DOWN_M = 0.40
MAX_UP_M = 0.20
XY_DEADBAND_M = 0.015      # re-plan when smoothed tag XY shifts >1.5 cm
XY_JUMP_REJECT_M = 0.10    # any single pose >10 cm from running avg → ignore
TRACK_RATE_HZ = 1.5        # tracking loop rate (slow enough for plan + move)
POSE_SMOOTH_N = 5          # running-average window for /fine_loc/tag_N poses

# Auto-descent on first lock: drop this far below current TCP z, but never
# closer than INITIAL_APPROACH_STANDOFF_M to the tag's own z. Removes the
# "arm just sits there after acquire waiting for the ↓ key" feel. The
# final target is also clamped to never exceed the current TCP z — an
# auto-descend must not accidentally ASCEND when the tag floor is above
# the current TCP (that just means we're already close to the tag).
INITIAL_DESCEND_M = 0.10
INITIAL_APPROACH_STANDOFF_M = 0.08

# Camera mount geometry (must match the static TF link_tcp → camera_link
# published by depth_camera.launch.py). Used to compute the TCP position
# that puts the CAMERA over the tag in 'camera' target mode. The launch's
# yaw=π correction is already baked into the TF chain, so these values
# are the raw translation in link_tcp exactly as declared in the launch.
CAM_OFFSET_X_IN_TCP = -0.06985
CAM_OFFSET_Y_IN_TCP = 0.0
CAM_OFFSET_Z_IN_TCP = 0.127

# Target modes:
#   'camera' — command TCP such that the camera's optical axis ends up on
#              the tag. Gives visual "locked on" feedback because the tag
#              stays in frame center. Use during tracking / approach.
#   'tcp'    — command TCP directly to the tag XY (the camera then sits
#              70 mm off-center in the image). Required for the final
#              grasp so the jaws close on the tool, not next to it.
TARGET_MODE_CAMERA = 'camera'
TARGET_MODE_TCP = 'tcp'
DEFAULT_TARGET_MODE = TARGET_MODE_CAMERA

# Safety gate: at handoff the camera is hovering directly above the
# pickup zone, so the tag's reported link_base XY must be within this
# radius of the TCP's link_base XY. A larger delta means the
# fine_localization TF chain is producing bogus poses (see memory:
# tool_bot_ros2 Phase 10b state), so we refuse to move and log loudly
# instead of letting the arm wander off.
SANITY_XY_RADIUS_M = 0.35

SWEET_PX = {2: 107.1, 3: 116.7, 4: 108.3}
SWEET_TOL_PX = 5.0
GRAB_HOLD_TICKS = 3
GRAB_RATE_HZ = 4.0


def tool_aligned_quat(x, y, phi=None):
    """RPY = (π, 0, phi) as (qx, qy, qz, qw).

    With phi=None, phi defaults to the radial direction atan2(y, x) — the
    IK-friendliest choice on the xArm5 because it lines up with J1's
    natural value and leaves J5 free to match any further constraint.
    Passing phi explicitly lets the caller set an arbitrary tool yaw
    (e.g. from the tag's own yaw in link_base), and IK solves J1+J5 to
    hit it.
    """
    if phi is None:
        phi = math.atan2(y, x)
    return (math.cos(phi / 2.0), math.sin(phi / 2.0), 0.0, 0.0)


def _quat_to_yaw_base(qx, qy, qz, qw):
    """Yaw (rad) of the tag's readable "up" direction in link_base.

    AprilTags are non-symmetric: the printed pattern has a canonical
    readable orientation, with the top of the pattern = tag's local
    −Y axis (dt_apriltags / OpenCV image convention: +X right, +Y down,
    +Z out of the tag). We want the camera's image-vertical to line up
    with the tag's readable vertical, which means the gripper's tool
    yaw should point along the tag's −Y axis in link_base — not its +X
    axis. Using +X put the image vertical 90° off the printed "up",
    which is the mismatch the user saw on the hammer.

    Derivation: −Y of tag in link_base = R @ [0, -1, 0] = -(R[:,1]).
    Standard quaternion → rotation matrix gives the 2nd column as:
        R[0,1] = 2*(qx*qy - qw*qz)
        R[1,1] = 1 - 2*(qx*qx + qz*qz)
    So −Y's horizontal components in link_base are:
        up_x = -2*(qx*qy - qw*qz) = 2*(qw*qz - qx*qy)
        up_y = -(1 - 2*(qx*qx + qz*qz)) = 2*(qx*qx + qz*qz) - 1
    We take atan2(up_y, up_x) and use it verbatim as the tool yaw.
    """
    up_x = 2.0 * (qw * qz - qx * qy)
    up_y = 2.0 * (qx * qx + qz * qz) - 1.0
    return math.atan2(up_y, up_x)


def _point(x, y, z):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _tcp_for_mode(tag_x, tag_y, phi, mode):
    """Return the TCP XY that achieves the requested target mode.

    In 'camera' mode the TCP is shifted by the inverse of the camera
    mount offset (expressed in link_base for the commanded tool yaw
    phi), so after the move the camera's optical axis lands on the tag.
    In 'tcp' mode we command TCP straight at the tag — correct for the
    final grasp, wrong for visual alignment.
    """
    if mode == TARGET_MODE_CAMERA:
        cos_p, sin_p = math.cos(phi), math.sin(phi)
        # R(π, 0, phi) @ (CAM_OFFSET_X, CAM_OFFSET_Y, *) horizontal part:
        dx = CAM_OFFSET_X_IN_TCP * cos_p + CAM_OFFSET_Y_IN_TCP * sin_p
        dy = CAM_OFFSET_X_IN_TCP * sin_p - CAM_OFFSET_Y_IN_TCP * cos_p
        return tag_x - dx, tag_y - dy
    return tag_x, tag_y


class GoTo(Node):

    def __init__(self):
        super().__init__('go_to')
        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter('move_speed', 0.15)
        self.declare_parameter('sweet_tol_px', SWEET_TOL_PX)
        self._tol_px = float(self.get_parameter('sweet_tol_px').value)

        self.move_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        self.gripper_client = ActionClient(
            self, GripperCommand, '/xarm_gripper/gripper_action',
            callback_group=self.cb_group)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # All subs and timers on the same reentrant group → no starvation
        # while a long-running thread is plotting a plan.
        self.create_subscription(
            String, '/voice_command/tool_request', self._tool_request_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            String, '/voice_command/home_request', self._home_request_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            String, '/detect_zone/complete', self._detect_zone_done_cb, 10,
            callback_group=self.cb_group)

        self.grab_pub = self.create_publisher(String, '/tool_approach/grab', 5)

        # Visualization: the projected target TCP pose + MarkerArray
        # showing target position, tag position, and approach arrow.
        # Consumed by RViz; NOT used by any control logic. Motion still
        # only happens on keyboard-driven commit.
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/go_to/target_pose', 5)
        self.viz_pub = self.create_publisher(
            MarkerArray, '/go_to/viz', 5)

        self._tag_sub = None
        self._pixel_sub = None
        self._target_tag_id = None

        self._lock = threading.Lock()
        self._latest_tag_pose = None        # PoseStamped (smoothed) or None
        self._latest_tag_stamp = None       # monotonic seconds
        self._pose_window = deque(maxlen=POSE_SMOOTH_N)  # recent (x,y,z) samples
        self._initial_tcp_z = None          # TCP z at first lock
        self._target_z = None               # current commanded z
        self._last_cmd_xy = None            # last commanded TCP (x, y)
        self._last_cmd_tag_xy = None        # tag (x, y) of last commanded move
        self._target_mode = DEFAULT_TARGET_MODE  # 'camera' or 'tcp'
        self._tracking_paused = False
        self._move_in_flight = False

        self._latest_pixel_px = None
        self._latest_pixel_stamp = None
        self._in_band_count = 0
        self._grab_armed = False
        self._first_viz_published = False  # one-shot log when viz starts
        self._step_idx = 0                 # cycled by 's' through STEP_SIZES_M

        self.create_timer(
            1.0 / TRACK_RATE_HZ, self._track_tick, callback_group=self.cb_group)
        self.create_timer(
            1.0 / GRAB_RATE_HZ, self._grab_tick, callback_group=self.cb_group)

        if sys.stdin.isatty():
            threading.Thread(target=self._keyboard_loop, daemon=True).start()
            sizes_str = '/'.join(f'{s*1000:.1f}mm' for s in STEP_SIZES_M)
            self.get_logger().info(
                'Keyboard (closed-loop commit): ↓/Enter descend+move, '
                '↑ raise+move, m commit-move (no z change), '
                f's cycle step ({sizes_str}), g CLOSE gripper (manual grab), '
                'o OPEN gripper, x toggle camera/tcp mode, h home, '
                'space pause viz, q quit. '
                'Target visualized on /go_to/target_pose + /go_to/viz — '
                'arm does NOT auto-move; every key press rebuilds IK from '
                'the live tag pose.')
        else:
            self.get_logger().warn('No TTY — keyboard disabled.')

        self.get_logger().info(
            f'go_to ready. Sweet sizes={SWEET_PX} tol={self._tol_px:.1f}px '
            f'track={TRACK_RATE_HZ}Hz step={STEP_M*1000:.0f}mm '
            f'XY_deadband={XY_DEADBAND_M*1000:.0f}mm.')
        self.get_logger().info(
            'Subscribed: /voice_command/tool_request, '
            '/voice_command/home_request, /detect_zone/complete.')

    # ---------------------- subscriptions ----------------------

    def _tool_request_cb(self, msg):
        fid = _parse_int(_parse_field(msg.data, 'fiducial'))
        self.get_logger().info(
            f'TOOL_REQUEST received: "{msg.data}" → parsed fiducial={fid}.')
        if fid not in TOOL_TAG_IDS:
            return
        self._reset_lock_state(target=fid)

    def _home_request_cb(self, msg):
        self.get_logger().warn(
            f'HOME_REQUEST received on /voice_command/home_request: '
            f'"{msg.data}" — dispatching home thread.')
        self._reset_lock_state(target=None)
        threading.Thread(target=self._go_home, daemon=True).start()

    def _detect_zone_done_cb(self, msg):
        fid = _parse_int(_parse_field(msg.data, 'fiducial'))
        self.get_logger().info(
            f'DETECT_ZONE_COMPLETE received: "{msg.data}" → fiducial={fid}.')
        if fid not in TOOL_TAG_IDS:
            return
        self._reset_lock_state(target=fid)
        with self._lock:
            self._tag_sub = self.create_subscription(
                PoseStamped, f'/fine_loc/tag_{fid}',
                self._tag_pose_cb, 10, callback_group=self.cb_group)
            self._pixel_sub = self.create_subscription(
                Float32, f'/fine_loc/tag_{fid}/pixel_size',
                self._pixel_cb, 10, callback_group=self.cb_group)
        self.get_logger().info(
            f'Subscribed to /fine_loc/tag_{fid} (+ pixel_size). '
            'Tracking will engage on first pose received.')

    def _reset_lock_state(self, target):
        with self._lock:
            self._target_tag_id = target
            self._latest_tag_pose = None
            self._latest_tag_stamp = None
            self._pose_window.clear()
            self._initial_tcp_z = None
            self._target_z = None
            self._last_cmd_xy = None
            self._last_cmd_tag_xy = None
            self._tracking_paused = False
            self._latest_pixel_px = None
            self._latest_pixel_stamp = None
            self._in_band_count = 0
            self._grab_armed = False
            self._first_viz_published = False
            if self._tag_sub is not None:
                self.destroy_subscription(self._tag_sub)
                self._tag_sub = None
            if self._pixel_sub is not None:
                self.destroy_subscription(self._pixel_sub)
                self._pixel_sub = None

    def _tag_pose_cb(self, msg):
        raw = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        with self._lock:
            # Outlier reject: if we already have a running mean, throw out
            # any single pose more than XY_JUMP_REJECT_M away from it.
            if len(self._pose_window) >= 3:
                mx = sum(p[0] for p in self._pose_window) / len(self._pose_window)
                my = sum(p[1] for p in self._pose_window) / len(self._pose_window)
                if math.hypot(raw[0] - mx, raw[1] - my) > XY_JUMP_REJECT_M:
                    self.get_logger().warn(
                        f'Rejected jumpy tag pose '
                        f'({raw[0]*1000:.0f},{raw[1]*1000:.0f})mm '
                        f'vs avg ({mx*1000:.0f},{my*1000:.0f})mm.')
                    return
            self._pose_window.append(raw)
            ax = sum(p[0] for p in self._pose_window) / len(self._pose_window)
            ay = sum(p[1] for p in self._pose_window) / len(self._pose_window)
            az = sum(p[2] for p in self._pose_window) / len(self._pose_window)
            # Build a smoothed PoseStamped snapshot.
            smoothed = PoseStamped()
            smoothed.header = msg.header
            smoothed.pose.position.x = ax
            smoothed.pose.position.y = ay
            smoothed.pose.position.z = az
            smoothed.pose.orientation = msg.pose.orientation
            self._latest_tag_pose = smoothed
            self._latest_tag_stamp = time.monotonic()
            if self._initial_tcp_z is None and len(self._pose_window) >= 3:
                tcp = self._tcp_position_unsafe()
                if tcp is not None:
                    self._initial_tcp_z = tcp[2]
                    # Auto-descend: drop INITIAL_DESCEND_M below current
                    # TCP z, but keep INITIAL_APPROACH_STANDOFF_M above
                    # the tag itself. Next tick replans to this z so the
                    # arm visibly moves closer right after acquire.
                    # Clamp to never ASCEND — if the tag-floor or safety
                    # floor is already above current TCP, that means we
                    # are already at/near the approach standoff and we
                    # should simply hold z and let keyboard take over,
                    # not command the arm upward.
                    descent_candidate = tcp[2] - INITIAL_DESCEND_M
                    tag_floor = az + INITIAL_APPROACH_STANDOFF_M
                    safety_floor = SAFETY_Z_FLOOR + 0.02
                    initial_target = max(
                        descent_candidate, tag_floor, safety_floor)
                    initial_target = min(initial_target, tcp[2])
                    self._target_z = initial_target
                    self.get_logger().warn(
                        f'AUTO_DESCEND components: '
                        f'tcp_z={tcp[2]*1000:+.0f}mm  '
                        f'tag_z={az*1000:+.0f}mm  '
                        f'descent_candidate={descent_candidate*1000:+.0f}mm  '
                        f'tag_floor={tag_floor*1000:+.0f}mm  '
                        f'safety_floor={safety_floor*1000:+.0f}mm  '
                        f'→ target_z={initial_target*1000:+.0f}mm  '
                        f'descent_from_tcp={(tcp[2]-initial_target)*1000:+.0f}mm')
                    # First-pose self-check: at hover the camera is ~
                    # directly above the tag, so tag XY in link_base
                    # should be near TCP XY (within a few cm due to the
                    # camera-to-TCP mount offset). If it's wildly off,
                    # fine_localization's pose is bogus — log loudly
                    # and let _track_tick's safety gate block the move.
                    dx = ax - tcp[0]
                    dy = ay - tcp[1]
                    d_xy = math.hypot(dx, dy)
                    verdict = 'SANE' if d_xy <= SANITY_XY_RADIUS_M else 'BOGUS'
                    tool = TOOL_NAMES.get(
                        self._target_tag_id, f'tag{self._target_tag_id}')
                    self.get_logger().warn(
                        f'FIRST tag pose [{verdict}] tool={tool} '
                        f'(id={self._target_tag_id}): '
                        f'tag=({ax*1000:+.0f},{ay*1000:+.0f},{az*1000:+.0f})mm  '
                        f'tcp=({tcp[0]*1000:+.0f},{tcp[1]*1000:+.0f},'
                        f'{tcp[2]*1000:+.0f})mm  '
                        f'delta_xy=({dx*1000:+.0f},{dy*1000:+.0f})mm '
                        f'|Δ|={d_xy*1000:.0f}mm  '
                        f'(limit={SANITY_XY_RADIUS_M*1000:.0f}mm)  '
                        f'auto_descend_to_z={initial_target*1000:+.0f}mm')
                    if d_xy > SANITY_XY_RADIUS_M:
                        self.get_logger().error(
                            'First tag pose is further from TCP than the '
                            'sanity radius. Refusing to track — check the '
                            'static TF link_tcp→camera_link in '
                            'depth_camera.launch.py and the /fine_loc/tag '
                            'output. Run `ros2 run xarm5_basic_position_cmd '
                            'pose_check` for a side-by-side readout.')
                    else:
                        # One-shot banner right after first lock. The
                        # refactored go_to does NOT auto-move; without
                        # this prompt a new operator stares at the log
                        # wondering why the arm is idle.
                        self.get_logger().warn(
                            '╔══ LOCKED ON (closed-loop viz ready) ══╗\n'
                            '║  Arm does NOT auto-move. Target is    ║\n'
                            '║  published live on /go_to/target_pose ║\n'
                            '║  + /go_to/viz (MarkerArray).          ║\n'
                            '║                                       ║\n'
                            '║  Press  m  → center camera on tag     ║\n'
                            '║  Press  ↓  → descend 10 mm and move   ║\n'
                            '║  Press  ↑  → raise   10 mm and move   ║\n'
                            '║  Press  x  → toggle camera/tcp mode   ║\n'
                            '╚═══════════════════════════════════════╝')

    def _pixel_cb(self, msg):
        v = float(msg.data)
        if not math.isfinite(v) or v <= 0.0:
            return
        with self._lock:
            self._latest_pixel_px = v
            self._latest_pixel_stamp = time.monotonic()

    # ---------------------- tracking tick ----------------------

    def _compute_target(self):
        """Snapshot live tag pose → (tcp_x, tcp_y, target_z, tag_yaw, mode).

        Closed-loop: every call reads _latest_tag_pose and recomputes.
        Returns a dict, or None if we aren't locked yet / the tag sample
        is stale. Holding the lock only for the pose read; yaw and TCP
        offset math happen outside the lock.
        """
        with self._lock:
            if self._latest_tag_pose is None or self._target_z is None:
                return None
            if self._latest_tag_stamp is None:
                return None
            if (time.monotonic() - self._latest_tag_stamp) > 2.0:
                return None
            pose = self._latest_tag_pose
            tag_x = pose.pose.position.x
            tag_y = pose.pose.position.y
            tag_z = pose.pose.position.z
            tag_q = (
                pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.w,
            )
            target_z = self._target_z
            mode = self._target_mode
            target_tid = self._target_tag_id
        tag_yaw = _quat_to_yaw_base(*tag_q)
        tcp_x, tcp_y = _tcp_for_mode(tag_x, tag_y, tag_yaw, mode)
        return {
            'tag_x': tag_x, 'tag_y': tag_y, 'tag_z': tag_z, 'tag_q': tag_q,
            'tcp_x': tcp_x, 'tcp_y': tcp_y, 'target_z': target_z,
            'tag_yaw': tag_yaw, 'mode': mode, 'target_tid': target_tid,
        }

    def _publish_target_viz(self, t):
        """Publish the projected target TCP pose + MarkerArray for RViz."""
        stamp = self.get_clock().now().to_msg()

        qx, qy, qz, qw = tool_aligned_quat(t['tcp_x'], t['tcp_y'], t['tag_yaw'])

        p = PoseStamped()
        p.header.frame_id = FRAME_ID
        p.header.stamp = stamp
        p.pose.position.x = t['tcp_x']
        p.pose.position.y = t['tcp_y']
        p.pose.position.z = t['target_z']
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        self.target_pose_pub.publish(p)

        arr = MarkerArray()

        # id=0: SPHERE at target TCP — green = safe / ready to commit.
        tcp_marker = Marker()
        tcp_marker.header.frame_id = FRAME_ID
        tcp_marker.header.stamp = stamp
        tcp_marker.ns = 'go_to'
        tcp_marker.id = 0
        tcp_marker.type = Marker.SPHERE
        tcp_marker.action = Marker.ADD
        tcp_marker.pose = p.pose
        tcp_marker.scale.x = 0.025
        tcp_marker.scale.y = 0.025
        tcp_marker.scale.z = 0.025
        tcp_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.9)
        arr.markers.append(tcp_marker)

        # id=1: CUBE at the detected tag center (actual tag z, not
        # target z) — yellow. 25.4 mm physical tag, 3 mm thick for
        # visibility. Orientation matches the tag so RViz shows which
        # way "up" points.
        tag_marker = Marker()
        tag_marker.header.frame_id = FRAME_ID
        tag_marker.header.stamp = stamp
        tag_marker.ns = 'go_to'
        tag_marker.id = 1
        tag_marker.type = Marker.CUBE
        tag_marker.action = Marker.ADD
        tag_marker.pose.position.x = t['tag_x']
        tag_marker.pose.position.y = t['tag_y']
        tag_marker.pose.position.z = t['tag_z']
        tag_marker.pose.orientation.x = t['tag_q'][0]
        tag_marker.pose.orientation.y = t['tag_q'][1]
        tag_marker.pose.orientation.z = t['tag_q'][2]
        tag_marker.pose.orientation.w = t['tag_q'][3]
        tag_marker.scale.x = 0.0254
        tag_marker.scale.y = 0.0254
        tag_marker.scale.z = 0.003
        tag_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
        arr.markers.append(tag_marker)

        # id=2: ARROW from target TCP down toward the tag — blue,
        # shows the gripper's approach direction.
        approach = Marker()
        approach.header.frame_id = FRAME_ID
        approach.header.stamp = stamp
        approach.ns = 'go_to'
        approach.id = 2
        approach.type = Marker.ARROW
        approach.action = Marker.ADD
        approach.points = [
            p.pose.position,
            _point(t['tag_x'], t['tag_y'], t['tag_z']),
        ]
        approach.scale.x = 0.004   # shaft diameter
        approach.scale.y = 0.010   # head diameter
        approach.scale.z = 0.014   # head length
        approach.color = ColorRGBA(r=0.1, g=0.3, b=1.0, a=0.85)
        arr.markers.append(approach)

        self.viz_pub.publish(arr)

    def _safety_check(self, tag_x, tag_y):
        """Reject plans where tag XY is absurdly far from current TCP.

        At hover the camera is roughly above the tag, so tag XY in
        link_base should be close to TCP XY. A big delta means the
        fine_localization pose is bogus — refuse to act on it.
        Returns (ok, d_xy_m, tcp_xyz_or_None).
        """
        tcp = self._tcp_position_unsafe()
        if tcp is None:
            return True, 0.0, None
        d_xy = math.hypot(tag_x - tcp[0], tag_y - tcp[1])
        return d_xy <= SANITY_XY_RADIUS_M, d_xy, tcp

    def _track_tick(self):
        """Closed-loop visualization tick — NO arm motion here.

        Every tick: read live tag pose, recompute projected target,
        publish pose + markers for RViz. Motion only fires when the
        operator presses a key (_adjust_z / _commit_move_to_target).
        """
        with self._lock:
            paused = self._tracking_paused
        if paused:
            return
        t = self._compute_target()
        if t is None:
            return
        ok, d_xy, tcp = self._safety_check(t['tag_x'], t['tag_y'])
        if not ok:
            self.get_logger().warn(
                f'VIZ gate: tag XY is {d_xy*1000:.0f}mm from TCP '
                f'(>limit {SANITY_XY_RADIUS_M*1000:.0f}mm). '
                f'tag=({t["tag_x"]*1000:+.0f},{t["tag_y"]*1000:+.0f}) '
                f'tcp=({tcp[0]*1000:+.0f},{tcp[1]*1000:+.0f}). '
                'Fine localization pose looks wrong — not publishing viz.',
                throttle_duration_sec=2.0)
            return
        self._publish_target_viz(t)
        if not self._first_viz_published:
            self._first_viz_published = True
            self.get_logger().warn(
                'VIZ FIRST PUBLISH — /go_to/target_pose + /go_to/viz are '
                'now live. Refactored closed-loop build IS running. '
                'Press m/↓/↑ to commit a move.')
        tool = TOOL_NAMES.get(t['target_tid'], f'tag{t["target_tid"]}')
        tcp_txt = (f'tcp({tcp[0]*1000:.0f},{tcp[1]*1000:.0f},'
                   f'{tcp[2]*1000:.0f})' if tcp is not None else 'tcp(?)')
        self.get_logger().info(
            f'VIZ[{tool}|{t["mode"]}] → {tcp_txt} → '
            f'target_tcp=({t["tcp_x"]*1000:.0f},{t["tcp_y"]*1000:.0f},'
            f'{t["target_z"]*1000:.0f})mm  '
            f'tag=({t["tag_x"]*1000:.0f},{t["tag_y"]*1000:.0f},'
            f'{t["tag_z"]*1000:.0f})mm  '
            f'tag_up_yaw={math.degrees(t["tag_yaw"]):+.0f}°  '
            f'radial_yaw={math.degrees(math.atan2(t["tag_y"], t["tag_x"])):+.0f}°',
            throttle_duration_sec=1.5)

    def _commit_move_to_target(self, reason):
        """Rebuild IK target from LIVE tag pose and execute the move.

        Called from the keyboard thread on ↓/↑/m. Each call is a fresh
        closed-loop sample: the cached _last_cmd_* fields are only used
        by the deadband check inside _track_tick (now vestigial), not
        by the IK request itself.
        """
        with self._lock:
            in_flight = self._move_in_flight
        if in_flight:
            self.get_logger().warn(
                f'COMMIT[{reason}] ignored — move already in flight.')
            return
        t = self._compute_target()
        if t is None:
            self.get_logger().warn(
                f'COMMIT[{reason}] ignored — no tag lock / stale pose.')
            return
        ok, d_xy, tcp = self._safety_check(t['tag_x'], t['tag_y'])
        if not ok:
            self.get_logger().error(
                f'COMMIT[{reason}] blocked: tag XY is {d_xy*1000:.0f}mm '
                f'from TCP (>limit {SANITY_XY_RADIUS_M*1000:.0f}mm). '
                'Fine localization pose looks wrong — refusing to move.')
            return
        self.get_logger().warn(
            f'COMMIT[{reason}] → tcp_target=({t["tcp_x"]*1000:+.0f},'
            f'{t["tcp_y"]*1000:+.0f},{t["target_z"]*1000:+.0f})mm '
            f'mode={t["mode"]} yaw={math.degrees(t["tag_yaw"]):+.0f}°')
        threading.Thread(
            target=self._do_move,
            args=(t['tag_x'], t['tag_y'], t['target_z'],
                  t['tag_yaw'], t['mode']),
            daemon=True).start()

    def _do_move(self, tag_x, tag_y, z, tag_yaw=None, mode=DEFAULT_TARGET_MODE):
        with self._lock:
            if self._move_in_flight:
                return
            self._move_in_flight = True
        try:
            if z < SAFETY_Z_FLOOR + 0.005:
                self.get_logger().warn(
                    f'z={z*1000:.0f}mm below floor — clamping.')
                z = SAFETY_Z_FLOOR + 0.005

            # Try tag-yaw first (so the gripper's jaws align with the
            # tool's long axis), then fall back to the radial yaw if IK
            # can't satisfy the tag-yaw request. Fallback is the
            # always-feasible choice on this 5-DOF arm.
            radial_yaw = math.atan2(tag_y, tag_x)
            candidates = []
            if tag_yaw is not None:
                candidates.append(('tag', tag_yaw))
            candidates.append(('radial', radial_yaw))

            joints = None
            used_label = None
            used_phi = None
            used_tcp = None
            for label, phi in candidates:
                tcp_x, tcp_y = _tcp_for_mode(tag_x, tag_y, phi, mode)
                pose = PoseStamped()
                pose.header.frame_id = FRAME_ID
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = tcp_x
                pose.pose.position.y = tcp_y
                pose.pose.position.z = z
                qx, qy, qz, qw = tool_aligned_quat(tcp_x, tcp_y, phi)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                joints = self._ik(pose)
                if joints is not None:
                    used_label = label
                    used_phi = phi
                    used_tcp = (tcp_x, tcp_y)
                    break
                self.get_logger().warn(
                    f'IK failed with {label} yaw={math.degrees(phi):+.0f}° '
                    f'tcp=({tcp_x*1000:.0f},{tcp_y*1000:.0f})mm; '
                    'trying next candidate.')
            if joints is None:
                return

            label = (f'track[{mode}/{used_label}] tag=({tag_x:.3f},'
                     f'{tag_y:.3f}) tcp=({used_tcp[0]:.3f},'
                     f'{used_tcp[1]:.3f}) z={z:.3f} '
                     f'yaw={math.degrees(used_phi):+.0f}°')
            ok = self._joint_move(joints, label)
            if ok:
                with self._lock:
                    self._last_cmd_xy = used_tcp
                    self._last_cmd_tag_xy = (tag_x, tag_y)
        finally:
            with self._lock:
                self._move_in_flight = False

    def _adjust_z(self, delta_m):
        """Step target_z by delta_m and immediately commit a move to the
        current LIVE target. No waiting for the next track tick."""
        with self._lock:
            if self._initial_tcp_z is None:
                self.get_logger().warn(
                    'No tag locked yet — cannot step Z. Run detect_zone first.')
                return
            new_z = (self._target_z or self._initial_tcp_z) + delta_m
            min_z = max(SAFETY_Z_FLOOR, self._initial_tcp_z - MAX_DOWN_M)
            max_z = self._initial_tcp_z + MAX_UP_M
            clamped = max(min_z, min(max_z, new_z))
            if clamped != new_z:
                self.get_logger().warn(
                    f'Z {new_z*1000:.0f}mm clamped to {clamped*1000:.0f}mm '
                    f'(range {min_z*1000:.0f}-{max_z*1000:.0f}mm).')
            self._target_z = clamped
            # Clear deadband anchors — they're vestigial now (tick no
            # longer auto-moves) but some other code path may still read
            # them; keep them consistent with "a fresh commit happened".
            self._last_cmd_xy = None
            self._last_cmd_tag_xy = None
        direction = 'DOWN' if delta_m < 0 else 'UP'
        self.get_logger().info(
            f'Z step {direction} {abs(delta_m)*1000:.0f}mm → '
            f'target_z={self._target_z*1000:.0f}mm — committing move.')
        self._commit_move_to_target(reason=f'z{direction.lower()}')

    # ---------------------- grab tick ----------------------

    def _grab_tick(self):
        with self._lock:
            if self._grab_armed:
                return
            tid = self._target_tag_id
            px = self._latest_pixel_px
            stamp = self._latest_pixel_stamp
        if tid is None or px is None or stamp is None:
            return
        if (time.monotonic() - stamp) > 1.0:
            with self._lock:
                self._in_band_count = 0
            return
        sweet = SWEET_PX.get(tid)
        if sweet is None:
            return
        in_band = abs(px - sweet) <= self._tol_px
        with self._lock:
            self._in_band_count = self._in_band_count + 1 if in_band else 0
            ready = (self._in_band_count >= GRAB_HOLD_TICKS
                     and not self._grab_armed)
            if ready:
                self._grab_armed = True
                self._tracking_paused = True
        if ready:
            self._fire_grab(tid, px, sweet)

    def _fire_grab(self, tid, px, sweet):
        delta = px - sweet
        m = String()
        m.data = f'fiducial={tid}|px={px:.1f}|sweet={sweet:.1f}|delta={delta:+.1f}'
        self.grab_pub.publish(m)
        self.get_logger().info(
            f'GRAB ARMED  → published /tool_approach/grab "{m.data}"')

    def _toggle_target_mode(self):
        with self._lock:
            new_mode = (TARGET_MODE_TCP
                        if self._target_mode == TARGET_MODE_CAMERA
                        else TARGET_MODE_CAMERA)
            self._target_mode = new_mode
            # Force a replan on the next tick so the switch is immediate.
            self._last_cmd_xy = None
            self._last_cmd_tag_xy = None
        self.get_logger().warn(
            f'Target mode → {new_mode}. '
            f'{"Camera will recenter on the tag." if new_mode == TARGET_MODE_CAMERA else "TCP will move over the tag (camera ~70 mm off-center is EXPECTED)."}')

    def _force_grab(self):
        with self._lock:
            tid = self._target_tag_id
            px = self._latest_pixel_px or 0.0
            sweet = SWEET_PX.get(tid, 0.0)
            self._grab_armed = True
            self._tracking_paused = True
        if tid is None:
            self.get_logger().warn('Force-grab ignored: no target tag.')
            return
        self._fire_grab(tid, px, sweet)

    # ---------------------- manual approach controls ----------------------

    def _cycle_step_size(self):
        """Rotate self._step_idx through STEP_SIZES_M.

        Used while keyboard-positioning the arm over a tool: start coarse
        (10 mm) to descend quickly, then drop to 5 mm and 1 mm for the
        final placement before pressing 'g' to close the gripper.
        """
        self._step_idx = (self._step_idx + 1) % len(STEP_SIZES_M)
        sizes_str = ' / '.join(
            f'{"[" if i == self._step_idx else " "}{s*1000:.1f}mm'
            f'{"]" if i == self._step_idx else " "}'
            for i, s in enumerate(STEP_SIZES_M))
        self.get_logger().warn(
            f'Approach step → {STEP_SIZES_M[self._step_idx]*1000:.1f}mm   '
            f'cycle: {sizes_str}')

    def _send_gripper_blocking(self, position, label):
        """Send a GripperCommand goal and wait for the result.

        Run from a worker thread (the keyboard loop's caller spawns one)
        so the action wait doesn't block the keyboard reader.
        """
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'GRIPPER[{label}]: action server '
                '/xarm_gripper/gripper_action not available — '
                'is the xarm driver up? (T1)')
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(GRIPPER_EFFORT)
        self.get_logger().warn(
            f'GRIPPER[{label}] → position={position:.3f} '
            f'effort={GRIPPER_EFFORT:.2f}')
        event = threading.Event()
        accepted = [False]

        def on_result(_f):
            event.set()

        def on_goal(f):
            gh = f.result()
            if gh is None or not gh.accepted:
                self.get_logger().error(
                    f'GRIPPER[{label}] goal rejected by action server.')
                event.set()
                return
            accepted[0] = True
            gh.get_result_async().add_done_callback(on_result)

        self.gripper_client.send_goal_async(goal).add_done_callback(on_goal)
        if not event.wait(timeout=GRIPPER_ACTION_TIMEOUT_S):
            self.get_logger().warn(f'GRIPPER[{label}] timed out.')
            return
        if accepted[0]:
            self.get_logger().info(f'GRIPPER[{label}] complete.')

    def _grab_close(self):
        """Manual grab: close the gripper. Operator-driven, no tag check.

        Called when the operator has visually positioned the arm over the
        tool with the keyboard. Also marks tracking paused + grab_armed
        so the auto-grab path doesn't re-fire on top of us.
        """
        with self._lock:
            self._grab_armed = True
            self._tracking_paused = True
        threading.Thread(
            target=self._send_gripper_blocking,
            args=(GRIPPER_CLOSE_POS, 'CLOSE'),
            daemon=True).start()

    def _grab_open(self):
        """Open the gripper to release / reset between manual attempts."""
        with self._lock:
            self._grab_armed = False
        threading.Thread(
            target=self._send_gripper_blocking,
            args=(GRIPPER_OPEN_POS, 'OPEN'),
            daemon=True).start()

    # ---------------------- home ----------------------

    def _go_home(self):
        self.get_logger().warn('HOME thread started.')
        deadline = time.monotonic() + 6.0
        owned = False
        while time.monotonic() < deadline:
            with self._lock:
                if not self._move_in_flight:
                    self._move_in_flight = True
                    owned = True
                    break
            time.sleep(0.05)
        if not owned:
            self.get_logger().error(
                'HOME: in-flight move did not clear in 6s. Forcing.')
            with self._lock:
                self._move_in_flight = True
                owned = True
        try:
            self.get_logger().warn(
                'HOME: planning all-zero joint goal via MoveGroup...')
            joints = dict(zip(JOINT_NAMES, HOME_JOINTS))
            ok = self._joint_move(joints, 'HOME(all-zero)')
            if ok:
                self.get_logger().warn('HOME: completed.')
            else:
                self.get_logger().error(
                    'HOME: MoveGroup did not accept/complete. '
                    'Check T1 terminal — move_group may have crashed.')
        finally:
            with self._lock:
                self._move_in_flight = False

    def _tcp_position_unsafe(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                FRAME_ID, 'link_tcp', rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None

    # ---------------------- IK + MoveGroup ----------------------

    def _ik(self, pose):
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IK service unavailable.')
            return None
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        req.ik_request.pose_stamped = pose
        event = threading.Event()
        holder = [None]
        self.ik_client.call_async(req).add_done_callback(
            lambda f: (holder.__setitem__(0, f.result()), event.set()))
        if not event.wait(timeout=3.0):
            self.get_logger().warn('IK call timeout.')
            return None
        resp = holder[0]
        if resp is None or resp.error_code.val != 1:
            code = resp.error_code.val if resp else 'None'
            self.get_logger().warn(
                f'IK failed (code={code}) for pose '
                f'({pose.pose.position.x:.3f},{pose.pose.position.y:.3f},'
                f'{pose.pose.position.z:.3f}).')
            return None
        jv = {n: p for n, p in zip(
            resp.solution.joint_state.name, resp.solution.joint_state.position)
            if n in JOINT_NAMES}
        if len(jv) != 5:
            return None
        return jv

    def _joint_move(self, joint_values, label):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'[{label}] MoveGroup action server NOT available — '
                'move_group is dead. Restart T1.')
            return False
        spd = float(self.get_parameter('move_speed').value)
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = PLANNING_GROUP
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = spd
        goal.request.max_acceleration_scaling_factor = spd
        c = Constraints()
        for name in JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = joint_values[name]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        event = threading.Event()
        accepted = [False]

        def on_goal(f):
            gh = f.result()
            if gh is None or not gh.accepted:
                event.set()
                return
            accepted[0] = True
            gh.get_result_async().add_done_callback(
                lambda _f2: event.set())

        self.move_client.send_goal_async(goal).add_done_callback(on_goal)
        if not event.wait(timeout=20.0):
            self.get_logger().warn(f'[{label}] Move timeout.')
            return False
        if not accepted[0]:
            self.get_logger().warn(f'[{label}] Move rejected.')
            return False
        self.get_logger().info(f'[{label}] Move done.')
        return True

    # ---------------------- keyboard ----------------------

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        try:
            old = termios.tcgetattr(fd)
        except termios.error:
            return
        try:
            tty.setraw(fd)
            while rclpy.ok():
                if not select.select([sys.stdin], [], [], 0.1)[0]:
                    continue
                c = sys.stdin.read(1)
                step = STEP_SIZES_M[self._step_idx]
                if c == '\x1b':
                    seq = sys.stdin.read(2)
                    if seq == '[A':
                        self._adjust_z(+step)
                    elif seq == '[B':
                        self._adjust_z(-step)
                elif c in ('\r', '\n'):
                    self._adjust_z(-step)
                elif c == ' ':
                    with self._lock:
                        self._tracking_paused = not self._tracking_paused
                    self.get_logger().info(
                        f'Tracking {"PAUSED" if self._tracking_paused else "RESUMED"}.')
                elif c == 's':
                    self._cycle_step_size()
                elif c == 'g':
                    self._grab_close()
                elif c == 'o':
                    self._grab_open()
                elif c == 'm':
                    self._commit_move_to_target(reason='manual')
                elif c == 'x':
                    self._toggle_target_mode()
                elif c == 'h':
                    threading.Thread(target=self._go_home, daemon=True).start()
                elif c in ('q', '\x03'):
                    self.get_logger().info('Quit from keyboard.')
                    rclpy.shutdown()
                    return
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _parse_field(text, field):
    for part in text.split('|'):
        if part.startswith(f'{field}='):
            return part.split('=', 1)[1]
    return None


def _parse_int(s):
    try:
        return int(s) if s is not None else None
    except ValueError:
        return None


def main(args=None):
    rclpy.init(args=args)
    node = GoTo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
