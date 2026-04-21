#!/usr/bin/env python3
"""
go_to.py — continuous AprilTag tracking + descent + grab + home.

Behaviour after detect_zone hovers over the pickup zone:

  /detect_zone/complete fiducial=N  → go_to subscribes to:
    /fine_loc/tag_N            (PoseStamped, in link_base)
    /fine_loc/tag_N/pixel_size (Float32)

  Tracking timer (TRACK_RATE_HZ): every tick reads the LATEST tag pose
  (not a one-time snapshot). If the tag has moved more than XY_DEADBAND_M
  from the last commanded XY, OR the operator changed the Z offset with
  the keyboard, plans an IK + joint-space move to (tag_x, tag_y, target_z)
  with the J5 yaw aligned to the tool. This means: the arm continuously
  follows the tag in XY while the operator paces the descent in Z.

  Keyboard:
    ↓ / Enter : descend 10 mm (target_z -= STEP_M)
    ↑         : raise   10 mm
    g         : force grab trigger
    space     : pause tracking
    h         : send arm home (all joints zero)
    q         : quit

  Voice:
    /voice_command/home_request → arm home (high-priority, with verbose log)
    /voice_command/tool_request → reset state, wait for new detect_zone

Grab arming: when /fine_loc/tag_N/pixel_size lands within sweet_px ± tol
for GRAB_HOLD_TICKS consecutive samples, publishes /tool_approach/grab
and pauses tracking until reset.

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

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest, PositionIKRequest,
)
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Float32, String


PLANNING_GROUP = 'xarm5'
FRAME_ID = 'link_base'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
SAFETY_Z_FLOOR = -0.110
TOOL_TAG_IDS = (2, 3, 4)
TOOL_NAMES = {2: 'phillips', 3: 'hammer', 4: 'flathead'}
HOME_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0)

STEP_M = 0.010
MAX_DOWN_M = 0.40
MAX_UP_M = 0.20
XY_DEADBAND_M = 0.015      # re-plan when smoothed tag XY shifts >1.5 cm
XY_JUMP_REJECT_M = 0.10    # any single pose >10 cm from running avg → ignore
TRACK_RATE_HZ = 1.5        # tracking loop rate (slow enough for plan + move)
POSE_SMOOTH_N = 5          # running-average window for /fine_loc/tag_N poses

# Auto-descent on first lock: drop this far below current TCP z, but never
# closer than INITIAL_APPROACH_STANDOFF_M to the tag's own z. Removes the
# "arm just sits there after acquire waiting for the ↓ key" feel.
INITIAL_DESCEND_M = 0.10
INITIAL_APPROACH_STANDOFF_M = 0.12

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
    """Yaw (rad) of the tag's local +X axis when expressed in link_base.

    The AprilTag detection produces a unique, non-symmetric orientation
    (each tag's pattern has a defined "up"), and callers must respect
    that — no 180° wrap, no axis swap. This function just returns the
    raw yaw of the tag's +X axis in link_base, and callers use it
    verbatim as the commanded tool yaw.
    """
    rx = 1.0 - 2.0 * (qy * qy + qz * qz)
    ry = 2.0 * (qx * qy + qw * qz)
    return math.atan2(ry, rx)


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

        self.create_timer(
            1.0 / TRACK_RATE_HZ, self._track_tick, callback_group=self.cb_group)
        self.create_timer(
            1.0 / GRAB_RATE_HZ, self._grab_tick, callback_group=self.cb_group)

        if sys.stdin.isatty():
            threading.Thread(target=self._keyboard_loop, daemon=True).start()
            self.get_logger().info(
                'Keyboard: ↓/Enter descend, ↑ raise, g grab, space pause, '
                'x toggle camera/tcp mode, h home, q quit.')
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
                    initial_target = tcp[2] - INITIAL_DESCEND_M
                    initial_target = max(
                        initial_target, az + INITIAL_APPROACH_STANDOFF_M)
                    initial_target = max(
                        initial_target, SAFETY_Z_FLOOR + 0.02)
                    self._target_z = initial_target
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

    def _pixel_cb(self, msg):
        v = float(msg.data)
        if not math.isfinite(v) or v <= 0.0:
            return
        with self._lock:
            self._latest_pixel_px = v
            self._latest_pixel_stamp = time.monotonic()

    # ---------------------- tracking tick ----------------------

    def _track_tick(self):
        with self._lock:
            if self._tracking_paused or self._move_in_flight:
                return
            if self._latest_tag_pose is None or self._target_z is None:
                return
            if (time.monotonic() - self._latest_tag_stamp) > 1.5:
                return  # stale tag — hold pose
            tag_x = self._latest_tag_pose.pose.position.x
            tag_y = self._latest_tag_pose.pose.position.y
            tag_q = (
                self._latest_tag_pose.pose.orientation.x,
                self._latest_tag_pose.pose.orientation.y,
                self._latest_tag_pose.pose.orientation.z,
                self._latest_tag_pose.pose.orientation.w,
            )
            target_z = self._target_z
            last_cmd_tag = self._last_cmd_tag_xy
            target_tid = self._target_tag_id
            mode = self._target_mode

        # Safety gate: never plan to an XY that's absurdly far from the
        # current TCP. At hover the camera is roughly above the tag, so
        # tag XY in link_base should be close to TCP XY; a huge delta
        # means the pose is bogus and we'd wander off blindly.
        tcp = self._tcp_position_unsafe()
        if tcp is not None:
            d_xy = math.hypot(tag_x - tcp[0], tag_y - tcp[1])
            if d_xy > SANITY_XY_RADIUS_M:
                self.get_logger().warn(
                    f'TRACK blocked: tag XY is {d_xy*1000:.0f}mm from TCP '
                    f'(>limit {SANITY_XY_RADIUS_M*1000:.0f}mm). '
                    f'tag=({tag_x*1000:+.0f},{tag_y*1000:+.0f}) '
                    f'tcp=({tcp[0]*1000:+.0f},{tcp[1]*1000:+.0f}). '
                    'Fine localization pose looks wrong — not moving.',
                    throttle_duration_sec=2.0)
                return

        # Re-plan when the tag XY shifted past the deadband OR no command
        # has been issued yet. Comparing against the last TAG XY (not last
        # TCP XY) keeps the deadband meaningful across mode changes.
        if last_cmd_tag is not None:
            dx = tag_x - last_cmd_tag[0]
            dy = tag_y - last_cmd_tag[1]
            if math.hypot(dx, dy) < XY_DEADBAND_M:
                return  # within deadband — skip
        tcp_txt = (f'tcp({tcp[0]*1000:.0f},{tcp[1]*1000:.0f},{tcp[2]*1000:.0f})'
                   if tcp is not None else 'tcp(?)')
        tool = TOOL_NAMES.get(target_tid, f'tag{target_tid}')
        radial_yaw = math.atan2(tag_y, tag_x)
        # Raw tag yaw from /fine_loc — used verbatim, no 180° wrap. The
        # AprilTag has a defined orientation (non-symmetric pattern), so
        # collapsing it toward the radial direction would swap axes.
        tag_yaw = _quat_to_yaw_base(*tag_q)
        self.get_logger().info(
            f'TRACK[{tool}|{mode}] → {tcp_txt} → tag({tag_x*1000:.0f},'
            f'{tag_y*1000:.0f}) z={target_z*1000:.0f}mm  '
            f'radial_yaw={math.degrees(radial_yaw):+.0f}°  '
            f'tag_yaw={math.degrees(tag_yaw):+.0f}°')
        threading.Thread(
            target=self._do_move,
            args=(tag_x, tag_y, target_z, tag_yaw, mode),
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
            # Force re-plan on next tick by clearing the last-cmd anchors
            # (both the TCP command and the tag-XY deadband reference).
            self._last_cmd_xy = None
            self._last_cmd_tag_xy = None
        self.get_logger().info(
            f'Z step → target_z={self._target_z*1000:.0f}mm (next tick replans).')

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
                if c == '\x1b':
                    seq = sys.stdin.read(2)
                    if seq == '[A':
                        self._adjust_z(+STEP_M)
                    elif seq == '[B':
                        self._adjust_z(-STEP_M)
                elif c in ('\r', '\n'):
                    self._adjust_z(-STEP_M)
                elif c == ' ':
                    with self._lock:
                        self._tracking_paused = not self._tracking_paused
                    self.get_logger().info(
                        f'Tracking {"PAUSED" if self._tracking_paused else "RESUMED"}.')
                elif c == 'g':
                    self._force_grab()
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
