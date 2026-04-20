#!/usr/bin/env python3
"""
tool_approach.py
----------------
Phase 10a: continuously align the gripper with an AprilTag's surface normal
("align mode"), and let the operator trace the gripper down along that
normal with keyboard ↑/↓ ("trace mode").

Subscribes:
  /fine_loc/tag_<id>                 (PoseStamped)  — target tag 6-DOF pose in link_base
  /fine_loc/tag_<id>/pixel_size      (Float32)      — tag edge length in pixels (RGB signal, always available)
  /fine_loc/tag_<id>/depth           (Float32)      — aligned-depth sample at tag center (only when realsense2_camera is live)
  /voice_command/tool_request        (String)       — selects target tag
  /detect_zone/complete              (String)       — auto-start after centering
  /tool_approach/cmd                 (String)       — headless command interface

Distance signal selection: pixel-size pinhole estimate is authoritative until
the tag's pixel_size exceeds `depth_activation_pixel_size_px` (default 70px
≈ 225 mm for a 25.4 mm tag with fx=615). Above that threshold, depth camera
readings take over. Below 190 mm (D435i minimum range) depth goes NaN and we
fall back to pixel-size even in "CLOSE" mode.

Commands (topic or keyboard):
  start <fid>          | → subscribe to /fine_loc/tag_<fid>, begin tracking
  stop                 | space → pause tracking (arm holds current pose)
  descend [mm]         | ↓     → reduce standoff by mm (default 10)
  raise   [mm]         | ↑     → increase standoff by mm (default 10)
  set <mm>             |        → set absolute standoff in mm
  quit                 | q     → shutdown

Flat-tag assumption for v1: if tag normal deviates from world +Z by >20°,
the update is skipped with a warning. Full 6-DOF approach is deferred.
"""

import math
import sys
import select
import termios
import threading
import time
import tty

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest, PositionIKRequest,
)
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Float32, String

import tf2_ros


PLANNING_GROUP = 'xarm5'
FRAME_ID = 'link_base'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
SAFETY_Z_FLOOR = -0.110
TOOL_TAG_IDS = (2, 3, 4)

DEFAULT_STANDOFF_M = 0.50
MIN_STANDOFF_M = 0.05
MAX_STANDOFF_M = 0.80
STEP_DEFAULT_M = 0.010
MAX_NORMAL_TILT_DEG = 20.0
ALIGN_POSITION_TOL_M = 0.003
ALIGN_RATE_HZ = 2.0


def tool_aligned_quat(x, y):
    """Tool-down quaternion with yaw = atan2(y, x). RPY = (pi, 0, atan2(y, x))."""
    phi = math.atan2(y, x)
    return (math.cos(phi / 2.0), math.sin(phi / 2.0), 0.0, 0.0)


def tag_normal_in_base(q):
    """Return the tag's local +Z axis expressed in link_base, given its quat."""
    nx = 2.0 * (q.x * q.z + q.w * q.y)
    ny = 2.0 * (q.y * q.z - q.w * q.x)
    nz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return (nx, ny, nz)


class ToolApproach(Node):

    def __init__(self):
        super().__init__('tool_approach')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('enable_keyboard', True)
        self.declare_parameter('move_speed', 0.15)
        # Depth camera only becomes authoritative once the tag is large enough
        # in the frame — before that, the pixel-size pinhole estimate is more
        # trustworthy than an out-of-range D435i depth sample.
        self.declare_parameter('depth_activation_pixel_size_px', 70.0)
        self.declare_parameter('pixel_dist_fx', 615.0)  # D435i 640x480 factory fx
        self.declare_parameter('tag_size_m', 0.0254)    # 1 inch tag

        self.move_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cb_group)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            String, '/voice_command/tool_request', self._tool_request_cb, 10)
        self.create_subscription(
            String, '/detect_zone/complete', self._detect_zone_done_cb, 10)
        self.create_subscription(
            String, '/tool_approach/cmd', self._cmd_cb, 10)

        self._per_tag_subs = {}
        self._per_tag_depth_subs = {}
        self._per_tag_pixel_subs = {}
        self._latest_tag_pose = None
        self._latest_tag_stamp = None
        self._latest_depth_m = None
        self._latest_depth_stamp = None
        self._latest_pixel_size_px = None
        self._latest_pixel_stamp = None

        self._target_tag_id = None
        self._standoff_m = DEFAULT_STANDOFF_M
        self._align_enabled = False
        self._move_in_flight = False
        self._lock = threading.Lock()

        self.create_timer(1.0 / ALIGN_RATE_HZ, self._align_tick,
                          callback_group=self.cb_group)
        self.create_timer(1.0, self._status_tick,
                          callback_group=self.cb_group)

        if self.get_parameter('enable_keyboard').value and sys.stdin.isatty():
            threading.Thread(target=self._keyboard_loop, daemon=True).start()
            self.get_logger().info('Keyboard: ↑ raise, ↓ descend, space stop, q quit.')
        else:
            self.get_logger().info('Keyboard disabled (not a TTY or disabled by param).')

        self.get_logger().info(
            'tool_approach ready. Waiting for start command. '
            f'Standoff default {DEFAULT_STANDOFF_M*1000:.0f}mm, '
            f'step {STEP_DEFAULT_M*1000:.0f}mm.')

    # ------------------------- command surface -------------------------

    def _tool_request_cb(self, msg):
        """voice-command payload: phrase=...|tool=...|fiducial=<id>"""
        fid = _parse_field(msg.data, 'fiducial')
        if fid is None:
            return
        try:
            fid_int = int(fid)
        except ValueError:
            return
        if fid_int in TOOL_TAG_IDS:
            self.get_logger().info(
                f'Voice request → will track tag {fid_int} after detect_zone completes. '
                'Pausing align so detect_zone can control the arm.')
            self._target_tag_id = fid_int
            with self._lock:
                self._align_enabled = False

    def _detect_zone_done_cb(self, msg):
        fid = _parse_field(msg.data, 'fiducial')
        if fid is None:
            return
        try:
            fid_int = int(fid)
        except ValueError:
            return
        if fid_int not in TOOL_TAG_IDS:
            return
        self.get_logger().info(
            f'detect_zone complete for fiducial {fid_int} → starting align.')
        self._start(fid_int)

    def _cmd_cb(self, msg):
        self._handle_command(msg.data.strip())

    def _handle_command(self, text):
        parts = text.split()
        if not parts:
            return
        cmd = parts[0].lower()
        try:
            if cmd == 'start':
                fid = int(parts[1])
                self._start(fid)
            elif cmd == 'stop':
                self._stop()
            elif cmd == 'descend':
                mm = float(parts[1]) if len(parts) > 1 else STEP_DEFAULT_M * 1000.0
                self._adjust_standoff(-mm / 1000.0)
            elif cmd in ('raise', 'up'):
                mm = float(parts[1]) if len(parts) > 1 else STEP_DEFAULT_M * 1000.0
                self._adjust_standoff(+mm / 1000.0)
            elif cmd == 'set':
                mm = float(parts[1])
                self._set_standoff(mm / 1000.0)
            elif cmd == 'quit':
                rclpy.shutdown()
            else:
                self.get_logger().warn(f'Unknown command: {text!r}')
        except (IndexError, ValueError) as e:
            self.get_logger().warn(f'Bad command {text!r}: {e}')

    # ------------------------- align state -----------------------------

    def _start(self, tag_id):
        with self._lock:
            if tag_id not in self._per_tag_subs:
                pose_topic = f'/fine_loc/tag_{tag_id}'
                self._per_tag_subs[tag_id] = self.create_subscription(
                    PoseStamped, pose_topic,
                    lambda m, tid=tag_id: self._tag_pose_cb(tid, m), 10)
                self.get_logger().info(f'Subscribed to {pose_topic}.')
            if tag_id not in self._per_tag_depth_subs:
                depth_topic = f'/fine_loc/tag_{tag_id}/depth'
                self._per_tag_depth_subs[tag_id] = self.create_subscription(
                    Float32, depth_topic,
                    lambda m, tid=tag_id: self._tag_depth_cb(tid, m), 10)
                self.get_logger().info(f'Subscribed to {depth_topic}.')
            if tag_id not in self._per_tag_pixel_subs:
                pixel_topic = f'/fine_loc/tag_{tag_id}/pixel_size'
                self._per_tag_pixel_subs[tag_id] = self.create_subscription(
                    Float32, pixel_topic,
                    lambda m, tid=tag_id: self._tag_pixel_cb(tid, m), 10)
                self.get_logger().info(f'Subscribed to {pixel_topic}.')
            self._target_tag_id = tag_id
            self._standoff_m = DEFAULT_STANDOFF_M
            self._align_enabled = True
            self._latest_depth_m = None
            self._latest_depth_stamp = None
            self._latest_pixel_size_px = None
            self._latest_pixel_stamp = None
        self.get_logger().info(
            f'ALIGN ENABLED on tag {tag_id}. Standoff={self._standoff_m*1000:.0f}mm.')

    def _stop(self):
        with self._lock:
            self._align_enabled = False
        self.get_logger().info('ALIGN PAUSED (arm holds current pose).')

    def _adjust_standoff(self, delta_m):
        with self._lock:
            new = self._standoff_m + delta_m
            self._set_standoff_unsafe(new)

    def _set_standoff(self, value_m):
        with self._lock:
            self._set_standoff_unsafe(value_m)

    def _set_standoff_unsafe(self, value_m):
        clamped = max(MIN_STANDOFF_M, min(MAX_STANDOFF_M, value_m))
        if clamped != value_m:
            self.get_logger().warn(
                f'Standoff {value_m*1000:.0f}mm clamped to {clamped*1000:.0f}mm '
                f'(range {MIN_STANDOFF_M*1000:.0f}-{MAX_STANDOFF_M*1000:.0f}mm).')
        self._standoff_m = clamped
        self.get_logger().info(f'Standoff set to {self._standoff_m*1000:.0f}mm.')

    def _tag_pose_cb(self, tag_id, msg):
        if tag_id != self._target_tag_id:
            return
        self._latest_tag_pose = msg
        self._latest_tag_stamp = time.monotonic()

    def _tag_depth_cb(self, tag_id, msg):
        if tag_id != self._target_tag_id:
            return
        val = float(msg.data)
        if not math.isfinite(val) or val <= 0.0:
            return
        self._latest_depth_m = val
        self._latest_depth_stamp = time.monotonic()

    def _tag_pixel_cb(self, tag_id, msg):
        if tag_id != self._target_tag_id:
            return
        val = float(msg.data)
        if not math.isfinite(val) or val <= 0.0:
            return
        self._latest_pixel_size_px = val
        self._latest_pixel_stamp = time.monotonic()

    def _pixel_distance_m(self):
        """Pinhole distance estimate from tag pixel size. None if unavailable."""
        if (self._latest_pixel_size_px is None
                or self._latest_pixel_stamp is None
                or (time.monotonic() - self._latest_pixel_stamp) > 1.0
                or self._latest_pixel_size_px <= 0.0):
            return None
        fx = float(self.get_parameter('pixel_dist_fx').value)
        tag_m = float(self.get_parameter('tag_size_m').value)
        return fx * tag_m / self._latest_pixel_size_px

    def _distance_mode(self):
        """Which signal is authoritative: FAR[pixel] or CLOSE[depth]."""
        threshold = float(self.get_parameter('depth_activation_pixel_size_px').value)
        depth_valid = (
            self._latest_depth_m is not None
            and self._latest_depth_stamp is not None
            and (time.monotonic() - self._latest_depth_stamp) < 2.0)
        if self._latest_pixel_size_px is None:
            return 'UNKNOWN', depth_valid
        if self._latest_pixel_size_px < threshold:
            return 'FAR[pixel]', depth_valid
        if depth_valid:
            return 'CLOSE[depth]', True
        return 'CLOSE[pixel-fallback]', False

    def _status_tick(self):
        """Periodic info log so the operator sees live distance + standoff."""
        if not self._align_enabled:
            return
        tcp = self._tcp_position()
        tcp_str = (
            f'tcp=({tcp[0]*1000:.0f},{tcp[1]*1000:.0f},{tcp[2]*1000:.0f})mm'
            if tcp else 'tcp=n/a')
        tag_str = (
            'tag_visible'
            if self._latest_tag_stamp is not None
            and (time.monotonic() - self._latest_tag_stamp) < 1.0
            else 'tag_LOST')

        mode, depth_valid = self._distance_mode()
        px_str = (
            f'pixel={self._latest_pixel_size_px:.0f}px'
            if self._latest_pixel_size_px is not None else 'pixel=n/a')
        pd = self._pixel_distance_m()
        pd_str = f'pixel_dist={pd*1000:.0f}mm' if pd is not None else 'pixel_dist=n/a'
        depth_str = (
            f'depth={self._latest_depth_m*1000:.0f}mm'
            if depth_valid else 'depth=n/a')

        self.get_logger().info(
            f'ALIGN tag={self._target_tag_id} standoff={self._standoff_m*1000:.0f}mm '
            f'{tag_str} {tcp_str} mode={mode} {px_str} {pd_str} {depth_str}')

    # ------------------------- align tick ------------------------------

    def _align_tick(self):
        if not self._align_enabled:
            return
        if self._move_in_flight:
            return
        if self._latest_tag_pose is None:
            return
        if self._latest_tag_stamp is None or (time.monotonic() - self._latest_tag_stamp) > 1.0:
            return

        pose = self._latest_tag_pose.pose
        nx, ny, nz = tag_normal_in_base(pose.orientation)
        # Normalize for safety.
        n_norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        nx, ny, nz = nx / n_norm, ny / n_norm, nz / n_norm

        # Flat-tag check: tag normal must be within MAX_NORMAL_TILT_DEG of world +Z.
        tilt_cos = abs(nz)
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, tilt_cos))))
        if tilt_deg > MAX_NORMAL_TILT_DEG:
            self.get_logger().warn(
                f'Tag normal tilted {tilt_deg:.1f}° (>{MAX_NORMAL_TILT_DEG:.0f}°). '
                'Non-horizontal tags not supported in v1.',
                throttle_duration_sec=3.0)
            return

        # Flip normal so it points "up" (same hemisphere as world +Z).
        sign = 1.0 if nz >= 0 else -1.0
        nx, ny, nz = sign * nx, sign * ny, sign * nz

        target_x = pose.position.x + self._standoff_m * nx
        target_y = pose.position.y + self._standoff_m * ny
        target_z = pose.position.z + self._standoff_m * nz

        if target_z < SAFETY_Z_FLOOR + 0.01:
            self.get_logger().warn(
                f'Target z {target_z*1000:.1f}mm below safety floor '
                f'({SAFETY_Z_FLOOR*1000:.1f}mm). Refusing move.',
                throttle_duration_sec=2.0)
            return

        # Already aligned?
        tcp = self._tcp_position()
        if tcp is not None:
            dx = target_x - tcp[0]
            dy = target_y - tcp[1]
            dz = target_z - tcp[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < ALIGN_POSITION_TOL_M:
                return

        # Dispatch a move on a worker thread so the timer stays responsive.
        with self._lock:
            if self._move_in_flight:
                return
            self._move_in_flight = True
        threading.Thread(
            target=self._do_move,
            args=(target_x, target_y, target_z),
            daemon=True,
        ).start()

    def _tcp_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                FRAME_ID, 'link_tcp', rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None

    def _do_move(self, x, y, z):
        try:
            pose = PoseStamped()
            pose.header.frame_id = FRAME_ID
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            qx, qy, qz, qw = tool_aligned_quat(x, y)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            joints = self._ik(pose)
            if joints is None:
                return
            self._joint_move(joints, f'align→({x:.3f},{y:.3f},{z:.3f})')
        finally:
            with self._lock:
                self._move_in_flight = False

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
                f'{pose.pose.position.z:.3f}).',
                throttle_duration_sec=2.0)
            return None
        jv = {n: p for n, p in zip(
            resp.solution.joint_state.name, resp.solution.joint_state.position)
            if n in JOINT_NAMES}
        if len(jv) != 5:
            return None
        return jv

    def _joint_move(self, joint_values, label):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('MoveGroup unavailable.')
            return
        spd = self.get_parameter('move_speed').value
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
        result_holder = [None]

        def on_goal(f):
            gh = f.result()
            if gh is None or not gh.accepted:
                event.set()
                return
            accepted[0] = True
            gh.get_result_async().add_done_callback(
                lambda f2: (result_holder.__setitem__(0, f2.result()), event.set()))

        self.move_client.send_goal_async(goal).add_done_callback(on_goal)
        if not event.wait(timeout=15.0):
            self.get_logger().warn(f'[{label}] Move timeout.')
            return
        if not accepted[0]:
            self.get_logger().warn(f'[{label}] Move rejected.')

    # ------------------------- keyboard --------------------------------

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
                        self._adjust_standoff(+STEP_DEFAULT_M)
                    elif seq == '[B':
                        self._adjust_standoff(-STEP_DEFAULT_M)
                elif c == ' ':
                    self._stop()
                elif c == 'q':
                    self.get_logger().info('Quit from keyboard.')
                    rclpy.shutdown()
                    return
                elif c == '\x03':
                    rclpy.shutdown()
                    return
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _parse_field(text, field):
    for part in text.split('|'):
        if part.startswith(f'{field}='):
            return part.split('=', 1)[1]
    return None


def main(args=None):
    rclpy.init(args=args)
    node = ToolApproach()
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
