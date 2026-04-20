#!/usr/bin/env python3
"""
go_to.py
--------
Fundamental AprilTag-tracking test.

Flow:
  voice → detect_zone (bird's eye + zone center + tool detect)
        → /detect_zone/complete {fiducial=N}
        → go_to snapshots the tag pose (from /fine_loc/tag_N) ONCE
          and derives a fixed approach axis: the tag's surface normal.
  Keyboard (TTY):
    ↓ or Enter : step 10 mm closer to the tag along the fixed normal
    ↑          : back off  10 mm
    space      : pause (no more moves until next ↓)
    h          : send arm to home (all joints zero)
    q          : quit
  Voice:
    "go home" / "initial position" → /voice_command/home_request → arm home.

The approach axis is LOCKED at the moment detect_zone completes, so arm
motion doesn't bounce around with tag-pose jitter. Target = tag_center
+ standoff · normal, where standoff monotonically decreases with each ↓.
Safety floor clamp prevents commanding below SAFETY_Z_FLOOR.
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
from std_msgs.msg import String


PLANNING_GROUP = 'xarm5'
FRAME_ID = 'link_base'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
SAFETY_Z_FLOOR = -0.110
TOOL_TAG_IDS = (2, 3, 4)

INITIAL_STANDOFF_M = 0.30
MIN_STANDOFF_M = 0.02
MAX_STANDOFF_M = 0.80
STEP_M = 0.010
HOME_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0)


def tool_aligned_quat(x, y):
    """Tool-down quaternion with yaw = atan2(y, x). RPY = (pi, 0, yaw).
    Returns (qx, qy, qz, qw)."""
    phi = math.atan2(y, x)
    return (math.cos(phi / 2.0), math.sin(phi / 2.0), 0.0, 0.0)


def tag_normal_in_base(q):
    """Tag +Z axis in link_base, given tag orientation quat."""
    nx = 2.0 * (q.x * q.z + q.w * q.y)
    ny = 2.0 * (q.y * q.z - q.w * q.x)
    nz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / n, ny / n, nz / n


class GoTo(Node):

    def __init__(self):
        super().__init__('go_to')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('move_speed', 0.15)

        self.move_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cb_group)

        self.create_subscription(
            String, '/voice_command/tool_request', self._tool_request_cb, 10)
        self.create_subscription(
            String, '/voice_command/home_request', self._home_request_cb, 10)
        self.create_subscription(
            String, '/detect_zone/complete', self._detect_zone_done_cb, 10)

        self._tag_sub = None
        self._target_tag_id = None

        # Locked snapshot (set when /detect_zone/complete fires).
        self._lock = threading.Lock()
        self._locked_center = None   # (x, y, z)
        self._locked_normal = None   # (nx, ny, nz) — flipped so nz >= 0
        self._standoff_m = INITIAL_STANDOFF_M
        self._move_in_flight = False

        if sys.stdin.isatty():
            threading.Thread(target=self._keyboard_loop, daemon=True).start()
            self.get_logger().info(
                'Keyboard: ↓/Enter closer, ↑ back off, space pause, h home, q quit.')
        else:
            self.get_logger().warn('No TTY — keyboard disabled.')

        self.get_logger().info(
            f'go_to ready. Waiting for /detect_zone/complete on tags {TOOL_TAG_IDS}.')

    # ---------------------- subscriptions ----------------------

    def _tool_request_cb(self, msg):
        """New voice request → clear old lock; detect_zone will drive."""
        fid = _parse_field(msg.data, 'fiducial')
        try:
            fid_int = int(fid) if fid is not None else None
        except ValueError:
            return
        if fid_int in TOOL_TAG_IDS:
            self.get_logger().info(
                f'Voice request for tag {fid_int} — clearing lock, waiting for detect_zone.')
            with self._lock:
                self._locked_center = None
                self._locked_normal = None
                self._target_tag_id = fid_int
                self._standoff_m = INITIAL_STANDOFF_M
                if self._tag_sub is not None:
                    self.destroy_subscription(self._tag_sub)
                    self._tag_sub = None

    def _home_request_cb(self, _msg):
        self.get_logger().info('Home request received → sending arm to home.')
        with self._lock:
            self._locked_center = None
            self._locked_normal = None
            if self._tag_sub is not None:
                self.destroy_subscription(self._tag_sub)
                self._tag_sub = None
        threading.Thread(target=self._go_home, daemon=True).start()

    def _detect_zone_done_cb(self, msg):
        fid = _parse_field(msg.data, 'fiducial')
        try:
            fid_int = int(fid) if fid is not None else None
        except ValueError:
            return
        if fid_int not in TOOL_TAG_IDS:
            return
        self.get_logger().info(
            f'detect_zone complete for tag {fid_int} → subscribing to /fine_loc/tag_{fid_int} '
            'for a single snapshot.')
        with self._lock:
            self._target_tag_id = fid_int
            self._standoff_m = INITIAL_STANDOFF_M
            self._locked_center = None
            self._locked_normal = None
            if self._tag_sub is not None:
                self.destroy_subscription(self._tag_sub)
        topic = f'/fine_loc/tag_{fid_int}'
        self._tag_sub = self.create_subscription(
            PoseStamped, topic, self._tag_snapshot_cb, 10)

    def _tag_snapshot_cb(self, msg):
        """Take the FIRST tag pose received after detect_zone done → lock it."""
        with self._lock:
            if self._locked_center is not None:
                return  # already locked
            p = msg.pose.position
            nx, ny, nz = tag_normal_in_base(msg.pose.orientation)
            if nz < 0:
                nx, ny, nz = -nx, -ny, -nz  # flip to point up
            self._locked_center = (p.x, p.y, p.z)
            self._locked_normal = (nx, ny, nz)
        self.get_logger().info(
            f'LOCK: tag {self._target_tag_id} center=({p.x*1000:.0f},'
            f'{p.y*1000:.0f},{p.z*1000:.0f})mm  normal=({nx:.2f},{ny:.2f},{nz:.2f})  '
            f'initial standoff={self._standoff_m*1000:.0f}mm')
        threading.Thread(target=self._move_to_current_target, daemon=True).start()

    # ---------------------- motion ----------------------

    def _move_to_current_target(self):
        with self._lock:
            if self._move_in_flight:
                return
            self._move_in_flight = True
        try:
            with self._lock:
                if self._locked_center is None or self._locked_normal is None:
                    return
                cx, cy, cz = self._locked_center
                nx, ny, nz = self._locked_normal
                s = self._standoff_m
            tx = cx + s * nx
            ty = cy + s * ny
            tz = cz + s * nz
            if tz < SAFETY_Z_FLOOR + 0.005:
                self.get_logger().warn(
                    f'Target z {tz*1000:.1f}mm below safety floor '
                    f'({SAFETY_Z_FLOOR*1000:.1f}mm). Clamped.')
                tz = SAFETY_Z_FLOOR + 0.005

            self.get_logger().info(
                f'GO → ({tx*1000:.0f},{ty*1000:.0f},{tz*1000:.0f})mm '
                f'standoff={s*1000:.0f}mm')

            pose = PoseStamped()
            pose.header.frame_id = FRAME_ID
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = tx
            pose.pose.position.y = ty
            pose.pose.position.z = tz
            qx, qy, qz, qw = tool_aligned_quat(tx, ty)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            joints = self._ik(pose)
            if joints is None:
                return
            self._joint_move(joints, f'go_to({tx:.3f},{ty:.3f},{tz:.3f})')
        finally:
            with self._lock:
                self._move_in_flight = False

    def _adjust_standoff(self, delta_m):
        with self._lock:
            if self._locked_center is None:
                self.get_logger().warn('Not locked — ignoring key.')
                return
            new = self._standoff_m + delta_m
            clamped = max(MIN_STANDOFF_M, min(MAX_STANDOFF_M, new))
            self._standoff_m = clamped
            if clamped != new:
                self.get_logger().warn(
                    f'Standoff clamped to {clamped*1000:.0f}mm range.')
        threading.Thread(target=self._move_to_current_target, daemon=True).start()

    def _go_home(self):
        with self._lock:
            if self._move_in_flight:
                self.get_logger().warn('Move in flight — home deferred.')
                return
            self._move_in_flight = True
        try:
            joints = dict(zip(JOINT_NAMES, HOME_JOINTS))
            self._joint_move(joints, 'HOME(all-zero)')
        finally:
            with self._lock:
                self._move_in_flight = False

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
            self.get_logger().warn('MoveGroup unavailable.')
            return
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
            return
        if not accepted[0]:
            self.get_logger().warn(f'[{label}] Move rejected.')
            return
        self.get_logger().info(f'[{label}] Move done.')

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
                    if seq == '[A':       # up
                        self._adjust_standoff(+STEP_M)
                    elif seq == '[B':     # down
                        self._adjust_standoff(-STEP_M)
                elif c == '\r' or c == '\n':
                    self._adjust_standoff(-STEP_M)
                elif c == ' ':
                    self.get_logger().info('Paused (no pending move).')
                elif c == 'h':
                    self._go_home()
                elif c == 'q' or c == '\x03':
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
