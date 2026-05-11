#!/usr/bin/env python3
"""
keyboard_controller.py - xArm5 Joint Keyboard Teleoperation
=============================================================
Publishes JointTrajectory directly to the trajectory controllers.
No MoveIt Servo required.

CONTROLS
════════
  Joint 1 :  Q (+)   A (-)
  Joint 2 :  W (+)   S (-)
  Joint 3 :  E (+)   D (-)
  Joint 4 :  R (+)   F (-)
  Joint 5 :  T (+)   G (-)

  Gripper :  Z = open   X = close   C = half open

  [ / ]    Decrease / Increase step size
  Space    Freeze (hold current commanded position)
  Esc      Quit
"""

import os
import sys
import tty
import termios
import select
import threading
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ── Joint names and limits ───────────────────────────────────────────────────
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

JOINT_LIMITS = [
    (-2 * math.pi,  2 * math.pi),   # joint1
    (-2.059,         2.094),          # joint2
    (-3.927,         0.192),          # joint3
    (-1.693,         math.pi),        # joint4
    (-2 * math.pi,  2 * math.pi),   # joint5
]

# ── Motion parameters ────────────────────────────────────────────────────────
DEFAULT_STEP   = 0.05   # rad per key event
STEP_MIN       = 0.01
STEP_MAX       = 0.20
STEP_INC       = 0.01
TRAJ_DURATION  = 0.15   # seconds for each trajectory segment

# ── Gripper ──────────────────────────────────────────────────────────────────
GRIPPER_OPEN   = 0.0
GRIPPER_CLOSE  = 0.85
GRIPPER_HALF   = 0.42
GRIPPER_JOINT  = 'drive_joint'

# ── Key → (joint_index, direction) ──────────────────────────────────────────
KEY_MAP = {
    'q': (0,  1.0),  'a': (0, -1.0),  # Joint 1
    'w': (1,  1.0),  's': (1, -1.0),  # Joint 2
    'e': (2,  1.0),  'd': (2, -1.0),  # Joint 3
    'r': (3,  1.0),  'f': (3, -1.0),  # Joint 4
    't': (4,  1.0),  'g': (4, -1.0),  # Joint 5
}


# ── TTY helpers ───────────────────────────────────────────────────────────────

_tty_file = None

def _get_tty():
    global _tty_file
    if _tty_file is None:
        try:
            _tty_file = open('/dev/tty', 'r+b', buffering=0)
        except OSError:
            _tty_file = False
    if _tty_file:
        return _tty_file.fileno(), _tty_file
    return sys.stdin.fileno(), sys.stdin


def read_key(timeout: float = 0.05) -> str:
    """Non-blocking key read. Returns '' if nothing pressed within timeout."""
    fd, stream = _get_tty()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([stream], [], [], timeout)
        if not r:
            return ''
        ch = os.read(fd, 1).decode('utf-8', errors='replace')
        if ch == '\x1b':
            r2, _, _ = select.select([stream], [], [], 0.02)
            if r2:
                ch2 = os.read(fd, 1).decode('utf-8', errors='replace')
                if ch2 == '[':
                    r3, _, _ = select.select([stream], [], [], 0.02)
                    if r3:
                        ch3 = os.read(fd, 1).decode('utf-8', errors='replace')
                        return '\x1b[' + ch3
            return ch
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Controller node ───────────────────────────────────────────────────────────

class KeyboardController(Node):

    def __init__(self):
        super().__init__('xarm5_keyboard')
        self.declare_parameter('dof', 5)

        # Publishers — direct to trajectory controllers
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/xarm5_traj_controller/joint_trajectory',
            10,
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/xarm_gripper_traj_controller/joint_trajectory',
            10,
        )
        # State
        self.step           = DEFAULT_STEP
        self.commanded_pos  = None   # initialised from first /joint_states
        self._pos_lock      = threading.Lock()
        self._running       = True

        # Subscribe to joint states once to get initial positions
        self._js_received   = threading.Event()
        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

        # Spin ROS in background
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        """Capture initial joint positions from /joint_states."""
        if self.commanded_pos is not None:
            return  # already initialised
        pos_by_name = dict(zip(msg.name, msg.position))
        with self._pos_lock:
            self.commanded_pos = [
                pos_by_name.get(j, 0.0) for j in ARM_JOINTS
            ]
        self.get_logger().info(
            'Initial arm positions: '
            + ', '.join(f'{j}={p:.3f}' for j, p in zip(ARM_JOINTS, self.commanded_pos))
        )
        self._js_received.set()
        self.destroy_subscription(self._js_sub)  # no longer needed

    def _spin(self):
        while self._running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    # ── Motion helpers ────────────────────────────────────────────────────

    def _clamp(self, idx: int, value: float) -> float:
        lo, hi = JOINT_LIMITS[idx]
        clamped = max(lo, min(hi, value))
        if clamped != value:
            self.get_logger().warn(
                f'joint{idx+1} limit reached '
                f'(limits [{lo:.2f}, {hi:.2f}])'
            )
        return clamped

    def _send_arm(self):
        """Publish the current commanded_pos to the arm trajectory controller."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = ARM_JOINTS
        pt = JointTrajectoryPoint()
        with self._pos_lock:
            pt.positions = list(self.commanded_pos)
        pt.velocities      = [0.0] * len(ARM_JOINTS)
        pt.time_from_start = Duration(sec=0, nanosec=int(TRAJ_DURATION * 1e9))
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def _move_joint(self, idx: int, direction: float):
        """Step joint idx by step * direction, clamped to limits, then publish."""
        with self._pos_lock:
            new_val = self._clamp(
                idx, self.commanded_pos[idx] + direction * self.step)
            self.commanded_pos[idx] = new_val
        self._send_arm()

    def _gripper_cmd(self, position: float):
        """Send gripper to target position (same approach as arm, 1-second move)."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = [GRIPPER_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions       = [position]
        pt.velocities      = [0.0]
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg.points = [pt]
        self.gripper_pub.publish(msg)

    # ── Display ───────────────────────────────────────────────────────────

    def _print_help(self):
        os.system('clear')
        print('╔═════════════════════════════════════════════════════════════╗')
        print('║              xArm5 Joint Keyboard Teleoperation             ║')
        print('╠═════════════════════════════════════════════════════════════╣')
        print('║  Joint 1 :   Q  (+ increase)     A  (- decrease)           ║')
        print('║  Joint 2 :   W  (+ increase)     S  (- decrease)           ║')
        print('║  Joint 3 :   E  (+ increase)     D  (- decrease)           ║')
        print('║  Joint 4 :   R  (+ increase)     F  (- decrease)           ║')
        print('║  Joint 5 :   T  (+ increase)     G  (- decrease)           ║')
        print('║                                                             ║')
        print('║  Gripper :   Z = open   X = close   C = half open          ║')
        print('║                                                             ║')
        print('║  [ / ]   Decrease / Increase step size                     ║')
        print('║  Space   Freeze (re-send current commanded position)        ║')
        print('║  Esc     Quit                                               ║')
        print('╚═════════════════════════════════════════════════════════════╝')
        self._status()

    def _status(self):
        with self._pos_lock:
            pos = list(self.commanded_pos) if self.commanded_pos else None
        if pos:
            pos_str = '   '.join(f'J{i+1}:{p:+.3f}' for i, p in enumerate(pos))
        else:
            pos_str = 'waiting for joint states…'
        print(f'\r  step: {self.step:.3f} rad  |  {pos_str}    ',
              end='', flush=True)

    # ── Main loop ─────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Waiting for /joint_states…')
        if not self._js_received.wait(timeout=10.0):
            self.get_logger().error(
                '/joint_states not received within 10 s. '
                'Is Terminal 1 (xarm5_sim.launch.py) running?'
            )
            return

        self._print_help()

        try:
            while self._running and rclpy.ok():
                key = read_key(timeout=0.05)
                if not key:
                    continue

                kl = key.lower()

                # ── Quit ───────────────────────────────────────────────────
                if key == '\x1b':
                    print('\n[keyboard] Quitting.')
                    break

                # ── Step size ──────────────────────────────────────────────
                if key == '[':
                    self.step = max(STEP_MIN, round(self.step - STEP_INC, 3))
                    self._status()
                    continue
                if key == ']':
                    self.step = min(STEP_MAX, round(self.step + STEP_INC, 3))
                    self._status()
                    continue

                # ── Freeze ─────────────────────────────────────────────────
                if key == ' ':
                    self._send_arm()
                    self._status()
                    continue

                # ── Gripper ────────────────────────────────────────────────
                if kl == 'z':
                    self._gripper_cmd(GRIPPER_OPEN)
                    self._status()
                    continue
                if kl == 'x':
                    self._gripper_cmd(GRIPPER_CLOSE)
                    self._status()
                    continue
                if kl == 'c':
                    self._gripper_cmd(GRIPPER_HALF)
                    self._status()
                    continue

                # ── Joint control ──────────────────────────────────────────
                if kl in KEY_MAP:
                    idx, direction = KEY_MAP[kl]
                    self._move_joint(idx, direction)
                    self._status()

        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            # Re-send current position one last time to hold the robot still
            if self.commanded_pos is not None:
                self._send_arm()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
