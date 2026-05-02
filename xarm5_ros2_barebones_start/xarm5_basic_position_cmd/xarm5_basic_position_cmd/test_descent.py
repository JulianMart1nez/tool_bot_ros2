#!/usr/bin/env python3
"""
test_descent.py
---------------
Manual test: open gripper → fine-loc → refine position → closed-loop descent → stop.
Does NOT close gripper, lift, or move to dropoff.

Run with the arm already at a pre-grasp position above a tool.

Usage:
  ros2 run xarm5_basic_position_cmd test_descent
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint, PositionIKRequest,
)
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import GripperCommand
from std_msgs.msg import Bool, Float32

import tf2_ros
import threading
import time
import math


TOOL_DOWN_QUAT = {'w': 0.0, 'x': 1.0, 'y': 0.0, 'z': 0.0}
PLANNING_GROUP = 'xarm5'
FRAME_ID = 'link_base'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
CAMERA_Z_OFFSET = 0.127  # camera is 5in above TCP
SAFETY_Z_FLOOR = -0.085  # never descend below this in link_base
DEPTH_MIN_RANGE = 0.190  # D435i minimum depth range in meters
DEPTH_RELIABLE_MIN_TCP_DIST = 0.060  # tcp_above_tool below this → camera is in noise range


class TestDescent(Node):

    def __init__(self):
        super().__init__('test_descent')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('descent_step_m', 0.005)
        self.declare_parameter('slowdown_far_m', 0.040)
        self.declare_parameter('slowdown_near_m', 0.010)
        self.declare_parameter('stop_distance_m', 0.0)
        self.declare_parameter('descent_speed_full', 0.3)
        self.declare_parameter('descent_speed_crawl', 0.05)
        self.declare_parameter('move_speed', 0.15)
        self.declare_parameter('move_acc', 0.15)
        self.declare_parameter('gripper_open_pos', 0.0)
        self.declare_parameter('gripper_effort', 0.0)
        self.declare_parameter('target_z', 0.038)

        self.move_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.cb_group)
        self.gripper_client = ActionClient(self, GripperCommand, '/xarm_gripper/gripper_action', callback_group=self.cb_group)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb_group)

        self.depth_monitor_pub = self.create_publisher(Bool, '/gripper/monitor_enabled', 10)
        self._tool_distance = None
        self._cart_distance = None
        self._tool_dist_stamp = None
        self.create_subscription(Float32, '/gripper/tool_distance', self._tool_dist_cb, 10, callback_group=self.cb_group)
        self.create_subscription(Float32, '/gripper/cart_distance', lambda m: setattr(self, '_cart_distance', m.data), 10, callback_group=self.cb_group)

        # Fine-localization now publishes the AprilTag's absolute position in
        # link_base continuously. We just consume the latest value.
        self._fine_loc_result = None
        self._fine_loc_stamp = None
        self._fine_loc_detected = None
        self.create_subscription(PoseStamped, '/fine_loc/result', self._fine_loc_cb, 10, callback_group=self.cb_group)
        self.create_subscription(Bool, '/fine_loc/tag_detected', lambda m: setattr(self, '_fine_loc_detected', m.data), 10, callback_group=self.cb_group)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Test descent ready. Starting in 3 seconds...')
        self.create_timer(3.0, self._run_once, callback_group=self.cb_group)
        self._ran = False

    def _param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _tool_dist_cb(self, msg):
        self._tool_distance = msg.data
        self._tool_dist_stamp = time.monotonic()

    def _depth_is_fresh(self, max_age=1.0):
        if self._tool_dist_stamp is None:
            return False
        return (time.monotonic() - self._tool_dist_stamp) < max_age

    def _fine_loc_cb(self, msg):
        self._fine_loc_result = msg
        self._fine_loc_stamp = time.monotonic()

    def _fine_loc_is_fresh(self, max_age=1.0):
        if self._fine_loc_stamp is None:
            return False
        return (time.monotonic() - self._fine_loc_stamp) < max_age

    def _get_tcp_position(self):
        try:
            tf = self.tf_buffer.lookup_transform('link_base', 'link_tcp', rclpy.time.Time())
            t = tf.transform.translation
            return t.x, t.y, t.z
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

    def _get_tcp_quat(self):
        try:
            tf = self.tf_buffer.lookup_transform('link_base', 'link_tcp', rclpy.time.Time())
            r = tf.transform.rotation
            return r.x, r.y, r.z, r.w
        except Exception as e:
            self.get_logger().error(f'TF lookup (quat) failed: {e}')
            return None

    def _send_action(self, client, goal, label, timeout=60.0):
        event = threading.Event()
        result_holder = [None]
        accepted = [False]
        def on_goal(f):
            gh = f.result()
            if gh is None or not gh.accepted:
                self.get_logger().error(f'[{label}] Rejected'); event.set(); return
            accepted[0] = True
            self.get_logger().info(f'[{label}] Executing...')
            gh.get_result_async().add_done_callback(lambda f2: (result_holder.__setitem__(0, f2.result()), event.set()))
        client.send_goal_async(goal).add_done_callback(on_goal)
        if not event.wait(timeout=timeout):
            self.get_logger().error(f'[{label}] Timed out'); return None
        return result_holder[0] if accepted[0] else None

    def _call_service(self, client, req, label, timeout=10.0):
        event = threading.Event()
        result_holder = [None]
        client.call_async(req).add_done_callback(lambda f: (result_holder.__setitem__(0, f.result()), event.set()))
        if not event.wait(timeout=timeout):
            self.get_logger().error(f'[{label}] Timed out'); return None
        return result_holder[0]

    def _tool_aligned_quat(self, x, y):
        """Tool-down quaternion with wrist yaw matching atan2(y, x).

        xArm5 has only 5 DOF — at any given XY, the wrist yaw is constrained
        to align with the radial direction from the base. Using this
        natural yaw is the only way IK reliably solves across the workspace.
        Equivalent to RPY = (pi, 0, atan2(y, x))."""
        phi = math.atan2(y, x)
        c = math.cos(phi / 2.0)
        s = math.sin(phi / 2.0)
        # q = q_yaw(phi) * q_roll(pi). For roll=pi: (1,0,0,0).
        # Result: (cos(phi/2), sin(phi/2), 0, 0) for (x, y, z, w).
        return (c, s, 0.0, 0.0)

    def _make_pose(self, x, y, z, quat=None):
        p = PoseStamped()
        p.header.frame_id = FRAME_ID
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        if quat is not None:
            qx, qy, qz, qw = quat
            p.pose.orientation.x = qx
            p.pose.orientation.y = qy
            p.pose.orientation.z = qz
            p.pose.orientation.w = qw
        else:
            p.pose.orientation.w = TOOL_DOWN_QUAT['w']
            p.pose.orientation.x = TOOL_DOWN_QUAT['x']
            p.pose.orientation.y = TOOL_DOWN_QUAT['y']
            p.pose.orientation.z = TOOL_DOWN_QUAT['z']
        return p

    def _move_to_pose(self, pose, label, speed=None):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            return False
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = PLANNING_GROUP
        # xArm5 is 5-DOF — collision-aware IK frequently rejects feasible
        # poses near the workspace edge. The MoveGroup planner that follows
        # still does collision checking, so safety is preserved.
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        req.ik_request.timeout.nanosec = 0
        req.ik_request.pose_stamped = pose
        resp = self._call_service(self.ik_client, req, f'{label}/IK')
        if resp is None:
            self.get_logger().error(f'[{label}] IK service call returned None'); return False
        if resp.error_code.val != 1:
            self.get_logger().error(
                f'[{label}] IK failed (error_code={resp.error_code.val}) '
                f'for pose=({pose.pose.position.x:.4f}, {pose.pose.position.y:.4f}, '
                f'{pose.pose.position.z:.4f})')
            return False
        jv = {n: p for n, p in zip(resp.solution.joint_state.name, resp.solution.joint_state.position) if n in JOINT_NAMES}
        if len(jv) != 5:
            return False
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            return False
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = PLANNING_GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        spd = speed if speed else self._param('move_speed')
        goal.request.max_velocity_scaling_factor = spd
        goal.request.max_acceleration_scaling_factor = spd
        c = Constraints()
        for name in JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name = name; jc.position = jv[name]
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        result = self._send_action(self.move_client, goal, label)
        if result and result.result.error_code.val == 1:
            self.get_logger().info(f'[{label}] Success'); return True
        self.get_logger().error(f'[{label}] Failed'); return False

    def _get_descent_speed(self, dist):
        far, near = self._param('slowdown_far_m'), self._param('slowdown_near_m')
        full, crawl = self._param('descent_speed_full'), self._param('descent_speed_crawl')
        if dist > far: return full
        if dist < near: return crawl
        return crawl + (dist - near) / (far - near) * (full - crawl)

    def _run_once(self):
        if self._ran: return
        self._ran = True
        threading.Thread(target=self._execute, daemon=True).start()

    def _execute(self):
        self.get_logger().info('========== TEST DESCENT START ==========')

        # Step 1: Open gripper
        self.get_logger().info('--- Step 1: Open gripper ---')
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Gripper not available — skipping open.')
        else:
            goal = GripperCommand.Goal()
            goal.command.position = self._param('gripper_open_pos')
            goal.command.max_effort = self._param('gripper_effort')
            self._send_action(self.gripper_client, goal, 'OPEN', timeout=15.0)

        # Step 2: Enable depth monitor and read current position
        self.get_logger().info('--- Step 2: Read current state ---')
        msg = Bool(); msg.data = True
        self.depth_monitor_pub.publish(msg)
        time.sleep(0.5)

        pos = self._get_tcp_position()
        if pos is None:
            self.get_logger().error('Cannot read TCP position. Aborting.')
            return
        cur_x, cur_y, cur_z = pos
        # Capture the current TCP orientation and reuse it for all subsequent
        # moves. xArm5 is 5-DOF so we cannot independently constrain XY position
        # AND a fixed yaw — preserving the operator-selected pre-grasp yaw is
        # what makes IK solvable across the workspace.
        cur_quat = self._get_tcp_quat()
        if cur_quat is None:
            self.get_logger().error('Cannot read TCP orientation. Aborting.')
            return
        self.get_logger().info(
            f'Current TCP: ({cur_x:.4f}, {cur_y:.4f}, {cur_z:.4f}) '
            f'quat=({cur_quat[0]:.3f}, {cur_quat[1]:.3f}, {cur_quat[2]:.3f}, {cur_quat[3]:.3f})')
        if self._tool_distance:
            self.get_logger().info(f'Depth: tool={self._tool_distance*1000:.1f}mm, cart={self._cart_distance*1000:.1f}mm' if self._cart_distance else f'Depth: tool={self._tool_distance*1000:.1f}mm')

        # Step 3: Fine localization (continuous — wait for a fresh sample)
        self.get_logger().info('--- Step 3: Fine localization ---')
        for _ in range(30):
            if self._fine_loc_is_fresh(max_age=0.5):
                break
            time.sleep(0.1)

        target_x, target_y = cur_x, cur_y
        if self._fine_loc_is_fresh(max_age=0.5) and self._fine_loc_result is not None:
            tag_x = self._fine_loc_result.pose.position.x
            tag_y = self._fine_loc_result.pose.position.y
            tag_z = self._fine_loc_result.pose.position.z
            self.get_logger().info(
                f'Tag absolute position in link_base: '
                f'x={tag_x*1000:.1f}mm, y={tag_y*1000:.1f}mm, z={tag_z*1000:.1f}mm')
            target_x, target_y = tag_x, tag_y
            self.get_logger().info(
                f'Move TCP from ({cur_x:.4f}, {cur_y:.4f}) → ({target_x:.4f}, {target_y:.4f}) '
                f'(dx={(target_x-cur_x)*1000:.1f}mm, dy={(target_y-cur_y)*1000:.1f}mm)')

            # Step 3b: Move to refined pre-grasp in small staged hops so the
            # tag stays in the camera FOV. A single big lateral jump moves the
            # fingertips toward the tag faster than the camera can re-acquire
            # it; staging lets the tag re-localize after each hop.
            self.get_logger().info('--- Step 3b: Staged refinement ---')
            max_hop = 0.020  # 20mm per hop
            for hop_idx in range(20):  # safety cap
                cp = self._get_tcp_position()
                if cp is None:
                    self.get_logger().error('TF lookup failed during staged refinement.')
                    return
                cx, cy, cz = cp

                # Refresh tag XY each hop from latest fine_loc
                if self._fine_loc_is_fresh(max_age=0.5) and self._fine_loc_result is not None:
                    target_x = self._fine_loc_result.pose.position.x
                    target_y = self._fine_loc_result.pose.position.y

                dx = target_x - cx
                dy = target_y - cy
                dist = math.hypot(dx, dy)
                if dist < 0.003:
                    self.get_logger().info(
                        f'[REFINE] Reached tag XY (residual {dist*1000:.1f}mm).')
                    break

                # Clamp this hop to max_hop in the direction of the tag
                if dist > max_hop:
                    scale = max_hop / dist
                    hop_x = cx + dx * scale
                    hop_y = cy + dy * scale
                else:
                    hop_x, hop_y = target_x, target_y

                self.get_logger().info(
                    f'[REFINE #{hop_idx}] hop {dist*1000:.1f}mm remaining → '
                    f'({hop_x:.4f}, {hop_y:.4f})')

                hop_pose = self._make_pose(hop_x, hop_y, cz,
                                           quat=self._tool_aligned_quat(hop_x, hop_y))
                if not self._move_to_pose(hop_pose, f'REFINE#{hop_idx}'):
                    self.get_logger().error('Staged refinement hop failed. Aborting.')
                    return
                # Pause so fine_loc can re-acquire the tag at the new pose
                time.sleep(0.5)
            else:
                self.get_logger().warn('Staged refinement hit safety cap (20 hops).')
        else:
            self.get_logger().warn('No tag detected — descending from current position.')

        # Step 4: Closed-loop descent
        self.get_logger().info('--- Step 4: Closed-loop descent ---')
        self.get_logger().info('>>> Use RViz STOP button to halt if anything looks wrong! <<<')
        time.sleep(1.0)

        # Re-read position after refinement move
        new_pos = self._get_tcp_position()
        if new_pos:
            cur_x, cur_y, cur_z = new_pos
            target_x, target_y = cur_x, cur_y

        step = self._param('descent_step_m')
        stop_dist = self._param('stop_distance_m')

        if self._tool_distance is None:
            self.get_logger().error('No depth reading — cannot compute descent target. Aborting.')
            return

        initial_tool_dist = self._tool_distance
        tcp_above_tool = initial_tool_dist - CAMERA_Z_OFFSET
        target_z = cur_z - tcp_above_tool + stop_dist
        target_z = max(target_z, SAFETY_Z_FLOOR)
        total_descent = cur_z - target_z
        self.get_logger().info(
            f'Current z={cur_z*1000:.1f}mm, tcp_above_tool={tcp_above_tool*1000:.1f}mm, '
            f'target_z={target_z*1000:.1f}mm (descend {total_descent*1000:.1f}mm)')
        self.get_logger().info(
            f'Safety z floor={SAFETY_Z_FLOOR*1000:.1f}mm, D435i min range={DEPTH_MIN_RANGE*1000:.0f}mm')

        z = cur_z
        open_loop = False
        remaining_at_switch = None

        iteration = 0
        while z > target_z + 0.001:
            if z <= SAFETY_Z_FLOOR:
                self.get_logger().info('Hit safety z floor. Stopping.')
                break

            # Continuously update XY target from the latest AprilTag observation.
            # The fine-loc node publishes the tag's absolute position in link_base,
            # so we steer the TCP directly to (tag_x, tag_y).
            xy_updated = ''
            if self._fine_loc_is_fresh(max_age=0.5) and self._fine_loc_result is not None:
                new_tx = self._fine_loc_result.pose.position.x
                new_ty = self._fine_loc_result.pose.position.y
                dx = (new_tx - target_x) * 1000
                dy = (new_ty - target_y) * 1000
                target_x, target_y = new_tx, new_ty
                xy_updated = f' [XY refresh: dx={dx:+.1f}mm, dy={dy:+.1f}mm]'

            depth_usable = (
                not open_loop
                and self._depth_is_fresh(max_age=2.0)
                and self._tool_distance is not None
                and (self._tool_distance - CAMERA_Z_OFFSET) >= DEPTH_RELIABLE_MIN_TCP_DIST
            )

            if depth_usable:
                tcp_dist = self._tool_distance - CAMERA_Z_OFFSET
                self.get_logger().info(
                    f'[#{iteration}] CLOSED-LOOP tcp_above_tool={tcp_dist*1000:.1f}mm{xy_updated}')

                if tcp_dist <= stop_dist:
                    self.get_logger().info('Contact distance reached. Stopping.')
                    break

                speed = self._get_descent_speed(tcp_dist)
                actual_step = min(step, tcp_dist - stop_dist)
            else:
                if not open_loop:
                    remaining_at_switch = z - target_z
                    open_loop = True
                    last_dist_mm = (self._tool_distance - CAMERA_Z_OFFSET) * 1000 if self._tool_distance else 0
                    self.get_logger().warn(
                        f'Depth unreliable (last tcp_above_tool={last_dist_mm:.1f}mm < '
                        f'{DEPTH_RELIABLE_MIN_TCP_DIST*1000:.0f}mm reliable threshold). '
                        f'Switching to OPEN-LOOP for remaining {remaining_at_switch*1000:.1f}mm to target_z={target_z*1000:.1f}mm')
                speed = self._param('descent_speed_crawl')
                actual_step = step
                self.get_logger().info(
                    f'[#{iteration}] OPEN-LOOP z={z*1000:.1f}mm, '
                    f'remaining={max(0, (z - actual_step - target_z))*1000:.1f}mm{xy_updated}')

            z -= actual_step
            if z < SAFETY_Z_FLOOR:
                z = SAFETY_Z_FLOOR
                self.get_logger().info('Clamping to safety z floor.')

            pose = self._make_pose(target_x, target_y, z,
                                   quat=self._tool_aligned_quat(target_x, target_y))
            self.get_logger().info(
                f'[#{iteration}] Descending to ({target_x:.4f}, {target_y:.4f}, {z*1000:.1f}mm) '
                f'(speed={speed:.3f})')

            if not self._move_to_pose(pose, f'DESC#{iteration}', speed=speed):
                self.get_logger().error('Descent move failed. Aborting.')
                break

            time.sleep(0.2)
            iteration += 1

        # Done
        off = Bool(); off.data = False
        self.depth_monitor_pub.publish(off)

        final_pos = self._get_tcp_position()
        self.get_logger().info('========== TEST DESCENT COMPLETE ==========')
        if final_pos:
            self.get_logger().info(f'Final TCP: ({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f})')
        if self._tool_distance:
            self.get_logger().info(f'Final depth: tool={self._tool_distance*1000:.1f}mm')
        self.get_logger().info('Gripper is OPEN. Arm is near the tool. Ctrl+C to exit.')


def main(args=None):
    rclpy.init(args=args)
    node = TestDescent()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
