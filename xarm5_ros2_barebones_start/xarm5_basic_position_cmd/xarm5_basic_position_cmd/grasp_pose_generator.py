#!/usr/bin/env python3
"""
grasp_pose_generator.py
-----------------------
ROS2 node that converts a tool position into a pick-and-place waypoint
sequence, then executes it through MoveIt2 + gripper action.

Approach: For each Cartesian target, first solve IK via /compute_ik
(TRAC-IK), then plan and execute in joint space. This is the same
approach RViz uses internally and works reliably for 5-DOF arms.

Prerequisites:
  ros2 launch xarm5_basic_position_cmd xarm5_moveit_with_table.launch.py \
    robot_ip:=192.168.1.234 add_gripper:=true

Usage:
  ros2 run xarm5_basic_position_cmd grasp_pose_generator

  # Trigger (coordinates in meters, in link_base frame):
  ros2 topic pub --once /tool_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'link_base'}, pose: {position: {x: 0.3, y: 0.0, z: 0.0}}}"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionIKRequest,
)
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import GripperCommand
from std_msgs.msg import Bool, Float32

import threading
import time


TOOL_DOWN_QUAT = {'w': 0.0, 'x': 1.0, 'y': 0.0, 'z': 0.0}
PLANNING_GROUP = 'xarm5'
FRAME_ID = 'link_base'
EEF_LINK = 'link_tcp'
GRIPPER_ACTION = '/xarm_gripper/gripper_action'
MOVE_ACTION = '/move_action'
IK_SERVICE = '/compute_ik'

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


class GraspPoseGenerator(Node):

    def __init__(self):
        super().__init__('grasp_pose_generator')

        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('approach_height', 100.0)    # mm
        self.declare_parameter('tool_height', 38.0)         # mm
        self.declare_parameter('drop_x', 200.0)             # mm
        self.declare_parameter('drop_y', 381.0)             # mm
        self.declare_parameter('drop_z', 127.0)             # mm
        self.declare_parameter('pickup_surface_z', 0.0)     # mm
        self.declare_parameter('gripper_open_pos', 0.0)     # drive_joint: 0.0=open, 0.85=closed
        self.declare_parameter('gripper_close_pos', 0.85)  # drive_joint: fully closed (grip tool)
        self.declare_parameter('gripper_effort', 0.0)
        self.declare_parameter('move_speed', 0.3)
        self.declare_parameter('move_acc', 0.3)

        # ── Depth-monitor parameters ─────────────────────────────────
        self.declare_parameter('use_depth_monitor', True)
        self.declare_parameter('descent_step_m', 0.005)       # 5mm per step
        self.declare_parameter('slowdown_far_m', 0.040)       # full speed above 40mm
        self.declare_parameter('slowdown_near_m', 0.010)      # crawl below 10mm
        self.declare_parameter('stop_distance_m', 0.002)      # stop at 2mm
        self.declare_parameter('descent_speed_full', 0.3)
        self.declare_parameter('descent_speed_crawl', 0.05)
        self.declare_parameter('safety_tolerance_m', 0.030)   # abort if measured vs expected differ by >30mm

        # ── Clients ──────────────────────────────────────────────────
        self.move_client = ActionClient(
            self, MoveGroup, MOVE_ACTION, callback_group=self.cb_group)
        self.gripper_client = ActionClient(
            self, GripperCommand, GRIPPER_ACTION, callback_group=self.cb_group)
        self.ik_client = self.create_client(
            GetPositionIK, IK_SERVICE, callback_group=self.cb_group)

        # ── Subscriber ───────────────────────────────────────────────
        self.subscription = self.create_subscription(
            PoseStamped, '/tool_pose', self.tool_pose_callback, 10,
            callback_group=self.cb_group)

        # ── Depth monitor interface ──────────────────────────────────
        self.depth_monitor_pub = self.create_publisher(
            Bool, '/gripper/monitor_enabled', 10)
        self._tool_distance = None
        self._cart_distance = None
        self.create_subscription(
            Float32, '/gripper/tool_distance', self._tool_dist_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            Float32, '/gripper/cart_distance', self._cart_dist_cb, 10,
            callback_group=self.cb_group)

        # ── Fine localization interface ──────────────────────────────
        self.fine_loc_pub = self.create_publisher(
            Bool, '/fine_loc/request', 10)
        self._fine_loc_result = None
        self._fine_loc_detected = None
        self.create_subscription(
            PoseStamped, '/fine_loc/result', self._fine_loc_result_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            Bool, '/fine_loc/tag_detected', self._fine_loc_detected_cb, 10,
            callback_group=self.cb_group)

        self.busy = False
        self.get_logger().info('Grasp pose generator ready. Waiting for /tool_pose...')

    # ── Helpers ───────────────────────────────────────────────────────

    def _mm(self, name):
        return self.get_parameter(name).get_parameter_value().double_value / 1000.0

    def _param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _make_pose(self, x_m, y_m, z_m):
        pose = PoseStamped()
        pose.header.frame_id = FRAME_ID
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x_m
        pose.pose.position.y = y_m
        pose.pose.position.z = z_m
        pose.pose.orientation.w = TOOL_DOWN_QUAT['w']
        pose.pose.orientation.x = TOOL_DOWN_QUAT['x']
        pose.pose.orientation.y = TOOL_DOWN_QUAT['y']
        pose.pose.orientation.z = TOOL_DOWN_QUAT['z']
        return pose

    # ── Blocking action/service helpers ───────────────────────────────

    def _send_action(self, client, goal, label, timeout=60.0):
        event = threading.Event()
        result_holder = [None]
        accepted = [False]

        def on_goal(future):
            gh = future.result()
            if gh is None or not gh.accepted:
                self.get_logger().error(f'[{label}] Goal rejected')
                event.set()
                return
            accepted[0] = True
            self.get_logger().info(f'[{label}] Executing...')
            gh.get_result_async().add_done_callback(on_result)

        def on_result(future):
            result_holder[0] = future.result()
            event.set()

        client.send_goal_async(goal).add_done_callback(on_goal)
        if not event.wait(timeout=timeout):
            self.get_logger().error(f'[{label}] Timed out')
            return None
        return result_holder[0] if accepted[0] else None

    def _call_service(self, client, request, label, timeout=10.0):
        event = threading.Event()
        result_holder = [None]

        def on_response(future):
            result_holder[0] = future.result()
            event.set()

        client.call_async(request).add_done_callback(on_response)
        if not event.wait(timeout=timeout):
            self.get_logger().error(f'[{label}] Service timed out')
            return None
        return result_holder[0]

    # ── IK solve → joint-space plan → execute ─────────────────────────

    def _solve_ik(self, pose_stamped, label):
        """Solve IK for a Cartesian pose, return joint values or None."""
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'[{label}] {IK_SERVICE} not available')
            return None

        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 500_000_000  # 0.5s
        req.ik_request.pose_stamped = pose_stamped

        resp = self._call_service(self.ik_client, req, f'{label}/IK')
        if resp is None:
            return None

        if resp.error_code.val != 1:
            self.get_logger().error(
                f'[{label}] IK failed (error_code={resp.error_code.val})')
            return None

        # Extract joint values from the IK solution
        joint_state = resp.solution.joint_state
        joint_values = {}
        for name, pos in zip(joint_state.name, joint_state.position):
            if name in JOINT_NAMES:
                joint_values[name] = pos

        if len(joint_values) != len(JOINT_NAMES):
            self.get_logger().error(f'[{label}] IK returned incomplete joints')
            return None

        self.get_logger().info(
            f'[{label}] IK solved: ' +
            ', '.join(f'{n}={joint_values[n]:.3f}' for n in JOINT_NAMES))
        return joint_values

    def _move_to_pose(self, pose_stamped, label="move"):
        """Solve IK for the target pose, then plan+execute in joint space."""
        self.get_logger().info(
            f'[{label}] Target: ({pose_stamped.pose.position.x:.3f}, '
            f'{pose_stamped.pose.position.y:.3f}, '
            f'{pose_stamped.pose.position.z:.3f})')

        # Step 1: Solve IK
        joint_values = self._solve_ik(pose_stamped, label)
        if joint_values is None:
            return False

        # Step 2: Plan in joint space (same as what RViz does)
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'[{label}] {MOVE_ACTION} not available')
            return False

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = PLANNING_GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = self._param('move_speed')
        goal.request.max_acceleration_scaling_factor = self._param('move_acc')

        # Joint-space goal constraints
        constraints = Constraints()
        for name in JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = joint_values[name]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        result = self._send_action(self.move_client, goal, label, timeout=60.0)
        if result is None:
            return False

        if result.result.error_code.val == 1:
            self.get_logger().info(f'[{label}] Success')
            return True
        else:
            self.get_logger().error(
                f'[{label}] Plan/execute failed (error_code={result.result.error_code.val})')
            return False

    # ── Gripper control ───────────────────────────────────────────────

    def _gripper(self, position, label="gripper"):
        self.get_logger().info(f'[{label}] Gripper to {position:.4f} m')

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f'[{label}] {GRIPPER_ACTION} not available — skipping.')
            return True

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self._param('gripper_effort')

        result = self._send_action(self.gripper_client, goal, label, timeout=15.0)
        if result is None:
            self.get_logger().error(f'[{label}] Gripper failed')
            return False

        self.get_logger().info(
            f'[{label}] Done (pos={result.result.position:.4f}, '
            f'reached={result.result.reached_goal})')
        return True

    def _open_gripper(self):
        return self._gripper(self._param('gripper_open_pos'), 'OPEN')

    def _close_gripper(self):
        return self._gripper(self._param('gripper_close_pos'), 'CLOSE')

    # ── Fine localization callbacks ──────────────────────────────────

    def _fine_loc_result_cb(self, msg):
        self._fine_loc_result = msg

    def _fine_loc_detected_cb(self, msg):
        self._fine_loc_detected = msg.data

    def _refine_pre_grasp(self, pre_grasp_pose):
        """Request fine localization and adjust pre-grasp XY if a tag is found."""
        self.get_logger().info('[FINE-LOC] Requesting fine localization...')
        self._fine_loc_result = None
        self._fine_loc_detected = None

        msg = Bool()
        msg.data = True
        self.fine_loc_pub.publish(msg)

        # Wait for result (up to 3 seconds)
        for _ in range(30):
            if self._fine_loc_detected is not None:
                break
            time.sleep(0.1)

        if self._fine_loc_detected is None:
            self.get_logger().warn('[FINE-LOC] Timed out — using coarse position.')
            return pre_grasp_pose

        if not self._fine_loc_detected:
            self.get_logger().warn('[FINE-LOC] No tool tag detected — using coarse position.')
            return pre_grasp_pose

        if self._fine_loc_result is None:
            self.get_logger().warn('[FINE-LOC] Detection succeeded but no result — using coarse position.')
            return pre_grasp_pose

        dx = self._fine_loc_result.pose.position.x
        dy = self._fine_loc_result.pose.position.y

        self.get_logger().info(
            f'[FINE-LOC] Correction: dx={dx*1000:.1f}mm, dy={dy*1000:.1f}mm')

        new_x = pre_grasp_pose.pose.position.x + dx
        new_y = pre_grasp_pose.pose.position.y + dy
        new_z = pre_grasp_pose.pose.position.z

        self.get_logger().info(
            f'[FINE-LOC] Adjusted pre-grasp: '
            f'({pre_grasp_pose.pose.position.x:.4f}, {pre_grasp_pose.pose.position.y:.4f}) → '
            f'({new_x:.4f}, {new_y:.4f})')

        return self._make_pose(new_x, new_y, new_z)

    # ── Depth monitor callbacks ─────────────────────────────────────

    def _tool_dist_cb(self, msg):
        self._tool_distance = msg.data

    def _cart_dist_cb(self, msg):
        self._cart_distance = msg.data

    def _set_depth_monitor(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.depth_monitor_pub.publish(msg)
        if enabled:
            self._tool_distance = None
            self._cart_distance = None
            time.sleep(0.3)

    def _get_descent_speed(self, distance_m):
        far = self._param('slowdown_far_m')
        near = self._param('slowdown_near_m')
        full = self._param('descent_speed_full')
        crawl = self._param('descent_speed_crawl')
        if distance_m > far:
            return full
        if distance_m < near:
            return crawl
        ratio = (distance_m - near) / (far - near)
        return crawl + ratio * (full - crawl)

    def _closed_loop_descent(self, start_pose, target_z, label, use_tool_dist=True):
        """Descend from start_pose to target_z using depth feedback for velocity scaling."""
        if not self.get_parameter('use_depth_monitor').value:
            pose = self._make_pose(
                start_pose.pose.position.x,
                start_pose.pose.position.y,
                target_z)
            return self._move_to_pose(pose, label)

        self._set_depth_monitor(True)

        step = self._param('descent_step_m')
        stop_dist = self._param('stop_distance_m')
        safety_tol = self._param('safety_tolerance_m')
        current_z = start_pose.pose.position.z
        x = start_pose.pose.position.x
        y = start_pose.pose.position.y

        self.get_logger().info(
            f'[{label}] Closed-loop descent: z={current_z:.4f} → {target_z:.4f}m')

        while current_z > target_z + 0.001:
            dist = self._tool_distance if use_tool_dist else self._cart_distance

            if dist is not None:
                if dist <= stop_dist:
                    self.get_logger().info(
                        f'[{label}] Contact distance reached ({dist*1000:.1f}mm). Stopping.')
                    break

                speed = self._get_descent_speed(dist)
                actual_step = min(step, dist - stop_dist)
            else:
                speed = self._param('descent_speed_crawl')
                actual_step = step

            next_z = max(target_z, current_z - actual_step)
            pose = self._make_pose(x, y, next_z)

            old_speed = self._param('move_speed')
            old_acc = self._param('move_acc')
            self.set_parameters([
                rclpy.Parameter('move_speed', rclpy.Parameter.Type.DOUBLE, speed),
                rclpy.Parameter('move_acc', rclpy.Parameter.Type.DOUBLE, speed),
            ])

            ok = self._move_to_pose(pose, f'{label}/z={next_z:.4f}')

            self.set_parameters([
                rclpy.Parameter('move_speed', rclpy.Parameter.Type.DOUBLE, old_speed),
                rclpy.Parameter('move_acc', rclpy.Parameter.Type.DOUBLE, old_acc),
            ])

            if not ok:
                self._set_depth_monitor(False)
                return False

            current_z = next_z

            if dist is not None and self._cart_distance is not None:
                expected_dist = current_z - target_z + stop_dist
                measured = self._tool_distance if use_tool_dist else self._cart_distance
                if measured is not None and abs(measured - expected_dist) > safety_tol:
                    self.get_logger().warn(
                        f'[{label}] Safety check: measured={measured*1000:.1f}mm, '
                        f'expected≈{expected_dist*1000:.1f}mm (tol={safety_tol*1000:.0f}mm)')

        self._set_depth_monitor(False)
        self.get_logger().info(f'[{label}] Descent complete at z={current_z:.4f}m')
        return True

    # ── Pick-and-place sequence ───────────────────────────────────────

    def tool_pose_callback(self, msg):
        if self.busy:
            self.get_logger().warn('Already executing. Ignoring.')
            return
        self.busy = True
        threading.Thread(target=self._run_sequence, args=(msg,), daemon=True).start()

    def _run_sequence(self, msg):
        tool_x = msg.pose.position.x
        tool_y = msg.pose.position.y
        tool_z = msg.pose.position.z

        self.get_logger().info(
            f'========== PICK-AND-PLACE START ==========\n'
            f'  Tool at: ({tool_x:.3f}, {tool_y:.3f}, {tool_z:.3f}) m')

        approach_h = self._mm('approach_height')
        tool_h = self._mm('tool_height')
        surface_z = self._mm('pickup_surface_z')
        drop_x = self._mm('drop_x')
        drop_y = self._mm('drop_y')
        drop_z = self._mm('drop_z')

        grasp_z = surface_z + tool_h

        pre_grasp = self._make_pose(tool_x, tool_y, grasp_z + approach_h)
        grasp     = self._make_pose(tool_x, tool_y, grasp_z)
        lift      = self._make_pose(tool_x, tool_y, grasp_z + approach_h)
        drop      = self._make_pose(drop_x, drop_y, drop_z)
        retreat   = self._make_pose(drop_x, drop_y, drop_z + approach_h)

        self.get_logger().info(
            f'  Waypoints (link_tcp frame, meters):\n'
            f'    PRE-GRASP: ({tool_x:.3f}, {tool_y:.3f}, {grasp_z + approach_h:.3f})\n'
            f'    GRASP:     ({tool_x:.3f}, {tool_y:.3f}, {grasp_z:.3f})\n'
            f'    LIFT:      ({tool_x:.3f}, {tool_y:.3f}, {grasp_z + approach_h:.3f})\n'
            f'    PRE-DROP:  ({drop_x:.3f}, {drop_y:.3f}, {drop_z + approach_h:.3f})\n'
            f'    DROP:      ({drop_x:.3f}, {drop_y:.3f}, {drop_z:.3f})\n'
            f'    RETREAT:   ({drop_x:.3f}, {drop_y:.3f}, {drop_z + approach_h:.3f})')

        pre_drop = self._make_pose(drop_x, drop_y, drop_z + approach_h)

        def _refine_and_descend():
            refined = self._refine_pre_grasp(pre_grasp)
            if refined.pose.position.x != pre_grasp.pose.position.x or \
               refined.pose.position.y != pre_grasp.pose.position.y:
                ok = self._move_to_pose(refined, 'REFINED-PRE-GRASP')
                if not ok:
                    return False
            return self._closed_loop_descent(refined, grasp_z, 'GRASP',
                                             use_tool_dist=True)

        steps = [
            (self._open_gripper,                                                     'Step 1: Open gripper'),
            (lambda: self._move_to_pose(pre_grasp, 'PRE-GRASP'),                    'Step 2: Pre-grasp (coarse)'),
            (_refine_and_descend,                                                    'Step 3: Fine-loc + descend'),
            (self._close_gripper,                                                    'Step 4: Close gripper'),
            (lambda: self._move_to_pose(lift, 'LIFT'),                               'Step 5: Lift'),
            (lambda: self._move_to_pose(pre_drop, 'PRE-DROP'),                       'Step 6: Move to pre-drop'),
            (lambda: self._closed_loop_descent(pre_drop, drop_z, 'DROP',
                                               use_tool_dist=False),                 'Step 7: Descend to drop'),
            (self._open_gripper,                                                     'Step 8: Release'),
            (lambda: self._move_to_pose(retreat, 'RETREAT'),                         'Step 9: Retreat'),
        ]

        for action, step_name in steps:
            self.get_logger().info(f'--- {step_name} ---')
            if not action():
                self.get_logger().error(f'ABORTED at: {step_name}')
                self.busy = False
                return

        self.get_logger().info('========== PICK-AND-PLACE COMPLETE ==========')
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = GraspPoseGenerator()
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
