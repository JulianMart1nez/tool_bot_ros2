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

import threading


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
            f'    DROP:      ({drop_x:.3f}, {drop_y:.3f}, {drop_z:.3f})\n'
            f'    RETREAT:   ({drop_x:.3f}, {drop_y:.3f}, {drop_z + approach_h:.3f})')

        steps = [
            (self._open_gripper,                                   'Step 1: Open gripper'),
            (lambda: self._move_to_pose(pre_grasp, 'PRE-GRASP'),  'Step 2: Pre-grasp'),
            (lambda: self._move_to_pose(grasp, 'GRASP'),           'Step 3: Descend to grasp'),
            (self._close_gripper,                                  'Step 4: Close gripper'),
            (lambda: self._move_to_pose(lift, 'LIFT'),             'Step 5: Lift'),
            (lambda: self._move_to_pose(drop, 'DROP'),             'Step 6: Move to drop'),
            (self._open_gripper,                                   'Step 7: Release'),
            (lambda: self._move_to_pose(retreat, 'RETREAT'),       'Step 8: Retreat'),
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
