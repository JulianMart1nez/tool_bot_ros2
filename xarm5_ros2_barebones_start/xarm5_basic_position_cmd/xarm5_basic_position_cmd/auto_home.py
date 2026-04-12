#!/usr/bin/env python3
"""
auto_home.py
------------
One-shot node that sends the xArm5 to the "home" named target (all joints
zero) via MoveIt's move_group action. Exits after the move completes.

Used in the launch file to auto-home the robot on startup.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints

import threading


class AutoHome(Node):

    def __init__(self):
        super().__init__('auto_home')
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def go_home(self):
        self.get_logger().info('Waiting for /move_action server...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('/move_action not available')
            return False

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = 'xarm5'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        # Use named target "home" — defined in SRDF as all joints at 0
        constraints = Constraints()
        constraints.joint_constraints = []

        # Joint angles for home (all zero)
        from moveit_msgs.msg import JointConstraint
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        for name in joint_names:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = 0.0
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info('Sending arm to home pose (all joints zero)...')

        event = threading.Event()
        result_holder = [None]

        def on_goal_response(future):
            gh = future.result()
            if gh is None or not gh.accepted:
                self.get_logger().error('Home goal rejected')
                event.set()
                return
            self.get_logger().info('Home goal accepted, executing...')
            gh.get_result_async().add_done_callback(on_result)

        def on_result(future):
            result_holder[0] = future.result()
            event.set()

        self.client.send_goal_async(goal).add_done_callback(on_goal_response)

        if not event.wait(timeout=60.0):
            self.get_logger().error('Home move timed out')
            return False

        if result_holder[0] and result_holder[0].result.error_code.val == 1:
            self.get_logger().info('Robot is at home pose.')
            return True
        else:
            code = result_holder[0].result.error_code.val if result_holder[0] else 'none'
            self.get_logger().error(f'Home move failed (error_code={code})')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = AutoHome()
    try:
        node.go_home()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
