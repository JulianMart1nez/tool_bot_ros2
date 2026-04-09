#!/usr/bin/env python3
"""
move_to_pose.py
---------------
Minimal xArm5 Cartesian move node.

Assumes the official UFactory xarm_api driver is already running:
  ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=<IP>

This node:
  1. Enables motion      (/xarm/motion_enable)
  2. Sets mode 0         (/xarm/set_mode)
  3. Sets state 0        (/xarm/set_state)
  4. Moves to target     (/xarm/set_position)

All units:
  x, y, z     : millimeters
  roll, pitch, yaw : degrees
  speed        : mm/s
  acc          : mm/s^2

Safe default target for xArm5 (upright base mount, table in front):
  x=300, y=0, z=300, roll=180, pitch=0, yaw=0
  (300 mm forward, centered, 300 mm above base, tool pointing down)
"""

import rclpy
from rclpy.node import Node

# Official UFactory message types - these come from xarm_msgs package
from xarm_msgs.srv import SetInt16       # set_mode, set_state, set_gripper_enable
from xarm_msgs.srv import SetInt16ById   # motion_enable (needs joint id)
from xarm_msgs.srv import MoveCartesian  # set_position (Cartesian move)


class MoveToPoser(Node):

    def __init__(self):
        super().__init__('move_to_pose')

        # ---------------------------------------------------------------
        # Declare parameters with safe defaults for xArm5.
        # Override any of these on the command line or via launch file.
        # ---------------------------------------------------------------
        self.declare_parameter('x',     300.0)   # mm forward
        self.declare_parameter('y',       0.0)   # mm lateral (0 = centered)
        self.declare_parameter('z',     300.0)   # mm above base
        self.declare_parameter('roll',  180.0)   # degrees (180 = tool down)
        self.declare_parameter('pitch',   0.0)   # degrees
        self.declare_parameter('yaw',     0.0)   # degrees
        self.declare_parameter('speed',  50.0)   # mm/s  (conservative for first run)
        self.declare_parameter('acc',   500.0)   # mm/s^2

        # ---------------------------------------------------------------
        # Create service clients.
        # These talk to the driver node started by xarm5_driver.launch.py.
        # ---------------------------------------------------------------
        self.cli_motion_enable = self.create_client(
            SetInt16ById, '/xarm/motion_enable')

        self.cli_set_mode = self.create_client(
            SetInt16, '/xarm/set_mode')

        self.cli_set_state = self.create_client(
            SetInt16, '/xarm/set_state')

        self.cli_set_position = self.create_client(
            MoveCartesian, '/xarm/set_position')

    # -------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------

    def _wait_for_service(self, client, name, timeout=5.0):
        """Block until the service is available or timeout."""
        self.get_logger().info(f'Waiting for service: {name}')
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'Service {name} not available after {timeout}s. '
                'Is the xarm5 driver running?')
            return False
        return True

    def _call_sync(self, client, request, description):
        """
        Send a request and spin until the future completes.
        Returns the response or None on failure.
        """
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error(f'{description}: no response (timeout or exception)')
            return None

        resp = future.result()
        if resp.ret != 0:
            self.get_logger().error(
                f'{description} returned error code {resp.ret}: {resp.message}')
        else:
            self.get_logger().info(f'{description}: OK (ret=0)')

        return resp

    # -------------------------------------------------------------------
    # Main sequence
    # -------------------------------------------------------------------

    def run(self):
        """Execute the full enable → mode → state → move sequence."""

        # -- Wait for all services to come up ---------------------------
        services = [
            (self.cli_motion_enable, '/xarm/motion_enable'),
            (self.cli_set_mode,      '/xarm/set_mode'),
            (self.cli_set_state,     '/xarm/set_state'),
            (self.cli_set_position,  '/xarm/set_position'),
        ]
        for client, name in services:
            if not self._wait_for_service(client, name):
                return  # driver not running, abort

        # -- Step 1: Enable motion (id=8 enables all joints) -------------
        req = SetInt16ById.Request()
        req.id = 8    # 8 = all joints; use 1-5 for individual joints
        req.data = 1  # 1 = enable, 0 = disable
        resp = self._call_sync(self.cli_motion_enable, req, 'motion_enable')
        if resp is None or resp.ret != 0:
            return

        # -- Step 2: Set mode 0 (position mode) --------------------------
        # Mode 0: firmware handles trajectory planning (recommended for beginners)
        # Mode 1: servo mode (real-time, advanced use only)
        req = SetInt16.Request()
        req.data = 0  # 0 = position mode
        resp = self._call_sync(self.cli_set_mode, req, 'set_mode(0)')
        if resp is None or resp.ret != 0:
            return

        # -- Step 3: Set state 0 (ready/running) -------------------------
        # State 0: ready to accept motion commands
        # State 4: stop state
        req = SetInt16.Request()
        req.data = 0  # 0 = ready
        resp = self._call_sync(self.cli_set_state, req, 'set_state(0)')
        if resp is None or resp.ret != 0:
            return

        # -- Step 4: Read target parameters ------------------------------
        x     = self.get_parameter('x').get_parameter_value().double_value
        y     = self.get_parameter('y').get_parameter_value().double_value
        z     = self.get_parameter('z').get_parameter_value().double_value
        roll  = self.get_parameter('roll').get_parameter_value().double_value
        pitch = self.get_parameter('pitch').get_parameter_value().double_value
        yaw   = self.get_parameter('yaw').get_parameter_value().double_value
        speed = self.get_parameter('speed').get_parameter_value().double_value
        acc   = self.get_parameter('acc').get_parameter_value().double_value

        self.get_logger().info(
            f'Target pose: x={x} y={y} z={z} '
            f'roll={roll} pitch={pitch} yaw={yaw} '
            f'speed={speed} mm/s  acc={acc} mm/s^2')

        # -- Step 5: Send Cartesian move ---------------------------------
        req = MoveCartesian.Request()
        # pose is [x, y, z, roll, pitch, yaw] in mm and degrees
        req.pose     = [float(x), float(y), float(z),
                        float(roll), float(pitch), float(yaw)]
        req.speed    = float(speed)
        req.acc      = float(acc)
        req.mvtime   = 0.0    # 0 = let firmware determine duration from speed
        req.wait     = True   # block until motion completes
        req.timeout  = 30.0   # seconds before giving up
        req.radius   = -1.0   # -1 = no blending radius (point-to-point)

        self.get_logger().info('Sending Cartesian move command...')
        resp = self._call_sync(self.cli_set_position, req, 'set_position')

        if resp is not None and resp.ret == 0:
            self.get_logger().info('Move completed successfully.')
        else:
            self.get_logger().error(
                'Move failed. Check robot state, error codes, and that '
                'the target is within the xArm5 workspace.')


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoser()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
