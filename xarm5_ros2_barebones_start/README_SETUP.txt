================================================================================
xArm5 ROS 2 Jazzy - Bare Minimum Setup & Test Guide
Robot: UFactory xArm5 with standard xArm gripper
ROS:   ROS 2 Jazzy on Ubuntu 24.04
================================================================================

================================================================================
>>> RESUME POINT - SETUP COMPLETED UP TO HERE (2026-04-08)
================================================================================

The following steps have already been completed and do NOT need to be repeated:

  [DONE] ROS 2 Jazzy confirmed installed
  [DONE] colcon and rosdep confirmed installed
  [DONE] ~/xarm_ws/src created
  [DONE] xarm_ros2 cloned (jazzy branch) with submodules at ~/xarm_ws/src/xarm_ros2
  [DONE] xarm5_basic_position_cmd package copied to ~/xarm_ws/src/
  [DONE] rosdep dependencies installed (all required rosdeps installed successfully)
  [DONE] Full colcon build completed - 13 packages, 0 errors, 1 harmless warning
  [DONE] Robot reachable: ping 192.168.1.234 returns 0% loss, <1ms RTT
  [DONE] Laptop Ethernet interface enxa0cec8775699 is at 192.168.1.100/24 (correct subnet)
  [DONE] xarm5_driver.launch.py confirmed present in xarm_api
  [DONE] move_to_pose executable confirmed registered in xarm5_basic_position_cmd

NEXT STEP TO PICK UP FROM:
  Section C - First Hardware Bring-Up, Step C1.
  The driver has NOT been launched yet. Start there.

  Before launching the driver:
    1. Ensure the robot is powered on and shows Ready in UFactory Studio
       http://192.168.1.234
    2. Open a terminal and source the workspace:
       source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash
    3. Then continue at section C below.

================================================================================
>>> END RESUME POINT
================================================================================


SAFETY - READ FIRST
-------------------
- Keep the robot workspace clear before every test.
- Use only small, slow motions until you know the robot behavior.
- Know where the E-stop is before powering on.
- Confirm the mounting surface is secure before commanding motion.
- Verify coordinate targets are within safe reach BEFORE issuing commands.
  xArm5 max reach: ~700 mm from base center.

================================================================================
A. VERIFY COMMUNICATION WITH THE CONTROL BOX
================================================================================

The xArm control box must be on the same subnet as your laptop.

A1. Find the robot IP
---------------------
The robot IP is set in UFactory Studio (the web UI on the control box).
Default factory IP is often 192.168.1.xxx. Check the label on the control box
or look in UFactory Studio under Settings > Network.

Common IP ranges:
  192.168.1.xxx  (factory default subnet)
  192.168.31.xxx (some units)

A2. Set your laptop to the same subnet
---------------------------------------
If the robot is at 192.168.1.234, your laptop Ethernet interface must be on
192.168.1.xxx (different last octet, same /24 subnet).

Check your current network interfaces:
  ip addr show

Set a static IP on the Ethernet interface (example with interface eth0):
  sudo ip addr add 192.168.1.100/24 dev eth0
  sudo ip link set eth0 up

Or use Network Manager:
  nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.100/24
  nmcli con mod "Wired connection 1" ipv4.method manual
  nmcli con up "Wired connection 1"

A3. Ping the robot
------------------
Replace 192.168.1.234 with your actual robot IP:

  ping -c 4 192.168.1.234

Expected output: 4 packets transmitted, 4 received, 0% packet loss, <5ms RTT.

If ping fails:
  - Wrong subnet on laptop                   → fix ip addr
  - Cable not connected or wrong port        → check physical connection
  - Control box powered off                  → power it on
  - IP address is wrong                      → verify in UFactory Studio
  - Firewall blocking ICMP                   → sudo ufw allow from 192.168.1.0/24

A4. Check robot state in UFactory Studio before using ROS
----------------------------------------------------------
Open a browser and go to: http://<robot_ip>
(UFactory Studio runs on port 80 on the control box.)

Before commanding anything from ROS:
  - Robot should show "Ready" or "Standby" state, not "Error" or "E-stop"
  - Motors should be enabled (no error codes shown)
  - No active E-stop (check the physical E-stop button is released)
  - "Enable" the robot in Studio first if this is a fresh power-on

A5. Common failure cases
-------------------------
  WRONG SUBNET     - Laptop and robot not on the same /24 network.
  WRONG IP         - Robot IP changed or mistyped. Verify in Studio.
  E-STOP ENGAGED   - Release the physical E-stop, then re-enable in Studio.
  MOTORS NOT ON    - Click "Enable Robot" in UFactory Studio.
  NOT IN READY     - Robot may be in error state. Clear errors in Studio.
  ROS SERVICES OFF - Some builds require enabling xarm_api services in config.
                     See xarm_ros2/xarm_api/config/xarm_params.yaml if services
                     are not discovered after driver launch.

================================================================================
B. INSTALLATION AND BUILD
================================================================================

B1. Source ROS 2 Jazzy
-----------------------
  source /opt/ros/jazzy/setup.bash

Add this to ~/.bashrc to avoid sourcing every session:
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

B2. Create workspace and clone xarm_ros2
-----------------------------------------
  mkdir -p ~/xarm_ws/src
  cd ~/xarm_ws/src
  git clone -b jazzy https://github.com/xArm-Developer/xarm_ros2.git --recursive

The --recursive flag is required. The repo has submodules.

B3. Install dependencies
-------------------------
  cd ~/xarm_ws
  rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

B4. Build
----------
  cd ~/xarm_ws
  colcon build

Expected: summary with no ERRORs. Warnings about deprecated CMake items are normal.

B5. Source the workspace
-------------------------
  source ~/xarm_ws/install/setup.bash

Add to ~/.bashrc if desired:
  echo "source ~/xarm_ws/install/setup.bash" >> ~/.bashrc

================================================================================
C. FIRST HARDWARE BRING-UP
================================================================================

Replace 192.168.1.234 with your actual robot IP in every command below.

C1. Launch the xArm5 driver
-----------------------------
In terminal 1 (keep this running the whole time):

  source /opt/ros/jazzy/setup.bash
  source ~/xarm_ws/install/setup.bash
  ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=192.168.1.234

Expected output: Driver starts, connects to robot, publishes joint states.
You will see lines like:
  [xarm_driver_node]: xArm5 connected ...
  [xarm_driver_node]: joint_states publisher started

If the launch file is named differently in your build, check available files:
  ls ~/xarm_ws/src/xarm_ros2/xarm_api/launch/
Note: the repo has xarm5_driver.launch.py - if you see only xarm6 files, the
repo may be on wrong branch. Verify: git -C ~/xarm_ws/src/xarm_ros2 branch

C2. Enable motion (in a new terminal)
--------------------------------------
Source ROS and workspace in every new terminal:
  source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash

Enable all joints (id=8 means all joints):
  ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"

Expected response: ret: 0, message: "" (ret=0 means success)

C3. Set mode and state
-----------------------
Set mode 0 (position mode, firmware-planned motion):
  ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"

Set state 0 (ready/motion state):
  ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"

ORDER MATTERS: always motion_enable → set_mode → set_state.

C4. Check joint states
-----------------------
  ros2 topic echo /xarm/joint_states --once

You should see joint positions for 5 joints.

C5. One small safe Cartesian test move
----------------------------------------
This moves to a conservative position above the table in front of the robot.
Units: x,y,z in mm | roll,pitch,yaw in degrees.

VERIFY this position is safe for YOUR mounting and environment before running:
  [300.0, 0.0, 300.0, 180.0, 0.0, 0.0]
  x=300mm forward, y=0mm center, z=300mm up, roll=180 (tool pointing down)

Speed is 50 mm/s. wait=true means the call blocks until motion completes.

  ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian \
    "{pose: [300.0, 0.0, 300.0, 180.0, 0.0, 0.0], speed: 50.0, acc: 500.0, mvtime: 0.0, wait: true, timeout: 10.0, radius: -1.0}"

Expected: robot moves slowly to that position. ret: 0 on success.

C6. Note on xArm5 vs xArm6 filenames
--------------------------------------
The repo uses model-specific launch files. If an example you find online
references xarm6_driver.launch.py, substitute xarm5_driver.launch.py.
The services (/xarm/set_position, /xarm/motion_enable, etc.) are identical
across xArm5/6/7 - only the joint count differs.

================================================================================
D. GRIPPER VERIFICATION
================================================================================

The xArm gripper is on the end of the arm and communicates via RS-485 through
the arm itself. The driver must be running first.

D1. Verify gripper services exist
-----------------------------------
  ros2 service list | grep gripper

You should see services including:
  /xarm/get_gripper_err_code
  /xarm/get_gripper_position
  /xarm/set_gripper_enable
  /xarm/set_gripper_position
  /xarm/set_gripper_speed

If none appear, the driver may not have gripper support enabled.
Check that add_gripper:=true (default) in the launch args:
  ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=<IP> add_gripper:=true

D2. Initialize the gripper
----------------------------
Enable the gripper:
  ros2 service call /xarm/set_gripper_enable xarm_msgs/srv/SetInt16 "{data: 1}"

Set gripper speed (1-5000, typical: 1500):
  ros2 service call /xarm/set_gripper_speed xarm_msgs/srv/SetFloat32 "{data: 1500.0}"

D3. Optional: test gripper open/close
---------------------------------------
Gripper position range: 0 (closed) to 850 (fully open).

Move to half open (wait for completion):
  ros2 service call /xarm/set_gripper_position xarm_msgs/srv/GripperMove \
    "{pos: 400.0, wait: true, timeout: 5.0}"

Close gripper:
  ros2 service call /xarm/set_gripper_position xarm_msgs/srv/GripperMove \
    "{pos: 0.0, wait: true, timeout: 5.0}"

Get current gripper position:
  ros2 service call /xarm/get_gripper_position xarm_msgs/srv/GetFloat32 "{}"

================================================================================
E. SAFETY NOTES (SHORT VERSION)
================================================================================

1. First motion should be SLOW (50 mm/s or less) and SHORT (50 mm or less).
2. Confirm the target pose coordinates match your actual mounting orientation.
   x=forward, y=left, z=up assumes standard upright base mounting.
3. Clear a minimum 1-meter radius around the robot before every test.
4. Keep the E-stop within arm's reach.
5. If the robot behaves unexpectedly, hit E-stop immediately.
6. ret=0 from a service means the command was accepted, not that it is safe.
   You must verify the target is physically valid before calling it.

================================================================================
END OF SETUP GUIDE
================================================================================
