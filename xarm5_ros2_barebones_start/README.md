# xArm5 ROS 2 Jazzy — Barebones Starter

A minimal, working ROS 2 project for the **UFactory xArm5** with the standard xArm gripper on **Ubuntu 24.04 + ROS 2 Jazzy**.

**Robot IP:** `192.168.1.234`  
**Laptop Ethernet interface:** `enxa0cec8775699` at `192.168.1.100/24`  
**ROS 2 workspace:** `~/xarm_ws`

---

## What Is In This Repo

```
xarm5_ros2_barebones_start/
├── README.md                          ← you are here
├── FULL_START_STEPS.txt               ← step-by-step operational checklist
└── xarm5_basic_position_cmd/          ← ROS 2 package
    ├── package.xml
    ├── setup.py / setup.cfg
    ├── resource/
    ├── launch/
    │   └── move_to_pose.launch.py     ← launch file for Cartesian moves
    └── xarm5_basic_position_cmd/
        ├── __init__.py
        └── move_to_pose.py            ← main motion node
```

The `xarm_ros2` driver lives separately at `~/xarm_ws/src/xarm_ros2` (official UFactory repo, not included here).

---

## Hardware

- UFactory xArm5 (5-DOF)
- Standard UFactory xArm gripper
- Ubuntu 24.04 laptop connected via Ethernet to the xArm control box

---

## One-Time Setup (Already Done — Skip Unless Starting Fresh)

```bash
# Install ROS 2 Jazzy (desktop-full)
# https://docs.ros.org/en/jazzy/Installation.html

# Create workspace and clone driver
mkdir -p ~/xarm_ws/src
cd ~/xarm_ws/src
git clone https://github.com/xArm-Developer/xarm_ros2 --branch jazzy --recursive

# Copy this package into the workspace
cp -r xarm5_basic_position_cmd ~/xarm_ws/src/

# Install dependencies and build
cd ~/xarm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

---

## Every Session — Quick Start

Open **4 terminals**. Use this layout:

```
┌─────────────────────┬─────────────────────┐
│  TL — Driver        │  TR — Enable/Control │
├─────────────────────┼─────────────────────┤
│  BL — Move commands │  BR — Claude / notes │
└─────────────────────┴─────────────────────┘
```

### Source command (run at top of EVERY terminal)
```bash
source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash
```

---

### TL — Start the driver (leave running all session)
```bash
source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash
ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=192.168.1.234
```

Wait for:
```
Tcp control connection successful
[TCP STATUS] CONTROL: 1, REPORT: 1
```

---

### TR — Enable the robot (run once after every driver start or e-stop)
```bash
ros2 service call /xarm/clean_error xarm_msgs/srv/Call "{}"
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```

All four should return `ret=0`. If any return non-zero, check robot state in UFactory Studio at `http://192.168.1.234`.

---

### BL — Send joint move commands

**Format:**
```bash
ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [J1, J2, J3, J4, J5], speed: 0.1, acc: 2.0, mvtime: 0.0, wait: true}"
```

- Angles are in **radians** (1 rad ≈ 57.3°)
- `speed: 0.1` is slow and safe for testing
- Keep commands on a **single line** — multiline paste breaks the shell

**Safe examples:**

Rotate base 11° right (joint 1 only — safest test):
```bash
ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [0.2, 0.0, 0.0, 0.0, 0.0], speed: 0.1, acc: 2.0, mvtime: 0.0, wait: true}"
```

Return to home (all zeros):
```bash
ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [0.0, 0.0, 0.0, 0.0, 0.0], speed: 0.1, acc: 2.0, mvtime: 0.0, wait: true}"
```

**WARNING — table collision risk:**  
Positive angles on J2 and J4 move the arm **down toward the table** in this mounting. Always test J1 first. Use UFactory Studio manual jog to explore safe ranges before sending large moves.

---

### Cartesian move (via the move_to_pose node)

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py
```

Override parameters:
```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=250.0 z:=350.0 speed:=30.0
```

Units: x/y/z in mm, roll/pitch/yaw in degrees, speed in mm/s.

---

## Verified Working Services

Confirmed correct service names as of 2026-04-08 (use `ros2 service list --show-types` to verify):

| Service | Type | Purpose |
|---|---|---|
| `/xarm/clean_error` | `xarm_msgs/srv/Call` | Clear error/e-stop state |
| `/xarm/motion_enable` | `xarm_msgs/srv/SetInt16ById` | Enable joints (id=8 = all) |
| `/xarm/set_mode` | `xarm_msgs/srv/SetInt16` | Set control mode (0=position) |
| `/xarm/set_state` | `xarm_msgs/srv/SetInt16` | Set robot state (0=ready) |
| `/xarm/set_servo_angle` | `xarm_msgs/srv/MoveJoint` | Joint move (radians) |
| `/xarm/set_servo_angle_j` | `xarm_msgs/srv/MoveJoint` | Joint move variant |
| `/xarm/set_position` | `xarm_msgs/srv/MoveCartesian` | Cartesian move (mm + degrees) |
| `/xarm/move_gohome` | `xarm_msgs/srv/MoveHome` | Move to factory home |
| `/xarm/vc_set_joint_velocity` | `xarm_msgs/srv/MoveVelocity` | Velocity control |
| `/xarm/get_servo_angle` | `xarm_msgs/srv/GetFloat32List` | Read joint angles |

---

## Return Codes

| ret | Meaning |
|---|---|
| 0 | Success |
| 1 | Warning / move blocked (check state) |
| 9 | Not enabled — run enable sequence |
| -9 | Not ready — run enable sequence |

---

## Robot State Reference

Check with: `ros2 topic echo /xarm/robot_states --once`

| `state` field | Meaning |
|---|---|
| 1 | In motion |
| 2 | Standby / sleep |
| 4 | Stopped (e-stop or error) |

After e-stop: `state` will be 4. Run the full enable sequence in TR to recover.

---

## Known Gotchas

1. **`/xarm/move_joint` does NOT work** — it appears in `ros2 service list` but has no type and never responds. The correct service is `/xarm/set_servo_angle`.

2. **Launch file name** — it is `xarm5_driver.launch.py`, not `xarm_driver.launch.py`.

3. **Multiline paste breaks ros2 service call** — always keep the command on one line in the terminal.

4. **E-stop resets everything** — after hitting e-stop, always run the full 4-command enable sequence before sending any moves.

5. **set_state returns ret=0 but robot stays in state=4** — this is expected if `clean_error` was not called first. Always call `clean_error` before the enable sequence.

6. **Positive J2/J4 = toward table** — for this mounting configuration, positive angles on joints 2 and 4 move the arm downward. Use small values and always test J1 first.

---

## Checking Robot Status

```bash
# Live joint angles
ros2 topic echo /xarm/joint_states --once

# Full robot state (mode, state, errors)
ros2 topic echo /xarm/robot_states --once

# All available services with types
ros2 service list --show-types | grep xarm
```

---

## AI Handoff Context

If you are an AI assistant picking up this project, read this section first.

**What has been accomplished:**
- Full ROS 2 Jazzy + xarm_ros2 workspace built and verified at `~/xarm_ws`
- Robot confirmed reachable at `192.168.1.234` (sub-1ms ping)
- Driver successfully connects, joint states stream correctly
- Joint moves confirmed working via `/xarm/set_servo_angle`
- Robot physically moved and verified responsive as of 2026-04-08

**What has NOT been tested yet:**
- Cartesian moves via `move_to_pose.launch.py` (node exists, not yet run successfully)
- Gripper control
- Any autonomous task or pick-and-place sequence

**The biggest trap:** The documentation and internet examples say to use `/xarm/move_joint` — this service exists in the list but does NOT work. Always use `/xarm/set_servo_angle` for joint moves.

**Safe first move to confirm the robot responds:**
```bash
# In any sourced terminal, after running the enable sequence:
ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [0.1, 0.0, 0.0, 0.0, 0.0], speed: 0.1, acc: 2.0, mvtime: 0.0, wait: true}"
```

This only moves joint 1 (base rotation) by ~6° and cannot hit the table.
