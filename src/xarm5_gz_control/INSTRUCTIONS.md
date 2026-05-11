# xArm5 Gazebo Harmonic Simulation — Complete Instructions
## Version 2.0 | ROS2 Jazzy | Ubuntu 24.04 | Gazebo Harmonic

---

## Prerequisites

These must be installed on your machine before cloning or building.

### Operating System
- **Ubuntu 24.04 (Noble)** — other distros are not tested

### ROS2 Jazzy
Follow the official install guide at https://docs.ros.org/en/jazzy/Installation.html
or run the quick install:

```bash
sudo apt update && sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Gazebo Harmonic + ROS2 bridge

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control
```

### MoveIt2

```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-py \
  ros-jazzy-xacro
```

### ros2_control (usually pulled in by the above, but install explicitly to be safe)

```bash
sudo apt install -y \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-joint-state-broadcaster
```

---

## How This Package Works

The key design decision: instead of using UFACTORY's `xarm_controller` C++ plugin
(which does not compile on Jazzy), simulation uses `gz_ros2_control/GazeboSimSystem`
which is already installed as `ros-jazzy-gz-ros2-control`.

---

## Step 1: Workspace Setup (One Time Only)

### 1a. Place folders in your workspace

Your workspace `src/` folder should look like this:

```
~/Xarm_project/src/
  ├── xarm_ros2/            ← UFACTORY ROS2 packages
  ├── xarm_python_sdk/      ← Python SDK (COLCON_IGNORE added)
  └── xarm5_gz_control/     ← This package
```

### 1b. Exclude packages that cannot build on Jazzy

Run these commands once — they tell colcon to skip packages that require
Classic Gazebo or have Jazzy-incompatible code:

```bash
# Exclude the Python SDK (broken setup.py)
touch ~/Xarm_project/src/xarm_python_sdk/COLCON_IGNORE

# Exclude RealSense plugin (requires Classic Gazebo gazebo_ros)
touch ~/Xarm_project/src/xarm_ros2/thirdparty/realsense_gazebo_plugin/COLCON_IGNORE

# Exclude demo and vision packages (not needed)
touch ~/Xarm_project/src/xarm_ros2/demo/mbot_demo/COLCON_IGNORE
touch ~/Xarm_project/src/xarm_ros2/xarm_vision/d435i_xarm_setup/COLCON_IGNORE

# Exclude xarm_gazebo (depends on xarm_controller which does not build on Jazzy)
touch ~/Xarm_project/src/xarm_ros2/xarm_gazebo/COLCON_IGNORE
```

### 1c. Build the workspace

```bash
cd ~/Xarm_project
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected result: The following packages build successfully:
- `uf_ros_lib`
- `xarm_msgs`
- `xarm_description`
- `xarm_moveit_config`
- `xarm_sdk`
- `xarm_api`
- `xarm5_gz_control`

The following packages will FAIL — this is expected and OK for simulation:
- `xarm_controller`  (Jazzy-incompatible headers — not needed for sim)
- `xarm_planner`     (depends on xarm_controller)
- `xarm_moveit_servo` (depends on xarm_controller)

### 1d. Source the workspace

```bash
source ~/Xarm_project/install/setup.bash
```

Add this to your `~/.bashrc` so it's always available:

```bash
echo "source ~/Xarm_project/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 2: Verify the Build

```bash
ros2 pkg list | grep -E "xarm5_gz|xarm_description|xarm_moveit|uf_ros" | sort
```

You should see:
```
uf_ros_lib
xarm5_gz_control
xarm_description
xarm_moveit_config
```

---

## Step 3: Running the Simulation

Open **2 separate terminal windows**. In each one, run:

```bash
source ~/Xarm_project/install/setup.bash
```

---

### TERMINAL 1 — Main Simulation

```bash
ros2 launch xarm5_gz_control xarm5_sim.launch.py
```

**What starts:**
- Gazebo Harmonic with the custom simulation world (table, red pick zone, green place zone, orange cube)
- xArm5 robot spawned on the table with full xArm gripper
- `joint_state_broadcaster` + `xarm5_traj_controller` + `xarm_gripper_traj_controller`
- MoveIt2 `move_group` server with collision checking
- RViz2 with MoveIt2 Motion Planning plugin

**Wait until you see this message in the terminal:**
```
[move_group]: You can start planning now!
```
This takes about 15–30 seconds. Do NOT start Terminal 2 until you see this.

---

### TERMINAL 2 — Keyboard Controller

```bash
ros2 launch xarm5_gz_control xarm5_keyboard.launch.py
```

**What starts:**
- Keyboard teleoperation node (direct joint trajectory control)

**The following controls will appear on screen:**

| Key | Action |
|-----|--------|
| **Q** | Joint 1: increase (rotate base +) |
| **A** | Joint 1: decrease (rotate base −) |
| **W** | Joint 2: increase (shoulder up) |
| **S** | Joint 2: decrease (shoulder down) |
| **E** | Joint 3: increase (elbow up) |
| **D** | Joint 3: decrease (elbow down) |
| **R** | Joint 4: increase (wrist pitch +) |
| **F** | Joint 4: decrease (wrist pitch −) |
| **T** | Joint 5: increase (wrist roll +) |
| **G** | Joint 5: decrease (wrist roll −) |
| **Z** | Gripper: open fully |
| **X** | Gripper: close fully |
| **C** | Gripper: half open |
| **[ / ]** | Decrease / Increase step size (default 0.05 rad) |
| **Space** | Freeze — re-send current commanded position |
| **Esc** | Quit keyboard controller |

> **Tip:** Each key press moves the selected joint by the current step size.
> Use **[** and **]** to fine-tune how far each press moves the arm.
> The status line at the bottom always shows the current step size and joint positions.

---


---

## Troubleshooting

### "package 'xarm_description' not found"
The build didn't complete. Run:
```bash
cd ~/Xarm_project
colcon build --symlink-install --packages-select xarm_description xarm_msgs uf_ros_lib xarm_moveit_config xarm5_gz_control
source install/setup.bash
```

### "xacro: command not found" or URDF errors at launch
```bash
sudo apt install -y ros-jazzy-xacro
```

### Gazebo opens but robot is not visible
The spawn may have timed out. Kill everything and relaunch. Also check:
```bash
ros2 topic echo /robot_description --once | head -5
```
If empty, the xacro processing failed — check the Terminal 1 output for errors.

### "move_group: could not find group 'xarm5'"
The SRDF was not generated correctly. Make sure `xarm_moveit_config` built:
```bash
ros2 pkg list | grep xarm_moveit_config
```

### Controllers not loading
Check controller status:
```bash
ros2 control list_controllers
```
Expected:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
xarm5_traj_controller[joint_trajectory_controller/JointTrajectoryController] active
xarm_gripper_traj_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### Servo server "not available"
The servo container takes a few seconds to start. Wait 5 seconds after launching
Terminal 2 before trying to move. If still unavailable:
```bash
ros2 node list | grep servo
ros2 service list | grep servo
```

### Fixing xarm_controller for real hardware (Jazzy patch)
The UFACTORY code uses a removed header. Patch it:
```bash
# Find and patch the broken include
sed -i 's|#include "hardware_interface/visibility_control.h"||g' \
  ~/Xarm_project/src/xarm_ros2/xarm_controller/include/xarm_controller/hardware/uf_robot_system_hardware.h

sed -i 's|#include "hardware_interface/visibility_control.h"||g' \
  ~/Xarm_project/src/xarm_ros2/xarm_controller/include/xarm_controller/hardware/uf_robot_fake_system_hardware.h

# Add the replacement macro definition to the top of each file
# (The macro was removed in ros2_control 4.x — define it as empty)
sed -i '1s/^/#pragma once\n#ifndef HARDWARE_INTERFACE_PUBLIC\n#define HARDWARE_INTERFACE_PUBLIC\n#endif\n/' \
  ~/Xarm_project/src/xarm_ros2/xarm_controller/include/xarm_controller/hardware/uf_robot_system_hardware.h

# Rebuild
cd ~/Xarm_project
colcon build --symlink-install --packages-select xarm_controller
```

---

## Package File Reference

```
xarm5_gz_control/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── moveit_config.yaml      ← MoveIt2 planning, kinematics, controllers
│   ├── ros2_controllers.yaml   ← gz_ros2_control controller config
│   └── servo_config.yaml       ← MoveIt Servo tuning + safety params
├── launch/
│   ├── xarm5_sim.launch.py     ← MAIN launch (mode:=sim or mode:=real)
│   └── xarm5_keyboard.launch.py ← Keyboard controller
├── scripts/
│   └── keyboard_controller.py  ← Keyboard teleoperation node
├── urdf/
│   └── xarm5_gz.urdf.xacro    ← xArm5+gripper URDF with gz_ros2_control
└── worlds/
    └── xarm5_demo.sdf          ← Gazebo Harmonic world (table + pick/place zones)
```
