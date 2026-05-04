# TRAC-IK Integration Plan for xArm5 with AprilTag Manipulation

**Branch:** `julian_tracik_integration`  
**Status:** Phase 0 Complete - Ready for Phase 1  
**Last Updated:** 2026-04-09  
**Target:** Autonomous tool manipulation with inverse kinematics and AprilTag perception

---

## Executive Summary

This plan guides development from the current working xArm5 low-level driver state through robust inverse kinematics (via TRAC-IK inside MoveIt) to autonomous AprilTag-driven tool manipulation. The path is incremental, with clear milestones and deliverables at each phase.

**Key Principle:** Validate each layer before integrating the next. Do not jump to TRAC-IK until Cartesian motion is verified. Do not add perception until deterministic motion is reliable.

---

## Current State (xarm5-ros2-starter branch)

✅ **Complete:**
- ROS 2 Jazzy on Ubuntu 24.04
- Official xarm_ros2 driver built and tested
- Network communication stable (192.168.1.234 reachable)
- Joint motion via `/xarm/set_servo_angle` confirmed working
- Minimal Cartesian command package exists (`move_to_pose.py`)
- Robot physically moved and responsive (as of 2026-04-08)

❌ **Not Yet Validated:**
- Cartesian moves via `move_to_pose` node on real hardware
- Gripper control (services exist, untested)
- MoveIt integration (intentionally excluded from starter)
- Any inverse kinematics layer
- Collision-aware planning
- Perception-driven motion

---

## Development Phases

### Phase 0: Branch Protection & Setup ✅ COMPLETE

**What was done:**
- Created `julian_tracik_integration` branch from `xarm5-ros2-starter`
- Preserved original branch as recovery point

**Status:** Ready for Phase 1

---

### Phase 1: Cartesian Motion Validation (THIS IS YOUR NEXT STEP)

**Goal:** Prove that Cartesian frame conventions are correct and consistent with real hardware before building anything on top.

**Why This First:**
- The starter docs say `move_to_pose` exists but "has not yet been validated on hardware"
- Without confirmed frame conventions, later IK results will be mathematically correct but physically wrong
- This is the foundation—get it right once

#### Phase 1a: Setup

**Prerequisites:**
1. ✅ Network: `192.168.1.234` reachable (already done)
2. ✅ Driver: xarm_ros2 built and ready to launch
3. ✅ Workspace: `~/xarm_ws` exists with 13 packages built

**If workspace not built yet:**
```bash
cd ~/xarm_ws
colcon build --symlink-install
```

#### Phase 1b: Cartesian Truth Check - Home Pose

**Command:**
```bash
# Terminal 1: Start driver
source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash
ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=192.168.1.234

# Terminal 2: Enable robot
source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash
ros2 service call /xarm/clean_error xarm_msgs/srv/Call "{}"
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"

# Terminal 3: Run home pose test
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py
```

**Expected Behavior:**
- Arm moves to x=300mm forward, y=0mm center, z=300mm height
- Tool rotates to roll=180° (pointing downward)
- Return code: `ret: 0`
- Movement is smooth, not jerky

**Log Template:**
```
TEST: Home Pose (x=300, y=0, z=300, roll=180, pitch=0, yaw=0)
├─ Command Sent: [timestamp]
├─ Arm Movement: [smooth/jerky/stall/error]
├─ Return Code: [ret value]
├─ Tool Orientation: [pointing down/up/sideways/unclear]
├─ End Position Check: [visual confirmation of actual position]
├─ Notes: [any anomalies]
└─ PASS/FAIL: [mark here]
```

#### Phase 1c: Cartesian Pose Grid

**Test Matrix:** Run 8 safe poses total

| Test # | x (mm) | y (mm) | z (mm) | roll (°) | pitch (°) | yaw (°) | Purpose |
|--------|--------|--------|--------|----------|-----------|---------|---------|
| 1 | 300 | 0 | 300 | 180 | 0 | 0 | Home/baseline |
| 2 | 350 | 0 | 300 | 180 | 0 | 0 | +X offset |
| 3 | 250 | 0 | 300 | 180 | 0 | 0 | -X offset |
| 4 | 300 | 0 | 350 | 180 | 0 | 0 | +Z offset |
| 5 | 300 | 0 | 250 | 180 | 0 | 0 | -Z offset (if safe) |
| 6 | 300 | 0 | 300 | 180 | 0 | 15 | Small yaw +15° |
| 7 | 300 | 0 | 300 | 180 | 0 | -15 | Small yaw -15° |
| 8 | 320 | 0 | 320 | 180 | 0 | 0 | Small x+z combo |

**For each pose, log:**
```
TEST: [description]
├─ Success: [PASS/FAIL]
├─ Return Code: [ret value]
├─ Motion Time: [seconds]
├─ Smoothness: [smooth/stepped/oscillating]
├─ Tool Orientation: [description]
├─ Frame Check: [x forward?] [y right?] [z up?]
├─ Joint Behavior: [any unusual limits?]
└─ Notes: [anything unexpected]
```

**Acceptance Criteria for Phase 1:**
- ✅ All 8 poses execute with ret=0
- ✅ Tool orientation is consistent with roll/pitch/yaw parameters
- ✅ Frame directions (x=forward, y=left, z=up) confirmed visually
- ✅ No joint limit warnings or collisions
- ✅ Motion is smooth and reproducible

**Deliverable:**
- `CARTESIAN_VALIDATION_TEST_MATRIX.txt` with 8 test results
- `FRAME_CONVENTIONS_VERIFIED.md` document listing:
  - Robot base frame location
  - Tool frame orientation
  - x/y/z direction meanings
  - Roll/pitch/yaw convention used
  - Safe upright mounting configuration

---

### Phase 2: MoveIt Bring-Up for xArm5

**Goal:** Get MoveIt (with RViz visualization) working on xArm5, validate planning layer separate from IK.

#### Phase 2a: Install & Configure MoveIt

**Check if xarm_moveit_config exists:**
```bash
find ~/xarm_ws/src -name "*moveit*" -type d
```

**If MoveIt config not present, build from official xarm_ros2:**
```bash
cd ~/xarm_ws/src
git clone https://github.com/xArm-Developer/xarm_ros2 --branch jazzy
cd ~/xarm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

**Locate key files:**
```bash
find ~/xarm_ws/src -name "kinematics.yaml" -o -name "xarm5_moveit_config"
```

#### Phase 2b: Launch MoveIt with RViz

**Command:**
```bash
# Terminal 1: Driver
ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=192.168.1.234

# Terminal 2: MoveIt + RViz
ros2 launch xarm_moveit_config moveit_planning_execution.launch.py robot_ip:=192.168.1.234
```

**Visual Verification in RViz:**
- [ ] Robot URDF loads correctly
- [ ] Joint state updates in real-time from driver
- [ ] No visual collision artifacts
- [ ] Planning group selector works (should have "manipulator" group)
- [ ] Interactive markers appear on end-effector

#### Phase 2c: Joint Goal Planning

**Command in RViz:**
1. Select Planning Group: "manipulator"
2. Select Planning Request: "Joint Space Goal"
3. Click "Randomize" or manually adjust joint 1 by +10°
4. Click "Plan" (should see trajectory in RViz)
5. Verify plan looks reasonable
6. Click "Execute" and observe motion

**Log:**
```
TEST: Joint Goal Planning
├─ Plan Time: [seconds]
├─ Path Quality: [smooth/jerky/unrealistic]
├─ Execution Success: [PASS/FAIL]
├─ Motion Smoothness: [smooth/stepped]
└─ Notes
```

#### Phase 2d: Pose Goal Planning

**Command in RViz:**
1. Select Planning Request: "Pose Space Goal"
2. Use interactive markers to set target pose (small offset from current)
3. Click "Plan"
4. Verify trajectory appears reasonable
5. Click "Execute"

**Log:**
```
TEST: Pose Goal Planning
├─ IK Solve Success: [yes/no - check planning log]
├─ Plan Time: [seconds]
├─ Path Quality: [smooth/jerky]
├─ Execution Success: [PASS/FAIL]
└─ Notes
```

**Acceptance Criteria for Phase 2:**
- ✅ MoveIt launches without errors
- ✅ RViz shows robot responding to joint state
- ✅ Joint goal planning succeeds
- ✅ Pose goal planning succeeds (IK is working, even if just KDL)
- ✅ One safe joint motion executes smoothly
- ✅ One safe Cartesian motion executes smoothly

**Deliverable:**
- `MOVEIT_LAUNCH_VERIFIED.md` with screenshots or descriptions of RViz state
- Copy of `kinematics.yaml` showing current solver

---

### Phase 3: TRAC-IK Solver Swap

**Goal:** Replace default KDL IK solver with TRAC-IK plugin in MoveIt.

#### Phase 3a: Install TRAC-IK Plugin

```bash
# Method 1: From apt
sudo apt install ros-jazzy-trac-ik

# Method 2: From source (if needed)
cd ~/xarm_ws/src
git clone https://github.com/traclabs/trac_ik.git --branch jazzy
cd ~/xarm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

**Verify Installation:**
```bash
ros2 pkg list | grep trac_ik
# Should show: trac_ik_kinematics_plugin (and others)
```

#### Phase 3b: Update kinematics.yaml

**Locate file:**
```bash
find ~/xarm_ws -name "kinematics.yaml" -path "*/xarm5*"
```

**Edit** `xarm_moveit_config/config/kinematics.yaml`:

**Before:**
```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_timeout: 0.05
```

**After:**
```yaml
manipulator:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.1
  solve_type: Speed
  # Optional tuning
  # position_only_ik: false
  # epsilon: 0.000001
```

**Rebuild:**
```bash
cd ~/xarm_ws
colcon build --symlink-install
```

#### Phase 3c: Test TRAC-IK with Pose Goals

**Re-launch MoveIt:**
```bash
# Terminal 1: Driver
ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=192.168.1.234

# Terminal 2: MoveIt (now with TRAC-IK)
ros2 launch xarm_moveit_config moveit_planning_execution.launch.py robot_ip:=192.168.1.234
```

**Test 10 Poses from Phase 1 test matrix:**
- Send each pose goal through MoveIt planning
- Check ROS logs for TRAC-IK solver calls:
  ```bash
  ros2 topic echo /rosout --filter "TRAC" | head -20
  ```
- Log: success, solve time, path quality

**Test Log Template:**
```
TRAC-IK Test Series (10 poses from Phase 1 matrix)
├─ Pose 1 (300, 0, 300): [SUCCESS/FAIL] Solve time: [ms]
├─ Pose 2 (350, 0, 300): [SUCCESS/FAIL] Solve time: [ms]
├─ Pose 3 (250, 0, 300): [SUCCESS/FAIL] Solve time: [ms]
├─ ... (continue for 10 poses)
└─ Summary: [X/10 successful] [avg solve time: ms]
```

**Acceptance Criteria for Phase 3:**
- ✅ TRAC-IK plugin installed and recognized by MoveIt
- ✅ kinematics.yaml updated with TRAC-IK solver
- ✅ MoveIt launches without IK solver errors
- ✅ ≥9/10 pose goals solve successfully
- ✅ Solve times reasonable (<0.5s typical)
- ✅ No geometric singularities or unexpected failures

**Deliverable:**
- Updated `kinematics.yaml` file
- `TRACIK_TEST_RESULTS.txt` with success/failure/timing for 10 poses

---

### Phase 4: Reachability Characterization

**Goal:** Map out what xArm5 can and cannot reliably solve, define grasp orientation constraints.

**Test Grid:**
- **Fixed tool orientation:** roll=180°, pitch=0°, yaw=0° (tool pointing down—typical grasp)
- **Yaw variations:** ±15°, ±30° (if safe)
- **Pre-grasp z offsets:** -50mm, -100mm, -150mm, -200mm (approach from above)
- **Lift heights:** 200mm, 300mm, 400mm, 500mm above table
- **Lateral positions:** grid over table (±200mm x, ±200mm y)

**For each pose:**
- Try to solve with TRAC-IK
- Log: success/failure, solve time, joint configuration, any warnings

**Deliverable:**
- Reachability map (JSON or CSV grid)
- Recommended grasp orientation family
- Safe pose template for manipulation tasks

---

### Phase 5: Grasp Pose Generator

**Goal:** Convert tool pose → deterministic grasp sequence.

**Create** `grasp_pose_generator.py`:
```python
def tool_pose_to_grasp_sequence(tool_pose):
    """
    tool_pose: [x, y, z, roll, pitch, yaw] in robot frame
    Returns: {
        'pre_grasp': [x, y, z+offset, ...],
        'grasp': [x, y, z, ...],
        'lift': [x, y, z+lift_height, ...],
        'pre_drop': [drop_x, drop_y, drop_z+offset, ...],
        'drop': [drop_x, drop_y, drop_z, ...],
        'retreat': [drop_x, drop_y, drop_z+offset, ...]
    }
    """
```

**Deterministic offsets:**
- Pre-grasp z offset: -100mm (approach from above)
- Grasp height: actual tool z
- Lift height: +300mm above grasp
- Pre-drop z offset: -100mm
- Drop location: hardcoded safe zone or from config

---

### Phase 6: Collision Scene Setup

**Goal:** Add table and safety geometry to MoveIt planning scene.

**Add to MoveIt config:**
- Table collision object (geometry, height, bounds)
- Tool staging area (no-drop zone)
- Drop zone (safe area)
- Safety margins

---

### Phase 7: Gripper Integration

**Goal:** Test gripper services and script pick-and-place sequence.

**Test Sequence:**
1. Open gripper
2. Move to pre-grasp
3. Move to grasp
4. Close gripper
5. Lift
6. Move to drop
7. Open gripper
8. Retreat

**Gripper Service Tests:**
```bash
# Enable
ros2 service call /xarm/set_gripper_enable xarm_msgs/srv/SetInt16 "{data: 1}"

# Set speed (1-5000)
ros2 service call /xarm/set_gripper_speed xarm_msgs/srv/SetFloat32 "{data: 1500.0}"

# Open (850 = fully open)
ros2 service call /xarm/set_gripper_position xarm_msgs/srv/GripperMove "{pos: 850.0, wait: true, timeout: 5.0}"

# Close (0 = closed)
ros2 service call /xarm/set_gripper_position xarm_msgs/srv/GripperMove "{pos: 0.0, wait: true, timeout: 5.0}"
```

---

### Phase 8: AprilTag Integration

**Goal:** Connect tag detection to autonomous pick-and-place.

**Pipeline:**
1. AprilTag node detects tool pose
2. Tool pose → grasp_pose_generator → target poses
3. MoveIt/TRAC-IK solves each target
4. Gripper controller executes sequence
5. Tool moved to drop zone

---

## Milestones

### Milestone A: Low-level Motion Validated ✅ READY
**Prerequisites:** Phase 1 complete
- Direct Cartesian service moves verified on hardware
- Frame directions understood (x/y/z, roll/pitch/yaw)
- Safe upright pose confirmed
- Test matrix: 8/8 poses successful

### Milestone B: MoveIt Stack Validated
**Prerequisites:** Phase 2 complete
- xArm5 visible and responsive in RViz
- Planning succeeds for joint and pose goals
- One safe joint motion executes
- One safe Cartesian motion executes

### Milestone C: TRAC-IK Active
**Prerequisites:** Phase 3 complete
- kinematics.yaml uses TRAC-IK plugin
- 10+ pose goals solve successfully
- Solve times reasonable

### Milestone D: Deterministic Pick Pipeline
**Prerequisites:** Phases 4, 5, 6, 7 complete
- Pre-grasp → grasp → lift → drop sequence reliable
- Gripper control integrated
- One tool picked and placed at one location

### Milestone E: AprilTag Integration Complete
**Prerequisites:** Phase 8 complete
- Tag-detected tool picked and placed autonomously
- Full end-to-end system operational

---

## Risk Management

| Risk | Severity | Mitigation |
|------|----------|-----------|
| TRAC-IK used before MoveIt is stable | High | Phase 2 must complete before Phase 3 |
| Overly strict pose goals for 5-DOF arm | High | Phase 4 characterizes reachable families |
| Bad frame alignment from AprilTag | High | Do Phase 1 frame validation first |
| Perception integration too early | Medium | Phases 1-7 with hard-coded poses first |
| Gripper timing not tuned | Medium | Phase 7 dedicated to gripper testing |

---

## Architecture: Four-Node System

Once complete, the system will be structured as:

```
┌─────────────────────┐
│  AprilTag Listener  │ (reads /tool_pose)
│  node               │
└──────────┬──────────┘
           │ tool_pose
           ▼
┌─────────────────────┐
│  Grasp Pose         │ (converts pose to sequence)
│  Generator          │
└──────────┬──────────┘
           │ target_poses
           ▼
┌─────────────────────┐         ┌──────────────────┐
│  MoveIt Executor    │────────▶│  TRAC-IK Solver  │
│  node               │ (queries for IK)
└──────────┬──────────┘         │  inside MoveIt   │
           │ joint_goals        └──────────────────┘
           ▼
┌─────────────────────┐
│  Gripper Controller │ (coordinates open/close)
│  node               │
└─────────────────────┘
```

---

## Next Immediate Steps

1. ✅ **Phase 0 Complete:** Branch created
2. **→ NEXT: Phase 1a-c: Validate Cartesian motion on hardware**
   - Run home pose test
   - Run pose grid (8 poses)
   - Document frame conventions
   - Build test matrix

3. **Then: Phase 2a-d: MoveIt bring-up**
4. **Then: Phase 3a-c: TRAC-IK swap**
5. **Then: Phases 4-8: Full system integration**

---

## How to Track Progress

- Update this file as each phase completes
- Maintain test matrices and logs in `/PHASE_X_RESULTS/`
- Tag git commits at each milestone
- Keep `xarm5-ros2-starter` branch untouched for recovery

---

**Status:** ✅ Phase 0 complete, ready for Phase 1  
**Next Action:** Run Cartesian validation test matrix  
**Estimated Time to Milestone A:** 2-4 hours (mostly testing)  
**Estimated Time to Milestone E:** 3-4 weeks (all phases)
