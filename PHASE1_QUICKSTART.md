# Phase 1 Quick-Start: Cartesian Motion Validation

**Goal:** Prove that Cartesian frame conventions are correct before building anything on top.

**Status:** READY TO START

---

## Pre-Flight Checklist

Before you start, verify these prerequisites:

- [ ] Network: Ping robot `ping 192.168.1.234` returns responses
- [ ] Workspace: `~/xarm_ws` exists with xarm_ros2 and xarm5_basic_position_cmd
- [ ] Workspace built: `colcon build` completed (13 packages, 0 errors)
- [ ] Source command ready: `source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash`
- [ ] Robot power: xArm5 control box powered on
- [ ] Workspace clear: No obstacles within 1 meter of arm
- [ ] E-stop: Know where it is and how to reach it

---

## Phase 1 Terminal Setup

Open 3 terminals. In each, run the source command first:

```bash
source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash
```

**Terminal 1 (Driver):**
```bash
ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=192.168.1.234
```
Wait for:
```
Tcp control connection successful
[TCP STATUS] CONTROL: 1, REPORT: 1
```
**Leave running.**

**Terminal 2 (Enable Robot):**
```bash
ros2 service call /xarm/clean_error xarm_msgs/srv/Call "{}"
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```
All should return `ret: 0`. If any fail, check UFactory Studio at http://192.168.1.234.

**Terminal 3 (Test Poses):**
Ready for pose testing below.

---

## Phase 1b: Home Pose Test

In Terminal 3:

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py
```

**What to Observe:**
1. Arm moves smoothly
2. Tool rotates to point downward
3. Movement completes in ~5-10 seconds
4. No jerking or collision warnings
5. Terminal shows: `Move completed successfully.`

**Log this test:**

```
═══════════════════════════════════════════════════════════════
TEST 1: HOME POSE (x=300, y=0, z=300, roll=180, pitch=0, yaw=0)
═══════════════════════════════════════════════════════════════

Command Executed: 
  ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py

Expected Position:
  - 300mm forward from base
  - Centered (0mm to right/left)
  - 300mm above base
  - Tool pointing downward

Observed Behavior:
  - Arm movement: [SMOOTH / JERKY / SLOW / STALLED]
  - Movement direction: [FORWARD / BACKWARD / UP / DOWN / OTHER]
  - Tool rotation: [POINTING DOWN / UP / SIDEWAYS / UNCLEAR]
  - Completion time: [_____ seconds]
  - Return code: [ret: 0 / ret: ???]
  - Any warnings: [YES / NO]

Visual Frame Check:
  - Does x-axis point forward? [YES / NO / UNCLEAR]
  - Does y-axis point left? [YES / NO / UNCLEAR]
  - Does z-axis point up? [YES / NO / UNCLEAR]
  - Roll=180 = tool down? [YES / NO / UNCLEAR]

Physical Reality Check:
  - Place a reference object (e.g., pen) in the expected position
  - Is the tool touching or very close to it? [YES / NO]
  - Is the pose stable? [YES / NO]

RESULT: [PASS ✓ / FAIL ✗]

Notes:
[Write any observations, anomalies, or concerns]
```

**If TEST 1 PASSES:** Continue to Phase 1c pose grid.

**If TEST 1 FAILS:** Debug before continuing.
- Check ROS logs: `ros2 topic echo /xarm/robot_states --once`
- Check in UFactory Studio: http://192.168.1.234
- Verify driver is running (Terminal 1 still shows connection)
- Try again after re-enabling robot (Terminal 2 commands)

---

## Phase 1c: Pose Grid Tests

Run each pose below. Keep Terminal 1 (driver) running the entire time.

For each pose, use Terminal 3:

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=<X> y:=<Y> z:=<Z> roll:=<ROLL> pitch:=<PITCH> yaw:=<YAW>
```

### Test 2: +X Offset (X forward)

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=350.0 y:=0.0 z:=300.0 roll:=180.0 pitch:=0.0 yaw:=0.0
```

```
TEST 2: +X OFFSET (x=350, y=0, z=300, roll=180, pitch=0, yaw=0)
├─ Expected: Arm moves 50mm further forward than TEST 1
├─ Movement: [smooth/jerky/stalled]
├─ Return code: [ret: ___]
├─ Position change visible? [YES / NO]
└─ RESULT: [PASS ✓ / FAIL ✗]
```

### Test 3: -X Offset (X backward)

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=250.0 y:=0.0 z:=300.0 roll:=180.0 pitch:=0.0 yaw:=0.0
```

```
TEST 3: -X OFFSET (x=250, y=0, z=300, roll=180, pitch=0, yaw=0)
├─ Expected: Arm moves 50mm closer than TEST 1
├─ Movement: [smooth/jerky/stalled]
├─ Return code: [ret: ___]
├─ Position change visible? [YES / NO]
└─ RESULT: [PASS ✓ / FAIL ✗]
```

### Test 4: +Z Offset (Z up)

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=300.0 y:=0.0 z:=350.0 roll:=180.0 pitch:=0.0 yaw:=0.0
```

```
TEST 4: +Z OFFSET (x=300, y=0, z=350, roll=180, pitch=0, yaw=0)
├─ Expected: Arm lifts 50mm higher than TEST 1
├─ Movement: [smooth/jerky/stalled]
├─ Return code: [ret: ___]
├─ Position change visible? [YES / NO]
└─ RESULT: [PASS ✓ / FAIL ✗]
```

### Test 5: -Z Offset (Z down) — CAUTION

**ONLY RUN IF TEST 4 PASSED SMOOTHLY.** This moves closer to table.

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=300.0 y:=0.0 z:=250.0 roll:=180.0 pitch:=0.0 yaw:=0.0
```

```
TEST 5: -Z OFFSET (x=300, y=0, z=250, roll=180, pitch=0, yaw=0)
├─ Expected: Arm descends 50mm closer to table
├─ Any collision warnings? [YES / NO]
├─ Movement: [smooth/jerky/COLLISION]
├─ Return code: [ret: ___]
└─ RESULT: [PASS ✓ / FAIL ✗ / SKIP (safe margin)]
```

### Test 6: Small Yaw +15°

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=300.0 y:=0.0 z:=300.0 roll:=180.0 pitch:=0.0 yaw:=15.0
```

```
TEST 6: SMALL YAW +15° (x=300, y=0, z=300, roll=180, pitch=0, yaw=15)
├─ Expected: Tool rotates ~15° around z-axis (base rotation visible)
├─ Rotation direction: [CLOCKWISE / COUNTERCLOCKWISE / UNCLEAR]
├─ Movement: [smooth/jerky]
├─ Return code: [ret: ___]
└─ RESULT: [PASS ✓ / FAIL ✗]
```

### Test 7: Small Yaw -15°

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=300.0 y:=0.0 z:=300.0 roll:=180.0 pitch:=0.0 yaw:=-15.0
```

```
TEST 7: SMALL YAW -15° (x=300, y=0, z=300, roll=180, pitch=0, yaw=-15)
├─ Expected: Tool rotates ~15° in opposite direction from TEST 6
├─ Rotation direction: [CLOCKWISE / COUNTERCLOCKWISE / UNCLEAR]
├─ Movement: [smooth/jerky]
├─ Return code: [ret: ___]
└─ RESULT: [PASS ✓ / FAIL ✗]
```

### Test 8: Small X+Z Combo

```bash
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py x:=320.0 y:=0.0 z:=320.0 roll:=180.0 pitch:=0.0 yaw:=0.0
```

```
TEST 8: X+Z COMBO (x=320, y=0, z=320, roll=180, pitch=0, yaw=0)
├─ Expected: Diagonal motion (forward + up)
├─ Motion quality: [smooth/jerky/stalled]
├─ Return code: [ret: ___]
├─ Both x and z changed? [YES / NO]
└─ RESULT: [PASS ✓ / FAIL ✗]
```

---

## Summary & Frame Validation

After all 8 tests, fill this out:

```
═══════════════════════════════════════════════════════════════
PHASE 1 SUMMARY
═══════════════════════════════════════════════════════════════

Test Results:
  TEST 1 (HOME):        [PASS / FAIL]
  TEST 2 (+X):         [PASS / FAIL]
  TEST 3 (-X):         [PASS / FAIL]
  TEST 4 (+Z):         [PASS / FAIL]
  TEST 5 (-Z):         [PASS / FAIL / SKIP]
  TEST 6 (YAW +15):    [PASS / FAIL]
  TEST 7 (YAW -15):    [PASS / FAIL]
  TEST 8 (X+Z):        [PASS / FAIL]

Total Passed: [___/8]

FRAME CONVENTION VALIDATION:
┌────────────────────────────────────────────┐
│ X-AXIS (forward/backward)                  │
│ ├─ Tests 2 & 3 show x-axis moves arm:      │
│ │  [FORWARD / BACKWARD / UNCLEAR]          │
│ └─ Direction correct? [YES / NO]           │
│                                            │
│ Y-AXIS (left/right)                        │
│ ├─ Not tested in this series but:          │
│ │  Robot should move [LEFT / RIGHT]        │
│ │  when y is positive                      │
│ └─ Assumption: [CORRECT / WRONG / UNKNOWN] │
│                                            │
│ Z-AXIS (up/down)                           │
│ ├─ Tests 4 & 5 show z-axis moves arm:      │
│ │  [UP / DOWN / UNCLEAR]                   │
│ └─ Direction correct? [YES / NO]           │
│                                            │
│ ROLL (around x-axis, tool pitch)           │
│ ├─ Tests use roll=180 (tool down):         │
│ │  [TOOL POINTS DOWN / UP / UNCLEAR]       │
│ └─ Convention correct? [YES / NO]          │
│                                            │
│ YAW (around z-axis, base rotation)         │
│ ├─ Tests 6 & 7 show yaw:                   │
│ │  [POSITIVE = CCW / CW / UNCLEAR]         │
│ └─ Convention correct? [YES / NO]          │
└────────────────────────────────────────────┘

SAFE MOUNTING CONFIGURATION:
  Base frame location: [___________]
  Table height: [_____ mm]
  Min safe z: [_____ mm] (above table)
  Max forward reach: [_____ mm]
  Max backward reach: [_____ mm]
  Workspace clear radius: [_____ mm]

NEXT MILESTONE: [READY FOR PHASE 2 / NEEDS DEBUG / RETRY]
```

---

## If Tests Fail

### Failure Mode: ret ≠ 0

- Check UFactory Studio: http://192.168.1.234
- Look for error codes or E-stop state
- Re-run enable sequence in Terminal 2
- Try again

### Failure Mode: No visible motion

- Verify driver shows `[TCP STATUS] CONTROL: 1` in Terminal 1
- Check `ros2 topic echo /xarm/joint_states --once` in a new terminal
- Joint angles should be non-zero if not at home
- If frozen, try small joint move:
  ```bash
  ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [0.1, 0.0, 0.0, 0.0, 0.0], speed: 0.1, acc: 2.0, mvtime: 0.0, wait: true}"
  ```

### Failure Mode: Jerky or erratic motion

- Reduce speed in move_to_pose command: `speed:=30.0`
- Check for obstacles in workspace
- Verify table is stable

---

## PASS/FAIL Decision

**PHASE 1 COMPLETE** if:
- ✅ 7/8 tests pass (can skip -Z if safety margin preferred)
- ✅ Frame directions confirmed (x, y, z correct)
- ✅ Roll/pitch/yaw convention matches reality
- ✅ No collision warnings
- ✅ Motion is smooth and repeatable

**NEXT STEP:** Move to Phase 2 (MoveIt Setup)

**Commit changes:**
```bash
cd ~/ros2_ws/src/tool_bot_ros2
git add CARTESIAN_VALIDATION_TEST_MATRIX.txt
git commit -m "Phase 1 complete: Cartesian validation passed (8/8 poses)"
```

---

**Created:** 2026-04-09  
**Phase:** 1 (Cartesian Validation)  
**Status:** Ready to execute
