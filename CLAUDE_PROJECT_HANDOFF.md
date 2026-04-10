# Claude Project Handoff: xArm5 TRAC-IK Integration

**Date:** April 10, 2026  
**Project:** Autonomous tool manipulation with inverse kinematics (TRAC-IK) and AprilTag perception  
**Working Directory:** `/home/julian/ros2_ws/src/tool_bot_ros2` (julian_tracik_integration branch)  
**Status:** Phase 0 complete → Ready to execute Phase 1

---

## Project Context

This is a **robotics project** to add robust inverse kinematics (TRAC-IK) to an xArm5 manipulator, enabling autonomous pick-and-place of tools detected via AprilTags.

**Key Tech Stack:**
- Robot: UFactory xArm5 (5-DOF arm + gripper)
- OS: Ubuntu 24.04 + ROS 2 Jazzy
- IK Solver: TRAC-IK (inside MoveIt2)
- Perception: AprilTag detection
- Driver: Official xarm_ros2 (built in `~/xarm_ws`)
- Application: Custom code in `~/ros2_ws/src/tool_bot_ros2`

**Current State:**
- ✅ Network: Robot at `192.168.1.234` reachable
- ✅ ROS 2 Jazzy: Installed on Ubuntu 24.04
- ✅ Workspaces: Both `xarm_ws` and `ros2_ws` built
- ✅ Planning: 8-phase development roadmap documented
- ❌ Execution: No hardware tests run yet

---

## Your Working Environment

### Workspaces

**Primary (USE THIS):**
```
~/ros2_ws/src/tool_bot_ros2/          ← Your application repo
├── julian_tracik_integration/         ← Active development branch
├── xarm5_ros2_barebones_start/        ← Starter package (unchanged)
├── TRACIK_INTEGRATION_PLAN.md         ← Full 8-phase roadmap
├── PHASE1_QUICKSTART.md               ← Phase 1 execution guide
└── PHASE0_COMPLETE.md                 ← Phase 0 summary
```

**Dependency (read-only):**
```
~/xarm_ws/src/xarm_ros2/               ← Official UFactory driver
├── xarm_api/                          ← Robot driver node
├── xarm_moveit_config/                ← MoveIt configurations
└── xarm_description/                  ← Robot URDF
```

### Source Command

**Always run this first in any new terminal:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash && source ~/ros2_ws/install/setup.bash
```

This sources both workspaces so you have access to:
- xarm_api driver (xarm_ws)
- xarm5_basic_position_cmd (ros2_ws)
- All ROS 2 Jazzy packages

---

## Git Repository

**URL:** `https://github.com/JulianMart1nez/tool_bot_ros2.git`  
**Current Branch:** `julian_tracik_integration`  
**Recovery Branch:** `xarm5-ros2-starter` (do NOT modify)

**Workflow:**
```bash
cd ~/ros2_ws/src/tool_bot_ros2

# Check status
git status

# Make changes, test, then commit
git add <files>
git commit -m "Phase X: <description>"

# Push to GitHub
git push origin julian_tracik_integration
```

---

## Development Roadmap (8 Phases)

### Phase 0: ✅ COMPLETE
- Created `julian_tracik_integration` branch
- Documented comprehensive 8-phase plan
- Created Phase 1 quickstart guide
- Setup task tracking

### Phase 1: 🚀 READY TO START — Cartesian Motion Validation
**Goal:** Prove frame conventions are correct before building anything else.

**What to do:**
1. Launch driver and verify connection to robot (`192.168.1.234`)
2. Enable robot (4-command sequence)
3. Run 8 Cartesian pose tests from home position
4. Log results and validate frame directions (x/y/z)
5. Document findings

**Deliverable:** `CARTESIAN_VALIDATION_RESULTS.txt` + frame conventions document

**Key File:** `PHASE1_QUICKSTART.md` (has all commands and templates)

**Success Criteria:**
- ✅ 7/8 poses execute with ret=0
- ✅ Motion is smooth and predictable
- ✅ Frame directions confirmed (x=forward, y=left, z=up)
- ✅ Results documented and committed

**Estimated Time:** 2-4 hours

---

### Phase 2: MoveIt Setup (After Phase 1 passes)
- Locate and launch xArm5 MoveIt configuration
- Verify RViz visualization
- Test joint goal planning
- Test pose goal planning

**Deliverable:** Working MoveIt launch, test results

---

### Phase 3: TRAC-IK Swap (After Phase 2 passes)
- Install TRAC-IK plugin: `sudo apt install ros-jazzy-trac-ik`
- Swap solver in `kinematics.yaml` (KDL → TRAC-IK)
- Test 10 pose goals with TRAC-IK active
- Verify solver is being called

**Deliverable:** Updated kinematics.yaml, test results

---

### Phases 4-8: Full System Integration
- Phase 4: Reachability characterization
- Phase 5: Grasp pose generator
- Phase 6: Collision scene setup
- Phase 7: Gripper integration
- Phase 8: AprilTag integration

---

## Key Files

| File | Purpose | Status |
|------|---------|--------|
| `TRACIK_INTEGRATION_PLAN.md` | Full 8-phase roadmap | ✅ Complete |
| `PHASE1_QUICKSTART.md` | Phase 1 execution guide | ✅ Ready |
| `PHASE0_COMPLETE.md` | Phase 0 summary | ✅ Complete |
| `xarm5_ros2_barebones_start/` | Starter package | ✅ Available |
| `CARTESIAN_VALIDATION_RESULTS.txt` | Phase 1 test results | ⏳ To create |

---

## Hardware Setup (Already Done)

**Network:**
- Robot IP: `192.168.1.234`
- Laptop IP: `192.168.1.100/24`
- Interface: `enxa0cec8775699`
- Routing: Configured
- Status: ✅ Verified working

**Physical:**
- xArm5 5-DOF arm + gripper
- Power: ON
- Workspace: Clear (no obstacles)
- E-stop: Know where it is

---

## Next Steps (Immediate)

### 1. Verify Setup
```bash
cd ~/ros2_ws/src/tool_bot_ros2
git status  # Should show clean working tree on julian_tracik_integration
```

### 2. Read Phase 1 Quickstart
- Open `PHASE1_QUICKSTART.md`
- Review Pre-Flight Checklist
- Review Terminal Setup section

### 3. Execute Phase 1
- Open 3 terminals
- Terminal 1: Launch driver
- Terminal 2: Enable robot + run tests
- Terminal 3: Monitor (if needed)
- Log all 8 pose results

### 4. Commit Results
```bash
git add CARTESIAN_VALIDATION_RESULTS.txt
git commit -m "Phase 1 complete: Cartesian validation test results"
git push origin julian_tracik_integration
```

---

## Terminal Sessions

### Terminal 1: Driver (Long-running)
```bash
source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash
ros2 launch xarm_api xarm5_driver.launch.py robot_ip:=192.168.1.234
# Keep running, do NOT close
```

### Terminal 2: Enable + Tests
```bash
source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash && source ~/ros2_ws/install/setup.bash

# Enable robot (4 commands)
ros2 service call /xarm/clean_error xarm_msgs/srv/Call "{}"
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"

# Run tests (see PHASE1_QUICKSTART.md for pose values)
ros2 launch xarm5_basic_position_cmd move_to_pose.launch.py [parameters]
```

### Terminal 3: Monitoring (Optional)
```bash
source /opt/ros/jazzy/setup.bash && source ~/xarm_ws/install/setup.bash

# Check services
ros2 service list | grep xarm

# Check joint states
ros2 topic echo /xarm/joint_states --once

# Check robot state
ros2 topic echo /xarm/robot_states --once
```

---

## Important Commands

### Check Network to Robot
```bash
ping -c 4 192.168.1.234  # Should respond
```

### Check ROS Services Available
```bash
ros2 service list | grep xarm  # Should show 10+ services
```

### Check Robot State
```bash
ros2 topic echo /xarm/robot_states --once
# state: 0 = ready, 1 = moving, 2 = standby, 4 = error
```

### Access Robot Web UI
```
http://192.168.1.234  # UFactory Studio (browser)
```

---

## Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| `source: /home/julian/xarm_ws/install/setup.bash: No such file or directory` | Run `cd ~/xarm_ws && colcon build --symlink-install` |
| Driver won't connect | Verify robot at 192.168.1.234 reachable: `ping 192.168.1.234` |
| Services not available | Verify driver is running (Terminal 1) and shows `[TCP STATUS] CONTROL: 1` |
| Service returns ret ≠ 0 | Check UFactory Studio at http://192.168.1.234 for errors |
| Robot won't move | Run enable sequence again (4 commands in Terminal 2) |
| Multiline service call fails | Keep command on ONE LINE (shell paste issue) |

---

## Documentation Reference

**Your Planning Docs:**
- `TRACIK_INTEGRATION_PLAN.md` — Full roadmap (8 phases, milestones, risks)
- `PHASE1_QUICKSTART.md` — Phase 1 detailed execution (commands, templates, checklist)
- `PHASE0_COMPLETE.md` — Executive summary

**External Docs:**
- xArm5 Service Reference: See `README.md` in `xarm5_ros2_barebones_start/`
- MoveIt Docs: https://moveit.picknik.ai/
- TRAC-IK Docs: https://github.com/traclabs/trac_ik

---

## Success Criteria

**Phase 1 Complete When:**
- ✅ 7/8 Cartesian poses execute successfully
- ✅ All return codes = 0
- ✅ Motion is smooth and reproducible
- ✅ Frame directions validated (x/y/z match expectations)
- ✅ Results documented in `CARTESIAN_VALIDATION_RESULTS.txt`
- ✅ Committed to branch with message "Phase 1 complete: ..."

**Phase 1 Failed If:**
- ❌ >1 pose fails to execute
- ❌ Return codes are non-zero
- ❌ Motion is erratic or jerky
- ❌ Frame directions don't match reality
- ❌ Results not documented

---

## Checkpoints

Track progress here:

- [ ] Phase 1a: Setup complete (driver running, robot enabled)
- [ ] Phase 1b: Home pose test passed
- [ ] Phase 1c: All 8 poses tested and logged
- [ ] Milestone A: Results documented, frame conventions validated
- [ ] Results committed to GitHub
- [ ] Ready for Phase 2

---

## Questions Before Starting?

- What is the actual robot workspace boundary? (affects safe pose limits)
- Is the table height known? (needed for -Z test limit)
- Any obstacles in workspace to avoid?
- What's the gripper tool offset (affects tool frame)?

---

## Final Notes

1. **This is iterative:** Phase 1 is a foundation. If it fails, we debug and retry.
2. **Safety first:** Clear workspace, know e-stop location, start with small moves.
3. **Document everything:** Every test result matters for later phases.
4. **No pressure on speed:** Accuracy and understanding matter more than speed.
5. **You have a recovery point:** If anything breaks, `git checkout xarm5-ros2-starter` restores the baseline.

---

**You are ready to begin Phase 1 immediately. Start with `PHASE1_QUICKSTART.md`.** 🚀

**Project Status:** Phase 1 ready to execute | Estimated 3-4 weeks to full system | All planning complete
