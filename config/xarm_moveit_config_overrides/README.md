# xarm_moveit_config overrides

This directory stores configuration files that override files in the upstream
`xarm_ros2` workspace. The `xarm_ros2` repo lives at `~/xarm_ws/src/xarm_ros2/`
and is NOT part of this project's git history — it's the official UFactory
driver. To preserve our local config changes (e.g., swapping KDL for TRAC-IK),
we keep copies here.

## Files

### `xarm5/kinematics.yaml`

Replaces the default KDL IK solver with **TRAC-IK** for the `xarm5` planning
group.

**Deploy:**

```bash
cp ~/ros2_ws/src/tool_bot_ros2/config/xarm_moveit_config_overrides/xarm5/kinematics.yaml \
   ~/xarm_ws/src/xarm_ros2/xarm_moveit_config/config/xarm5/kinematics.yaml
```

**Backup before overwriting** (recommended on a fresh checkout):

```bash
cp ~/xarm_ws/src/xarm_ros2/xarm_moveit_config/config/xarm5/kinematics.yaml \
   ~/xarm_ws/src/xarm_ros2/xarm_moveit_config/config/xarm5/kinematics.yaml.kdl_backup
```

**Rebuild is NOT required** if `xarm_ws` was built with `--symlink-install`.
The installed `kinematics.yaml` in `~/xarm_ws/install/...` is a symlink to the
source copy.

## Why TRAC-IK for xArm5

The xArm5 is a 5-DOF arm, which means not all 6-DOF Cartesian poses are
achievable — the IK solver must handle redundancy and constraints gracefully.
KDL (the MoveIt default) uses a pseudoinverse Jacobian approach that struggles
near joint limits and singularities. TRAC-IK runs two strategies in parallel
(Newton-Raphson + SQP optimization) and returns whichever converges first,
giving much higher reliability for constrained arms.

`solve_type: Distance` means TRAC-IK returns the IK solution closest to the
current joint configuration, producing smoother motion between successive
plans.
