#!/usr/bin/env python3
"""
Phase 1: Cartesian Motion Validation
Uses xArm Python SDK directly (bypasses ROS driver state management bug).

Tests 8 poses from PHASE1_QUICKSTART.md to validate frame conventions.
"""

from xarm.wrapper import XArmAPI
import time
import sys

ROBOT_IP = "192.168.1.234"

TESTS = [
    {"name": "HOME",       "pose": [300.0, 0.0, 300.0, 180.0, 0.0, 0.0],
     "expect": "300mm forward, centered, 300mm up, tool down"},
    {"name": "+X OFFSET",  "pose": [350.0, 0.0, 300.0, 180.0, 0.0, 0.0],
     "expect": "50mm further forward than TEST 1"},
    {"name": "-X OFFSET",  "pose": [250.0, 0.0, 300.0, 180.0, 0.0, 0.0],
     "expect": "50mm closer than TEST 1"},
    {"name": "+Z OFFSET",  "pose": [300.0, 0.0, 350.0, 180.0, 0.0, 0.0],
     "expect": "50mm higher than TEST 1"},
    {"name": "-Z OFFSET",  "pose": [300.0, 0.0, 250.0, 180.0, 0.0, 0.0],
     "expect": "50mm lower (closer to table) — CAUTION"},
    {"name": "YAW +15",    "pose": [300.0, 0.0, 300.0, 180.0, 0.0, 15.0],
     "expect": "Tool rotates ~15 deg around z-axis"},
    {"name": "YAW -15",    "pose": [300.0, 0.0, 300.0, 180.0, 0.0, -15.0],
     "expect": "Tool rotates ~15 deg opposite direction"},
    {"name": "X+Z COMBO",  "pose": [320.0, 0.0, 320.0, 180.0, 0.0, 0.0],
     "expect": "Diagonal motion (forward + up) from home"},
]

def main():
    print("=" * 65)
    print("PHASE 1: CARTESIAN MOTION VALIDATION")
    print("=" * 65)
    print()

    # Connect
    arm = XArmAPI(ROBOT_IP)
    arm.connect()
    print(f"Connected. Firmware: v{arm.version}, State: {arm.state}, Error: {arm.error_code}")
    print(f"Current position: {arm.position}")
    print(f"Current angles:   {arm.angles[:5]}")
    print()

    # Enable
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)

    if arm.error_code != 0:
        print(f"ERROR: Robot has error code {arm.error_code} after enable. Aborting.")
        arm.disconnect()
        return

    print(f"Robot enabled. State: {arm.state}, Mode: {arm.mode}, Error: {arm.error_code}")
    print()

    # Go to factory home first
    print("Moving to factory home (all joints zero)...")
    arm.move_gohome(speed=30, wait=True)
    time.sleep(0.5)
    print(f"At factory home. Position: {arm.position}")
    print()

    results = []

    for i, test in enumerate(TESTS):
        test_num = i + 1
        name = test["name"]
        pose = test["pose"]
        expect = test["expect"]

        print("=" * 65)
        print(f"TEST {test_num}: {name}")
        print(f"  Target: x={pose[0]}, y={pose[1]}, z={pose[2]}, "
              f"roll={pose[3]}, pitch={pose[4]}, yaw={pose[5]}")
        print(f"  Expected: {expect}")
        print("-" * 65)

        if test_num == 5:
            print("  ** CAUTION: Moving closer to table. Watch carefully. **")

        for sec in range(5, 0, -1):
            print(f"  Executing TEST {test_num} in {sec}...", flush=True)
            time.sleep(1)
        print(f"  GO!", flush=True)

        ret = arm.set_position(*pose, speed=50, wait=True)
        pos_after = arm.position
        err_after = arm.error_code

        result = {
            "test": test_num,
            "name": name,
            "ret": ret,
            "position": [round(p, 1) for p in pos_after],
            "error": err_after,
        }
        results.append(result)

        status = "PASS" if ret == 0 and err_after == 0 else "FAIL"
        print(f"  Return code: {ret}")
        print(f"  Position after: {result['position']}")
        print(f"  Error code: {err_after}")
        print(f"  RESULT: {status}")
        print()

        if err_after != 0:
            print(f"  ** Error detected! Clearing and re-enabling... **")
            arm.clean_error()
            arm.motion_enable(enable=True)
            arm.set_mode(0)
            arm.set_state(0)
            time.sleep(0.5)

        # Return to home between tests (except consecutive tests at same base pose)
        if test_num < len(TESTS):
            print("  Returning to home pose...")
            arm.set_position(300.0, 0.0, 300.0, 180.0, 0.0, 0.0, speed=50, wait=True)
            time.sleep(0.5)

    # Summary
    print()
    print("=" * 65)
    print("PHASE 1 SUMMARY")
    print("=" * 65)
    passed = sum(1 for r in results if r["ret"] == 0 and r["error"] == 0)
    total = len(results)

    for r in results:
        status = "PASS" if r["ret"] == 0 and r["error"] == 0 else "FAIL"
        print(f"  TEST {r['test']} ({r['name']}): {status}  (ret={r['ret']}, err={r['error']})")

    print()
    print(f"Total: {passed}/{total} passed")
    print()

    if passed >= 7:
        print("PHASE 1: PASSED — Ready for Phase 2")
    else:
        print("PHASE 1: NEEDS DEBUG — Review failures above")

    # Return to factory home
    print()
    print("Returning to factory home...")
    arm.move_gohome(speed=30, wait=True)
    arm.disconnect()
    print("Done. Robot disconnected.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted by user.")
        sys.exit(1)
