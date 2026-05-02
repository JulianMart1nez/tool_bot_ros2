#!/usr/bin/env python3
"""
auto_go_to.py — fully autonomous voice → grab → dropoff → home.

Subclasses GoTo and replaces the keyboard fallback with a state-machine
timer. Reuses every helper in the parent (IK, MoveGroup, gripper action,
dropoff sequence, planning-scene toggles) — none of that logic is
duplicated here.

Flow per "give me <tool>" voice command:
  1) WAIT_DETECT_ZONE: parent's _detect_zone_done_cb subscribes to
     /fine_loc/tag_<id>; we just watch for self._tag_sub != None.
  2) WAIT_TAG_LOCK: first smoothed pose triggers AUTO_DESCEND in the
     parent's _tag_pose_cb (sets self._initial_tcp_z + self._target_z).
     We wait for that.
  3) SETTLE_INITIAL: 2 s pause for the AUTO_DESCEND move to land.
  4) DESCEND: every DESCEND_PERIOD_S, if not move_in_flight and
     _grab_armed not set, step _adjust_z(-DESCEND_STEP_M). Abort if
     pixel_size signal disappears for >PIXEL_TIMEOUT_S, or if z hits
     the safety floor with no grab_armed. The parent's _grab_tick is
     the authority on _grab_armed (sweet_px ± tol for HOLD_TICKS).
  5) GRAB: call self._grab_close() (non-blocking). Wait on
     _gripper_idle so the close completes before we transit.
  6) DROPOFF: spawn self._dropoff_sequence() in a thread (matches the
     keyboard 'd' behavior). Wait on _dropoff_done.
  7) HOME: call self._go_home() inline (it owns _move_in_flight while
     planning). Done.

Aborts: parent's tool_request and home_request callbacks set
_autonomous_abort. Every transition checks it and resets to IDLE.
"""

import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .go_to import (
    GoTo,
    SWEET_PX,
    SWEET_TOL_PX,
    GRAB_HOLD_TICKS,
    TOOL_NAMES,
)


# Tunables — see top-of-file design notes.
DESCEND_PERIOD_S = 1.0    # wall-clock cadence between -10 mm steps
DESCEND_STEP_M = 0.010    # match STEP_SIZES_M[0] (10 mm)
SETTLE_INITIAL_S = 2.0    # let AUTO_DESCEND land before stepping further
DROPOFF_TIMEOUT_S = 60.0  # max wall time for full dropoff sequence
HOME_TIMEOUT_S = 30.0
WAIT_DETECT_ZONE_S = 60.0  # detect_zone bird's-eye+scan can take ~30 s
WAIT_TAG_LOCK_S = 15.0
GRAB_SETTLE_S = 5.0       # gripper close completion timeout
TICK_HZ = 5.0
# At touchdown (target_z within tolerance of safety floor) without
# pixel-arm, we force-grab IF the tag's pixel_size ever climbed to
# this fraction of sweet during the run. The reasoning: if we got
# close to alignment but the tag fell out of FOV near the bottom
# (the 70 mm camera offset bites at close range), the geometry
# still works — grab anyway. If the max-seen never approached
# sweet, alignment was bad and we should home instead of pinching
# air or worse.
FORCE_GRAB_PIXEL_FRACTION = 0.55


# State labels (plain strings — readable in logs).
S_IDLE = 'IDLE'
S_WAIT_DETECT = 'WAIT_DETECT_ZONE'
S_WAIT_LOCK = 'WAIT_TAG_LOCK'
S_SETTLE = 'SETTLE_INITIAL'
S_DESCEND = 'DESCEND'
S_GRAB = 'GRAB'
S_DROPOFF = 'DROPOFF'
S_HOME = 'HOME'
S_ABORT = 'ABORT'


class AutoGoTo(GoTo):
    def __init__(self):
        super().__init__()
        self._auto_state = S_IDLE
        self._auto_state_entered = time.monotonic()
        # Latch: starts the state machine on first tool_request after
        # __init__. We don't kick off in WAIT_DETECT until the parent's
        # _tool_request_cb has set _target_tag_id, otherwise we'd race
        # with detect_zone publishing /detect_zone/complete.
        self._last_seen_target_tid = None
        # Last pixel_size sample stamp the state machine observed —
        # used for the descent abort. The parent already records
        # _latest_pixel_stamp; we also track the wall time of the last
        # _adjust_z call so we don't fire a step at faster than
        # DESCEND_PERIOD_S even if the tick is faster.
        self._last_descent_step_t = 0.0
        self._descend_start_pixel = None  # for "is the tag growing?" log
        # Max pixel_size observed during the current descent. Used at
        # floor-touchdown to decide between force-grab (we got close)
        # and home (we never aligned). Reset on every fresh DESCEND.
        self._max_pixel_seen = 0.0
        # Generation counter: every fresh run gets a new id. Worker
        # threads carry the gen they started with and refuse to
        # mutate state if a new request has bumped the counter (i.e.
        # the previous run was aborted and a new one is now active).
        # Without this, a slow dropoff worker could call
        # _reset_lock_state on the NEW run's target_tid as it tears
        # down — stomping the new request.
        self._auto_gen = 0
        self.create_timer(
            1.0 / TICK_HZ, self._auto_tick, callback_group=self.cb_group)
        self.get_logger().warn(
            'AUTO_GO_TO node up. Speak a tool request to start the '
            'fully-autonomous sequence. Voice "go home" or any new tool '
            'request aborts the current run.')

    # ------------------------------------------------------------------
    # State-machine timer
    # ------------------------------------------------------------------

    def _auto_tick(self):
        # Honor abort first — both /voice_command/tool_request and
        # /voice_command/home_request set this. The reset-handler
        # callbacks (parent) clear _target_tag_id / state independent of
        # us, so we just collapse to IDLE here.
        if self._autonomous_abort.is_set():
            self._autonomous_abort.clear()
            if self._auto_state not in (S_IDLE, S_HOME):
                self.get_logger().warn(
                    f'AUTO[{self._auto_state}] aborted by external request.')
            # Bump generation so any in-flight worker thread (grab,
            # dropoff, home) sees its gen as stale and refuses to
            # mutate state when it eventually completes.
            self._auto_gen += 1
            # Reset the edge-trigger memory so that even a re-request
            # for the SAME tool re-enters WAIT_DETECT (otherwise the
            # tid-comparison would suppress it).
            self._last_seen_target_tid = None
            self._enter(S_IDLE)
            return

        st = self._auto_state
        if st == S_IDLE:
            self._tick_idle()
        elif st == S_WAIT_DETECT:
            self._tick_wait_detect()
        elif st == S_WAIT_LOCK:
            self._tick_wait_lock()
        elif st == S_SETTLE:
            self._tick_settle()
        elif st == S_DESCEND:
            self._tick_descend()
        # GRAB / DROPOFF / HOME run as one-shot worker threads; their
        # exit transitions back via _enter(). The tick does nothing
        # while those are in progress.

    def _enter(self, new_state):
        old = self._auto_state
        if old != new_state:
            self.get_logger().warn(
                f'AUTO state: {old} → {new_state}')
            self._auto_state = new_state
            self._auto_state_entered = time.monotonic()
            if new_state == S_DESCEND:
                self._last_descent_step_t = 0.0
                with self._lock:
                    self._descend_start_pixel = self._latest_pixel_px
                self._max_pixel_seen = 0.0

    def _elapsed(self):
        return time.monotonic() - self._auto_state_entered

    # ------------------------------------------------------------------
    # Per-state handlers
    # ------------------------------------------------------------------

    def _tick_idle(self):
        # Parent's _tool_request_cb sets _target_tag_id. Edge-trigger:
        # only kick the state machine when the id actually changes from
        # None → something, so we don't relaunch on the same request.
        with self._lock:
            tid = self._target_tag_id
        if tid is not None and tid != self._last_seen_target_tid:
            self._last_seen_target_tid = tid
            self._auto_gen += 1
            tool = TOOL_NAMES.get(tid, f'fid={tid}')
            self.get_logger().warn(
                f'AUTO[gen={self._auto_gen}]: target acquired = {tool} '
                f'(fid={tid}). Waiting for detect_zone to complete.')
            self._enter(S_WAIT_DETECT)
        elif tid is None:
            self._last_seen_target_tid = None

    def _tick_wait_detect(self):
        # _detect_zone_done_cb (parent) creates the per-tag subscriber
        # only when /detect_zone/complete payload matches.
        with self._lock:
            sub_present = self._tag_sub is not None
            tid = self._target_tag_id
        if tid is None:
            self._enter(S_IDLE)
            return
        if sub_present:
            self.get_logger().warn(
                'AUTO: /detect_zone/complete received, fine_loc subscriber '
                'live. Waiting for first tag pose lock.')
            self._enter(S_WAIT_LOCK)
            return
        if self._elapsed() > WAIT_DETECT_ZONE_S:
            self.get_logger().error(
                f'AUTO: detect_zone did not complete within '
                f'{WAIT_DETECT_ZONE_S:.0f}s — aborting.')
            self._enter(S_HOME)
            gen = self._auto_gen
            threading.Thread(target=self._auto_home_then_idle,
                             args=(gen,), daemon=True).start()

    def _tick_wait_lock(self):
        # AUTO_DESCEND in parent's _tag_pose_cb sets _initial_tcp_z and
        # _target_z. That's our signal that smoothing has converged
        # (>=3 samples), the tag XY is sane, and the parent has already
        # commanded the first descent move.
        with self._lock:
            locked = (self._initial_tcp_z is not None
                      and self._target_z is not None)
            tid = self._target_tag_id
        if tid is None:
            self._enter(S_IDLE)
            return
        if locked:
            self.get_logger().warn(
                'AUTO: tag locked, AUTO_DESCEND fired. Settling for '
                f'{SETTLE_INITIAL_S:.0f}s before fine descent.')
            self._enter(S_SETTLE)
            return
        if self._elapsed() > WAIT_TAG_LOCK_S:
            self.get_logger().error(
                f'AUTO: no tag pose lock within {WAIT_TAG_LOCK_S:.0f}s — '
                'aborting. Check fine_localization output and '
                f'_latest_tag_stamp.')
            self._enter(S_HOME)
            gen = self._auto_gen
            threading.Thread(target=self._auto_home_then_idle,
                             args=(gen,), daemon=True).start()

    def _tick_settle(self):
        if self._elapsed() < SETTLE_INITIAL_S:
            return
        # Wait for the AUTO_DESCEND move to actually finish before we
        # start stacking 10 mm steps on top of it.
        with self._lock:
            in_flight = self._move_in_flight
        if in_flight:
            return
        self._enter(S_DESCEND)

    def _tick_descend(self):
        # Authoritative grab signal: parent's _grab_tick sets _grab_armed
        # when pixel_size in sweet_px ± SWEET_TOL_PX for GRAB_HOLD_TICKS.
        with self._lock:
            armed = self._grab_armed
            in_flight = self._move_in_flight
            tid = self._target_tag_id
            target_z = self._target_z
            initial_z = self._initial_tcp_z
            px = self._latest_pixel_px
            px_stamp = self._latest_pixel_stamp
        if tid is None or initial_z is None:
            # Parent reset us. Fall through to IDLE.
            self._enter(S_IDLE)
            return
        # Track best pixel_size we've seen this descent. When the tag
        # falls out of FOV near touchdown (the 70 mm camera offset
        # bites at close range), this lets us decide at floor-time
        # whether we got close enough to grab anyway.
        if px is not None and px_stamp is not None:
            silence = time.monotonic() - px_stamp
            if silence < 1.0 and px > self._max_pixel_seen:
                self._max_pixel_seen = px

        if armed:
            self.get_logger().warn(
                f'AUTO: grab armed at pixel_size={px:.1f}px '
                f'(sweet={SWEET_PX.get(tid, 0):.1f} ± {self._tol_px:.0f}). '
                'Closing gripper.')
            self._enter(S_GRAB)
            gen = self._auto_gen
            threading.Thread(
                target=self._auto_grab_then_dropoff,
                args=(gen,), daemon=True).start()
            return

        # Floor-touchdown without arming. Two paths:
        #  - We got close to sweet at some point during descent:
        #    force-grab. The geometry-based fall-out (tag exits FOV
        #    when camera is offset 70 mm and very close) is the
        #    expected failure mode of the pixel signal, NOT a sign
        #    that alignment was bad.
        #  - We never approached sweet: alignment was bad. Home.
        floor = self._current_safety_floor()
        if target_z is not None and target_z <= floor + 0.005:
            sweet = SWEET_PX.get(tid, 0.0)
            threshold = sweet * FORCE_GRAB_PIXEL_FRACTION
            # Wait until any in-flight floor move has actually landed
            # before we decide — gripper closing mid-descent is bad.
            if in_flight:
                return
            if self._max_pixel_seen >= threshold:
                self.get_logger().warn(
                    f'AUTO: floor reached without pixel-arm, but '
                    f'max_seen pixel_size={self._max_pixel_seen:.1f}px '
                    f'≥ {threshold:.1f}px ({FORCE_GRAB_PIXEL_FRACTION*100:.0f}%'
                    f' of sweet={sweet:.1f}). Force-grabbing.')
                self._enter(S_GRAB)
                gen = self._auto_gen
                threading.Thread(
                    target=self._auto_grab_then_dropoff,
                    args=(gen,), daemon=True).start()
                return
            self.get_logger().error(
                f'AUTO: target_z={target_z*1000:.0f}mm at safety floor '
                f'({floor*1000:.0f}mm). max_seen pixel_size='
                f'{self._max_pixel_seen:.1f}px never reached '
                f'{threshold:.1f}px ({FORCE_GRAB_PIXEL_FRACTION*100:.0f}%'
                f' of sweet={sweet:.1f}). Aborting to home.')
            self._enter(S_HOME)
            gen = self._auto_gen
            threading.Thread(target=self._auto_home_then_idle,
                             args=(gen,), daemon=True).start()
            return

        # Step down only if previous step has settled AND enough wall
        # time has passed since the last step.
        now = time.monotonic()
        if (in_flight
                or (now - self._last_descent_step_t) < DESCEND_PERIOD_S):
            return
        self._last_descent_step_t = now
        self.get_logger().info(
            f'AUTO: stepping down {DESCEND_STEP_M*1000:.0f}mm '
            f'(target_z={target_z*1000:+.0f}mm, '
            f'pixel_size={px}px → sweet={SWEET_PX.get(tid, 0):.1f})')
        # _adjust_z commits a fresh IK + joint-move to the live tag XY
        # at the new z. Same path the keyboard ↓ takes.
        self._adjust_z(-DESCEND_STEP_M)

    # ------------------------------------------------------------------
    # Worker threads (long blocking ops kept off the timer thread)
    # ------------------------------------------------------------------

    def _auto_grab_then_dropoff(self, gen):
        def stale():
            if gen != self._auto_gen:
                self.get_logger().warn(
                    f'AUTO worker gen={gen} stale '
                    f'(current={self._auto_gen}) — exiting without '
                    'state mutation.')
                return True
            return False

        try:
            self._grab_close()
            if not self._gripper_idle.wait(timeout=GRAB_SETTLE_S):
                self.get_logger().error(
                    'AUTO: gripper did not finish closing within '
                    f'{GRAB_SETTLE_S:.0f}s — aborting to home.')
                if stale():
                    return
                self._enter(S_HOME)
                self._auto_home_then_idle(gen)
                return
            if stale():
                return

            self.get_logger().warn('AUTO: gripper closed. Dropoff starting.')
            self._enter(S_DROPOFF)
            threading.Thread(
                target=self._dropoff_sequence, daemon=True).start()
            if not self._dropoff_done.wait(timeout=DROPOFF_TIMEOUT_S):
                self.get_logger().error(
                    f'AUTO: dropoff did not complete in '
                    f'{DROPOFF_TIMEOUT_S:.0f}s. Going home anyway.')
            if stale():
                return
            self.get_logger().warn(
                'AUTO: dropoff complete. Transiting home.')
            self._enter(S_HOME)
            self._auto_home_then_idle(gen)
        except Exception as e:  # pragma: no cover — safety net
            self.get_logger().error(
                f'AUTO: exception in grab/dropoff worker: {e!r}. '
                'Falling back to home.')
            if stale():
                return
            self._enter(S_HOME)
            self._auto_home_then_idle(gen)

    def _auto_home_then_idle(self, gen=None):
        try:
            self._go_home()
        except Exception as e:  # pragma: no cover
            self.get_logger().error(
                f'AUTO: exception during home: {e!r}.')
        finally:
            # If a newer run has bumped the gen counter, do NOT clear
            # the lock state — the new run owns _target_tag_id now and
            # we must not stomp it.
            if gen is not None and gen != self._auto_gen:
                self.get_logger().warn(
                    f'AUTO home-worker gen={gen} stale '
                    f'(current={self._auto_gen}) — preserving new '
                    'run state.')
                return
            self._reset_lock_state(target=None)
            self._last_seen_target_tid = None
            self._enter(S_IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = AutoGoTo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
