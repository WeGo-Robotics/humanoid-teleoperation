"""Unit tests for the XR safety layer.

Deliberately dependency-free (numpy + stdlib only) so they run without DDS,
Vuer, MuJoCo or a robot:

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from safety import (  # noqa: E402
    Action, FaultKind, SafetyConfig, SafetyFSM, SafetyState, XRLiveness,
)


def pose(x=0.0, y=0.0, z=0.0, rot=None):
    m = np.eye(4)
    if rot is not None:
        m[0:3, 0:3] = rot
    m[0:3, 3] = (x, y, z)
    return m


def rot_z(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0.0],
                     [np.sin(a), np.cos(a), 0.0],
                     [0.0, 0.0, 1.0]])


class Rig:
    """Drives an FSM on a fake clock with a synthetic XR stream."""

    DT = 1.0 / 30.0

    def __init__(self, config=None):
        self.fsm = SafetyFSM(config or SafetyConfig())
        self.now = 100.0          # arbitrary monotonic origin
        self.seq = 0
        self.head = pose(0.0, 0.0, 1.6)
        self.left = pose(0.15, 0.20, -0.10)
        self.right = pose(0.15, -0.20, -0.10)
        self.session_up = True
        self.worn = None
        self.left_tracked = True
        self.right_tracked = True
        self._rx = self.now
        self._jitter = 0.0

    def step(self, dt=None, fresh=True, drift=True):
        """Advance one control cycle.

        fresh: whether a new pose event arrived (False simulates a dead link).
        drift: apply sub-millimetre jitter, as a real tracker always does.
        """
        self.now += dt if dt is not None else self.DT
        if fresh:
            self.seq += 1
            self._rx = self.now
            if drift:
                self._jitter += 1e-5
                self.head[0, 3] = self._jitter
        liveness = XRLiveness(
            seq=self.seq, last_rx=self._rx, session_up=self.session_up,
            left_tracked=self.left_tracked, right_tracked=self.right_tracked,
            worn=self.worn,
        )
        return self.fsm.update(self.now, liveness, self.head, self.left, self.right)

    def run(self, cycles, **kw):
        v = None
        for _ in range(cycles):
            v = self.step(**kw)
        return v

    def arm(self):
        assert self.fsm.arm(self.now)
        return self.step()


class TestHealthyStream(unittest.TestCase):
    def test_passes_when_everything_is_fine(self):
        rig = Rig()
        rig.arm()
        v = rig.run(30)
        self.assertIs(v.action, Action.PASS)
        self.assertIs(v.state, SafetyState.FOLLOWING)
        self.assertEqual(v.faults, ())
        self.assertIsNotNone(v.left_wrist)

    def test_idle_before_arming(self):
        rig = Rig()
        v = rig.run(10)
        self.assertIs(v.action, Action.IDLE)
        self.assertIs(v.state, SafetyState.IDLE)


class TestDoffByStaleness(unittest.TestCase):
    """The primary Phase 0 doff detector: pose events simply stop arriving."""

    def test_holds_then_latches(self):
        rig = Rig()
        rig.arm()
        rig.run(10)

        # 100ms of silence is under the 200ms deadline -- still following.
        rig.run(3, fresh=False)
        self.assertIs(rig.fsm.state, SafetyState.FOLLOWING)

        # Cross the staleness deadline -> hold, arms frozen.
        v = rig.run(4, fresh=False)
        self.assertIs(v.action, Action.HOLD)
        self.assertIs(v.state, SafetyState.HOLD)
        self.assertIn(FaultKind.STALE, [f.kind for f in v.faults])

        # Keep it dead -> latched stop.
        v = rig.run(30, fresh=False)
        self.assertIs(rig.fsm.state, SafetyState.SAFE_STOP)
        self.assertTrue(rig.fsm.latched)

    def test_safe_stop_action_is_emitted_exactly_once(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        actions = [rig.step(fresh=False).action for _ in range(60)]
        self.assertEqual(actions.count(Action.SAFE_STOP), 1,
                         "caller must be told to run its stop routine once")
        self.assertIs(actions[-1], Action.IDLE)

    def test_recovers_from_a_brief_dropout(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        v = rig.run(8, fresh=False)
        self.assertIs(v.action, Action.HOLD)
        v = rig.run(5)                      # data comes back
        self.assertIs(v.action, Action.PASS)
        self.assertIs(v.state, SafetyState.FOLLOWING)


class TestFrozenPayload(unittest.TestCase):
    """Sequence numbers advancing while the payload never changes -- a client
    re-sending a cached frame. Staleness alone would not catch this."""

    def test_identical_payload_holds(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        v = rig.run(15, drift=False)        # ~500ms of bit-identical poses
        self.assertIs(v.action, Action.HOLD)
        self.assertIn(FaultKind.FROZEN, [f.kind for f in v.faults])


class TestPresenceSignal(unittest.TestCase):
    """When the device can report presence (Phase 3 native app), honour it."""

    def test_worn_false_latches_immediately(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        rig.worn = False
        v = rig.step()
        self.assertIs(v.action, Action.SAFE_STOP)
        self.assertIn(FaultKind.OPERATOR_ABSENT, [f.kind for f in v.faults])

    def test_worn_none_is_not_a_fault(self):
        rig = Rig()
        rig.worn = None                     # Vuer cannot report presence
        rig.arm()
        v = rig.run(20)
        self.assertIs(v.action, Action.PASS)


class TestLinkLoss(unittest.TestCase):
    def test_session_down_latches(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        rig.session_up = False
        v = rig.step()
        self.assertIs(v.action, Action.SAFE_STOP)
        self.assertIn(FaultKind.LINK_DOWN, [f.kind for f in v.faults])


class TestTrackingLoss(unittest.TestCase):
    def test_brief_hand_dropout_is_tolerated(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        rig.left_tracked = False
        v = rig.run(5)                      # ~165ms, inside the 300ms grace
        self.assertIs(v.action, Action.PASS)

    def test_sustained_hand_dropout_holds(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        rig.left_tracked = False
        v = rig.run(15)
        self.assertIs(v.action, Action.HOLD)
        self.assertIn(FaultKind.TRACKING_LOST, [f.kind for f in v.faults])


class TestHeadJump(unittest.TestCase):
    """The specific failure mode from tv_wrapper.py:296 -- head moves, hands do
    not, and the head-relative wrist target is displaced by the same amount."""

    def _doff_transient(self, rig):
        # Headset yanked down 0.5m in one cycle. Wrist targets are head-relative,
        # so they rise by the same amount.
        rig.head[2, 3] -= 0.5
        rig.left[2, 3] += 0.5
        rig.right[2, 3] += 0.5

    def test_single_transient_holds(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        self._doff_transient(rig)
        v = rig.step()
        self.assertIs(v.action, Action.HOLD)
        kinds = [f.kind for f in v.faults]
        self.assertIn(FaultKind.HEAD_JUMP, kinds)
        self.assertIn(FaultKind.WRIST_JUMP, kinds)

    def test_repeated_transients_latch(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        for _ in range(3):
            self._doff_transient(rig)
            v = rig.step()
        self.assertIs(rig.fsm.state, SafetyState.SAFE_STOP)
        self.assertIs(v.action, Action.SAFE_STOP)

    def test_ordinary_motion_does_not_trip(self):
        rig = Rig()
        rig.arm()
        rig.run(5)
        # A brisk crouch: 0.4m over 0.5s, plus a 90 deg head turn over 0.5s.
        for i in range(15):
            rig.head[2, 3] -= 0.4 / 15.0
            rig.head[0:3, 0:3] = rot_z(90.0 * (i + 1) / 15.0)
            v = rig.step()
        self.assertIs(v.action, Action.PASS, f"false trip: {v.reason}")

    def test_rotation_spike_trips(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        rig.head[0:3, 0:3] = rot_z(45.0)    # 45 deg in one 33ms cycle = 1350 deg/s
        v = rig.step()
        self.assertIn(FaultKind.HEAD_JUMP, [f.kind for f in v.faults])


class TestClamp(unittest.TestCase):
    def test_fast_wrist_motion_is_rate_limited(self):
        rig = Rig()
        rig.arm()
        v = rig.run(5)
        before = v.left_wrist[2, 3]
        # 0.1m in one 33ms cycle = 3 m/s: over the 2 m/s clamp, under the 4 m/s trip.
        rig.left[2, 3] += 0.1
        v = rig.step()
        self.assertIs(v.action, Action.PASS)
        self.assertTrue(v.clamped)
        step = v.left_wrist[2, 3] - before
        self.assertAlmostEqual(step, 2.0 * Rig.DT, places=6)
        self.assertLess(step, 0.1, "clamp must bound the commanded displacement")

    def test_normal_motion_is_untouched(self):
        rig = Rig()
        rig.arm()
        rig.run(5)
        rig.left[2, 3] += 0.01              # 0.3 m/s
        v = rig.step()
        self.assertFalse(v.clamped)
        self.assertAlmostEqual(v.left_wrist[2, 3], rig.left[2, 3], places=9)

    def test_recovery_from_hold_does_not_snap(self):
        """The output baseline must not advance while held, so resuming walks
        the target back at the clamp rate rather than jumping to it."""
        rig = Rig()
        rig.arm()
        v = rig.run(5)
        held_at = v.left_wrist[2, 3]
        rig.run(8, fresh=False)             # hold
        rig.left[2, 3] += 0.5               # operator moved a lot meanwhile
        v = rig.step()
        self.assertIs(v.action, Action.PASS)
        self.assertLessEqual(abs(v.left_wrist[2, 3] - held_at), 2.0 * Rig.DT + 1e-9)


class TestGapRecovery(unittest.TestCase):
    """Velocity across a data gap is undefined, not large. Recovering from a
    dropout must not be reported as an anomalous jump."""

    def test_repeated_dropouts_do_not_latch_via_jump_escalation(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        for _ in range(5):
            rig.run(8, fresh=False)         # ~265ms gap -> HOLD
            rig.left[2, 3] += 0.25          # operator moved during the gap
            rig.right[2, 3] += 0.25
            v = rig.run(4)                  # data returns
            self.assertIs(v.action, Action.PASS, f"tripped on recovery: {v.reason}")
        self.assertIs(rig.fsm.state, SafetyState.FOLLOWING)

    def test_real_jump_still_trips_after_a_gap(self):
        """Abstaining across the gap must not disable the detector afterwards."""
        rig = Rig()
        rig.arm()
        rig.run(10)
        rig.run(8, fresh=False)
        rig.run(4)                          # recovered, baseline re-established
        rig.head[2, 3] -= 0.5               # now a genuine transient
        rig.left[2, 3] += 0.5
        v = rig.step()
        self.assertIs(v.action, Action.HOLD)
        self.assertIn(FaultKind.HEAD_JUMP, [f.kind for f in v.faults])


class TestLatchAndAcknowledge(unittest.TestCase):
    def test_arm_is_refused_while_latched(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        rig.fsm.estop(rig.now)
        self.assertTrue(rig.fsm.latched)
        self.assertFalse(rig.fsm.arm(rig.now))
        self.assertIs(rig.fsm.state, SafetyState.SAFE_STOP)

    def test_acknowledge_then_rearm(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        rig.fsm.estop(rig.now)
        self.assertTrue(rig.fsm.acknowledge(rig.now))
        self.assertIs(rig.fsm.state, SafetyState.IDLE)
        self.assertTrue(rig.fsm.arm(rig.now))
        v = rig.run(10)
        self.assertIs(v.action, Action.PASS)

    def test_link_recovery_alone_never_resumes_motion(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        rig.run(60, fresh=False)            # latch via dead link
        self.assertTrue(rig.fsm.latched)
        v = rig.run(30)                     # link is healthy again
        self.assertIsNot(v.action, Action.PASS)
        self.assertTrue(rig.fsm.latched)

    def test_disarm_does_not_clear_a_latch(self):
        rig = Rig()
        rig.arm()
        rig.fsm.estop(rig.now)
        rig.fsm.disarm(rig.now)
        self.assertIs(rig.fsm.state, SafetyState.SAFE_STOP)


class TestSnapshot(unittest.TestCase):
    def test_snapshot_shape(self):
        rig = Rig()
        rig.arm()
        rig.run(10)
        snap = rig.fsm.snapshot()
        self.assertEqual(snap["state"], "following")
        self.assertTrue(snap["link_up"])
        self.assertFalse(snap["latched"])
        self.assertEqual(snap["faults"], [])
        for key in ("state", "faults", "reason", "stale_ms", "link_up", "worn",
                    "clamped", "latched"):
            self.assertIn(key, snap)


if __name__ == "__main__":
    unittest.main(verbosity=2)
