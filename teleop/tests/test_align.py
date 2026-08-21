"""Tests for the start-alignment gate.

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from safety.align import AlignConfig, AlignGate, pose_error  # noqa: E402


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


ROBOT_L = pose(0.15, 0.25, 0.45)
ROBOT_R = pose(0.15, -0.25, 0.45)


class Rig:
    DT = 1.0 / 30.0

    def __init__(self, config=None):
        self.gate = AlignGate(config or AlignConfig())
        self.now = 500.0
        self.gate.reset(self.now)
        self.left = pose(0.15, 0.25, 0.45)     # start perfectly aligned
        self.right = pose(0.15, -0.25, 0.45)
        self.confirming = False
        self.skip = False

    def step(self, dt=None):
        self.now += dt if dt is not None else self.DT
        return self.gate.update(self.now, ROBOT_L, ROBOT_R,
                                self.left, self.right, self.confirming,
                                self.skip)

    def run(self, seconds):
        report = None
        for _ in range(int(seconds / self.DT)):
            report = self.step()
        return report


class TestPoseError(unittest.TestCase):
    def test_identical_poses(self):
        p, r = pose_error(pose(1, 2, 3), pose(1, 2, 3))
        self.assertAlmostEqual(p, 0.0)
        self.assertAlmostEqual(r, 0.0)

    def test_pure_translation(self):
        p, r = pose_error(pose(0, 0, 0), pose(0.3, 0.4, 0.0))
        self.assertAlmostEqual(p, 0.5)
        self.assertAlmostEqual(r, 0.0)

    def test_pure_rotation(self):
        p, r = pose_error(pose(rot=rot_z(0)), pose(rot=rot_z(30)))
        self.assertAlmostEqual(p, 0.0)
        self.assertAlmostEqual(r, 30.0, places=4)


class TestGuidedPathRequiresBothAgreements(unittest.TestCase):
    """Default path (`skip_requested=False`): unchanged from the original

    design -- position and the confirm gesture must both hold continuously."""

    def test_aligned_but_not_confirming_never_accepts(self):
        rig = Rig()
        rig.confirming = False
        report = rig.run(5.0)
        self.assertFalse(report.accepted)
        self.assertTrue(report.within_tolerance)
        self.assertIn("hold both hands", report.reason)

    def test_confirming_but_not_aligned_never_accepts(self):
        """The operator cannot gesture their way past the host's check --

        not without also holding skip (see TestSkipPath)."""
        rig = Rig()
        rig.left = pose(0.15, 0.25, 1.10)      # 65cm high
        rig.confirming = True
        report = rig.run(5.0)
        self.assertFalse(report.accepted)
        self.assertFalse(report.within_tolerance)
        self.assertIn("or hold both A/X to skip", report.reason)

    def test_both_agreeing_accepts_after_the_hold(self):
        rig = Rig()
        rig.confirming = True
        early = rig.run(1.0)
        self.assertFalse(early.accepted, "accepted before the hold elapsed")
        self.assertGreater(early.progress, 0.4)
        later = rig.run(1.5)
        self.assertTrue(later.accepted)
        self.assertTrue(rig.gate.accepted)

    def test_acceptance_latches(self):
        rig = Rig()
        rig.confirming = True
        rig.run(2.5)
        self.assertTrue(rig.gate.accepted)
        rig.confirming = False
        rig.left = pose(9, 9, 9)
        self.assertTrue(rig.step().accepted, "acceptance must not un-accept")


class TestSkipPath(unittest.TestCase):
    """Holding both A/X buttons (`skip_requested`) alongside the confirm

    gesture waives the position agreement -- but not the gesture itself, and
    not continuity: see module docstring in align.py."""

    def test_skip_without_confirm_gesture_never_accepts(self):
        """Skip only relaxes the position check; it is not a second way in."""
        rig = Rig()
        rig.left = pose(0.15, 0.25, 1.10)
        rig.confirming = False
        rig.skip = True
        report = rig.run(5.0)
        self.assertFalse(report.accepted)

    def test_skip_plus_confirm_accepts_far_out_of_position(self):
        rig = Rig()
        rig.left = pose(0.15, 0.25, 1.10)      # 65cm high -- well out of tolerance
        rig.confirming = True
        rig.skip = True
        report = rig.run(5.0)
        self.assertTrue(report.accepted)
        self.assertFalse(report.within_tolerance, "still reported, just not gating")

    def test_releasing_skip_before_the_hold_elapses_resets_it(self):
        rig = Rig()
        rig.left = pose(0.15, 0.25, 1.10)
        rig.confirming = True
        rig.skip = True
        mid = rig.run(1.0)
        self.assertGreater(mid.progress, 0.4)
        rig.skip = False
        after = rig.step()
        self.assertEqual(after.held_s, 0.0, "letting go of skip must reset the hold")

    def test_skip_is_irrelevant_once_already_in_tolerance(self):
        rig = Rig(AlignConfig(hold_s=0.2))
        rig.confirming = True
        rig.skip = True
        report = rig.run(0.5)
        self.assertTrue(report.accepted)
        self.assertTrue(report.within_tolerance)


class TestHoldResets(unittest.TestCase):
    def test_drifting_out_of_position_resets_the_hold_on_the_guided_path(self):
        rig = Rig()
        rig.confirming = True
        mid = rig.run(1.0)
        self.assertGreater(mid.progress, 0.4)

        rig.left = pose(0.15, 0.25, 1.10)       # operator drifts
        after = rig.step()
        self.assertEqual(after.held_s, 0.0)
        self.assertEqual(after.progress, 0.0)

        rig.left = pose(0.15, 0.25, 0.45)       # back in position
        resumed = rig.run(0.5)
        self.assertLess(resumed.progress, 0.9, "hold restarted, not resumed")
        self.assertFalse(resumed.accepted)

    def test_drifting_out_of_position_does_not_reset_the_hold_on_the_skip_path(self):
        rig = Rig()
        rig.confirming = True
        rig.skip = True
        mid = rig.run(1.0)
        self.assertGreater(mid.progress, 0.4)

        rig.left = pose(0.15, 0.25, 1.10)       # operator drifts, still confirming+skip
        after = rig.step()
        self.assertGreater(after.held_s, 0.9, "position drift must not reset a skip hold")

        resumed = rig.run(1.5)
        self.assertTrue(resumed.accepted)

    def test_releasing_the_gesture_resets_the_hold(self):
        rig = Rig()
        rig.confirming = True
        rig.run(1.5)
        rig.confirming = False
        self.assertEqual(rig.step().held_s, 0.0)

    def test_total_time_is_not_what_matters(self):
        """Ten seconds of intermittent agreement must not add up to acceptance."""
        rig = Rig()
        for _ in range(10):
            rig.confirming = True
            rig.run(0.5)
            rig.confirming = False
            rig.run(0.2)
        self.assertFalse(rig.gate.accepted)


class TestTolerance(unittest.TestCase):
    def test_just_inside_position_tolerance(self):
        rig = Rig(AlignConfig(pos_tol_m=0.10, hold_s=0.2))
        rig.left = pose(0.15, 0.25, 0.45 + 0.09)
        rig.confirming = True
        self.assertTrue(rig.run(0.5).accepted)

    def test_just_outside_position_tolerance(self):
        rig = Rig(AlignConfig(pos_tol_m=0.10, hold_s=0.2))
        rig.left = pose(0.15, 0.25, 0.45 + 0.11)
        rig.confirming = True
        self.assertFalse(rig.run(0.5).accepted)

    def test_orientation_alone_can_block(self):
        rig = Rig(AlignConfig(rot_tol_deg=25.0, hold_s=0.2))
        rig.left = pose(0.15, 0.25, 0.45, rot=rot_z(40))
        rig.confirming = True
        report = rig.run(0.5)
        self.assertFalse(report.accepted)
        self.assertGreater(report.left_rot_err, 25.0)

    def test_both_wrists_must_pass(self):
        rig = Rig(AlignConfig(hold_s=0.2))
        rig.right = pose(0.15, -0.25, 1.10)
        rig.confirming = True
        self.assertFalse(rig.run(0.5).accepted)


class TestFailureModes(unittest.TestCase):
    def test_missing_robot_pose_is_never_treated_as_aligned(self):
        """FK failure must read as not-in-tolerance and block the guided

        path, same as any other out-of-tolerance reading."""
        gate = AlignGate(AlignConfig(hold_s=0.2))
        gate.reset(0.0)
        report = None
        for i in range(20):
            report = gate.update(i * 0.03, None, None, ROBOT_L, ROBOT_R, True)
        self.assertFalse(report.accepted)
        self.assertFalse(report.within_tolerance)

    def test_missing_robot_pose_still_accepts_via_skip(self):
        gate = AlignGate(AlignConfig(hold_s=0.2))
        gate.reset(0.0)
        report = None
        for i in range(20):
            report = gate.update(i * 0.03, None, None, ROBOT_L, ROBOT_R, True,
                                 True)
        self.assertTrue(report.accepted)
        self.assertFalse(report.within_tolerance)

    def test_timeout(self):
        rig = Rig(AlignConfig(timeout_s=1.0))
        rig.confirming = False
        report = rig.run(2.0)
        self.assertTrue(report.timed_out)
        self.assertFalse(report.accepted)

    def test_reset_clears_acceptance(self):
        rig = Rig()
        rig.confirming = True
        rig.run(2.5)
        self.assertTrue(rig.gate.accepted)
        rig.gate.reset(rig.now)
        self.assertFalse(rig.gate.accepted)


class TestGuidanceText(unittest.TestCase):
    """The `reason` string carries direction + closeness so a layman operator

    on the headset (which just renders it verbatim, see TeleopHud.cs) has
    something actionable to do with it."""

    def test_guidance_names_the_far_wrist_and_direction(self):
        rig = Rig()
        rig.left = pose(0.15, 0.25 + 0.20, 0.45)   # 20cm further left than robot
        rig.confirming = True
        report = rig.step()
        self.assertIn("L", report.reason)
        self.assertIn("right", report.reason, "operator is left of target -> move right")

    def test_guidance_only_names_wrists_out_of_tolerance(self):
        rig = Rig()
        rig.left = pose(0.15, 0.25, 0.45)          # in tolerance
        rig.right = pose(0.15, -0.25, 1.10)        # 65cm off
        rig.confirming = True
        report = rig.step()
        self.assertNotIn("L ", report.reason)
        self.assertIn("R ", report.reason)

    def test_guidance_omits_axes_under_a_centimetre(self):
        rig = Rig()
        rig.left = pose(0.15, 0.25, 0.45 + 0.20)   # pure up/down offset, 20cm
        rig.confirming = True
        report = rig.step()
        self.assertNotIn("fwd", report.reason)
        self.assertNotIn("back", report.reason)
        self.assertNotIn("left", report.reason)
        self.assertNotIn("right", report.reason)


class TestTelemetry(unittest.TestCase):
    def test_as_dict_is_json_safe(self):
        import json
        rig = Rig()
        json.dumps(rig.step().as_dict())

    def test_infinite_errors_serialise_as_null(self):
        gate = AlignGate()
        gate.reset(0.0)
        d = gate.update(0.01, None, None, ROBOT_L, ROBOT_R, False).as_dict()
        self.assertIsNone(d["left_pos_err"])

    def test_reported_errors_match_reality(self):
        rig = Rig()
        rig.left = pose(0.15, 0.25, 0.45 + 0.20)
        report = rig.step()
        self.assertAlmostEqual(report.left_pos_err, 0.20, places=6)
        self.assertAlmostEqual(report.right_pos_err, 0.0, places=6)
        self.assertAlmostEqual(report.worst_pos_err, 0.20, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
