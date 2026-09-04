"""Tests for the start-alignment gate.

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import json
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
        self.assertIn("hold both triggers", report.reason)

    def test_confirming_but_not_aligned_never_accepts(self):
        """The operator cannot gesture their way past the host's check --

        not without also holding skip (see TestSkipPath)."""
        rig = Rig()
        rig.left = pose(0.15, 0.25, 1.10)      # 65cm high
        rig.confirming = True
        report = rig.run(5.0)
        self.assertFalse(report.accepted)
        self.assertFalse(report.within_tolerance)
        self.assertIn("or hold X + A together to skip", report.reason)

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
        # hold_s pinned, not inherited. This test needs 1.5 s to land in the
        # MIDDLE of the hold, and it silently stopped doing that when the
        # default dropped to 1.2 s -- at which point 1.5 s accepts the gate and
        # the reset being tested never gets a chance to happen.
        rig = Rig(AlignConfig(hold_s=2.0))
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
        rig = Rig(AlignConfig(scale_free=False, pos_tol_m=0.10, hold_s=0.2))
        rig.left = pose(0.15, 0.25, 0.45 + 0.09)
        rig.confirming = True
        self.assertTrue(rig.run(0.5).accepted)

    def test_just_outside_position_tolerance(self):
        rig = Rig(AlignConfig(scale_free=False, pos_tol_m=0.10, hold_s=0.2))
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
        rig = Rig(AlignConfig(scale_free=False))
        rig.left = pose(0.15, 0.25 + 0.20, 0.45)   # 20cm further left than robot
        rig.confirming = True
        report = rig.step()
        self.assertIn("L", report.reason)
        self.assertIn("right", report.reason, "operator is left of target -> move right")

    def test_guidance_only_names_wrists_out_of_tolerance(self):
        rig = Rig(AlignConfig(scale_free=False))
        rig.left = pose(0.15, 0.25, 0.45)          # in tolerance
        rig.right = pose(0.15, -0.25, 1.10)        # 65cm off
        rig.confirming = True
        report = rig.step()
        self.assertNotIn("L ", report.reason)
        self.assertIn("R ", report.reason)

    def test_guidance_omits_axes_under_a_centimetre(self):
        rig = Rig(AlignConfig(scale_free=False))
        rig.left = pose(0.15, 0.25, 0.45 + 0.20)   # pure up/down offset, 20cm
        rig.confirming = True
        report = rig.step()
        self.assertNotIn("fwd", report.reason)
        self.assertNotIn("back", report.reason)
        self.assertNotIn("left", report.reason)
        self.assertNotIn("right", report.reason)


class TestHeadRelativeTargets(unittest.TestCase):
    """The device cannot anchor a marker from a scalar distance. These are the

    3D positions the headset places its rings at, and they must stay the exact
    inverse of the chain in transforms.py -- if they drift, every marker moves
    while the gate itself still passes."""

    def test_target_is_fk_wrist_minus_the_waist_offset(self):
        cfg = AlignConfig(scale_free=False)
        rig = Rig(cfg)
        report = rig.step()
        # ROBOT_L sits at (0.15, 0.25, 0.45) in the pelvis frame
        self.assertAlmostEqual(report.left_target[0], 0.15 - cfg.waist_offset_x, places=6)
        self.assertAlmostEqual(report.left_target[1], 0.25, places=6)
        self.assertAlmostEqual(report.left_target[2], 0.45 - cfg.waist_offset_z, places=6)

    def test_waist_offset_matches_the_transform_chain(self):
        """A copy of the constant would let the two drift apart silently."""
        from xr.transforms import WAIST_OFFSET_X, WAIST_OFFSET_Z
        cfg = AlignConfig()
        self.assertEqual(cfg.waist_offset_x, WAIST_OFFSET_X)
        self.assertEqual(cfg.waist_offset_z, WAIST_OFFSET_Z)

    def test_no_target_when_fk_is_unavailable(self):
        """Sending zeros would put both rings on the operator's own head."""
        gate = AlignGate(AlignConfig(hold_s=0.2))
        gate.reset(0.0)
        report = gate.update(0.03, None, None, ROBOT_L, ROBOT_R, False)
        self.assertIsNone(report.left_target)
        self.assertIsNone(report.right_target)
        self.assertIsNone(report.as_dict()["left_target"])

    def test_targets_survive_acceptance_and_timeout_branches(self):
        """The markers must not blink out when the gate changes branch."""
        rig = Rig(AlignConfig(hold_s=0.2))
        rig.confirming = True
        accepted = rig.run(0.6)
        self.assertTrue(accepted.accepted)
        self.assertIsNotNone(accepted.left_target)

        rig2 = Rig(AlignConfig(timeout_s=0.5))
        rig2.confirming = False
        timed = rig2.run(1.0)
        self.assertTrue(timed.timed_out)
        self.assertIsNotNone(timed.left_target)

    def test_as_dict_targets_are_json_safe_lists(self):
        import json
        rig = Rig()
        d = rig.step().as_dict()
        json.dumps(d)
        self.assertIsInstance(d["left_target"], list)
        self.assertEqual(len(d["left_target"]), 3)


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
        rig = Rig(AlignConfig(scale_free=False))
        rig.left = pose(0.15, 0.25, 0.45 + 0.20)
        report = rig.step()
        self.assertAlmostEqual(report.left_pos_err, 0.20, places=6)
        self.assertAlmostEqual(report.right_pos_err, 0.0, places=6)
        self.assertAlmostEqual(report.worst_pos_err, 0.20, places=6)


class TestScaleFreeGate(unittest.TestCase):
    """The gate an experience-centre visitor actually has to pass.

    The fixtures here are the real G1's forward kinematics at zero joint
    angles, read off assets/g1/g1_body29_hand14.urdf, because the failure this
    class exists to prevent only shows up at the robot's true proportions: its
    wrist sits 25cm in front of and 9.5cm ABOVE the pelvis origin, while the
    transform chain's waist offset assumes an operator whose head is 45cm above
    their own. An adult holding the robot's initial pose correctly still lands
    10-20cm from the absolute target -- more than the entire position
    tolerance -- so the absolute gate could not be passed by standing right.
    See docs section 15.2 and align.py's docstring.
    """

    # Pelvis frame, q = 0, from the URDF. The wrist frame is identity there.
    G1_L = pose(0.249774, 0.148652, 0.09523)
    G1_R = pose(0.249774, -0.148642, 0.09523)

    @staticmethod
    def operator(scale, cfg=None):
        """The mapped wrist poses of an operator of arbitrary size holding the

        robot's initial pose exactly. `scale` is the ratio of their arm to the
        G1's: 1.0 is a person built like the robot, 1.6 is a typical adult.

        This reproduces the transform chain -- the gate is handed
        (op_wrist - op_head) + WAIST_OFFSET, not the raw wrist -- so what is
        being scaled here is the only thing an operator's body actually
        changes: how far their hand is from their head."""
        c = cfg or AlignConfig()
        off = np.array([c.waist_offset_x, 0.0, c.waist_offset_z])
        out = []
        for fk in (TestScaleFreeGate.G1_L, TestScaleFreeGate.G1_R):
            ref = fk[0:3, 3] - off          # what the robot asks of the head
            out.append(pose(*(ref * scale + off)))
        return out

    def hold(self, cfg, left, right, seconds=1.0, confirming=True):
        gate = AlignGate(cfg)
        now = 100.0
        gate.reset(now)
        report = None
        for _ in range(int(seconds / 0.02)):
            now += 0.02
            report = gate.update(now, self.G1_L, self.G1_R, left, right,
                                 confirming)
        return report

    # -- the regression -------------------------------------------------
    def test_a_typical_adult_holding_the_pose_fails_the_absolute_gate(self):
        """The defect, stated as a test. Nothing about this operator is wrong;

        they are simply not the size of a G1."""
        left, right = self.operator(1.6)
        report = self.hold(AlignConfig(scale_free=False, hold_s=0.2), left, right)
        self.assertFalse(report.accepted)
        self.assertGreater(report.worst_pos_err, AlignConfig().pos_tol_m)

    def test_the_same_operator_passes_the_scale_free_gate(self):
        left, right = self.operator(1.6)
        report = self.hold(AlignConfig(hold_s=0.2), left, right)
        self.assertTrue(report.accepted)
        self.assertLess(report.worst_dir_err, AlignConfig().dir_tol_deg)

    def test_every_size_of_operator_passes(self):
        """A child and a tall adult hold the same shape and both proceed. This

        is the requirement: whatever the scale, matching the initial pose is
        enough."""
        for scale in (0.7, 1.0, 1.3, 1.6, 2.0):
            with self.subTest(scale=scale):
                left, right = self.operator(scale)
                report = self.hold(AlignConfig(hold_s=0.2), left, right)
                self.assertTrue(report.accepted, f"scale {scale} was refused")

    # -- it is still a gate ---------------------------------------------
    def test_arms_hanging_at_the_sides_is_refused(self):
        """Scale-free must not mean shape-free. The G1's initial pose has the

        forearms forward; standing at rest with the arms straight down is a
        different pose and has to be refused, or the gate guards nothing."""
        cfg = AlignConfig(hold_s=0.2)
        off = np.array([cfg.waist_offset_x, 0.0, cfg.waist_offset_z])
        left = pose(*(np.array([0.0, 0.20, -0.65]) + off))
        right = pose(*(np.array([0.0, -0.20, -0.65]) + off))
        report = self.hold(cfg, left, right)
        self.assertFalse(report.accepted)
        self.assertFalse(report.within_tolerance)

    def test_hands_above_the_head_is_refused(self):
        cfg = AlignConfig(hold_s=0.2)
        off = np.array([cfg.waist_offset_x, 0.0, cfg.waist_offset_z])
        left = pose(*(np.array([0.10, 0.25, 0.45]) + off))
        right = pose(*(np.array([0.10, -0.25, 0.45]) + off))
        report = self.hold(cfg, left, right)
        self.assertFalse(report.accepted)

    def test_one_wrist_out_of_shape_blocks(self):
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.6, cfg)
        off = np.array([cfg.waist_offset_x, 0.0, cfg.waist_offset_z])
        right = pose(*(np.array([0.0, -0.20, -0.70]) + off))
        report = self.hold(cfg, left, right)
        self.assertFalse(report.accepted)
        self.assertIn("R ", report.reason)
        self.assertNotIn("L ", report.reason)

    def test_orientation_still_blocks_when_the_shape_is_right(self):
        """Direction-only would let an operator pass with the controllers

        upside down, which is a real way to start following badly."""
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.6, cfg)
        left = pose(*left[0:3, 3], rot=rot_z(40))
        report = self.hold(cfg, left, right)
        self.assertFalse(report.accepted)
        self.assertGreater(report.left_rot_err, cfg.rot_tol_deg)

    def test_a_hand_held_at_the_head_has_no_direction(self):
        """Normalising a near-zero vector would turn tracking noise into a

        lucky pass. It reads as out of tolerance instead."""
        cfg = AlignConfig(hold_s=0.2)
        off = np.array([cfg.waist_offset_x, 0.0, cfg.waist_offset_z])
        left = pose(*(np.array([0.0, 0.01, -0.01]) + off))
        _, right = self.operator(1.6, cfg)
        report = self.hold(cfg, left, right)
        self.assertFalse(report.accepted)
        self.assertFalse(np.isfinite(report.left_dir_err))

    # -- what the headset is told ---------------------------------------
    def test_the_ring_does_not_move_when_the_hand_moves(self):
        """The first version of this placed the marker at the operator's

        *current* reach, which meant reaching toward it pushed it away and it
        could only ever be caught by rotating. A marker has to be a fixed thing
        you move toward."""
        cfg = AlignConfig(hold_s=0.2)
        gate = AlignGate(cfg)
        gate.reset(0.0)
        seen = []
        for scale in (0.6, 1.0, 1.4, 1.9):
            left, right = self.operator(scale, cfg)
            report = gate.update(0.1, self.G1_L, self.G1_R, left, right, False,
                                 head_height_m=1.63)
            seen.append(tuple(report.left_target))
        for other in seen[1:]:
            for a, b in zip(seen[0], other):
                self.assertAlmostEqual(a, b, places=9,
                                       msg="the ring followed the hand")

    def test_the_ring_is_sized_to_the_operator(self):
        """A child and a tall adult are not sent to the same point in space.

        Eye height comes free from the FloorLevel tracking origin, so this
        needs nothing of a first-time visitor -- there is no calibration step
        to run in an experience centre."""
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.3, cfg)
        reach = {}
        for eye in (1.15, 1.63, 1.78):
            gate = AlignGate(cfg)
            gate.reset(0.0)
            report = gate.update(0.1, self.G1_L, self.G1_R, left, right, False,
                                 head_height_m=eye)
            reach[eye] = float(np.linalg.norm(report.left_target))
        self.assertLess(reach[1.15], reach[1.63])
        self.assertLess(reach[1.63], reach[1.78])

    def test_the_ring_keeps_the_shape_whatever_the_size(self):
        """Scaling moves the marker along the required direction and nowhere

        else, so a taller operator gets the same pose further out -- not a
        different pose."""
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.3, cfg)
        off = np.array([cfg.waist_offset_x, 0.0, cfg.waist_offset_z])
        want = self.G1_L[0:3, 3] - off
        for eye in (1.15, 1.78):
            with self.subTest(eye=eye):
                gate = AlignGate(cfg)
                gate.reset(0.0)
                report = gate.update(0.1, self.G1_L, self.G1_R, left, right,
                                     False, head_height_m=eye)
                got = np.array(report.left_target)
                cos = float(np.dot(want, got) /
                            (np.linalg.norm(want) * np.linalg.norm(got)))
                self.assertAlmostEqual(cos, 1.0, places=9)

    def test_a_crouch_does_not_shrink_the_markers(self):
        """Held as a maximum, not sampled once and not averaged: the operator

        is standing when alignment starts, and a nod or a glance at the floor
        must not pull the rings in underneath them mid-hold."""
        cfg = AlignConfig(hold_s=0.2)
        gate = AlignGate(cfg)
        gate.reset(0.0)
        left, right = self.operator(1.3, cfg)
        tall = gate.update(0.1, self.G1_L, self.G1_R, left, right, False,
                           head_height_m=1.70)
        stooped = gate.update(0.2, self.G1_L, self.G1_R, left, right, False,
                              head_height_m=1.20)
        self.assertEqual(tall.left_target, stooped.left_target)

    def test_a_nonsense_head_height_cannot_move_the_markers(self):
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.3, cfg)
        for eye in (0.0, -1.0, 9.0, float("nan"), float("inf")):
            with self.subTest(eye=eye):
                gate = AlignGate(cfg)
                gate.reset(0.0)
                report = gate.update(0.1, self.G1_L, self.G1_R, left, right,
                                     False, head_height_m=eye)
                self.assertEqual(report.operator_scale, 1.0)

    def test_the_operator_scale_stays_inside_its_clamp(self):
        """Belt and braces. The plausibility window on head height already

        bounds most of this -- 0.9 to 2.2 m against a 1.25 m nominal gives 0.72
        to 1.76 -- so the clamp only bites at the very bottom. It stays because
        the two limits are set independently and a change to either should not
        be able to put a ring somewhere absurd."""
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.3, cfg)
        for eye in (0.90, 0.95, 1.10, 1.25, 1.63, 1.90, 2.20):
            with self.subTest(eye=eye):
                gate = AlignGate(cfg)
                gate.reset(0.0)
                k = gate.update(0.1, self.G1_L, self.G1_R, left, right, False,
                                head_height_m=eye).operator_scale
                self.assertGreaterEqual(k, cfg.operator_scale_min)
                self.assertLessEqual(k, cfg.operator_scale_max)
        gate = AlignGate(cfg)
        gate.reset(0.0)
        self.assertAlmostEqual(
            gate.update(0.1, self.G1_L, self.G1_R, left, right, False,
                        head_height_m=0.90).operator_scale,
            cfg.operator_scale_min, places=6)

    def test_head_height_cannot_change_the_verdict(self):
        """The whole reason it is safe to guess at the operator's build: the

        estimate sizes the markers and touches nothing that gates. An operator
        holding the pose passes whatever height the headset reports, and one
        who is not holding it fails the same way."""
        cfg = AlignConfig(hold_s=0.2)
        good_l, good_r = self.operator(1.6, cfg)
        off = np.array([cfg.waist_offset_x, 0.0, cfg.waist_offset_z])
        bad_l = pose(*(np.array([0.0, 0.20, -0.65]) + off))
        bad_r = pose(*(np.array([0.0, -0.20, -0.65]) + off))
        for eye in (None, 0.95, 1.15, 1.63, 2.15):
            with self.subTest(eye=eye):
                gate = AlignGate(cfg)
                gate.reset(0.0)
                self.assertTrue(gate.update(0.1, self.G1_L, self.G1_R, good_l,
                                            good_r, False,
                                            head_height_m=eye).within_tolerance)
                gate = AlignGate(cfg)
                gate.reset(0.0)
                self.assertFalse(gate.update(0.1, self.G1_L, self.G1_R, bad_l,
                                             bad_r, False,
                                             head_height_m=eye).within_tolerance)

    def test_the_ring_radius_is_the_tolerance_at_that_distance(self):
        """The device used to hold this as a 0.10f constant "matching the

        gate's position tolerance", which the gate no longer has. An angle has
        no size until it is put at a distance."""
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.3, cfg)
        gate = AlignGate(cfg)
        gate.reset(0.0)
        report = gate.update(0.1, self.G1_L, self.G1_R, left, right, False,
                             head_height_m=1.63)
        d = float(np.linalg.norm(report.left_target))
        self.assertAlmostEqual(report.left_radius,
                               d * np.tan(np.radians(cfg.dir_tol_deg)),
                               places=9)
        self.assertGreater(report.left_radius, 0.0)

    def test_the_ring_keeps_the_direction_the_robot_asked_for(self):
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.6, cfg)
        report = self.hold(cfg, left, right, seconds=0.04, confirming=False)
        off = np.array([cfg.waist_offset_x, 0.0, cfg.waist_offset_z])
        want = self.G1_L[0:3, 3] - off
        got = np.array(report.left_target)
        cos = float(np.dot(want, got) /
                    (np.linalg.norm(want) * np.linalg.norm(got)))
        self.assertAlmostEqual(cos, 1.0, places=6)

    def test_each_wrist_gets_its_own_verdict(self):
        """The headset renders these directly. If the gate did not publish

        them the console would have to re-derive "is this hand in position"
        from the error numbers against a threshold of its own, and a console
        that decides for itself what the host means is how builds 12 and 13
        both went wrong."""
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.6, cfg)
        off = np.array([cfg.waist_offset_x, 0.0, cfg.waist_offset_z])
        right = pose(*(np.array([0.0, -0.20, -0.70]) + off))
        report = self.hold(cfg, left, right, seconds=0.04, confirming=False)
        self.assertTrue(report.left_ok)
        self.assertFalse(report.right_ok)

    def test_both_verdicts_true_is_exactly_within_tolerance(self):
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.6, cfg)
        report = self.hold(cfg, left, right, seconds=0.04, confirming=False)
        self.assertTrue(report.left_ok)
        self.assertTrue(report.right_ok)
        self.assertTrue(report.within_tolerance)

    def test_a_wrist_that_is_turned_wrong_is_not_ok(self):
        """The verdict covers that wrist's orientation too, so the console

        cannot tick a hand the gate is refusing on rotation."""
        cfg = AlignConfig(hold_s=0.2)
        left, right = self.operator(1.6, cfg)
        left = pose(*left[0:3, 3], rot=rot_z(40))
        report = self.hold(cfg, left, right, seconds=0.04, confirming=False)
        self.assertFalse(report.left_ok)
        self.assertTrue(report.right_ok)

    def test_no_verdict_survives_a_failed_fk(self):
        gate = AlignGate(AlignConfig(hold_s=0.2))
        gate.reset(0.0)
        left, right = self.operator(1.6)
        report = gate.update(0.1, None, None, left, right, True)
        self.assertFalse(report.left_ok)
        self.assertFalse(report.right_ok)


    def test_direction_errors_reach_the_dashboard(self):
        left, right = self.operator(1.6)
        report = self.hold(AlignConfig(hold_s=0.2), left, right,
                           seconds=0.04, confirming=False)
        d = report.as_dict()
        self.assertIn("left_dir_err", d)
        self.assertIn("right_dir_err", d)
        json.dumps(d)


if __name__ == "__main__":
    unittest.main(verbosity=2)
