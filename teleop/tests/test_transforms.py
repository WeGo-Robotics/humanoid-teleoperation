"""Tests for the OpenXR -> robot transform.

These are checks against physical meaning, not against a transcription of the
original code -- comparing a port to itself would prove nothing. Each test
states where a direction in the real world should end up.

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xr.transforms import (  # noqa: E402
    CONST_HEAD_POSE, T_OPENXR_ROBOT, T_ROBOT_OPENXR, WAIST_OFFSET_X,
    WAIST_OFFSET_Z, change_basis, is_usable, openxr_to_robot,
)


def pose(x=0.0, y=0.0, z=0.0, rot=None):
    m = np.eye(4)
    if rot is not None:
        m[0:3, 0:3] = rot
    m[0:3, 3] = (x, y, z)
    return m


class TestBasisMatrices(unittest.TestCase):
    def test_the_two_matrices_are_inverses(self):
        np.testing.assert_allclose(T_ROBOT_OPENXR @ T_OPENXR_ROBOT, np.eye(4), atol=1e-12)

    def test_rotation_part_is_orthonormal(self):
        r = T_ROBOT_OPENXR[0:3, 0:3]
        np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-12)

    def test_it_is_a_proper_rotation_not_a_reflection(self):
        """det must be +1. A reflection would silently mirror left and right."""
        self.assertAlmostEqual(np.linalg.det(T_ROBOT_OPENXR[0:3, 0:3]), 1.0)


class TestPhysicalDirections(unittest.TestCase):
    """OpenXR is y-up / z-back / x-right; the robot is z-up / y-left / x-front."""

    def _map(self, xyz):
        return (T_ROBOT_OPENXR @ np.array([*xyz, 1.0]))[0:3]

    def test_up_becomes_plus_z(self):
        np.testing.assert_allclose(self._map((0, 1, 0)), (0, 0, 1), atol=1e-12)

    def test_right_becomes_minus_y(self):
        np.testing.assert_allclose(self._map((1, 0, 0)), (0, -1, 0), atol=1e-12)

    def test_backward_becomes_minus_x(self):
        np.testing.assert_allclose(self._map((0, 0, 1)), (-1, 0, 0), atol=1e-12)

    def test_forward_becomes_plus_x(self):
        np.testing.assert_allclose(self._map((0, 0, -1)), (1, 0, 0), atol=1e-12)


class TestChangeOfBasisIsASimilarity(unittest.TestCase):
    def test_translation_transforms_like_a_point(self):
        out = change_basis(pose(1.0, 2.0, 3.0))
        np.testing.assert_allclose(out[0:3, 3], (-3.0, -1.0, 2.0), atol=1e-12)

    def test_identity_stays_identity(self):
        np.testing.assert_allclose(change_basis(np.eye(4)), np.eye(4), atol=1e-12)

    def test_rotation_angle_is_preserved(self):
        """A similarity transform re-expresses a rotation; it must not change
        how far it rotates."""
        a = np.radians(37.0)
        rot = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        out = change_basis(pose(rot=rot))
        cos = (np.trace(out[0:3, 0:3]) - 1.0) / 2.0
        self.assertAlmostEqual(np.degrees(np.arccos(np.clip(cos, -1, 1))), 37.0, places=6)


class TestFullChain(unittest.TestCase):
    def test_wrist_is_expressed_relative_to_the_head(self):
        """The whole doff hazard lives in this subtraction: moving the head
        alone must move the target."""
        head = pose(0.0, 1.6, 0.0)
        wrist = pose(0.0, 1.6, -0.3)          # 30cm in front of the head
        _h, left, _r, _v = openxr_to_robot(head, wrist, wrist, hand_tracking=False)
        # 30cm forward becomes +x, plus the waist offsets
        np.testing.assert_allclose(
            left[0:3, 3], (0.3 + WAIST_OFFSET_X, 0.0, WAIST_OFFSET_Z), atol=1e-9)

    def test_lowering_the_head_raises_the_target(self):
        """Exactly the doff mechanism, in one assertion."""
        wrist = pose(0.0, 1.6, -0.3)
        _h, before, _r, _v = openxr_to_robot(pose(0, 1.6, 0), wrist, wrist, False)
        _h, after, _r, _v = openxr_to_robot(pose(0, 1.1, 0), wrist, wrist, False)
        self.assertAlmostEqual(after[2, 3] - before[2, 3], 0.5, places=9)

    def test_waist_offset_applied_once_per_side(self):
        head = pose(0, 0, 0)
        _h, left, right, _v = openxr_to_robot(head, pose(), pose(), False)
        for target in (left, right):
            self.assertAlmostEqual(target[0, 3], WAIST_OFFSET_X, places=9)
            self.assertAlmostEqual(target[2, 3], WAIST_OFFSET_Z, places=9)

    def test_hand_tracking_applies_a_ninety_degree_roll(self):
        head = pose(0, 1.6, 0)
        wrist = pose(0, 1.6, -0.3)
        _h, ctrl, _r, _v = openxr_to_robot(head, wrist, wrist, hand_tracking=False)
        _h, hand, _r, _v = openxr_to_robot(head, wrist, wrist, hand_tracking=True)
        rel = ctrl[0:3, 0:3].T @ hand[0:3, 0:3]
        angle = np.degrees(np.arccos(np.clip((np.trace(rel) - 1) / 2, -1, 1)))
        self.assertAlmostEqual(angle, 90.0, places=6)

    def test_left_and_right_rolls_are_opposite(self):
        head = pose(0, 1.6, 0)
        wrist = pose(0, 1.6, -0.3)
        _h, left, right, _v = openxr_to_robot(head, wrist, wrist, hand_tracking=True)
        # same input pose both sides -> the two rolls must differ by 180 degrees
        rel = left[0:3, 0:3].T @ right[0:3, 0:3]
        angle = np.degrees(np.arccos(np.clip((np.trace(rel) - 1) / 2, -1, 1)))
        self.assertAlmostEqual(angle, 180.0, places=6)

    def test_translation_is_unaffected_by_the_hand_roll(self):
        head = pose(0, 1.6, 0)
        wrist = pose(0.1, 1.7, -0.3)
        _h, ctrl, _r, _v = openxr_to_robot(head, wrist, wrist, False)
        _h, hand, _r, _v = openxr_to_robot(head, wrist, wrist, True)
        np.testing.assert_allclose(ctrl[0:3, 3], hand[0:3, 3], atol=1e-12)


class TestUnusablePoses(unittest.TestCase):
    def test_singular_pose_is_rejected(self):
        self.assertFalse(is_usable(np.zeros((4, 4))))

    def test_nan_pose_is_rejected(self):
        bad = np.eye(4)
        bad[0, 3] = np.nan
        self.assertFalse(is_usable(bad))

    def test_wrong_shape_is_rejected(self):
        self.assertFalse(is_usable(np.eye(3)))

    def test_valid_pose_is_accepted(self):
        self.assertTrue(is_usable(pose(1, 2, 3)))

    def test_validity_is_reported_not_hidden(self):
        """A substituted fallback must be visible to the caller so the side can
        be marked untracked -- never silently acted on."""
        _h, _l, _r, valid = openxr_to_robot(np.zeros((4, 4)), pose(), np.zeros((4, 4)),
                                            hand_tracking=False)
        head_ok, left_ok, right_ok = valid
        self.assertFalse(head_ok)
        self.assertTrue(left_ok)
        self.assertFalse(right_ok)

    def test_bad_head_falls_back_to_the_constant(self):
        head, _l, _r, _v = openxr_to_robot(np.zeros((4, 4)), pose(), pose(), False)
        np.testing.assert_allclose(head, change_basis(CONST_HEAD_POSE), atol=1e-12)

    def test_fallback_pose_is_not_rolled_in_hand_mode(self):
        """Rotating a fabricated pose would place it somewhere the operator
        never was."""
        _h, left, _r, _v = openxr_to_robot(pose(0, 1.6, 0), np.zeros((4, 4)),
                                           pose(), hand_tracking=True)
        _h2, left2, _r2, _v2 = openxr_to_robot(pose(0, 1.6, 0), np.zeros((4, 4)),
                                               pose(), hand_tracking=False)
        np.testing.assert_allclose(left, left2, atol=1e-12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
