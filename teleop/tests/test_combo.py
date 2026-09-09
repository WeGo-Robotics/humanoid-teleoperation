"""Tests for the in-VR start gesture.

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xr.combo import HoldCombo, start_gesture_held  # noqa: E402


class Frame:
    """Just the four fields the gesture reads."""

    def __init__(self, la=False, lb=False, ra=False, rb=False):
        self.left_ctrl_aButton = la
        self.left_ctrl_bButton = lb
        self.right_ctrl_aButton = ra
        self.right_ctrl_bButton = rb


class StartGestureHeldTest(unittest.TestCase):

    def test_x_and_b_with_y_and_a_up_is_the_gesture(self):
        self.assertTrue(start_gesture_held(Frame(la=True, rb=True)))

    def test_one_hand_alone_is_not(self):
        # One button that arms a humanoid is a button someone leans on.
        self.assertFalse(start_gesture_held(Frame(la=True)))
        self.assertFalse(start_gesture_held(Frame(rb=True)))

    def test_nothing_held_is_not(self):
        self.assertFalse(start_gesture_held(Frame()))

    def test_the_estop_pair_is_not_a_start(self):
        """Y+B stops the robot on the device. The host must not read a start
        out of it, whichever message arrives first."""
        self.assertFalse(start_gesture_held(Frame(lb=True, rb=True)))
        self.assertFalse(start_gesture_held(
            Frame(la=True, lb=True, ra=True, rb=True)))

    def test_the_align_skip_pair_is_not_a_start(self):
        """X+A waives the align position check.

        These must stay disjoint: an operator who holds the start gesture into
        alignment and then pulls both triggers would otherwise skip the very
        check the start was for.
        """
        self.assertFalse(start_gesture_held(Frame(la=True, ra=True)))

    def test_holding_the_start_gesture_is_not_the_skip_gesture(self):
        """The converse, stated as the invariant it is."""
        held = Frame(la=True, rb=True)
        self.assertTrue(start_gesture_held(held))
        skip = held.left_ctrl_aButton and held.right_ctrl_aButton
        self.assertFalse(skip)

    def test_adding_either_excluded_button_blocks_it(self):
        self.assertFalse(start_gesture_held(Frame(la=True, rb=True, lb=True)))
        self.assertFalse(start_gesture_held(Frame(la=True, rb=True, ra=True)))


class HoldComboTest(unittest.TestCase):

    def setUp(self):
        self.combo = HoldCombo(hold_s=0.75)

    def test_a_tap_does_not_fire(self):
        self.assertFalse(self.combo.update(0.0, True))
        self.assertFalse(self.combo.update(0.3, True))
        self.assertFalse(self.combo.update(0.4, False))

    def test_a_full_hold_fires_once(self):
        self.assertFalse(self.combo.update(0.0, True))
        self.assertFalse(self.combo.update(0.5, True))
        self.assertTrue(self.combo.update(0.8, True))
        # Still held: the caller polls at 30 Hz and must not get 30 starts.
        self.assertFalse(self.combo.update(0.9, True))
        self.assertFalse(self.combo.update(5.0, True))

    def test_release_re_arms(self):
        self.assertTrue(self.combo.update(1.0, True) or
                        self.combo.update(2.0, True))
        self.combo.update(3.0, False)
        self.assertFalse(self.combo.update(3.1, True))
        self.assertTrue(self.combo.update(4.0, True))

    def test_releasing_mid_hold_restarts_the_clock(self):
        self.combo.update(0.0, True)
        self.combo.update(0.6, True)          # nearly there
        self.combo.update(0.7, False)         # let go
        self.combo.update(0.8, True)          # start again
        self.assertFalse(self.combo.update(1.4, True))   # 0.6s into the new hold
        self.assertTrue(self.combo.update(1.6, True))

    def test_progress_reports_the_hold_and_stops_at_the_fire(self):
        self.assertEqual(self.combo.progress(0.0), 0.0)
        self.combo.update(0.0, True)
        self.assertAlmostEqual(self.combo.progress(0.375), 0.5)
        self.combo.update(0.8, True)          # fires
        self.assertEqual(self.combo.progress(0.9), 0.0)

    def test_reset_abandons_a_hold_in_progress(self):
        self.combo.update(0.0, True)
        self.combo.reset()
        # The hold has to be earned again from zero, so a gesture begun under
        # one meaning cannot complete under another.
        self.assertFalse(self.combo.update(0.8, True))
        self.assertTrue(self.combo.update(1.6, True))

    def test_zero_hold_fires_immediately(self):
        instant = HoldCombo(hold_s=0.0)
        self.assertTrue(instant.update(0.0, True))
        self.assertFalse(instant.update(0.1, True))


if __name__ == "__main__":
    unittest.main()
