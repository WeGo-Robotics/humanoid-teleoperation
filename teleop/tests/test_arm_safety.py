"""Unit tests for the arm safe-stop primitives.

`logging_mp` is stubbed so this runs without the robot toolchain installed:

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import os
import sys
import threading
import types
import unittest

import numpy as np

# Stub logging_mp before importing the module under test -- it is a robot-side
# dependency and has nothing to do with the behaviour being verified here.
if "logging_mp" not in sys.modules:
    stub = types.ModuleType("logging_mp")
    stub.get_logger = lambda *a, **kw: types.SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
        error=lambda *a, **k: None, debug=lambda *a, **k: None)
    stub.basic_config = lambda *a, **kw: None
    stub.INFO = 20
    sys.modules["logging_mp"] = stub

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "robot_control"))

from arm_safety import ArmSafetyMixin  # noqa: E402


class FakeArm(ArmSafetyMixin):
    """Stands in for a *_ArmController, exposing the contract the mixin needs."""

    def __init__(self, measured=None, has_state=True):
        self.ctrl_lock = threading.Lock()
        self.q_target = np.zeros(14)
        self.arm_velocity_limit = 20.0
        self._speed_gradual_max = False
        self._measured = measured if measured is not None else np.full(14, 0.7)
        self._has_state = has_state
        self.home_calls = 0

    def get_current_dual_arm_q(self):
        if not self._has_state:
            raise AttributeError("'NoneType' object has no attribute 'motor_state'")
        return self._measured.copy()

    def ctrl_dual_arm_go_home(self):
        self.home_calls += 1
        self.q_target = np.zeros(14)


class TestHold(unittest.TestCase):
    def test_hold_freezes_at_the_measured_pose(self):
        arm = FakeArm(measured=np.full(14, 0.7))
        arm.q_target = np.full(14, 1.9)      # a bad target was in flight
        arm.hold()
        np.testing.assert_allclose(arm.q_target, np.full(14, 0.7))

    def test_hold_latches_and_does_not_chase_a_sagging_arm(self):
        arm = FakeArm(measured=np.full(14, 0.7))
        arm.hold()
        # The arm droops under gravity over the next few cycles.
        for sag in (0.68, 0.66, 0.64):
            arm._measured = np.full(14, sag)
            arm.hold()
        np.testing.assert_allclose(arm.q_target, np.full(14, 0.7),
                                   err_msg="hold target must not follow the arm down")

    def test_release_allows_recapture(self):
        arm = FakeArm(measured=np.full(14, 0.7))
        arm.hold()
        arm.release_hold()
        arm._measured = np.full(14, 0.4)
        arm.hold()
        np.testing.assert_allclose(arm.q_target, np.full(14, 0.4))

    def test_hold_without_joint_state_keeps_last_target(self):
        arm = FakeArm(has_state=False)
        arm.q_target = np.full(14, 0.3)
        arm.hold()                            # must not raise
        np.testing.assert_allclose(arm.q_target, np.full(14, 0.3))

    def test_hold_target_is_a_copy(self):
        measured = np.full(14, 0.7)
        arm = FakeArm(measured=measured)
        arm.hold()
        arm._measured[:] = 99.0               # later readings must not alias in
        np.testing.assert_allclose(arm.q_target, np.full(14, 0.7))


class TestVelocityLimit(unittest.TestCase):
    def test_setting_a_limit_cancels_the_ramp(self):
        """Without this the 250Hz control thread recomputes arm_velocity_limit
        from the ramp and overwrites the safe value within 4ms."""
        arm = FakeArm()
        arm._speed_gradual_max = True
        arm.set_velocity_limit(3.0)
        self.assertFalse(arm._speed_gradual_max)
        self.assertEqual(arm.arm_velocity_limit, 3.0)

    def test_restore_returns_to_nominal(self):
        arm = FakeArm()
        arm.set_velocity_limit(3.0)
        arm.restore_velocity_limit()
        self.assertEqual(arm.arm_velocity_limit, ArmSafetyMixin.NOMINAL_ARM_VELOCITY)


class TestSafeStop(unittest.TestCase):
    def test_freezes_before_slowing_and_homing(self):
        order = []
        arm = FakeArm(measured=np.full(14, 0.7))

        real_hold, real_limit, real_home = (
            arm.hold, arm.set_velocity_limit, arm.ctrl_dual_arm_go_home)
        arm.hold = lambda: (order.append("hold"), real_hold())[1]
        arm.set_velocity_limit = lambda v: (order.append(f"limit:{v}"),
                                            real_limit(v))[1]
        arm.ctrl_dual_arm_go_home = lambda: (order.append("home"), real_home())[1]

        arm.safe_stop()
        self.assertEqual(order, ["hold", f"limit:{ArmSafetyMixin.SAFE_ARM_VELOCITY}",
                                 "home"])

    def test_homes_at_the_reduced_velocity(self):
        arm = FakeArm()
        arm.safe_stop()
        self.assertEqual(arm.arm_velocity_limit, ArmSafetyMixin.SAFE_ARM_VELOCITY)
        self.assertEqual(arm.home_calls, 1)

    def test_freeze_only_mode_does_not_home(self):
        arm = FakeArm(measured=np.full(14, 0.7))
        arm.safe_stop(go_home=False)
        self.assertEqual(arm.home_calls, 0)
        np.testing.assert_allclose(arm.q_target, np.full(14, 0.7))

    def test_survives_a_failing_go_home(self):
        arm = FakeArm()

        def boom():
            raise RuntimeError("dds down")
        arm.ctrl_dual_arm_go_home = boom
        arm.safe_stop()                        # must not propagate
        self.assertEqual(arm.arm_velocity_limit, ArmSafetyMixin.SAFE_ARM_VELOCITY)

    def test_leaves_hold_released_for_the_next_arm_cycle(self):
        arm = FakeArm()
        arm.safe_stop()
        self.assertFalse(arm._hold_engaged)


if __name__ == "__main__":
    unittest.main(verbosity=2)
