"""Safe-stop primitives shared by every arm controller.

Kept separate from `robot_arm.py` so it can be exercised without a DDS stack or
a robot -- these are the behaviours that run when something has already gone
wrong, which makes them exactly the ones worth testing offline.
"""
import numpy as np
import logging_mp

logger_mp = logging_mp.get_logger(__name__)


class ArmSafetyMixin:
    """Mixed into every `*_ArmController`.

    Written against the contract all five controllers already satisfy
    (`ctrl_lock`, `q_target`, `arm_velocity_limit`, `_speed_gradual_max`,
    `get_current_dual_arm_q()`, `ctrl_dual_arm_go_home()`), so it needs no
    per-class duplication.

    Motivation: before this, the only ways to stop were `ctrl_dual_arm_go_home()`
    -- which snaps `q_target` to zeros and drives there at the full 20-30 rad/s
    limit -- and killing the process. Neither is a safe response to the operator
    disappearing mid-motion.

    Note: `clip_arm_q_target` is bypassed in `simulation_mode`, so the velocity
    limit has no effect there. `hold()` works in both.
    """

    NOMINAL_ARM_VELOCITY = 20.0   # matches every controller's __init__ default
    SAFE_ARM_VELOCITY = 3.0       # while stopping or homing after a fault

    def hold(self):
        """Freeze the arms where they physically are, right now.

        Latching is deliberate. Re-sampling the measured position every cycle
        would let the target follow the arm as it sags under gravity, walking it
        downward for as long as the hold lasts. The first call captures the
        pose; later calls are no-ops until `release_hold()`.
        """
        if getattr(self, "_hold_engaged", False):
            return
        try:
            current = self.get_current_dual_arm_q()
        except Exception as e:
            # No lowstate yet. Leaving q_target untouched still freezes the arm
            # at its last commanded target, which is the safe fallback.
            logger_mp.warning(f"[ArmSafety] hold(): no joint state ({e}); "
                              f"holding last commanded target")
            self._hold_engaged = True
            return
        with self.ctrl_lock:
            self.q_target = np.asarray(current).copy()
        self._hold_engaged = True
        logger_mp.info("[ArmSafety] hold engaged")

    def release_hold(self):
        """Allow normal target tracking to resume. Call before commanding again."""
        self._hold_engaged = False

    def set_velocity_limit(self, velocity):
        """Set the joint velocity ceiling used by `clip_arm_q_target`.

        Also clears `_speed_gradual_max`: the 250Hz control thread recomputes
        `arm_velocity_limit` from the ramp on every cycle while that flag is set,
        so without clearing it this value would be overwritten within 4ms.
        """
        self._speed_gradual_max = False
        self.arm_velocity_limit = float(velocity)
        logger_mp.info(f"[ArmSafety] arm velocity limit -> {velocity:.1f} rad/s")

    def restore_velocity_limit(self):
        self.set_velocity_limit(self.NOMINAL_ARM_VELOCITY)

    def safe_stop(self, go_home=True):
        """Stop following after a fault: freeze, slow down, optionally home.

        The order matters. Freezing first arrests whatever motion was in flight;
        only then is the velocity ceiling dropped and the (large) trip home
        allowed to start.
        """
        logger_mp.warning("[ArmSafety] SAFE STOP")
        self.hold()
        self.set_velocity_limit(self.SAFE_ARM_VELOCITY)
        if go_home:
            try:
                self.ctrl_dual_arm_go_home()
            except Exception as e:
                logger_mp.error(f"[ArmSafety] safe_stop go-home failed: {e}")
        self.release_hold()
