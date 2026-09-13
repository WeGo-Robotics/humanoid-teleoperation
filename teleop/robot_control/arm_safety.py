"""Safe-stop primitives shared by every arm controller.

Kept separate from `robot_arm.py` so it can be exercised without a DDS stack or
a robot -- these are the behaviours that run when something has already gone
wrong, which makes them exactly the ones worth testing offline.
"""
import time

import numpy as np
import logging_mp

logger_mp = logging_mp.get_logger(__name__)


class ArmSafetyMixin:
    """Mixed into every `*_ArmController`.

    Written against the contract all five controllers already satisfy
    (`ctrl_lock`, `q_target`, `tauff_target`, `arm_velocity_limit`,
    `_speed_gradual_max`, `get_current_dual_arm_q()`, `ctrl_dual_arm_go_home()`),
    so it needs no per-class duplication.

    Motivation: before this, the only ways to stop were `ctrl_dual_arm_go_home()`
    -- which snaps `q_target` to zeros and drives there at the full 20-30 rad/s
    limit -- and killing the process. Neither is a safe response to the operator
    disappearing mid-motion.

    Why the homing move is slowed by moving the *target* slowly, and never by
    lowering `arm_velocity_limit`: `clip_arm_q_target` steps from the *measured*
    position, so that limit does not cap speed directly -- it caps how far the
    command may lead the arm, `arm_velocity_limit * control_dt` radians, and so
    caps the PD torque at kp times that. An earlier version dropped it to
    3 rad/s to home "slowly". That is a 0.012 rad lead: under 1 Nm at the
    shoulders and 0.5 Nm at the wrists, less than gravity. On the robot the arm
    could not be driven home at all; it drifted wherever gravity and the stale
    feed-forward torque pushed it, and the go-home wait gave up after 5 s with
    the arms twisted part-way. `clip_arm_q_target` is bypassed in
    `simulation_mode`, which is why the same e-stop looked fine in simulation.
    """

    NOMINAL_ARM_VELOCITY = 20.0   # matches every controller's __init__ default

    #: Peak joint speed of the homing move, rad/s. Reached only by the joint
    #: with the furthest to go; every other joint moves proportionally slower,
    #: so all of them arrive together.
    HOME_PEAK_SPEED = 1.0
    #: Floor on the homing duration, so a short trip is still a gentle one.
    HOME_MIN_S = 1.0
    #: How often the homing move updates the target. The 250Hz control thread
    #: interpolates nothing, so this is the step size the motors see.
    HOME_RATE_HZ = 100.0
    #: A joint further than this from home after the move is reported. Matches
    #: the tolerance the controllers' own go-home waits for.
    HOME_TOLERANCE = 0.05

    # Seams for tests, which drive the homing move on a simulated clock.
    def _now(self):
        return time.monotonic()

    def _wait(self, seconds):
        time.sleep(seconds)

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

        Do not lower it to slow the arm down -- see the class docstring. It is
        a torque ceiling in disguise.
        """
        self._speed_gradual_max = False
        self.arm_velocity_limit = float(velocity)
        logger_mp.info(f"[ArmSafety] arm velocity limit -> {velocity:.1f} rad/s")

    def restore_velocity_limit(self):
        self.set_velocity_limit(self.NOMINAL_ARM_VELOCITY)

    def glide_home(self, gravity=None):
        """Walk the commanded pose from where it is to home, slowly and smoothly.

        Minimum-jerk in time, so the arm starts and stops without a velocity
        step, and peaks at `HOME_PEAK_SPEED`. The feed-forward torque follows
        the path too. The one in place was computed by the IK for the last pose
        teleop commanded; holding it fixed while the arm moves pushes the arm
        off the path, and at home it is simply wrong -- the G1's forearms are
        horizontal at zero, so the elbows need real torque there.

        `gravity(q)` returns the gravity-compensation torques for `q`; pass the
        IK model's `gravity_torque`. Without it, or if it fails, the
        feed-forward is left as it was, which is what the controllers' own
        go-home has always done.
        """
        with self.ctrl_lock:
            q0 = np.array(self.q_target, dtype=float)
            tau0 = np.array(self.tauff_target, dtype=float)
        home = np.zeros_like(q0)
        dist = float(np.max(np.abs(home - q0))) if q0.size else 0.0
        # A minimum-jerk profile peaks at 1.875x its average speed.
        duration = max(1.875 * dist / self.HOME_PEAK_SPEED, self.HOME_MIN_S)
        logger_mp.info(f"[ArmSafety] homing: {dist:.2f} rad over {duration:.1f}s")

        dt = 1.0 / self.HOME_RATE_HZ
        t0 = self._now()
        while True:
            u = min((self._now() - t0) / duration, 1.0)
            s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
            q = q0 + (home - q0) * s
            tau = tau0
            if gravity is not None:
                try:
                    tau = np.asarray(gravity(q), dtype=float).reshape(tau0.shape)
                except Exception as e:
                    # Once, not at 100Hz: fall back for the rest of the move.
                    logger_mp.error(f"[ArmSafety] gravity model failed ({e}); "
                                    f"homing with the feed-forward unchanged")
                    gravity = None
            with self.ctrl_lock:
                self.q_target = q
                self.tauff_target = tau
            if u >= 1.0:
                return
            self._wait(dt)

    def safe_stop(self, go_home=True, gravity=None):
        """Stop following after a fault: freeze, then optionally home slowly.

        The order matters. Freezing first arrests whatever motion was in flight
        before the (large) trip home starts. The velocity ceiling is put back to
        nominal, never lowered: the slowness comes from `glide_home` moving the
        target slowly, and the ceiling is what gives the PD enough authority to
        follow it. See the class docstring for what happened when it didn't.
        """
        logger_mp.warning("[ArmSafety] SAFE STOP")
        self.hold()
        self.restore_velocity_limit()
        if go_home:
            try:
                self.glide_home(gravity)
                # The target is already home; this waits for the arms to settle
                # and, in motion mode, hands them back to the balance controller.
                self.ctrl_dual_arm_go_home()
                self._report_home()
            except Exception as e:
                logger_mp.error(f"[ArmSafety] safe_stop go-home failed: {e}")
        self.release_hold()

    def _report_home(self):
        """Say so when the arms did not make it home.

        The controllers' go-home gives up silently after ~5s, which is how an
        arm stranded half-way could look, from the log, like a clean stop.
        """
        try:
            q = np.asarray(self.get_current_dual_arm_q(), dtype=float)
        except Exception:
            return
        if not q.size:
            return
        worst = int(np.argmax(np.abs(q)))
        if abs(q[worst]) > self.HOME_TOLERANCE:
            logger_mp.error(f"[ArmSafety] arms did NOT reach home: joint {worst} "
                            f"is {q[worst]:+.3f} rad off")
        else:
            logger_mp.info("[ArmSafety] arms home")
