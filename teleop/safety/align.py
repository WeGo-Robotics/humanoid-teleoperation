"""Start-alignment gate.

The last unguarded hazard in the system. Pressing 시작 used to go straight from
"waiting" to full IK following of whatever pose the operator happened to be in;
if their arms were nowhere near the robot's, the first control cycle commanded a
large discontinuity. The README handled this with a human instruction ("align
your arm to the robot's initial pose"), which is not a guard.

The gate requires **two independent agreements**, held continuously:

1. *The host agrees.* Forward kinematics of the robot's current arm joints gives
   where its wrists actually are; the operator's wrist targets must be within
   tolerance of that. This is computed from robot state, so a headset that lies
   or mis-transforms cannot talk its way past it.
2. *The operator agrees.* A deliberate two-handed gesture (both pinches, or both
   triggers) held for `hold_s`.

Either one lapsing resets the hold to zero. Requiring both to persist -- rather
than sampling them once at the moment of confirmation -- is what stops an
operator from drifting out of position during the countdown.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class AlignConfig:
    pos_tol_m: float = 0.10      # per-wrist position tolerance
    rot_tol_deg: float = 25.0    # per-wrist orientation tolerance
    hold_s: float = 2.0          # continuous agreement before accepting
    timeout_s: float = 120.0     # give up and return to idle


@dataclass(frozen=True)
class AlignReport:
    within_tolerance: bool
    operator_confirming: bool
    accepted: bool
    timed_out: bool
    held_s: float
    progress: float                      # 0..1 through the hold
    left_pos_err: float = float("inf")   # metres
    right_pos_err: float = float("inf")
    left_rot_err: float = float("inf")   # degrees
    right_rot_err: float = float("inf")
    reason: str = ""

    @property
    def worst_pos_err(self) -> float:
        return max(self.left_pos_err, self.right_pos_err)

    @property
    def worst_rot_err(self) -> float:
        return max(self.left_rot_err, self.right_rot_err)

    def as_dict(self) -> dict:
        """Compact form for the IPC heartbeat / dashboard."""
        def clean(v):
            return None if not np.isfinite(v) else round(float(v), 4)
        return {
            "within_tolerance": self.within_tolerance,
            "confirming": self.operator_confirming,
            "accepted": self.accepted,
            "timed_out": self.timed_out,
            "held_s": round(self.held_s, 2),
            "progress": round(self.progress, 3),
            "left_pos_err": clean(self.left_pos_err),
            "right_pos_err": clean(self.right_pos_err),
            "left_rot_err": clean(self.left_rot_err),
            "right_rot_err": clean(self.right_rot_err),
            "reason": self.reason,
        }


def pose_error(actual: np.ndarray, target: np.ndarray) -> Tuple[float, float]:
    """(position error in metres, orientation error in degrees)."""
    pos = float(np.linalg.norm(np.asarray(actual)[0:3, 3] - np.asarray(target)[0:3, 3]))
    rel = np.asarray(actual)[0:3, 0:3].T @ np.asarray(target)[0:3, 0:3]
    cos = (np.trace(rel) - 1.0) / 2.0
    if not np.isfinite(cos):
        return pos, float("inf")
    rot = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
    return pos, rot


class AlignGate:
    def __init__(self, config: Optional[AlignConfig] = None):
        self.cfg = config or AlignConfig()
        self.reset(0.0)

    def reset(self, now: float) -> None:
        self._started = now
        self._hold_since: Optional[float] = None
        self._accepted = False

    @property
    def accepted(self) -> bool:
        return self._accepted

    def update(self, now: float,
               robot_left: Optional[np.ndarray], robot_right: Optional[np.ndarray],
               operator_left: np.ndarray, operator_right: np.ndarray,
               operator_confirming: bool) -> AlignReport:
        """One evaluation cycle.

        `robot_left` / `robot_right` come from FK of the current arm joints.
        `None` means FK failed -- reported as not-aligned, never as aligned.
        """
        if self._accepted:
            return self._report(True, True, True, False, self.cfg.hold_s, 1.0,
                                reason="accepted")

        elapsed = now - self._started
        if elapsed > self.cfg.timeout_s:
            return self._report(False, operator_confirming, False, True, 0.0, 0.0,
                                reason="alignment timed out")

        if robot_left is None or robot_right is None:
            self._hold_since = None
            return self._report(False, operator_confirming, False, False, 0.0, 0.0,
                                reason="robot pose unavailable — cannot verify")

        lp, lr = pose_error(operator_left, robot_left)
        rp, rr = pose_error(operator_right, robot_right)
        within = (lp <= self.cfg.pos_tol_m and rp <= self.cfg.pos_tol_m
                  and lr <= self.cfg.rot_tol_deg and rr <= self.cfg.rot_tol_deg)

        # Both agreements must hold *continuously*; either lapsing restarts the
        # countdown. Sampling them once at the end would let the operator drift
        # out of position while the timer ran.
        if within and operator_confirming:
            if self._hold_since is None:
                self._hold_since = now
            held = now - self._hold_since
        else:
            self._hold_since = None
            held = 0.0

        progress = min(1.0, held / self.cfg.hold_s) if self.cfg.hold_s > 0 else 1.0
        accepted = held >= self.cfg.hold_s
        if accepted:
            self._accepted = True

        if accepted:
            reason = "accepted"
        elif not within:
            reason = self._deviation_hint(lp, rp, lr, rr)
        elif not operator_confirming:
            reason = "in position — hold both hands to confirm"
        else:
            reason = f"hold… {progress * 100:.0f}%"

        return self._report(within, operator_confirming, accepted, False, held,
                            progress, lp, rp, lr, rr, reason)

    def _deviation_hint(self, lp, rp, lr, rr) -> str:
        worst = []
        if lp > self.cfg.pos_tol_m:
            worst.append(f"left {lp * 100:.0f}cm")
        if rp > self.cfg.pos_tol_m:
            worst.append(f"right {rp * 100:.0f}cm")
        if lr > self.cfg.rot_tol_deg:
            worst.append(f"left {lr:.0f}°")
        if rr > self.cfg.rot_tol_deg:
            worst.append(f"right {rr:.0f}°")
        return "move to the robot's pose — off by " + ", ".join(worst)

    def _report(self, within, confirming, accepted, timed_out, held, progress,
                lp=float("inf"), rp=float("inf"), lr=float("inf"),
                rr=float("inf"), reason="") -> AlignReport:
        return AlignReport(
            within_tolerance=within, operator_confirming=confirming,
            accepted=accepted, timed_out=timed_out, held_s=held,
            progress=progress, left_pos_err=lp, right_pos_err=rp,
            left_rot_err=lr, right_rot_err=rr, reason=reason)
