"""Plausibility guard on operator motion.

Two jobs, deliberately separated:

* **observe()** -- an *anomaly detector*. Measures true instantaneous head and
  wrist speed and trips when they exceed what a human can do. Should never fire
  in normal operation.
* **clamp()** -- a *rate limiter*. Bounds how far a wrist target can move in one
  control cycle. Fires constantly and harmlessly; its purpose is to put a
  ceiling on the damage of any transient the detectors have not caught yet.

Why both. The dangerous quantity in this system is the head-relative wrist
target (`p_hand - p_head`, see tv_wrapper.py:296), so a head-only transient
displaces the arms just as much as a hand movement does. A doff is not
instantaneous -- the headset comes off over ~0.5-1s at 0.5-1.5 m/s, which
overlaps the speed of an ordinary crouch and therefore *cannot* be reliably
separated from legitimate motion by speed alone. The watchdog catches the doff
when pose events stop; the clamp bounds how far the arms can travel in the
window before that happens.

Two baselines are tracked on purpose:

* `_prev_in`  advances every frame, so measured velocity is always the real
  frame-to-frame velocity -- even while the FSM is holding.
* `_prev_out` advances only when a target is actually emitted, so recovering
  from a HOLD walks the target back at the clamp rate instead of snapping, and
  does not produce a spurious trip on the first frame after the hold.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np

from .types import Fault, FaultKind, JumpGuardConfig


def _rot_angle_deg(r_prev: np.ndarray, r_now: np.ndarray) -> float:
    """Geodesic angle between two rotation matrices, in degrees."""
    rel = r_prev.T @ r_now
    cos = (np.trace(rel) - 1.0) / 2.0
    if not np.isfinite(cos):
        return 0.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


@dataclass(frozen=True)
class JumpReport:
    faults: Tuple[Fault, ...]
    head_mps: float = 0.0
    head_dps: float = 0.0
    left_mps: float = 0.0
    right_mps: float = 0.0
    trips_in_window: int = 0
    escalate: bool = False   # too many trips too fast -> latch a stop

    @property
    def healthy(self) -> bool:
        return not self.faults


class JumpGuard:
    def __init__(self, config: Optional[JumpGuardConfig] = None):
        self.cfg = config or JumpGuardConfig()
        self._trips: Deque[float] = deque()
        self.reset(0.0)

    def reset(self, now: float) -> None:
        """Drop all history. Call when (re)entering FOLLOWING so the first frame
        establishes a baseline instead of measuring against a stale one."""
        self._prev_in = None          # (head, left, right) as of the last observe
        self._prev_t = now
        self._dt = self.cfg.max_dt
        self._prev_out = None         # (left, right) as last emitted
        self._trips.clear()

    def rebaseline(self, now: float) -> None:
        """Forget the motion baseline, keeping the output rate limiter intact.

        Called while data continuity is broken (stale, frozen, untracked). Across
        a gap the operator's velocity is *undefined*, not large -- measuring the
        pre-gap pose against the post-gap pose over one frame's dt would
        manufacture a spike and trip the detector on every recovery. The clamp
        still walks the target in smoothly, which is the behaviour that actually
        keeps the arms safe here.
        """
        self._prev_in = None
        self._prev_t = now

    # ------------------------------------------------------------------
    # anomaly detection
    # ------------------------------------------------------------------
    def observe(self, now: float, head: np.ndarray,
                left: np.ndarray, right: np.ndarray) -> JumpReport:
        dt = float(np.clip(now - self._prev_t, self.cfg.min_dt, self.cfg.max_dt))
        self._prev_t = now
        self._dt = dt

        if self._prev_in is None:
            self._prev_in = (head.copy(), left.copy(), right.copy())
            return JumpReport(faults=())

        p_head, p_left, p_right = self._prev_in
        head_mps = float(np.linalg.norm(head[0:3, 3] - p_head[0:3, 3])) / dt
        head_dps = _rot_angle_deg(p_head[0:3, 0:3], head[0:3, 0:3]) / dt
        left_mps = float(np.linalg.norm(left[0:3, 3] - p_left[0:3, 3])) / dt
        right_mps = float(np.linalg.norm(right[0:3, 3] - p_right[0:3, 3])) / dt

        self._prev_in = (head.copy(), left.copy(), right.copy())

        faults = []
        if head_mps > self.cfg.head_lin_mps:
            faults.append(Fault(FaultKind.HEAD_JUMP, f"{head_mps:.1f} m/s"))
        elif head_dps > self.cfg.head_ang_dps:
            faults.append(Fault(FaultKind.HEAD_JUMP, f"{head_dps:.0f} deg/s"))
        worst_wrist = max(left_mps, right_mps)
        if worst_wrist > self.cfg.wrist_lin_mps:
            faults.append(Fault(FaultKind.WRIST_JUMP, f"{worst_wrist:.1f} m/s"))

        if faults:
            self._trips.append(now)
        while self._trips and (now - self._trips[0]) > self.cfg.trip_window_s:
            self._trips.popleft()

        return JumpReport(
            faults=tuple(faults),
            head_mps=head_mps, head_dps=head_dps,
            left_mps=left_mps, right_mps=right_mps,
            trips_in_window=len(self._trips),
            escalate=len(self._trips) >= self.cfg.trips_to_escalate,
        )

    # ------------------------------------------------------------------
    # rate limiting
    # ------------------------------------------------------------------
    def clamp(self, left: np.ndarray,
              right: np.ndarray) -> Tuple[np.ndarray, np.ndarray, bool]:
        """Rate-limit the emitted wrist targets. Only call this when the target
        is actually going to be commanded -- it advances the output baseline."""
        max_step = self.cfg.clamp_wrist_mps * self._dt

        if self._prev_out is None:
            self._prev_out = (left.copy(), right.copy())
            return left, right, False

        out_left, hit_l = self._limit(self._prev_out[0], left, max_step)
        out_right, hit_r = self._limit(self._prev_out[1], right, max_step)
        self._prev_out = (out_left.copy(), out_right.copy())
        return out_left, out_right, (hit_l or hit_r)

    @staticmethod
    def _limit(prev: np.ndarray, target: np.ndarray,
               max_step: float) -> Tuple[np.ndarray, bool]:
        """Limit translation only. Orientation passes through -- a wrist
        rotation cannot translate the end effector across the workspace, and
        slerp-limiting it in Phase 0 buys less than it costs."""
        out = target.copy()
        delta = target[0:3, 3] - prev[0:3, 3]
        dist = float(np.linalg.norm(delta))
        if dist > max_step > 0.0:
            out[0:3, 3] = prev[0:3, 3] + delta * (max_step / dist)
            return out, True
        return out, False
