"""Start-alignment gate.

The last unguarded hazard in the system. Pressing 시작 used to go straight from
"waiting" to full IK following of whatever pose the operator happened to be in;
if their arms were nowhere near the robot's, the first control cycle commanded a
large discontinuity. The README handled this with a human instruction ("align
your arm to the robot's initial pose"), which is not a guard.

The gate requires **two independent agreements, held continuously**:

1. *The host agrees.* Forward kinematics of the robot's current arm joints
   (`ArmFKMixin.forward_kinematics(q)`) gives where its wrists actually are;
   the operator's wrist targets must be within tolerance of that. Computed
   from robot state, so a headset that lies or mis-transforms cannot talk its
   way past it.
2. *The operator agrees.* A deliberate two-handed gesture (both pinches, or
   both triggers) held for `hold_s`.

Either lapsing resets the hold to zero.

**A layman operator cannot satisfy (1) blind.** Passthrough shows the robot,
but nothing marks where their hand needs to be relative to it, so the first
on-hardware session found the hold almost never accumulating -- see
docs §14.7. Two things followed from that, both host-side (per `TeleopHud`'s
own header: "if you find yourself wanting to add a condition here that
changes behaviour ... it belongs on the host"):

* `reason` now carries **directional guidance** for each out-of-tolerance
  wrist -- which way to move, how far, and a rough closeness percentage --
  rather than just "off by 30cm". It is plain text so it renders on the HUD
  with no protocol or Unity change.
* An explicit, deliberate **skip**: holding both A/X buttons (`skip_requested`)
  *together with* the confirm gesture waives requirement (1) for that hold,
  falling through to confirm-gesture-only acceptance. This is not a second,
  weaker gate -- the operator still has to hold a continuous two-handed
  gesture for the full `hold_s`, they are just no longer also required to
  have found the exact pose first. What makes the skip path safe is the same
  thing that makes the guided path's first following-cycle safe:
  `robot_arm.py`'s per-cycle joint velocity limiter (`clip_arm_q_target`,
  ramped in by `speed_gradual_max`) bounds how fast the commanded target can
  move regardless of where it starts, unconditionally, from the moment
  following begins. Skipping the position check changes how far the robot
  eases to catch up; it does not remove the cap on how fast.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from teleop.xr.transforms import WAIST_OFFSET_X, WAIST_OFFSET_Z


@dataclass
class AlignConfig:
    pos_tol_m: float = 0.10        # per-wrist position tolerance
    rot_tol_deg: float = 25.0      # per-wrist orientation tolerance
    hold_s: float = 2.0            # continuous agreement before accepting
    timeout_s: float = 120.0       # give up and return to idle
    guidance_range_m: float = 0.75  # informational only: distance at which the
                                     # HUD's closeness percentage reads 0%
    #: Must match transforms.WAIST_OFFSET_X / _Z. Imported rather than
    #: re-typed so the two cannot drift apart: if the transform chain's offset
    #: changes and this does not, every headset marker moves to the wrong
    #: place while the gate itself still passes, which is the worst way for
    #: these two to disagree.
    waist_offset_x: float = WAIST_OFFSET_X
    waist_offset_z: float = WAIST_OFFSET_Z


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
    #: Where the operator's wrists have to be, expressed the same way the
    #: device expresses them: relative to the operator's own head, in robot
    #: axes (+x front, +y left, +z up). This is the inverse of the chain in
    #: transforms.py, which sends
    #:     target = (op_wrist - op_head) + WAIST_OFFSET
    #: so solving for the operator's half gives
    #:     op_wrist - op_head = robot_fk_wrist - WAIST_OFFSET
    #: which is exactly what a headset needs to anchor a marker in the world.
    #: `None` when forward kinematics is unavailable -- the device must not
    #: draw a target it cannot place.
    left_target: Optional[Tuple[float, float, float]] = None
    right_target: Optional[Tuple[float, float, float]] = None

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
            # Sent as plain lists so JsonUtility on the device can deserialise
            # them without a custom converter; omitted entirely when FK is
            # unavailable rather than sent as zeros, which would place both
            # markers on the operator's own head.
            "left_target": None if self.left_target is None
                           else [round(float(v), 4) for v in self.left_target],
            "right_target": None if self.right_target is None
                            else [round(float(v), 4) for v in self.right_target],
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
        self._last_within = False
        self._last_errs = (float("inf"), float("inf"), float("inf"), float("inf"))
        self._last_targets = (None, None)

    @property
    def accepted(self) -> bool:
        return self._accepted

    def update(self, now: float,
               robot_left: Optional[np.ndarray], robot_right: Optional[np.ndarray],
               operator_left: np.ndarray, operator_right: np.ndarray,
               operator_confirming: bool,
               skip_requested: bool = False) -> AlignReport:
        """One evaluation cycle.

        `robot_left` / `robot_right` come from FK of the current arm joints.
        `None` means FK failed -- reported as not-in-tolerance, and (guided
        path) blocks acceptance same as any other out-of-tolerance reading.

        `skip_requested` is the operator holding both A/X buttons: see module
        docstring. It only ever *relaxes* requirement (1); it cannot substitute
        for the confirm gesture, and it is re-read every cycle like everything
        else that feeds this hold -- letting go of either resets it the same
        way.
        """
        if self._accepted:
            lp, rp, lr, rr = self._last_errs
            return self._report(self._last_within, True, True, False,
                                self.cfg.hold_s, 1.0, lp, rp, lr, rr,
                                reason="accepted")

        elapsed = now - self._started
        if elapsed > self.cfg.timeout_s:
            return self._report(False, operator_confirming, False, True, 0.0, 0.0,
                                reason="alignment timed out")

        left_delta = right_delta = None
        left_target = right_target = None
        if robot_left is None or robot_right is None:
            within, lp, rp, lr, rr = False, float("inf"), float("inf"), float("inf"), float("inf")
        else:
            lp, lr = pose_error(operator_left, robot_left)
            rp, rr = pose_error(operator_right, robot_right)
            within = (lp <= self.cfg.pos_tol_m and rp <= self.cfg.pos_tol_m
                      and lr <= self.cfg.rot_tol_deg and rr <= self.cfg.rot_tol_deg)
            left_delta = np.asarray(robot_left)[0:3, 3] - np.asarray(operator_left)[0:3, 3]
            right_delta = np.asarray(robot_right)[0:3, 3] - np.asarray(operator_right)[0:3, 3]
            left_target = self._head_relative_target(robot_left)
            right_target = self._head_relative_target(robot_right)
        self._last_within, self._last_errs = within, (lp, rp, lr, rr)
        self._last_targets = (left_target, right_target)

        # Both agreements must hold *continuously*; either lapsing restarts the
        # countdown. Sampling once at the end would let the operator drift out
        # of position -- or let go of skip, or of the confirm gesture -- during
        # the countdown and still get credit.
        gate_satisfied = operator_confirming and (within or skip_requested)
        if gate_satisfied:
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
        elif not operator_confirming:
            reason = "hold both hands to confirm"
            if not within:
                reason += " — " + self._guidance(lp, rp, left_delta, right_delta)
        elif not (within or skip_requested):
            reason = (self._guidance(lp, rp, left_delta, right_delta) +
                      " — or hold both A/X to skip")
        else:
            reason = f"hold… {progress * 100:.0f}%"
            if skip_requested and not within:
                reason += " (position check skipped)"

        return self._report(within, operator_confirming, accepted, False, held,
                            progress, lp, rp, lr, rr, reason)

    def _head_relative_target(self, robot_wrist: np.ndarray) -> Tuple[float, float, float]:
        """Undo the waist offset so the device can place a marker.

        transforms.openxr_to_robot() sends
            target = (op_wrist - op_head) + (WAIST_OFFSET_X, 0, WAIST_OFFSET_Z)
        and this gate compares that against FK of the robot's own joints. Where
        the operator's hand actually has to be is therefore

            op_wrist - op_head = robot_fk_wrist - WAIST_OFFSET

        Note this is *not* the robot's own head-to-hand geometry: the chain
        never uses the robot's head, only a fixed human-shaped offset, so the
        robot's height plays no part in where the operator holds their hands.
        """
        p = np.asarray(robot_wrist, dtype=float)[0:3, 3]
        return (float(p[0] - self.cfg.waist_offset_x),
                float(p[1]),
                float(p[2] - self.cfg.waist_offset_z))

    def _closeness_pct(self, pos_err: float) -> float:
        """0% at `guidance_range_m` away, 100% inside tolerance. Informational

        only -- a rough sense of scale for the HUD, not a threshold anything
        acts on."""
        if not np.isfinite(pos_err):
            return 0.0
        if pos_err <= self.cfg.pos_tol_m:
            return 100.0
        span = max(self.cfg.guidance_range_m - self.cfg.pos_tol_m, 1e-6)
        return max(0.0, 100.0 * (1.0 - (pos_err - self.cfg.pos_tol_m) / span))

    @staticmethod
    def _direction_hint(delta: np.ndarray) -> str:
        """`delta` = target - actual, robot frame (+x fwd, +y left, +z up --

        see docs/xr_automation_and_safety_plan.md §9, test_transforms.py's
        TestPhysicalDirections). Axes under 1cm are omitted so the hint does
        not jitter with noise once the operator is close."""
        x, y, z = delta
        parts = []
        if abs(x) >= 0.01:
            parts.append(f"{'fwd' if x > 0 else 'back'} {abs(x) * 100:.0f}cm")
        if abs(y) >= 0.01:
            parts.append(f"{'left' if y > 0 else 'right'} {abs(y) * 100:.0f}cm")
        if abs(z) >= 0.01:
            parts.append(f"{'up' if z > 0 else 'down'} {abs(z) * 100:.0f}cm")
        return ", ".join(parts) if parts else "in position"

    def _guidance(self, lp, rp, left_delta, right_delta) -> str:
        if left_delta is None or right_delta is None:
            return "robot pose unavailable"
        bits = []
        if lp > self.cfg.pos_tol_m:
            bits.append(f"L {self._direction_hint(left_delta)} "
                        f"({self._closeness_pct(lp):.0f}%)")
        if rp > self.cfg.pos_tol_m:
            bits.append(f"R {self._direction_hint(right_delta)} "
                        f"({self._closeness_pct(rp):.0f}%)")
        return "  ".join(bits) if bits else "in position"

    def _report(self, within, confirming, accepted, timed_out, held, progress,
                lp=float("inf"), rp=float("inf"), lr=float("inf"),
                rr=float("inf"), reason="") -> AlignReport:
        # Targets ride on every report, including the accepted and timed-out
        # early returns, so the device never sees a frame where the markers
        # blink out because the gate took a different branch.
        lt, rt = self._last_targets
        return AlignReport(
            within_tolerance=within, operator_confirming=confirming,
            accepted=accepted, timed_out=timed_out, held_s=held,
            progress=progress, left_pos_err=lp, right_pos_err=rp,
            left_rot_err=lr, right_rot_err=rr, reason=reason,
            left_target=lt, right_target=rt)
