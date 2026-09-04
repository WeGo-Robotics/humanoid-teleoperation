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
* An explicit, deliberate **skip**: holding X and A together (`skip_requested`)
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

**And the position check is on shape, not on metres.** Requirement (1) as
originally written compared the operator's mapped wrist position against the
robot's in absolute metres, which a member of the public cannot satisfy: the
G1's arm is about 0.32 m from shoulder to wrist and an adult's is about 0.52 m,
and the transform chain's `WAIST_OFFSET` is a single human-shaped constant that
cannot be right for every operator. Holding the URDF's initial pose correctly
still left the hands 10-20 cm from the target -- more than the whole tolerance.
This gate runs in an experience centre where the operator is a first-time
visitor and there is no calibration step, so the requirement is: *whatever the
operator's scale, if they are holding the robot's initial pose, proceed.*

`scale_free` (the default) therefore compares the **direction** of each
head-to-wrist vector against the direction the robot's own FK asks for -- as
elevation and azimuth, separately, so that "forearms forward" counts for as
much as "hands low"; see `_direction_error_deg` -- and lets the magnitude be
whatever that operator's arm is.

The markers are a separate question from the gate, and are sized from the
operator's eye height -- free to read, since the tracking origin is FloorLevel,
and requiring nothing of a first-time visitor. A marker derived from where the
hand *currently* is would move whenever the hand moved; these depend only on
the robot's pose and the operator's height, so they hold still, and they sit in
a different place for a tall adult than for a child. Nothing about that
estimate can change whether the gate passes, which is what makes it safe to
guess at: being 10% out moves a ring a few centimetres, where the same error
feeding the gate would refuse someone who was standing correctly. Set
`scale_free=False` for the original absolute-position gate, which is still the
right one for a repeatable bench test with a known operator.

"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from teleop.xr.transforms import WAIST_OFFSET_X, WAIST_OFFSET_Z


@dataclass
class AlignConfig:
    #: Per-wrist position tolerance. Read ONLY when `scale_free` is False --
    #: the scale-free gate below tests `dir_tol_deg` and never looks at this.
    #: Kept in step with the scale-free tolerances so switching paths does not
    #: silently tighten the gate.
    pos_tol_m: float = 0.16
    rot_tol_deg: float = 40.0      # per-wrist orientation tolerance
    #: Gate on the direction of the head-to-wrist vector rather than on its
    #: endpoint, so an operator of any size passes by holding the right shape.
    #: See the module docstring; False restores the absolute-position gate.
    scale_free: bool = True
    #: Per-wrist direction tolerance, and the tolerance that actually decides
    #: the default gate.
    #:
    #: Raised from 20 after bench testing on 2026-09-04. The targets come from
    #: the G1's own arm pose, and the robot is not shaped like the operator:
    #: its wrists sit 29.7 cm apart where a human's are 40 cm or more. Holding
    #: the same shape at a human's shoulder width is worth about 6.5 deg of
    #: direction error at 40 cm of separation and 12.2 deg at 50 cm, before any
    #: of the fore/aft and height difference that comes with not being a 1.34 m
    #: robot. Twenty degrees left very little of that budget for the operator's
    #: own aim.
    dir_tol_deg: float = 30.0
    #: A hand this close to the head has no reliable direction -- normalising
    #: it would amplify tracking noise into large angles -- so it reads as
    #: out-of-tolerance rather than as a lucky pass.
    min_reach_m: float = 0.12
    #: Below this fraction of the reach, the horizontal part of a head-to-wrist
    #: vector is too short to carry a bearing -- a hand hanging dead below the
    #: head. Reported as unmeasurable, which the gate treats as a refusal.
    min_horizontal_frac: float = 0.10
    #: The operator's eye height that WAIST_OFFSET implicitly assumes. Its
    #: 0.45 m eye-to-waist is about a third of stature, so the constant encodes
    #: a person roughly 1.34 m tall -- near the G1's own height, which is not a
    #: coincidence: the offset was derived from the robot, not from a human.
    #: Used only to size the markers, never to gate; see `_operator_scale`.
    nominal_eye_height_m: float = 1.25
    #: Clamp on that ratio. A headset reporting nonsense, or an operator who
    #: crouches or sits, must not be able to fling the rings somewhere absurd.
    operator_scale_min: float = 0.75
    operator_scale_max: float = 1.80
    hold_s: float = 1.2            # continuous agreement before accepting
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
    #: Angle between the operator's head-to-wrist direction and the one the
    #: robot's FK asks for. This is what the scale-free gate tests; the
    #: position errors above stay populated either way so a log can still
    #: answer "how far out were they in metres".
    left_dir_err: float = float("inf")   # degrees
    right_dir_err: float = float("inf")
    #: Per-wrist verdict, under whichever check the gate is actually running
    #: and including that wrist's orientation. The headset renders these
    #: rather than re-deriving them from the errors above against a threshold
    #: of its own: a device that decides for itself what "in position" means
    #: can tick a box the host is still refusing, which is exactly the class
    #: of defect that cost builds 12 and 13 (docs sections 15.2, 16.1).
    left_ok: bool = False
    right_ok: bool = False
    #: Radius the headset should draw each ring at, metres. The gate's
    #: tolerance is an angle, and an angle has no size until it is put at a
    #: distance; this is that angle at the marker's distance. Sent rather than
    #: held on the device, which used to mirror the old 0.10 m position
    #: tolerance in a constant of its own.
    left_radius: float = 0.0
    right_radius: float = 0.0
    #: The operator's size relative to the one WAIST_OFFSET assumes. 1.0 when
    #: no head height has been seen yet.
    operator_scale: float = 1.0
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
    def worst_dir_err(self) -> float:
        return max(self.left_dir_err, self.right_dir_err)

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
            "left_dir_err": clean(self.left_dir_err),
            "right_dir_err": clean(self.right_dir_err),
            "left_ok": self.left_ok,
            "right_ok": self.right_ok,
            "left_radius": round(float(self.left_radius), 4),
            "right_radius": round(float(self.right_radius), 4),
            "operator_scale": round(float(self.operator_scale), 3),
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
        self._last_errs = (float("inf"), float("inf"), float("inf"),
                           float("inf"), float("inf"), float("inf"))
        self._last_targets = (None, None)
        self._last_sides = (False, False)
        self._last_radii = (0.0, 0.0)
        #: Tallest plausible eye height seen since this align began. Held, and
        #: taken as a maximum rather than sampled once or averaged: the
        #: operator is standing when alignment starts, and a nod, a crouch or
        #: a glance at the floor must not shrink the markers underneath them.
        self._eye_height: Optional[float] = None

    @property
    def accepted(self) -> bool:
        return self._accepted

    def update(self, now: float,
               robot_left: Optional[np.ndarray], robot_right: Optional[np.ndarray],
               operator_left: np.ndarray, operator_right: np.ndarray,
               operator_confirming: bool,
               skip_requested: bool = False,
               head_height_m: Optional[float] = None) -> AlignReport:
        """One evaluation cycle.

        `robot_left` / `robot_right` come from FK of the current arm joints.
        `None` means FK failed -- reported as not-in-tolerance, and (guided
        path) blocks acceptance same as any other out-of-tolerance reading.

        `skip_requested` is the operator holding X and A together: see module
        docstring. It only ever *relaxes* requirement (1); it cannot substitute
        for the confirm gesture, and it is re-read every cycle like everything
        else that feeds this hold -- letting go of either resets it the same
        way.

        `head_height_m` is the operator's eye height above the floor, which the
        tracking origin being FloorLevel makes free to read. It sizes the
        markers to the person and nothing else -- **it cannot affect whether
        the gate passes.** That separation is the point: an anthropometric
        guess that is 10% out moves a ring by a few centimetres, where the same
        guess feeding the gate would refuse an operator who was standing
        correctly.
        """
        if head_height_m is not None and np.isfinite(head_height_m) \
                and 0.9 <= float(head_height_m) <= 2.2:
            h = float(head_height_m)
            self._eye_height = h if self._eye_height is None \
                else max(self._eye_height, h)
        if self._accepted:
            lp, rp, lr, rr, ld, rd = self._last_errs
            return self._report(self._last_within, True, True, False,
                                self.cfg.hold_s, 1.0, lp, rp, lr, rr, ld, rd,
                                reason="accepted")

        elapsed = now - self._started
        if elapsed > self.cfg.timeout_s:
            return self._report(False, operator_confirming, False, True, 0.0, 0.0,
                                reason="alignment timed out")

        left_delta = right_delta = None
        left_target = right_target = None
        ld = rd = float("inf")
        if robot_left is None or robot_right is None:
            within, lp, rp, lr, rr = False, float("inf"), float("inf"), float("inf"), float("inf")
        else:
            _, lr = pose_error(operator_left, robot_left)
            _, rr = pose_error(operator_right, robot_right)
            rot_ok = lr <= self.cfg.rot_tol_deg and rr <= self.cfg.rot_tol_deg

            # Head-relative on both sides: the operator's mapped wrist and the
            # robot's FK wrist, each with the waist offset taken back off, so
            # the two vectors start from the same place and only their shape
            # is being compared.
            op_l = self._head_relative(operator_left)
            op_r = self._head_relative(operator_right)
            ref_l = self._head_relative(robot_left)
            ref_r = self._head_relative(robot_right)

            ld = self._direction_error_deg(op_l, ref_l)
            rd = self._direction_error_deg(op_r, ref_r)

            if self.cfg.scale_free:
                # The ring goes at THIS operator's reach along the required
                # direction. Placing it at the robot's own distance is what
                # made the gate unsatisfiable for anyone whose arm is not the
                # length the waist offset assumes.
                left_target = self._scaled_target(ref_l)
                right_target = self._scaled_target(ref_r)
                # Reported against the ring the operator is actually being
                # shown, so the HUD's colour, the guidance arrows and the
                # number all describe the same thing.
                lp = float(np.linalg.norm(np.asarray(left_target) - op_l))
                rp = float(np.linalg.norm(np.asarray(right_target) - op_r))
                within = (ld <= self.cfg.dir_tol_deg
                          and rd <= self.cfg.dir_tol_deg and rot_ok)
                left_delta = np.asarray(left_target) - op_l
                right_delta = np.asarray(right_target) - op_r
            else:
                lp = float(np.linalg.norm(op_l - ref_l))
                rp = float(np.linalg.norm(op_r - ref_r))
                left_target = self._head_relative_target(robot_left)
                right_target = self._head_relative_target(robot_right)
                within = (lp <= self.cfg.pos_tol_m and rp <= self.cfg.pos_tol_m
                          and rot_ok)
                left_delta = ref_l - op_l
                right_delta = ref_r - op_r
        left_ok = (robot_left is not None and self._side_ok(lp, ld)
                   and lr <= self.cfg.rot_tol_deg)
        right_ok = (robot_right is not None and self._side_ok(rp, rd)
                    and rr <= self.cfg.rot_tol_deg)
        self._last_within, self._last_errs = within, (lp, rp, lr, rr, ld, rd)
        self._last_targets = (left_target, right_target)
        self._last_sides = (left_ok, right_ok)
        self._last_radii = (
            0.0 if left_target is None else self._ring_radius(left_target),
            0.0 if right_target is None else self._ring_radius(right_target))

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
            reason = "hold both triggers to confirm"
            if not within:
                reason += " — " + self._guidance(lp, rp, left_delta,
                                                 right_delta, ld, rd)
        elif not (within or skip_requested):
            # "X + A together", not "both A/X": the host requires both face
            # buttons at once, and the looser wording reads as either-one --
            # which is the readout error that cost a hardware session.
            reason = (self._guidance(lp, rp, left_delta, right_delta, ld, rd) +
                      " — or hold X + A together to skip")
        else:
            reason = f"hold… {progress * 100:.0f}%"
            if skip_requested and not within:
                reason += " (position check skipped)"

        return self._report(within, operator_confirming, accepted, False, held,
                            progress, lp, rp, lr, rr, ld, rd, reason)

    def _waist_offset(self) -> np.ndarray:
        return np.array([self.cfg.waist_offset_x, 0.0, self.cfg.waist_offset_z])

    def _head_relative(self, pose: np.ndarray) -> np.ndarray:
        """Strip the waist offset back off a pose in the IK frame.

        Applied to the operator's mapped wrist this recovers `op_wrist -
        op_head`, which is what they physically control; applied to the robot's
        FK wrist it gives the same quantity the robot is asking for. Comparing
        the two here is what makes the comparison about the operator's own arm
        rather than about the pelvis of a machine that is a different size.
        """
        return np.asarray(pose, dtype=float)[0:3, 3] - self._waist_offset()

    def _direction_error_deg(self, op_vec: np.ndarray, ref_vec: np.ndarray) -> float:
        """How far the operator's head-to-wrist direction is from the robot's,
        as the worse of two angles: elevation and azimuth.

        Deliberately NOT the plain 3D angle between the vectors. The G1's
        initial pose puts its wrist 0.10m forward, 0.15m out and 0.36m below
        where the chain expects the operator's head, so the vector is dominated
        by its downward component and the horizontal part -- the part that says
        *forearms forward*, which is the whole character of the pose -- is a
        quarter of its length. Under a single 3D angle, an operator standing
        with their arms hanging straight at their sides scores about 15 degrees
        off and sails through a 20 degree gate. Splitting the comparison gives
        the forward/lateral split equal weight with the height, and that same
        operator now reads 34 degrees out on azimuth and is refused.

        Infinite when either vector is shorter than `min_reach_m`, or when the
        horizontal part is too small a fraction of it to have a bearing: a hand
        at the head, or hanging dead below it, has no direction to measure and
        normalising it would turn a centimetre of tracking noise into tens of
        degrees of swing. Unmeasurable reads as out of tolerance, never as a
        pass.
        """
        n_op = float(np.linalg.norm(op_vec))
        n_ref = float(np.linalg.norm(ref_vec))
        if n_op < self.cfg.min_reach_m or n_ref < self.cfg.min_reach_m:
            return float("inf")

        def elevation(v, n):
            return float(np.degrees(np.arcsin(np.clip(v[2] / n, -1.0, 1.0))))

        def azimuth(v, n):
            flat = float(np.hypot(v[0], v[1]))
            if flat < self.cfg.min_horizontal_frac * n:
                return None
            return float(np.degrees(np.arctan2(v[1], v[0])))

        el_err = abs(elevation(op_vec, n_op) - elevation(ref_vec, n_ref))

        az_op = azimuth(op_vec, n_op)
        az_ref = azimuth(ref_vec, n_ref)
        if az_op is None or az_ref is None:
            return float("inf")
        az_err = abs((az_op - az_ref + 180.0) % 360.0 - 180.0)

        return max(el_err, az_err)

    def _operator_scale(self) -> float:
        """How big this operator is, relative to the one WAIST_OFFSET assumes.

        Read from eye height, which the FloorLevel tracking origin gives for
        free and which needs nothing from the operator -- there is no
        calibration step to run in an experience centre, and a first-time
        visitor will not perform one. Clamped, because a headset reporting
        nonsense, or an operator who sits down, must not be able to move the
        markers somewhere absurd.

        Sizing only. Nothing downstream of this can change the gate's verdict.
        """
        if self._eye_height is None:
            return 1.0
        k = self._eye_height / max(self.cfg.nominal_eye_height_m, 1e-6)
        return float(np.clip(k, self.cfg.operator_scale_min,
                             self.cfg.operator_scale_max))

    def _scaled_target(self, ref_vec: np.ndarray) -> Tuple[float, float, float]:
        """Where to put this operator's marker: the required pose, scaled to

        their body.

        The first version of this placed the ring at the operator's *current*
        reach along the required direction. That was wrong in a way that is
        obvious the moment a person is wearing it: the marker is derived from
        where their hand is, so it moves whenever their hand moves. Reaching
        toward it pushes it away, and it can only ever be caught by rotating,
        never by reaching. It also meant two operators of the same size got
        different markers depending on how they happened to be standing when
        alignment began.

        A marker has to be a fixed thing you move toward. This one depends only
        on the robot's pose and the operator's height, so it is stationary for
        the whole of the align, and it is in a different place for a tall adult
        than for a child -- which is the entire point.
        """
        k = self._operator_scale()
        v = np.asarray(ref_vec, dtype=float) * k
        return (float(v[0]), float(v[1]), float(v[2]))

    def _ring_radius(self, target: Tuple[float, float, float]) -> float:
        """The gate's angular tolerance, drawn at the marker's distance.

        The device used to hold this as a 0.10 m constant "matching the gate's
        position tolerance", which the gate no longer has. An angle has no size
        until you put it at a distance, so the size belongs here, with the
        angle.

        Approximate on purpose, and in the forgiving direction: the acceptance
        region is a box in elevation and azimuth, not a cone, so a hand just
        outside the ring can still pass. The ring says roughly where and
        roughly how close; whether that hand actually counts is `left_ok`,
        which is the host's answer and the one the console colours from.
        """
        d = float(np.linalg.norm(np.asarray(target, dtype=float)))
        return float(d * np.tan(np.radians(self.cfg.dir_tol_deg)))

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

    def _closeness_pct(self, pos_err: float, dir_err: float = float("inf")) -> float:
        """0% far out, 100% inside tolerance. Informational only -- a rough

        sense of scale for the HUD, not a threshold anything acts on.

        Reads whichever quantity the gate is actually testing, so the bar
        cannot sit at 100% while the gate refuses: degrees in scale-free mode,
        metres in absolute mode."""
        if self.cfg.scale_free:
            if not np.isfinite(dir_err):
                return 0.0
            if dir_err <= self.cfg.dir_tol_deg:
                return 100.0
            span = max(90.0 - self.cfg.dir_tol_deg, 1e-6)
            return max(0.0, 100.0 * (1.0 - (dir_err - self.cfg.dir_tol_deg) / span))
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

    def _side_ok(self, pos_err: float, dir_err: float) -> bool:
        """Whether one wrist passes the check the gate is actually running, so
        guidance names exactly the wrists that are blocking and no others."""
        if self.cfg.scale_free:
            return dir_err <= self.cfg.dir_tol_deg
        return pos_err <= self.cfg.pos_tol_m

    def _guidance(self, lp, rp, left_delta, right_delta,
                  ld=float("inf"), rd=float("inf")) -> str:
        if left_delta is None or right_delta is None:
            return "robot pose unavailable"
        bits = []
        # The hint stays in centimetres even in scale-free mode: the delta is
        # measured to the ring at the operator's own reach, so "left 12cm" is a
        # move they can actually make. Telling a first-time visitor to correct
        # by eighteen degrees is not.
        if not self._side_ok(lp, ld):
            bits.append(f"L {self._direction_hint(left_delta)} "
                        f"({self._closeness_pct(lp, ld):.0f}%)")
        if not self._side_ok(rp, rd):
            bits.append(f"R {self._direction_hint(right_delta)} "
                        f"({self._closeness_pct(rp, rd):.0f}%)")
        return "  ".join(bits) if bits else "in position"

    def _report(self, within, confirming, accepted, timed_out, held, progress,
                lp=float("inf"), rp=float("inf"), lr=float("inf"),
                rr=float("inf"), ld=float("inf"), rd=float("inf"),
                reason="") -> AlignReport:
        # Targets ride on every report, including the accepted and timed-out
        # early returns, so the device never sees a frame where the markers
        # blink out because the gate took a different branch.
        lt, rt = self._last_targets
        lok, rok = self._last_sides
        lrad, rrad = self._last_radii
        return AlignReport(
            within_tolerance=within, operator_confirming=confirming,
            accepted=accepted, timed_out=timed_out, held_s=held,
            progress=progress, left_pos_err=lp, right_pos_err=rp,
            left_rot_err=lr, right_rot_err=rr,
            left_dir_err=ld, right_dir_err=rd,
            left_ok=lok, right_ok=rok,
            left_radius=lrad, right_radius=rrad,
            operator_scale=self._operator_scale(), reason=reason,
            left_target=lt, right_target=rt)
