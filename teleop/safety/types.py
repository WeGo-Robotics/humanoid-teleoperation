"""Shared vocabulary for the XR safety layer.

Everything here is dependency-free (numpy only) so it can be imported by the
teleop process, the dashboard, and the tests without pulling in DDS, Vuer or
MuJoCo.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

import numpy as np


class SafetyState(Enum):
    """Where the safety layer thinks the session is."""
    IDLE      = "idle"        # not following; arms are not driven from XR
    FOLLOWING = "following"   # XR data is trusted and driving the arms
    HOLD      = "hold"        # transient fault; arms frozen, session recoverable
    SAFE_STOP = "safe_stop"   # latched fault; needs an explicit operator ack


class Action(Enum):
    """What the control loop must do with this cycle."""
    IDLE      = "idle"        # do not command the arms at all
    PASS      = "pass"        # solve IK and command the (possibly clamped) target
    HOLD      = "hold"        # freeze the arms where they are
    SAFE_STOP = "safe_stop"   # freeze, then slow-home, and latch following off


class FaultKind(Enum):
    LINK_DOWN       = "link_down"        # no XR session attached at all
    STALE           = "stale"            # no fresh pose event within the deadline
    FROZEN          = "frozen"           # events arriving but the payload never changes
    TRACKING_LOST   = "tracking_lost"    # device reports a hand/controller is untracked
    OPERATOR_ABSENT = "operator_absent"  # device reports the headset is not worn
    HEAD_JUMP       = "head_jump"        # head moved faster than a human head can
    WRIST_JUMP      = "wrist_jump"       # wrist target moved implausibly fast
    ESTOP           = "estop"            # explicit operator/dashboard stop


#: Faults that mean the operator is gone or unreachable. These latch immediately —
#: there is no plausible reading under which continuing to drive the arms is safe.
TERMINAL_FAULTS = frozenset({
    FaultKind.LINK_DOWN,
    FaultKind.OPERATOR_ABSENT,
    FaultKind.ESTOP,
})

#: Faults that may be a momentary glitch. These hold the arms; if they persist
#: past `SafetyConfig.hold_to_stop_s` the FSM escalates them to a latched stop.
TRANSIENT_FAULTS = frozenset({
    FaultKind.STALE,
    FaultKind.FROZEN,
    FaultKind.TRACKING_LOST,
    FaultKind.HEAD_JUMP,
    FaultKind.WRIST_JUMP,
})


@dataclass(frozen=True)
class Fault:
    kind: FaultKind
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.kind.value}({self.detail})" if self.detail else self.kind.value


@dataclass(frozen=True)
class XRLiveness:
    """Transport-level snapshot of the XR link, produced by the XR adapter.

    This is the whole contract between "how we talk to the headset" and "is it
    safe to move". A Vuer browser session and a native Quest app fill it in from
    different places; nothing downstream can tell them apart.
    """
    seq: int                        # monotonic counter, bumped per pose event
    last_rx: float                  # time.monotonic() of the newest pose event
    session_up: bool                # a device session is currently attached
    left_tracked: bool = True
    right_tracked: bool = True
    worn: Optional[bool] = None     # None => this device cannot report presence

    @staticmethod
    def offline() -> "XRLiveness":
        return XRLiveness(seq=0, last_rx=0.0, session_up=False,
                          left_tracked=False, right_tracked=False, worn=None)


@dataclass(frozen=True)
class SafetyVerdict:
    """The safety layer's answer for one control cycle."""
    action: Action
    state: SafetyState
    faults: Tuple[Fault, ...] = ()
    # Rate-limited wrist targets. Only meaningful when action is PASS; the control
    # loop must feed *these* to IK rather than the raw XR poses.
    left_wrist: Optional[np.ndarray] = None
    right_wrist: Optional[np.ndarray] = None
    # Telemetry, surfaced on the heartbeat for the dashboard.
    stale_for: float = 0.0
    clamped: bool = False

    @property
    def allow_motion(self) -> bool:
        return self.action is Action.PASS

    @property
    def reason(self) -> str:
        return ", ".join(str(f) for f in self.faults) if self.faults else ""


@dataclass
class WatchdogConfig:
    """Deadlines for "is the XR data still trustworthy".

    Starting values. They need one tuning pass in sim and one supervised pass on
    the robot with the arms clear -- see docs/xr_automation_and_safety_plan.md.
    """
    stale_s: float = 0.20     # no pose event for this long -> HOLD
    dead_s: float = 1.00      # ...for this long -> latched stop
    freeze_s: float = 0.30    # identical payload for this long -> HOLD
    tracking_grace_s: float = 0.30   # tolerate brief per-hand dropouts


@dataclass
class JumpGuardConfig:
    """Plausibility limits on operator motion.

    `clamp_wrist_mps` is a rate limiter and fires constantly in normal use; the
    `*_trip_*` values are anomaly detectors and should never fire in normal use.
    """
    head_lin_mps: float = 2.0        # head translation speed trip
    head_ang_dps: float = 720.0      # head rotation speed trip
    wrist_lin_mps: float = 4.0       # wrist translation speed trip
    clamp_wrist_mps: float = 2.0     # Cartesian rate limit applied to targets
    trip_window_s: float = 1.0
    trips_to_escalate: int = 3
    min_dt: float = 1.0 / 240.0      # dt clamp, so a stalled loop cannot
    max_dt: float = 1.0 / 10.0       # manufacture or mask a velocity spike


@dataclass
class SafetyConfig:
    watchdog: WatchdogConfig = field(default_factory=WatchdogConfig)
    jump: JumpGuardConfig = field(default_factory=JumpGuardConfig)
    hold_to_stop_s: float = 1.0   # continuous HOLD longer than this latches a stop
