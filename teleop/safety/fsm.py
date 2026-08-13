"""The safety finite state machine.

Sits between "we have XR poses" and "we command the arms". The control loop
calls `update()` once per cycle and does exactly what the verdict says.

Design rules, in priority order:

1. **Absence of evidence is a fault.** The gate is recent positive proof that
   the operator is present and tracked -- not the absence of proof that they
   left. Silence fails closed.
2. **SAFE_STOP latches.** Nothing clears it except an explicit operator
   acknowledgement. A link that comes back up is never sufficient reason to
   resume motion.
3. **Re-entering FOLLOWING resets the baselines.** No transition inherits stale
   motion history.
"""
from __future__ import annotations

from typing import Callable, Optional, Tuple

import numpy as np

from .jump_guard import JumpGuard, JumpReport
from .types import (
    Action, Fault, FaultKind, SafetyConfig, SafetyState, SafetyVerdict,
    TERMINAL_FAULTS, XRLiveness,
)
from .watchdog import XRWatchdog

EventSink = Callable[[str, str], None]   # (level, message)


def pose_signature(head: np.ndarray, left: np.ndarray, right: np.ndarray) -> int:
    """Content hash of a pose payload, for freeze detection.

    Real XR poses always jitter, so bit-identical payloads across cycles mean
    the producer stopped updating -- not that the operator held still.
    """
    return hash((head.tobytes(), left.tobytes(), right.tobytes()))


class SafetyFSM:
    def __init__(self, config: Optional[SafetyConfig] = None,
                 on_event: Optional[EventSink] = None):
        self.cfg = config or SafetyConfig()
        self._on_event = on_event or (lambda level, msg: None)
        self._watchdog = XRWatchdog(self.cfg.watchdog)
        self._jump = JumpGuard(self.cfg.jump)

        self._state = SafetyState.IDLE
        self._hold_since: Optional[float] = None
        self._stop_emitted = False
        self._last_faults: Tuple[Fault, ...] = ()
        self._last_stale = 0.0
        self._last_link_up = False
        self._last_worn: Optional[bool] = None
        self._last_clamped = False

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------
    @property
    def state(self) -> SafetyState:
        return self._state

    @property
    def latched(self) -> bool:
        return self._state is SafetyState.SAFE_STOP

    def _goto(self, state: SafetyState, why: str = "") -> None:
        if state is self._state:
            return
        level = "error" if state is SafetyState.SAFE_STOP else (
            "warning" if state is SafetyState.HOLD else "info")
        self._on_event(level, f"safety {self._state.value} -> {state.value}"
                              + (f" ({why})" if why else ""))
        self._state = state

    # ------------------------------------------------------------------
    # operator / loop commands
    # ------------------------------------------------------------------
    def arm(self, now: float) -> bool:
        """Enter FOLLOWING. Refused while a stop is latched."""
        if self._state is SafetyState.SAFE_STOP:
            self._on_event("warning", "arm refused: safe-stop latched, acknowledge first")
            return False
        self._watchdog.reset(now)
        self._jump.reset(now)
        self._hold_since = None
        self._goto(SafetyState.FOLLOWING, "armed")
        return True

    def disarm(self, now: float) -> None:
        """Leave FOLLOWING for an ordinary reason (pause / stop button)."""
        if self._state is SafetyState.SAFE_STOP:
            return
        self._hold_since = None
        self._goto(SafetyState.IDLE, "disarmed")

    def estop(self, now: float, detail: str = "operator") -> None:
        self._latch(now, (Fault(FaultKind.ESTOP, detail),))

    def acknowledge(self, now: float) -> bool:
        """Clear a latched stop. This is the only way out of SAFE_STOP."""
        if self._state is not SafetyState.SAFE_STOP:
            return False
        self._last_faults = ()
        self._hold_since = None
        self._stop_emitted = False
        self._goto(SafetyState.IDLE, "acknowledged")
        return True

    def _latch(self, now: float, faults: Tuple[Fault, ...]) -> None:
        self._last_faults = faults
        self._hold_since = None
        if self._state is not SafetyState.SAFE_STOP:
            self._stop_emitted = False
            self._goto(SafetyState.SAFE_STOP,
                       ", ".join(str(f) for f in faults) or "unspecified")

    # ------------------------------------------------------------------
    # per-cycle evaluation
    # ------------------------------------------------------------------
    def update(self, now: float, liveness: XRLiveness, head: np.ndarray,
               left_wrist: np.ndarray, right_wrist: np.ndarray) -> SafetyVerdict:
        wd = self._watchdog.update(now, liveness, pose_signature(head, left_wrist, right_wrist))
        self._last_stale = wd.stale_for
        self._last_link_up = wd.link_up
        self._last_worn = liveness.worn

        # Latched: keep reporting telemetry, emit the stop action exactly once so
        # the caller runs its safe-stop routine a single time.
        if self._state is SafetyState.SAFE_STOP:
            if not self._stop_emitted:
                self._stop_emitted = True
                return self._verdict(Action.SAFE_STOP, self._last_faults)
            return self._verdict(Action.IDLE, self._last_faults)

        # Not following: telemetry only. The jump guard stays untouched so that
        # arm() always starts from a clean baseline.
        if self._state is SafetyState.IDLE:
            self._last_faults = wd.faults
            return self._verdict(Action.IDLE, wd.faults)

        # --- armed (FOLLOWING or HOLD) -------------------------------------
        # While the data stream is broken the poses in hand are stale, frozen or
        # untracked, so frame-to-frame velocity is meaningless. Abstain from
        # measuring it and re-baseline instead; the rate limiter still bounds the
        # target when the stream recovers.
        if wd.faults:
            self._jump.rebaseline(now)
            jump = JumpReport(faults=())
        else:
            jump = self._jump.observe(now, head, left_wrist, right_wrist)
        faults = wd.faults + jump.faults
        self._last_faults = faults

        terminal = tuple(f for f in faults if f.kind in TERMINAL_FAULTS)
        if terminal:
            self._latch(now, terminal)
            self._stop_emitted = True
            return self._verdict(Action.SAFE_STOP, terminal)

        if jump.escalate:
            why = (Fault(FaultKind.HEAD_JUMP,
                         f"{jump.trips_in_window} trips in "
                         f"{self.cfg.jump.trip_window_s:.0f}s"),)
            self._latch(now, why)
            self._stop_emitted = True
            return self._verdict(Action.SAFE_STOP, why)

        if faults:
            if self._hold_since is None:
                self._hold_since = now
            held = now - self._hold_since
            if held >= self.cfg.hold_to_stop_s:
                escalated = faults + (Fault(FaultKind.LINK_DOWN,
                                            f"held {held:.1f}s"),)
                self._latch(now, escalated)
                self._stop_emitted = True
                return self._verdict(Action.SAFE_STOP, escalated)
            self._goto(SafetyState.HOLD, ", ".join(str(f) for f in faults))
            return self._verdict(Action.HOLD, faults)

        # Healthy.
        self._hold_since = None
        self._goto(SafetyState.FOLLOWING, "recovered")
        out_left, out_right, clamped = self._jump.clamp(left_wrist, right_wrist)
        self._last_clamped = clamped
        return self._verdict(Action.PASS, (), left=out_left, right=out_right,
                             clamped=clamped)

    def _verdict(self, action: Action, faults: Tuple[Fault, ...],
                 left: Optional[np.ndarray] = None,
                 right: Optional[np.ndarray] = None,
                 clamped: bool = False) -> SafetyVerdict:
        return SafetyVerdict(action=action, state=self._state, faults=faults,
                             left_wrist=left, right_wrist=right,
                             stale_for=self._last_stale, clamped=clamped)

    # ------------------------------------------------------------------
    # telemetry
    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        """Compact state for the IPC heartbeat / dashboard."""
        return {
            "state": self._state.value,
            "faults": [f.kind.value for f in self._last_faults],
            "reason": ", ".join(str(f) for f in self._last_faults),
            "stale_ms": int(self._last_stale * 1000.0),
            "link_up": self._last_link_up,
            "worn": self._last_worn,
            "clamped": self._last_clamped,
            "latched": self.latched,
        }
