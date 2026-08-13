"""Liveness watchdog for the XR link.

Answers one question per cycle: *is the pose data in front of me actually
current?* It never looks at pose values -- that is `JumpGuard`'s job. It looks
only at arrival timing, sequence progress and the device's own validity flags.

This is the primary doff detector in Phase 0. When the operator lifts the
headset off, the Quest proximity sensor blurs the WebXR session and pose events
stop within a few hundred milliseconds; `stale_s` catches that. The definitive
presence signal (`XRLiveness.worn`) arrives with the native app in Phase 3, and
this class already honours it when the device supplies one.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from .types import Fault, FaultKind, WatchdogConfig, XRLiveness


@dataclass(frozen=True)
class WatchdogReport:
    faults: Tuple[Fault, ...]
    stale_for: float          # seconds since the last pose event (0 if never seen)
    frozen_for: float         # seconds the payload has been bit-identical
    link_up: bool

    @property
    def healthy(self) -> bool:
        return not self.faults


class XRWatchdog:
    def __init__(self, config: Optional[WatchdogConfig] = None):
        self.cfg = config or WatchdogConfig()
        self.reset(0.0)

    def reset(self, now: float) -> None:
        self._last_seq = -1
        self._last_seq_change = now
        self._last_signature = None
        self._last_signature_change = now
        self._left_lost_since = None
        self._right_lost_since = None

    def update(self, now: float, liveness: XRLiveness, signature: int) -> WatchdogReport:
        """`signature` is a cheap content hash of the current pose payload.

        Sequence progress alone is not enough: a client that re-sends a cached
        frame keeps `seq` moving while the data is dead. Comparing the payload
        catches that; comparing `seq` catches a client that stops sending.
        """
        faults = []

        if not liveness.session_up:
            # No session attached at all -- nothing below can be meaningful.
            self._last_signature = None
            return WatchdogReport(
                faults=(Fault(FaultKind.LINK_DOWN, "no XR session"),),
                stale_for=0.0, frozen_for=0.0, link_up=False,
            )

        # --- explicit presence, when the device can report it -------------------
        if liveness.worn is False:
            faults.append(Fault(FaultKind.OPERATOR_ABSENT, "headset not worn"))

        # --- arrival freshness ---------------------------------------------------
        if liveness.seq != self._last_seq:
            self._last_seq = liveness.seq
            self._last_seq_change = now
        # Prefer the producer's own receive stamp; fall back to sequence progress
        # if the adapter cannot supply one.
        last_rx = liveness.last_rx if liveness.last_rx > 0.0 else self._last_seq_change
        stale_for = max(0.0, now - last_rx)

        if stale_for >= self.cfg.dead_s:
            faults.append(Fault(FaultKind.LINK_DOWN, f"no data for {stale_for:.2f}s"))
        elif stale_for >= self.cfg.stale_s:
            faults.append(Fault(FaultKind.STALE, f"{stale_for * 1000:.0f}ms"))

        # --- payload freeze ------------------------------------------------------
        if signature != self._last_signature:
            self._last_signature = signature
            self._last_signature_change = now
        frozen_for = now - self._last_signature_change
        if frozen_for >= self.cfg.freeze_s:
            faults.append(Fault(FaultKind.FROZEN, f"{frozen_for * 1000:.0f}ms"))

        # --- per-hand tracking validity -----------------------------------------
        # Brief dropouts are normal when a hand leaves the headset's field of
        # view, so each side gets a grace period before it counts as a fault.
        lost = []
        self._left_lost_since = self._track_side(
            liveness.left_tracked, self._left_lost_since, now)
        self._right_lost_since = self._track_side(
            liveness.right_tracked, self._right_lost_since, now)
        for side, since in (("left", self._left_lost_since),
                            ("right", self._right_lost_since)):
            if since is not None and (now - since) >= self.cfg.tracking_grace_s:
                lost.append(side)
        if lost:
            faults.append(Fault(FaultKind.TRACKING_LOST, "+".join(lost)))

        return WatchdogReport(faults=tuple(faults), stale_for=stale_for,
                              frozen_for=frozen_for, link_up=True)

    @staticmethod
    def _track_side(tracked: bool, lost_since: Optional[float],
                    now: float) -> Optional[float]:
        if tracked:
            return None
        return now if lost_since is None else lost_since
