"""XR safety layer for humanoid teleoperation.

The control loop never decides for itself whether it is safe to move. It hands
the current XR liveness snapshot and pose payload to `SafetyFSM.update()` and
obeys the returned `Action`:

    verdict = fsm.update(now, liveness, head, left_wrist, right_wrist)
    if verdict.action is Action.PASS:
        # use verdict.left_wrist / verdict.right_wrist -- they are rate-limited
    elif verdict.action is Action.HOLD:
        arm_ctrl.hold()
    elif verdict.action is Action.SAFE_STOP:
        arm_ctrl.safe_stop()

See docs/xr_automation_and_safety_plan.md for the rationale and the threshold
tuning procedure.
"""
from .fsm import SafetyFSM, pose_signature
from .jump_guard import JumpGuard, JumpReport
from .types import (
    Action, Fault, FaultKind, JumpGuardConfig, SafetyConfig, SafetyState,
    SafetyVerdict, TERMINAL_FAULTS, TRANSIENT_FAULTS, WatchdogConfig, XRLiveness,
)
from .watchdog import WatchdogReport, XRWatchdog

__all__ = [
    "Action", "Fault", "FaultKind", "JumpGuard", "JumpGuardConfig", "JumpReport",
    "SafetyConfig", "SafetyFSM", "SafetyState", "SafetyVerdict", "TERMINAL_FAULTS",
    "TRANSIENT_FAULTS", "WatchdogConfig", "WatchdogReport", "XRLiveness",
    "XRWatchdog", "pose_signature",
]
