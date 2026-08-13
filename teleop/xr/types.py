"""Device-neutral XR payload.

`XRFrame` deliberately carries only what the control loop actually consumes --
not the ~40 fields of televuer's `TeleData`. Narrowing it here documents the
real dependency surface between "some XR device" and "the robot moves", and is
what makes a second device implementation tractable: a native Quest app has to
produce these seventeen values and nothing else.
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Optional

import numpy as np

try:
    from ..safety.types import XRLiveness
except (ImportError, ValueError):
    # 'beyond top-level package' raises ValueError, not ImportError, when
    # teleop/ itself is the sys.path root rather than its parent.
    from safety.types import XRLiveness


def _zeros2():
    return np.zeros(2)


@dataclass(frozen=True)
class XRFrame:
    """One coherent XR payload, paired with the liveness of the link that
    produced it. Poses follow the robot convention (z up, y left, x front);
    wrist poses are already in the IK target frame."""

    liveness: XRLiveness
    head_pose: np.ndarray                    # (4,4) head in the world frame
    left_wrist_pose: np.ndarray              # (4,4) IK target frame
    right_wrist_pose: np.ndarray

    # hand tracking (None in controller mode)
    left_hand_pos: Optional[np.ndarray] = None    # (25,3)
    right_hand_pos: Optional[np.ndarray] = None

    # hand-tracking inputs
    left_hand_pinch: bool = False
    right_hand_pinch: bool = False
    left_hand_pinchValue: float = 10.0
    right_hand_pinchValue: float = 10.0

    # controller inputs
    left_ctrl_trigger: bool = False
    right_ctrl_trigger: bool = False
    left_ctrl_triggerValue: float = 10.0
    right_ctrl_triggerValue: float = 10.0
    left_ctrl_aButton: bool = False
    right_ctrl_aButton: bool = False
    left_ctrl_thumbstick: bool = False
    right_ctrl_thumbstick: bool = False
    left_ctrl_thumbstickValue: np.ndarray = field(default_factory=_zeros2)
    right_ctrl_thumbstickValue: np.ndarray = field(default_factory=_zeros2)

    @property
    def confirm_gesture(self) -> bool:
        """A deliberate two-handed confirmation.

        Both hands are required on purpose: the align gate is the last thing
        standing between the operator and a moving robot, and a one-handed
        gesture is far too easy to trigger by accident while getting into
        position. Works in either input mode without the caller caring which.
        """
        return ((self.left_hand_pinch and self.right_hand_pinch)
                or (self.left_ctrl_trigger and self.right_ctrl_trigger))

    @classmethod
    def field_names(cls):
        return {f.name for f in fields(cls)}
