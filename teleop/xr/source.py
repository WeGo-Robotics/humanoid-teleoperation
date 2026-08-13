"""The seam between "some XR device" and "the robot moves".

The control loop talks to an `XRSource` and never learns which device is
attached. Two implementations exist:

* `VuerXRSource`   -- the Quest browser + Vuer server that ships today.
* `NativeXRSource` -- the XrLink WebSocket server the Quest app connects to.

Everything safety-relevant lives above this interface, in `teleop.safety`, so a
new device cannot accidentally opt out of it.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from .types import XRFrame


class XRSource(ABC):
    """A source of XR tracking data and a sink for host-to-device messages."""

    #: Short identifier for logs and the dashboard.
    name: str = "xr"

    @abstractmethod
    def read(self) -> XRFrame:
        """Newest payload plus the liveness of the link that produced it.

        Never blocks and never raises: a source with nothing to report returns a
        frame whose `liveness.session_up` is False. Absence of data is a normal,
        expected condition that the safety layer is built to handle -- an
        exception here would be a far worse way to express it.
        """

    def send(self, message: dict) -> bool:
        """Push a host-to-device message (session state, align prompt, abort).

        Returns False when the transport has no way to deliver it. Optional by
        design: the browser transport is receive-only.
        """
        return False

    def render_to_xr(self, image: np.ndarray) -> None:
        """Show a camera frame in the headset. No-op if unsupported."""

    def close(self) -> None:
        """Release the transport."""


def wrist_targets(frame: XRFrame):
    return frame.left_wrist_pose, frame.right_wrist_pose
