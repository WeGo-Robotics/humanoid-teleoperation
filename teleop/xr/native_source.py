"""`XRSource` over the XrLink native transport.

Turns the link server's snapshot into the same `XRFrame` the Vuer source
produces, so the control loop and the whole safety layer are unchanged.

The interesting difference from `VuerXRSource` is that `worn` is a *measured*
value here rather than `None`. That is what upgrades presence from an inference
("data stopped arriving") to an observation ("the proximity sensor says the
headset is off") -- but only as a fast path. The watchdog still runs, because
the OS may suspend the app on doff before it can transmit (docs §10.3).
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np

try:
    from ..safety.types import XRLiveness
except (ImportError, ValueError):
    # 'beyond top-level package' raises ValueError, not ImportError, when
    # teleop/ itself is the sys.path root rather than its parent.
    from safety.types import XRLiveness
from .link_server import LinkSnapshot, XrLinkServer
from .source import XRSource
from .transforms import openxr_to_robot
from .types import XRFrame

_IDENTITY = np.eye(4)


class NativeXRSource(XRSource):
    name = "xrlink"

    def __init__(self, server: XrLinkServer):
        self._server = server

    @property
    def server(self) -> XrLinkServer:
        return self._server

    def read(self) -> XRFrame:
        snap = self._server.snapshot()
        return self._to_frame(snap)

    @staticmethod
    def _to_frame(snap: LinkSnapshot) -> XRFrame:
        frame = snap.frame
        if frame is None:
            # Connected but nothing decoded yet, or not connected at all. Either
            # way there is no pose to act on; identity poses are inert and the
            # liveness fields tell the safety layer not to trust them.
            return XRFrame(
                liveness=XRLiveness(
                    seq=snap.seq, last_rx=snap.rx_monotonic,
                    session_up=snap.connected,
                    left_tracked=False, right_tracked=False, worn=snap.worn),
                head_pose=_IDENTITY.copy(),
                left_wrist_pose=_IDENTITY.copy(),
                right_wrist_pose=_IDENTITY.copy(),
            )

        inputs = frame.inputs
        # From the control channel, not the tracking frame -- see LinkSnapshot.
        buttons = snap.buttons
        # Raw OpenXR on the wire; the transform lives here, shared with every
        # other device implementation. `valid` reports any pose that had to be
        # replaced by a fallback, which then counts as untracked rather than
        # being acted on as if it were real.
        head, left, right, valid = openxr_to_robot(
            frame.head, frame.left_wrist, frame.right_wrist,
            hand_tracking=frame.hand_mode)
        head_ok, left_ok, right_ok = valid
        return XRFrame(
            liveness=XRLiveness(
                seq=snap.seq,
                last_rx=snap.rx_monotonic,
                session_up=snap.connected,
                left_tracked=frame.left_tracked and left_ok and head_ok,
                right_tracked=frame.right_tracked and right_ok and head_ok,
                worn=snap.worn,
            ),
            head_pose=head,
            left_wrist_pose=left,
            right_wrist_pose=right,
            left_hand_pos=frame.left_joints,
            right_hand_pos=frame.right_joints,
            # Pinch/trigger analog values follow televuer's inverted convention
            # (10.0 open -> 0.0 fully closed) so downstream gripper code needs
            # no per-source special-casing.
            left_hand_pinchValue=inputs.get("left_pinch_value", 10.0),
            right_hand_pinchValue=inputs.get("right_pinch_value", 10.0),
            left_hand_pinch=inputs.get("left_pinch_value", 10.0) < 1.0,
            right_hand_pinch=inputs.get("right_pinch_value", 10.0) < 1.0,
            left_ctrl_triggerValue=inputs.get("left_trigger_value", 10.0),
            right_ctrl_triggerValue=inputs.get("right_trigger_value", 10.0),
            left_ctrl_trigger=inputs.get("left_trigger_value", 10.0) < 1.0,
            right_ctrl_trigger=inputs.get("right_trigger_value", 10.0) < 1.0,
            left_ctrl_aButton="left_a" in buttons,
            right_ctrl_aButton="right_a" in buttons,
            left_ctrl_thumbstick="left_thumb" in buttons,
            right_ctrl_thumbstick="right_thumb" in buttons,
            left_ctrl_thumbstickValue=np.array(
                [inputs.get("left_thumb_x", 0.0), inputs.get("left_thumb_y", 0.0)]),
            right_ctrl_thumbstickValue=np.array(
                [inputs.get("right_thumb_x", 0.0), inputs.get("right_thumb_y", 0.0)]),
        )

    def send(self, message: dict) -> bool:
        kind = message.get("t")
        if not kind:
            return False
        payload = {k: v for k, v in message.items() if k != "t"}
        return self._server.send(kind, **payload)

    def render_to_xr(self, image) -> None:
        """Video reaches the headset over WebRTC from PC2, not over XrLink --
        the link stays small and on the control path only."""

    def close(self) -> None:
        self._server.stop()
