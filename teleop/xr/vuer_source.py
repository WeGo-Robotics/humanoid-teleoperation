"""`XRSource` over the Vuer browser transport.

Adapts what ships today. It is the fallback once the native app lands, and the
reference implementation for the interface in the meantime -- the align gate and
everything in `teleop.safety` work through it unchanged.
"""
from __future__ import annotations

from dataclasses import asdict

import numpy as np

try:
    from ..safety.types import XRLiveness
except (ImportError, ValueError):
    # 'beyond top-level package' raises ValueError, not ImportError, when
    # teleop/ itself is the sys.path root rather than its parent.
    from safety.types import XRLiveness
from .source import XRSource
from .types import XRFrame

try:
    import logging_mp
    logger_mp = logging_mp.get_logger(__name__)
except ImportError:      # pragma: no cover - robot-side dependency
    import logging
    logger_mp = logging.getLogger(__name__)

_IDENTITY = np.eye(4)


class VuerXRSource(XRSource):
    name = "vuer"

    def __init__(self, tv_wrapper, use_hand_tracking: bool):
        self._tv = tv_wrapper
        self._hand = use_hand_tracking
        self._warned_send = False

    def read(self) -> XRFrame:
        data = self._tv.get_tele_data()
        # Paired, not sampled separately: the safety layer must judge the
        # freshness of the exact payload it is about to act on.
        liveness = XRLiveness(**asdict(self._tv.get_link_status()))
        return XRFrame(
            liveness=liveness,
            head_pose=data.head_pose,
            left_wrist_pose=data.left_wrist_pose,
            right_wrist_pose=data.right_wrist_pose,
            left_hand_pos=data.left_hand_pos if self._hand else None,
            right_hand_pos=data.right_hand_pos if self._hand else None,
            left_hand_pinch=bool(data.left_hand_pinch),
            right_hand_pinch=bool(data.right_hand_pinch),
            left_hand_pinchValue=float(data.left_hand_pinchValue),
            right_hand_pinchValue=float(data.right_hand_pinchValue),
            left_ctrl_trigger=bool(data.left_ctrl_trigger),
            right_ctrl_trigger=bool(data.right_ctrl_trigger),
            left_ctrl_triggerValue=float(data.left_ctrl_triggerValue),
            right_ctrl_triggerValue=float(data.right_ctrl_triggerValue),
            left_ctrl_aButton=bool(data.left_ctrl_aButton),
            right_ctrl_aButton=bool(data.right_ctrl_aButton),
            left_ctrl_thumbstick=bool(data.left_ctrl_thumbstick),
            right_ctrl_thumbstick=bool(data.right_ctrl_thumbstick),
            left_ctrl_thumbstickValue=np.asarray(data.left_ctrl_thumbstickValue),
            right_ctrl_thumbstickValue=np.asarray(data.right_ctrl_thumbstickValue),
        )

    def send(self, message: dict) -> bool:
        """Not supported: the browser transport is receive-only.

        This is the concrete reason the align prompt cannot be shown *in* the
        headset on this transport -- the operator reads it off the dashboard
        instead, and confirms with the two-handed gesture. The native source
        implements this properly.
        """
        if not self._warned_send:
            self._warned_send = True
            logger_mp.warning(
                "[VuerXRSource] host-to-device messaging is unavailable on the "
                "browser transport; align prompts show on the dashboard only")
        return False

    def render_to_xr(self, image) -> None:
        self._tv.render_to_xr(image)

    def close(self) -> None:
        self._tv.close()
