"""Head-camera frames from the image client to the XR device.

The browser transport already sent the robot's head camera to the headset;
the native XrLink transport never did, so the Quest client's centre panel had
nothing live to show. This closes that gap without adding a port, a protocol
or a dependency: frames go out as binary messages on the WebSocket the device
has already connected.

Everything here is deliberately lossy. Video is the one thing on this link
that must never delay anything else, because the same connection carries the
messages that stop the robot. So:

  * frames are read from the image client's ring buffer, which already keeps
    only the most recent one,
  * the pump skips rather than waits when the socket is busy,
  * and it downsizes and re-encodes rather than forwarding whatever the
    camera produced, because a 1080p JPEG is several hundred kilobytes and
    the panel it lands on is a few hundred pixels across.

If the encode or the send fails, the frame is dropped and the loop continues.
A stalled video feed is a degraded session; a stalled control channel is a
runaway robot.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import cv2
import numpy as np

try:
    import logging_mp
    logger_mp = logging_mp.get_logger(__name__)
except ImportError:      # pragma: no cover - robot-side dependency
    import logging
    logger_mp = logging.getLogger(__name__)


class VideoPump:
    """Background thread pushing head-camera JPEGs to the XR device."""

    def __init__(self,
                 get_frame: Callable[[], Optional[np.ndarray]],
                 send: Callable[[bytes], bool],
                 fps: float = 20.0,
                 width: int = 640,
                 quality: int = 70):
        """
        Args:
            get_frame: returns the newest BGR frame, or None. Must not
                block. ImageClient.get_head_frame returns (frame, fps), so
                wrap it rather than passing it directly.
            send: hands one encoded frame to the transport; returns False if
                it was not delivered.
            fps: upper bound on send rate. The camera's own rate wins if lower.
            width: frames wider than this are scaled down, preserving aspect.
            quality: JPEG quality, 1-100.
        """
        self._get_frame = get_frame
        self._send = send
        self._interval = 1.0 / max(fps, 1.0)
        self._width = int(width)
        self._params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self.sent = 0
        self.encode_failures = 0
        self.send_failures = 0

    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="xr-video",
                                        daemon=True)
        self._thread.start()
        logger_mp.info("[XrVideo] pump started")

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=timeout)

    # ------------------------------------------------------------------
    def _run(self) -> None:
        last_id = None
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                frame = self._get_frame()
                if frame is not None:
                    # The ring buffer hands back the same object until a new
                    # frame lands. Re-encoding it would burn CPU to send the
                    # device a picture it already has.
                    ident = id(frame), frame.shape
                    if ident != last_id:
                        last_id = ident
                        self._encode_and_send(frame)
            except Exception as e:  # never let a bad frame kill the pump
                logger_mp.warning(f"[XrVideo] frame skipped: {e!r}")

            elapsed = time.monotonic() - started
            self._stop.wait(max(0.0, self._interval - elapsed))

    def _encode_and_send(self, frame: np.ndarray) -> None:
        if frame.ndim != 3 or frame.shape[2] != 3:
            self.encode_failures += 1
            return

        h, w = frame.shape[:2]
        if w > self._width:
            scale = self._width / float(w)
            frame = cv2.resize(frame, (self._width, max(1, int(round(h * scale)))),
                               interpolation=cv2.INTER_AREA)

        ok, buf = cv2.imencode(".jpg", frame, self._params)
        if not ok:
            self.encode_failures += 1
            return

        if self._send(buf.tobytes()):
            self.sent += 1
        else:
            self.send_failures += 1
