"""XrLink server: the host end of the native-app transport.

An asyncio `websockets` server on its own daemon thread, so the 30 Hz control
loop never awaits anything. The loop reads a lock-protected snapshot; the server
thread writes it.

**Pose convention on the wire.** Protocol v2 carries *raw OpenXR* poses (y up,
z back, x right) exactly as the device's XR nodes report them. The whole
OpenXR->robot chain runs on the host in `transforms.py`, so the safety-relevant
geometry exists in one reviewed implementation rather than one per client. This
server does not transform anything; `NativeXRSource` does, on the way out.

Single operator by design: a second connection **supersedes** the first rather
than being rejected, for the same reason Vuer sessions do. A headset that
reconnects after a Wi-Fi blip must not be locked out by the corpse of its own
previous connection.
"""
from __future__ import annotations

import asyncio
import threading
import time
from dataclasses import dataclass, replace
from typing import Optional, Tuple

import numpy as np

from .codec import (BUTTON_NAMES, CodecError, TrackingFrame, decode_control,
                    decode_tracking, encode_control)

try:
    import logging_mp
    logger_mp = logging_mp.get_logger(__name__)
except ImportError:      # pragma: no cover - robot-side dependency
    import logging
    logger_mp = logging.getLogger(__name__)


@dataclass(frozen=True)
class LinkSnapshot:
    """Everything the control loop needs, captured atomically."""
    connected: bool = False
    frame: Optional[TrackingFrame] = None
    rx_monotonic: float = 0.0        # host clock when the frame landed
    seq: int = 0
    dropped: int = 0                 # frames missing from the sequence
    decode_errors: int = 0
    worn: Optional[bool] = None
    battery: Optional[float] = None
    device: str = ""
    estop_requested: bool = False
    #: Currently-held buttons, from the control channel. Level-triggered, not
    #: edge-triggered: the device re-sends the full held set whenever it
    #: changes, so a dropped message self-corrects on the next press or
    #: release instead of leaving a button stuck down forever.
    buttons: Tuple[str, ...] = ()


class XrLinkServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 8443,
                 ssl_context=None, on_estop=None):
        """`ssl_context` should be a real one in production; tests use plain ws.

        `on_estop` is invoked from the server thread when the device requests an
        emergency stop, so it must be cheap and thread-safe.
        """
        self._host = host
        self._port = port
        self._ssl = ssl_context
        self._on_estop = on_estop

        self._lock = threading.Lock()
        self._snap = LinkSnapshot()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._server = None
        self._client = None
        self._ready = threading.Event()
        self._running = False

        # Head-camera video. Counters are plain ints touched only from the
        # caller's thread and the loop's done-callback, both of which run one
        # at a time per frame, so they need no lock -- they are diagnostics,
        # not control.
        self.video_sent = 0
        self.video_dropped = 0
        self._video_inflight = 0

    #: Frames allowed on the wire at once before new ones are dropped. One in
    #: flight plus one queued: enough to keep the link busy, few enough that a
    #: slow socket cannot build a backlog the operator would see as lag.
    _VIDEO_MAX_INFLIGHT = 2

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def start(self, timeout: float = 5.0) -> bool:
        if self._running:
            return True
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="xrlink")
        self._thread.start()
        if not self._ready.wait(timeout):
            logger_mp.error("[XrLink] server failed to start within %.1fs", timeout)
            return False
        return True

    def _run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        try:
            loop.run_until_complete(self._serve())
        except Exception as e:
            logger_mp.error(f"[XrLink] server loop failed: {e!r}")
            self._ready.set()      # unblock start(), which will report failure
        finally:
            try:
                loop.run_until_complete(loop.shutdown_asyncgens())
            except Exception:
                pass
            loop.close()

    async def _serve(self):
        import websockets
        self._stop_future = asyncio.get_event_loop().create_future()
        self._server = await websockets.serve(
            self._handle, self._host, self._port, ssl=self._ssl,
            ping_interval=5, ping_timeout=5, max_queue=4)
        self.port = self._actual_port()
        logger_mp.info(f"[XrLink] listening on {self._host}:{self.port}")
        self._ready.set()
        await self._stop_future
        self._server.close()
        await self._server.wait_closed()

    def _actual_port(self):
        try:
            return self._server.sockets[0].getsockname()[1]
        except Exception:
            return self._port

    def stop(self):
        self._running = False
        loop, fut = self._loop, getattr(self, "_stop_future", None)
        if loop is not None and fut is not None:
            loop.call_soon_threadsafe(lambda: fut.done() or fut.set_result(None))
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        with self._lock:
            self._snap = LinkSnapshot()

    # ------------------------------------------------------------------
    # connection handling
    # ------------------------------------------------------------------
    async def _handle(self, ws, path=None):
        previous, self._client = self._client, ws
        if previous is not None:
            logger_mp.warning("[XrLink] superseding previous connection")
            try:
                await previous.close(code=1012, reason="superseded")
            except Exception:
                pass

        peer = getattr(ws, "remote_address", None)
        logger_mp.info(f"[XrLink] device connected: {peer}")
        with self._lock:
            self._snap = LinkSnapshot(connected=True)

        try:
            async for message in ws:
                self._on_message(message)
        except Exception as e:
            logger_mp.warning(f"[XrLink] connection error: {e!r}")
        finally:
            logger_mp.warning(f"[XrLink] device disconnected: {peer}")
            if self._client is ws:
                self._client = None
                # session_up goes false immediately. The safety layer treats
                # that as terminal, which is the whole point.
                with self._lock:
                    # Buttons are level-triggered, so a set held at the moment
                    # the link dropped would otherwise stay held forever. The
                    # safety layer stops the robot on disconnect regardless;
                    # this just refuses to keep a stale press around for
                    # whatever reconnects next.
                    self._snap = replace(self._snap, connected=False, worn=None,
                                         buttons=())

    def _on_message(self, message):
        if isinstance(message, (bytes, bytearray)):
            self._on_tracking(bytes(message))
        else:
            self._on_control(message)

    def _on_tracking(self, payload: bytes):
        try:
            frame = decode_tracking(payload)
        except CodecError as e:
            with self._lock:
                self._snap = replace(self._snap,
                                     decode_errors=self._snap.decode_errors + 1)
            # Rate-limited: a version mismatch would otherwise log at 90Hz.
            if self._snap.decode_errors % 100 == 1:
                logger_mp.error(f"[XrLink] bad tracking frame: {e}")
            return

        now = time.monotonic()
        with self._lock:
            prev = self._snap
            dropped = prev.dropped
            if prev.frame is not None:
                gap = (frame.seq - prev.seq) & 0xFFFFFFFF
                if 1 < gap < 1000:          # ignore wrap and reconnect jumps
                    dropped += gap - 1
            self._snap = replace(prev, frame=frame, rx_monotonic=now,
                                 seq=frame.seq, dropped=dropped,
                                 worn=frame.worn)

    def _on_control(self, text):
        try:
            msg = decode_control(text)
        except CodecError as e:
            logger_mp.error(f"[XrLink] bad control message: {e}")
            return

        kind = msg.get("t")
        unknown: Tuple[str, ...] = ()
        with self._lock:
            if kind == "hello":
                self._snap = replace(self._snap, device=str(msg.get("dev", "")))
                logger_mp.info(f"[XrLink] hello from {msg.get('dev')} "
                               f"proto={msg.get('proto')}")
            elif kind == "presence":
                self._snap = replace(self._snap, worn=bool(msg.get("worn")))
            elif kind == "buttons":
                pressed = msg.get("pressed") or ()
                if not isinstance(pressed, (list, tuple)):
                    unknown, pressed = (), ()
                else:
                    unknown = tuple(b for b in pressed if b not in BUTTON_NAMES)
                    pressed = tuple(b for b in pressed if b in BUTTON_NAMES)
                self._snap = replace(self._snap, buttons=pressed)
            elif kind == "status":
                battery = msg.get("battery")
                self._snap = replace(
                    self._snap,
                    battery=float(battery) if battery is not None else None)
            elif kind == "estop":
                self._snap = replace(self._snap, estop_requested=True)

        if unknown:
            # Not fatal -- the known buttons in the same message were kept -- but
            # it means the app and the host disagree about the vocabulary, and
            # some control the operator is pressing is doing nothing.
            logger_mp.warning(f"[XrLink] ignoring unknown buttons: {unknown}")

        if kind == "estop":
            logger_mp.error("[XrLink] device requested EMERGENCY STOP")
            if callable(self._on_estop):
                try:
                    self._on_estop()
                except Exception as e:
                    logger_mp.error(f"[XrLink] estop callback failed: {e!r}")

    # ------------------------------------------------------------------
    # control-loop facing API (thread-safe)
    # ------------------------------------------------------------------
    def snapshot(self) -> LinkSnapshot:
        with self._lock:
            return self._snap

    def clear_estop(self):
        with self._lock:
            self._snap = replace(self._snap, estop_requested=False)

    def send(self, kind: str, **payload) -> bool:
        """Queue a host->device control message. Returns False if undeliverable."""
        ws, loop = self._client, self._loop
        if ws is None or loop is None:
            return False
        try:
            text = encode_control(kind, **payload)
        except CodecError as e:
            logger_mp.error(f"[XrLink] refusing to send: {e}")
            return False
        try:
            asyncio.run_coroutine_threadsafe(self._send(ws, text), loop)
        except Exception as e:
            logger_mp.warning(f"[XrLink] send failed: {e!r}")
            return False
        return True

    def send_video(self, jpeg: bytes) -> bool:
        """Queue one head-camera frame to the device, as a binary frame.

        Binary rather than a base64 field on a control message: at 30 fps a
        third more bytes on every frame is real bandwidth, and it keeps video
        off the same path as the messages that stop the robot. The device
        tells the two apart by opcode.

        Frames are dropped rather than queued when the socket is behind. A
        late video frame has no value -- the next one is already better -- and
        letting them accumulate would put latency on the control channel they
        share.
        """
        ws, loop = self._client, self._loop
        if ws is None or loop is None or not jpeg:
            return False

        if self._video_inflight >= self._VIDEO_MAX_INFLIGHT:
            self.video_dropped += 1
            return False

        self._video_inflight += 1

        def _done(_):
            self._video_inflight -= 1

        try:
            fut = asyncio.run_coroutine_threadsafe(self._send_bytes(ws, jpeg), loop)
            fut.add_done_callback(_done)
        except Exception as e:
            self._video_inflight -= 1
            logger_mp.warning(f"[XrLink] video send failed: {e!r}")
            return False

        self.video_sent += 1
        return True

    @staticmethod
    async def _send_bytes(ws, payload: bytes):
        try:
            await ws.send(payload)
        except Exception as e:
            logger_mp.debug(f"[XrLink] video frame dropped: {e!r}")

    @staticmethod
    async def _send(ws, text):
        try:
            await ws.send(text)
        except Exception as e:
            logger_mp.debug(f"[XrLink] send dropped: {e!r}")
