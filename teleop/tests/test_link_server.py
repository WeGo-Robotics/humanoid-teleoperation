"""End-to-end tests for the XrLink server against a real WebSocket client.

Plain ws:// over loopback -- TLS is an `ssl_context` the caller supplies and is
orthogonal to everything under test here.

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import asyncio
import contextlib
import json
import os
import sys
import threading
import time
import types
import unittest

import numpy as np

if "logging_mp" not in sys.modules:
    _m = types.ModuleType("logging_mp")
    _m.get_logger = lambda *a, **kw: types.SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
        error=lambda *a, **k: None, debug=lambda *a, **k: None)
    _m.basic_config = lambda *a, **kw: None
    _m.INFO = 20
    sys.modules["logging_mp"] = _m

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import websockets  # noqa: E402

from xr.codec import INPUT_FIELDS, TrackingFrame, encode_tracking  # noqa: E402
from xr.link_server import XrLinkServer  # noqa: E402
from xr.native_source import NativeXRSource  # noqa: E402


def pose(x=0.0, y=0.0, z=0.0):
    m = np.eye(4)
    m[0:3, 3] = (x, y, z)
    return m


def tracking(seq=1, worn=True, hand_mode=False, **kw):
    defaults = dict(
        seq=seq, t_device=float(seq) / 90.0, worn=worn, left_tracked=True,
        right_tracked=True, hand_mode=hand_mode,
        # raw OpenXR: y up, z back. Head 1.6m up, wrists 30cm in front.
        head=pose(0.0, 1.6, 0.0),
        left_wrist=pose(0.2, 1.5, -0.3), right_wrist=pose(-0.2, 1.5, -0.3),
        inputs={k: 0.0 for k in INPUT_FIELDS},
        left_joints=np.zeros((25, 3)) if hand_mode else None,
        right_joints=np.zeros((25, 3)) if hand_mode else None,
    )
    defaults.update(kw)
    return encode_tracking(TrackingFrame(**defaults))


def wait_until(predicate, timeout=5.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class Device:
    """A minimal Quest stand-in: its own asyncio loop on its own thread."""

    def __init__(self, port):
        self._url = f"ws://127.0.0.1:{port}"
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        self._ws = None
        self.received = []

    def _run(self, coro, timeout=5.0):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout)

    def connect(self):
        async def go():
            self._ws = await websockets.connect(self._url)
            asyncio.ensure_future(self._reader())
        self._run(go())
        return self

    async def _reader(self):
        try:
            async for msg in self._ws:
                self.received.append(json.loads(msg))
        except Exception:
            pass

    def send_bytes(self, payload):
        self._run(self._ws.send(payload))

    def send_json(self, **msg):
        self._run(self._ws.send(json.dumps(msg)))

    def close(self):
        if getattr(self, "_closed", False):
            return
        self._closed = True
        if self._ws is not None and self._loop.is_running():
            with contextlib.suppress(Exception):
                self._run(self._ws.close())
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)
        with contextlib.suppress(Exception):
            self._loop.close()


class LinkFixture(unittest.TestCase):
    def setUp(self):
        self.estops = []
        self.server = XrLinkServer(host="127.0.0.1", port=0,
                                   on_estop=lambda: self.estops.append(time.time()))
        self.assertTrue(self.server.start(), "server did not start")
        self.source = NativeXRSource(self.server)
        self.devices = []

    def tearDown(self):
        for d in self.devices:
            with contextlib.suppress(Exception):
                d.close()
        with contextlib.suppress(Exception):
            self.server.stop()

    def device(self):
        d = Device(self.server.port).connect()
        self.devices.append(d)
        self.assertTrue(wait_until(lambda: self.server.snapshot().connected))
        return d


class TestConnection(LinkFixture):
    def test_no_client_means_session_down(self):
        frame = self.source.read()
        self.assertFalse(frame.liveness.session_up)
        self.assertIsNone(frame.liveness.worn)

    def test_connect_marks_session_up(self):
        self.device()
        self.assertTrue(self.source.read().liveness.session_up)

    def test_disconnect_marks_session_down_immediately(self):
        d = self.device()
        d.send_bytes(tracking(seq=1))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.seq == 1))
        d.close()
        self.assertTrue(wait_until(
            lambda: not self.source.read().liveness.session_up))

    def test_reconnect_supersedes_rather_than_being_rejected(self):
        """A headset returning from a Wi-Fi blip must not be locked out by its
        own previous connection."""
        first = self.device()
        second = self.device()
        second.send_bytes(tracking(seq=7))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.seq == 7))
        self.assertTrue(self.source.read().liveness.session_up)
        del first


class TestTracking(LinkFixture):
    def test_frame_reaches_the_control_loop(self):
        d = self.device()
        d.send_bytes(tracking(seq=3, worn=True))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.seq == 3))

        frame = self.source.read()
        self.assertTrue(frame.liveness.worn)
        self.assertTrue(frame.liveness.left_tracked)
        # The host applies the OpenXR->robot transform (protocol v2 carries raw
        # OpenXR). Head 1.6m up becomes robot +z; the wrist 30cm in front and
        # 10cm below the head becomes +x, then picks up the waist offsets.
        np.testing.assert_allclose(frame.head_pose[0:3, 3], [0, 0, 1.6], atol=1e-6)
        np.testing.assert_allclose(frame.left_wrist_pose[0:3, 3],
                                   [0.3 + 0.15, -0.2, -0.1 + 0.45], atol=1e-6)

    def test_hand_joints_arrive(self):
        d = self.device()
        joints = np.arange(75, dtype=float).reshape(25, 3)
        d.send_bytes(tracking(seq=1, hand_mode=True, left_joints=joints,
                              right_joints=joints))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.seq == 1))
        np.testing.assert_allclose(self.source.read().left_hand_pos, joints, atol=1e-3)

    def test_worn_false_reaches_liveness(self):
        """The fast doff path: a measured flag rather than inferred silence."""
        d = self.device()
        d.send_bytes(tracking(seq=1, worn=True))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.worn is True))
        d.send_bytes(tracking(seq=2, worn=False))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.worn is False))

    def test_rx_timestamp_advances(self):
        d = self.device()
        d.send_bytes(tracking(seq=1))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.seq == 1))
        first = self.source.read().liveness.last_rx
        time.sleep(0.05)
        d.send_bytes(tracking(seq=2))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.seq == 2))
        self.assertGreater(self.source.read().liveness.last_rx, first)

    def test_dropped_frames_are_counted(self):
        d = self.device()
        for seq in (1, 2, 6):
            d.send_bytes(tracking(seq=seq))
        self.assertTrue(wait_until(lambda: self.server.snapshot().seq == 6))
        self.assertEqual(self.server.snapshot().dropped, 3)

    def test_a_malformed_frame_does_not_disturb_good_state(self):
        d = self.device()
        d.send_bytes(tracking(seq=5))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.seq == 5))

        d.send_bytes(b"\x00\x01\x02 garbage")
        self.assertTrue(wait_until(lambda: self.server.snapshot().decode_errors == 1))

        # The last good pose must survive, and the link must stay up.
        frame = self.source.read()
        self.assertEqual(frame.liveness.seq, 5)
        self.assertTrue(frame.liveness.session_up)
        np.testing.assert_allclose(frame.left_wrist_pose[0:3, 3],
                                   [0.3 + 0.15, -0.2, -0.1 + 0.45], atol=1e-6)

        d.send_bytes(tracking(seq=6))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.seq == 6))


class TestControlChannel(LinkFixture):
    def test_presence_message_updates_worn(self):
        d = self.device()
        d.send_json(t="presence", worn=False)
        self.assertTrue(wait_until(lambda: self.server.snapshot().worn is False))

    def test_hello_records_the_device(self):
        d = self.device()
        d.send_json(t="hello", proto=1, dev="quest3", session="abc")
        self.assertTrue(wait_until(lambda: self.server.snapshot().device == "quest3"))

    def test_status_records_battery(self):
        d = self.device()
        d.send_json(t="status", battery=0.72)
        self.assertTrue(wait_until(
            lambda: self.server.snapshot().battery == 0.72))

    def test_estop_fires_the_callback(self):
        d = self.device()
        d.send_json(t="estop")
        self.assertTrue(wait_until(lambda: len(self.estops) == 1))
        self.assertTrue(self.server.snapshot().estop_requested)
        self.server.clear_estop()
        self.assertFalse(self.server.snapshot().estop_requested)

    def test_unknown_control_message_is_ignored_not_fatal(self):
        d = self.device()
        d.send_json(t="nonsense", foo=1)
        d.send_json(t="presence", worn=True)
        self.assertTrue(wait_until(lambda: self.server.snapshot().worn is True))

    def test_host_can_push_a_prompt(self):
        d = self.device()
        self.assertTrue(self.source.send({
            "t": "prompt_align",
            "target": {"left": [1, 2], "right": [3, 4]},
            "tol": {"pos_m": 0.08, "rot_deg": 15},
        }))
        self.assertTrue(wait_until(lambda: any(
            m.get("t") == "prompt_align" for m in d.received)))
        msg = [m for m in d.received if m["t"] == "prompt_align"][0]
        self.assertEqual(msg["tol"]["pos_m"], 0.08)

    def test_send_without_a_client_reports_failure(self):
        self.assertFalse(self.source.send({"t": "state", "session": "READY"}))

    def test_host_refuses_to_send_a_device_message(self):
        self.device()
        self.assertFalse(self.source.send({"t": "presence", "worn": True}))


class TestButtons(LinkFixture):
    """Buttons ride the control channel, but two of them stop the robot:
    right_a quits the session and both thumbstick clicks damp the base."""

    def armed_device(self):
        d = self.device()
        d.send_bytes(tracking(seq=1))
        self.assertTrue(wait_until(lambda: self.source.read().liveness.seq == 1))
        return d

    def test_pressed_buttons_reach_the_control_loop(self):
        d = self.armed_device()
        d.send_json(t="buttons", pressed=["right_a", "left_thumb"])
        self.assertTrue(wait_until(lambda: self.source.read().right_ctrl_aButton))

        frame = self.source.read()
        self.assertTrue(frame.left_ctrl_thumbstick)
        self.assertFalse(frame.right_ctrl_thumbstick)
        self.assertFalse(frame.left_ctrl_aButton)

    def test_release_clears_them(self):
        """Level-triggered: an empty set is how the device says 'released'."""
        d = self.armed_device()
        d.send_json(t="buttons", pressed=["right_a"])
        self.assertTrue(wait_until(lambda: self.source.read().right_ctrl_aButton))
        d.send_json(t="buttons", pressed=[])
        self.assertTrue(wait_until(
            lambda: not self.source.read().right_ctrl_aButton))

    def test_unknown_names_are_dropped_but_known_ones_survive(self):
        d = self.armed_device()
        d.send_json(t="buttons", pressed=["right_a", "left_grip", "menu"])
        self.assertTrue(wait_until(lambda: self.source.read().right_ctrl_aButton))
        self.assertEqual(self.server.snapshot().buttons, ("right_a",))

    def test_a_malformed_pressed_field_clears_rather_than_crashes(self):
        d = self.armed_device()
        d.send_json(t="buttons", pressed=["right_a"])
        self.assertTrue(wait_until(lambda: self.source.read().right_ctrl_aButton))
        d.send_json(t="buttons", pressed="right_a")     # a string, not a list
        self.assertTrue(wait_until(
            lambda: not self.source.read().right_ctrl_aButton))
        self.assertTrue(self.source.read().liveness.session_up)

    def test_disconnect_does_not_leave_a_button_held(self):
        d = self.armed_device()
        d.send_json(t="buttons", pressed=["left_thumb", "right_thumb"])
        self.assertTrue(wait_until(lambda: self.source.read().left_ctrl_thumbstick))
        d.close()
        self.assertTrue(wait_until(
            lambda: not self.source.read().liveness.session_up))
        self.assertEqual(self.server.snapshot().buttons, ())

    def test_buttons_before_any_pose_are_harmless(self):
        d = self.device()
        d.send_json(t="buttons", pressed=["right_a"])
        self.assertTrue(wait_until(
            lambda: self.server.snapshot().buttons == ("right_a",)))
        # No tracking frame yet, so there is nothing to act on and the frame
        # reports untracked rather than a press against identity poses.
        frame = self.source.read()
        self.assertFalse(frame.right_ctrl_aButton)
        self.assertFalse(frame.liveness.left_tracked)


class TestBindFailureIsReported(unittest.TestCase):
    """start() used to return True when the listen failed.

    The server thread sets its ready event on the failure path as well as the
    success path, so waiting on that alone said "running" for a server that had
    never bound. The commonest cause is the port already being held -- another
    host still up, or a previous one not yet torn down -- and the symptom was a
    host sitting there waiting for a device that could never arrive, which
    reads as a headset problem and is not. Found while setting up the XR
    Simulator host, where two hosts on one machine is the normal way to trip
    it.
    """

    def test_second_server_on_the_same_port_fails_to_start(self):
        first = XrLinkServer(host="127.0.0.1", port=0)
        self.assertTrue(first.start())
        try:
            second = XrLinkServer(host="127.0.0.1", port=first.port)
            try:
                self.assertFalse(
                    second.start(timeout=3.0),
                    "a server that could not bind reported success")
            finally:
                second.stop()
        finally:
            first.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)
