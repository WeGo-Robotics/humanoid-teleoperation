"""Socket-level tests for the teleop IPC transport.

Real ZMQ sockets over loopback TCP (the production `ipc://@abstract` addresses
are Linux-only, and the behaviours under test are transport-independent).

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import contextlib
import json
import os
import socket as pysocket
import sys
import threading
import time
import types
import unittest

if "logging_mp" not in sys.modules:
    stub = types.ModuleType("logging_mp")
    stub.get_logger = lambda *a, **kw: types.SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
        error=lambda *a, **k: None, debug=lambda *a, **k: None)
    stub.basic_config = lambda *a, **kw: None
    stub.INFO = 20
    sys.modules["logging_mp"] = stub

import zmq  # noqa: E402

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "utils"))

from ipc import IPC_Client, IPC_Server  # noqa: E402

HB_FPS = 50.0          # fast heartbeat keeps the tests short
REQ_TIMEOUT_MS = 300


def free_port():
    with contextlib.closing(pysocket.socket()) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_until(predicate, timeout=3.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class ServerFixture(unittest.TestCase):
    """A real IPC_Server plus a raw REQ socket, so tests can send arbitrary
    bytes -- including frames a well-behaved IPC_Client would never emit."""

    def setUp(self):
        self.data_addr = f"tcp://127.0.0.1:{free_port()}"
        self.hb_addr = f"tcp://127.0.0.1:{free_port()}"
        self.pressed = []
        self.state = {"START": False, "READY": True}
        self.server = IPC_Server(
            on_press=self.pressed.append, get_state=lambda: self.state,
            hb_fps=HB_FPS, data_addr=self.data_addr, hb_addr=self.hb_addr)
        self.server.start()
        self._clients = []
        self._ctx = zmq.Context()

    def tearDown(self):
        for c in self._clients:
            with contextlib.suppress(Exception):
                c.stop()
        with contextlib.suppress(Exception):
            self.server.stop()
        with contextlib.suppress(Exception):
            self._ctx.destroy(linger=0)

    def client(self):
        c = IPC_Client(hb_fps=HB_FPS, data_addr=self.data_addr,
                       hb_addr=self.hb_addr, req_timeout_ms=REQ_TIMEOUT_MS)
        self._clients.append(c)
        return c

    def raw_req(self):
        s = self._ctx.socket(zmq.REQ)
        s.setsockopt(zmq.LINGER, 0)
        s.setsockopt(zmq.RCVTIMEO, 1500)
        s.setsockopt(zmq.REQ_RELAXED, 1)
        s.setsockopt(zmq.REQ_CORRELATE, 1)
        s.connect(self.data_addr)
        return s

    def send_raw(self, payload):
        """One raw request/reply exchange on a throwaway socket."""
        s = self.raw_req()
        try:
            s.send(payload if isinstance(payload, bytes) else payload.encode())
            return json.loads(s.recv().decode())
        finally:
            s.close(0)


class TestReplyContract(ServerFixture):
    def test_valid_command_is_dispatched(self):
        rep = self.send_raw(json.dumps({"reqid": "1", "cmd": "CMD_START"}))
        self.assertEqual(rep["status"], "ok")
        self.assertEqual(rep["repid"], "1")
        self.assertTrue(wait_until(lambda: self.pressed == ["r"]))

    def test_safety_commands_are_dispatched(self):
        for cmd, key in (("CMD_ESTOP", "e"), ("CMD_ACK_FAULT", "a")):
            rep = self.send_raw(json.dumps({"reqid": "x", "cmd": cmd}))
            self.assertEqual(rep["status"], "ok", cmd)
            self.assertTrue(wait_until(lambda k=key: k in self.pressed), cmd)

    def test_unknown_command_is_rejected(self):
        rep = self.send_raw(json.dumps({"reqid": "1", "cmd": "CMD_NOPE"}))
        self.assertEqual(rep["status"], "error")
        self.assertIn("not supported", rep["msg"])

    def test_missing_reqid_is_rejected(self):
        rep = self.send_raw(json.dumps({"cmd": "CMD_START"}))
        self.assertEqual(rep["status"], "error")
        self.assertIn("reqid", rep["msg"])


class TestMalformedInputDoesNotWedgeTheServer(ServerFixture):
    """Regression for the REP state-machine deadlock.

    recv_json() consumes the message and *then* raises on bad JSON, leaving the
    REP socket owing a reply it never sends. It will not deliver another request
    after that -- the command channel is deaf for the rest of the session,
    including to CMD_STOP. Each test here sends a bad frame and then asserts the
    server still answers a good one.
    """

    def _assert_still_serving(self):
        rep = self.send_raw(json.dumps({"reqid": "after", "cmd": "CMD_START"}))
        self.assertEqual(rep["status"], "ok",
                         "server stopped answering after a malformed request")
        self.assertEqual(rep["repid"], "after")

    def test_non_json_bytes(self):
        rep = self.send_raw(b"\x00\x01 not json at all")
        self.assertEqual(rep["status"], "error")
        self.assertIn("malformed", rep["msg"])
        self._assert_still_serving()

    def test_valid_json_that_is_not_an_object(self):
        rep = self.send_raw(json.dumps([1, 2, 3]))
        self.assertEqual(rep["status"], "error")
        self.assertIn("malformed", rep["msg"])
        self._assert_still_serving()

    def test_empty_frame(self):
        rep = self.send_raw(b"")
        self.assertEqual(rep["status"], "error")
        self._assert_still_serving()

    def test_several_bad_frames_in_a_row(self):
        for payload in (b"{", b"]", b"\xff\xfe", json.dumps("string")):
            rep = self.send_raw(payload)
            self.assertEqual(rep["status"], "error")
        self._assert_still_serving()


class TestHeartbeat(ServerFixture):
    def test_client_comes_online_and_sees_state(self):
        c = self.client()
        self.assertTrue(wait_until(c.is_online), "never came online")
        self.assertEqual(c.latest_state().get("READY"), True)

    def test_state_updates_propagate(self):
        c = self.client()
        self.assertTrue(wait_until(c.is_online))
        self.state = {"START": True, "READY": False, "XR": {"state": "following"}}
        self.assertTrue(wait_until(
            lambda: c.latest_state().get("XR", {}).get("state") == "following"))

    def test_goes_offline_when_the_server_stops(self):
        c = self.client()
        self.assertTrue(wait_until(c.is_online))
        self.server.stop()
        self.assertTrue(wait_until(lambda: not c.is_online(), timeout=3.0),
                        "client kept reporting online after the server died")
        self.assertEqual(c.latest_state(), {})

    def test_heartbeat_age_grows_after_the_server_stops(self):
        c = self.client()
        self.assertTrue(wait_until(c.is_online))
        self.assertLess(c.heartbeat_age(), 1.0)
        self.server.stop()
        self.assertTrue(wait_until(lambda: c.heartbeat_age() == float("inf"),
                                   timeout=3.0))


class TestClientCommands(ServerFixture):
    def test_send_data_round_trip(self):
        c = self.client()
        self.assertTrue(wait_until(c.is_online))
        rep = c.send_data("CMD_START")
        self.assertEqual(rep["status"], "ok")
        self.assertTrue(wait_until(lambda: "r" in self.pressed))

    def test_offline_send_is_refused_but_stop_can_force(self):
        c = IPC_Client(hb_fps=HB_FPS, data_addr=self.data_addr,
                       hb_addr=f"tcp://127.0.0.1:{free_port()}",  # no publisher
                       req_timeout_ms=REQ_TIMEOUT_MS)
        self._clients.append(c)
        self.assertEqual(c.send_data("CMD_START")["status"], "error")
        # A shutdown must still be attempted even with no heartbeat.
        rep = c.send_data("CMD_STOP", require_online=False)
        self.assertEqual(rep["status"], "ok")
        self.assertTrue(wait_until(lambda: "q" in self.pressed))


class TestRequestTimeoutDoesNotPoisonTheSocket(unittest.TestCase):
    """Regression for the REQ state-machine hazard.

    A plain REQ socket that times out waiting for a reply is stuck in "must
    recv"; the next send raises EFSM and the command channel is dead. REQ_RELAXED
    plus REQ_CORRELATE let the next request through and discard the stale reply.
    """

    def setUp(self):
        self.addr = f"tcp://127.0.0.1:{free_port()}"
        self.ctx = zmq.Context()
        self.rep = self.ctx.socket(zmq.REP)
        self.rep.setsockopt(zmq.LINGER, 0)
        self.rep.setsockopt(zmq.RCVTIMEO, 2000)
        self.rep.bind(self.addr)
        self.client = IPC_Client(hb_fps=HB_FPS, data_addr=self.addr,
                                 hb_addr=f"tcp://127.0.0.1:{free_port()}",
                                 req_timeout_ms=REQ_TIMEOUT_MS)
        self._responder_stop = None
        self._responder = None

    def tearDown(self):
        # Stop the responder *first*. unittest runs cleanups after tearDown, so
        # registering this with addCleanup would let the context be destroyed
        # while the thread is still blocked in rep.recv() -- libzmq aborts the
        # process when a socket is closed under a thread using it.
        if self._responder_stop is not None:
            self._responder_stop.set()
            self._responder.join(timeout=3.0)
        with contextlib.suppress(Exception):
            self.client.stop()
        with contextlib.suppress(Exception):
            self.rep.close(0)
        with contextlib.suppress(Exception):
            self.ctx.destroy(linger=0)

    def _serve_in_background(self):
        """Answer every request that arrives, until stopped."""
        stop = threading.Event()
        seen = []

        def responder():
            while not stop.is_set():
                try:
                    msg = json.loads(self.rep.recv().decode())
                except zmq.Again:
                    continue
                except Exception:
                    return
                seen.append(msg["cmd"])
                with contextlib.suppress(Exception):
                    self.rep.send_json({"repid": msg["reqid"], "status": "ok",
                                        "msg": "ok"})

        t = threading.Thread(target=responder, daemon=True)
        t.start()
        self._responder_stop, self._responder = stop, t   # torn down in tearDown
        return seen

    def test_next_command_works_after_a_timeout(self):
        # 1. a request nobody answers in time
        rep = self.client.send_data("CMD_START", require_online=False)
        self.assertEqual(rep["status"], "error")
        self.assertIn("timeout", rep["msg"])

        # 2. the server starts behaving. The abandoned request gets a late
        #    reply, which must not be mistaken for the answer to the next one.
        seen = self._serve_in_background()
        rep = self.client.send_data("CMD_STOP", require_online=False)
        self.assertEqual(rep["status"], "ok", f"channel dead after a timeout: {rep}")
        self.assertTrue(wait_until(lambda: "CMD_STOP" in seen))


if __name__ == "__main__":
    unittest.main(verbosity=2)
