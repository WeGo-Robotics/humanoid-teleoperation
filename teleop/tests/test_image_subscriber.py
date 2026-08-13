"""Tests for the image subscriber's reconnect behaviour and manager lifecycle.

Uses real ZMQ PUB/SUB over loopback with a publisher the test can kill and
restart. `cv2` is stubbed if absent -- JPEG decoding is not what is under test.

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import contextlib
import os
import socket as pysocket
import sys
import threading
import time
import types
import unittest

import numpy as np

def _logging_stub():
    m = types.ModuleType("logging_mp")
    m.get_logger = lambda *a, **kw: types.SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
        error=lambda *a, **k: None, debug=lambda *a, **k: None)
    m.basic_config = lambda *a, **kw: None
    m.INFO = 20
    return m


def _cv2_stub():
    m = types.ModuleType("cv2")
    m.IMREAD_COLOR = 1
    # The subscriber only needs "bytes in, array out" to exercise its loop.
    m.imdecode = lambda buf, flag: np.asarray(buf, dtype=np.uint8).copy()
    m.imencode = lambda ext, img, params=None: (True, np.frombuffer(b"x", np.uint8))
    return m


def _yaml_stub():
    m = types.ModuleType("yaml")
    m.safe_load = lambda *a, **kw: {}
    m.YAMLError = Exception
    return m


# Robot-side dependencies that have nothing to do with the transport behaviour
# under test. Real ones are used when present.
for _name, _factory in (("logging_mp", _logging_stub), ("cv2", _cv2_stub),
                        ("yaml", _yaml_stub)):
    if _name not in sys.modules:
        try:
            __import__(_name)
        except ImportError:
            sys.modules[_name] = _factory()


import cv2  # noqa: E402  (real one if installed, stub otherwise)
import zmq  # noqa: E402


def jpeg_frame():
    """A real JPEG. The subscriber decodes with cv2.imdecode, which returns None
    for anything that is not a decodable image -- so junk bytes would look
    exactly like 'no frame' and the tests would prove nothing."""
    ok, buf = cv2.imencode(".jpg", np.zeros((8, 8, 3), dtype=np.uint8))
    assert ok
    return buf.tobytes()


sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "teleimager", "src"))

from teleimager.image_client import ZMQ_SubscriberManager  # noqa: E402


def free_port():
    with contextlib.closing(pysocket.socket()) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_until(predicate, timeout=8.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class Publisher:
    """A PUB socket pushing frames, which the test can stop and restart."""

    def __init__(self, port):
        self.port = port
        self._ctx = None
        self._sock = None
        self._stop = threading.Event()
        self._thread = None

    def start(self, payload=None):
        payload = jpeg_frame() if payload is None else payload
        self._stop.clear()
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 1)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(f"tcp://127.0.0.1:{self.port}")

        def loop():
            while not self._stop.is_set():
                with contextlib.suppress(Exception):
                    self._sock.send(payload)
                time.sleep(0.02)

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        with contextlib.suppress(Exception):
            self._ctx.destroy(linger=0)
        self._sock = None
        self._ctx = None


class SubscriberFixture(unittest.TestCase):
    def setUp(self):
        self.port = free_port()
        self.pub = Publisher(self.port)
        self.managers = []

    def tearDown(self):
        with contextlib.suppress(Exception):
            self.pub.stop()
        for m in self.managers:
            with contextlib.suppress(Exception):
                m.close()
        with contextlib.suppress(Exception):
            ZMQ_SubscriberManager._instance = None

    def manager(self):
        m = ZMQ_SubscriberManager.get_instance()
        self.managers.append(m)
        return m

    def got_frame(self, mgr):
        img, _fps = mgr.subscribe("127.0.0.1", self.port)
        return img is not None


class TestSubscriberReconnect(SubscriberFixture):
    def test_receives_frames(self):
        self.pub.start()
        mgr = self.manager()
        self.assertTrue(wait_until(lambda: self.got_frame(mgr)))

    def test_reports_no_frame_while_the_publisher_is_down(self):
        self.pub.start()
        mgr = self.manager()
        self.assertTrue(wait_until(lambda: self.got_frame(mgr)))
        self.pub.stop()
        self.assertTrue(wait_until(lambda: not self.got_frame(mgr)),
                        "kept serving a stale frame after the publisher died")

    def test_recovers_when_the_publisher_comes_back(self):
        """The whole point: a mid-session server restart must heal itself."""
        self.pub.start()
        mgr = self.manager()
        self.assertTrue(wait_until(lambda: self.got_frame(mgr)))

        self.pub.stop()
        self.assertTrue(wait_until(lambda: not self.got_frame(mgr)))

        self.pub.start()
        self.assertTrue(wait_until(lambda: self.got_frame(mgr), timeout=10.0),
                        "subscriber never recovered after the publisher restarted")

    def test_subscriber_thread_survives_a_recv_error(self):
        """Previously any recv/decode error killed the thread for good."""
        self.pub.start()
        mgr = self.manager()
        self.assertTrue(wait_until(lambda: self.got_frame(mgr)))

        thread = mgr._subscriber_threads[("127.0.0.1", self.port)]
        self.assertTrue(thread.is_alive())

        # Force the receive loop to raise from inside _recv_loop.
        original = thread._decode_image
        boom = {"count": 0}

        def exploding(_bytes):
            boom["count"] += 1
            raise RuntimeError("decode blew up")

        thread._decode_image = exploding
        self.assertTrue(wait_until(lambda: boom["count"] > 0))
        thread._decode_image = original

        self.assertTrue(thread.is_alive(), "subscriber thread died on an error")
        self.assertTrue(wait_until(lambda: self.got_frame(mgr), timeout=10.0),
                        "subscriber did not resume after an error")


class TestManagerLifecycle(SubscriberFixture):
    def test_a_new_client_works_after_close(self):
        """close() used to set _running=False on the class, so every later
        ImageClient in the process raised 'SubscriberManager is closed'."""
        self.pub.start()
        first = self.manager()
        self.assertTrue(wait_until(lambda: self.got_frame(first)))
        first.close()

        second = self.manager()
        self.assertIsNot(second, first, "closed manager was handed out again")
        self.assertTrue(wait_until(lambda: self.got_frame(second), timeout=10.0),
                        "second manager could not subscribe after the first closed")

    def test_close_is_idempotent(self):
        mgr = self.manager()
        mgr.close()
        mgr.close()

    def test_state_is_not_shared_between_instances(self):
        first = self.manager()
        first.close()
        second = self.manager()
        self.assertTrue(second._running)
        self.assertEqual(second._subscriber_threads, {})


if __name__ == "__main__":
    unittest.main(verbosity=2)
