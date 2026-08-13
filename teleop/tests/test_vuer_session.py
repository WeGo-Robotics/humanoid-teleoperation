"""Tests for TeleVuer's XR session lifecycle.

`vuer` is stubbed so the real module (and a live server) are not needed -- only
the session-tracking wrapper is under test, and it is pure asyncio.

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import asyncio
import os
import sys
import types
import unittest
from multiprocessing import Value


def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    return m


if "logging_mp" not in sys.modules:
    sys.modules["logging_mp"] = _stub(
        "logging_mp",
        get_logger=lambda *a, **kw: types.SimpleNamespace(
            info=lambda *a, **k: None, warning=lambda *a, **k: None,
            error=lambda *a, **k: None, debug=lambda *a, **k: None),
        basic_config=lambda *a, **kw: None, INFO=20)

if "vuer" not in sys.modules:
    sys.modules["vuer"] = _stub("vuer", Vuer=object)
    sys.modules["vuer.schemas"] = _stub(
        "vuer.schemas", ImageBackground=object, Hands=object,
        MotionControllers=object, WebRTCVideoPlane=object,
        WebRTCStereoVideoPlane=object)

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "televuer", "src"))

from televuer.televuer import TeleVuer  # noqa: E402


class FakeVuer:
    """Just enough of TeleVuer for the session wrapper, without spawning
    a Vuer server process."""
    _with_session_tracking = TeleVuer._with_session_tracking

    def __init__(self):
        self.xr_session_count_shared = Value('i', 0)

    @property
    def sessions(self):
        return self.xr_session_count_shared.value


def run(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class TestSessionSupersession(unittest.TestCase):
    """Every main_* body is an infinite `while True: session.upsert(...)` on
    fixed bgChildren keys. Nothing used to tear the old one down, so each
    reconnect stacked another render loop against the same keys."""

    def setUp(self):
        self.fake = FakeVuer()
        self.started = []
        self.cancelled = []

    def _body(self):
        async def body(session):
            self.started.append(session)
            try:
                while True:
                    await asyncio.sleep(0.005)
            except asyncio.CancelledError:
                self.cancelled.append(session)
                raise
        return body

    def test_reconnect_cancels_the_previous_session(self):
        async def scenario():
            wrapped = self.fake._with_session_tracking(self._body())
            first = asyncio.ensure_future(wrapped("s1"))
            await asyncio.sleep(0.05)
            self.assertEqual(self.fake.sessions, 1)

            second = asyncio.ensure_future(wrapped("s2"))   # reconnect
            await asyncio.sleep(0.05)

            self.assertEqual(self.started, ["s1", "s2"])
            self.assertEqual(self.cancelled, ["s1"],
                             "the superseded session kept running")
            self.assertEqual(self.fake.sessions, 1,
                             "exactly one session must remain attached")

            second.cancel()
            await asyncio.gather(first, second, return_exceptions=True)
            await asyncio.sleep(0.05)
            self.assertEqual(self.fake.sessions, 0)
        run(scenario())

    def test_many_reconnects_leave_one_session(self):
        async def scenario():
            wrapped = self.fake._with_session_tracking(self._body())
            tasks = []
            for i in range(5):
                tasks.append(asyncio.ensure_future(wrapped(f"s{i}")))
                await asyncio.sleep(0.02)
            self.assertEqual(self.fake.sessions, 1)
            self.assertEqual(self.cancelled, ["s0", "s1", "s2", "s3"])
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            await asyncio.sleep(0.05)
            self.assertEqual(self.fake.sessions, 0)
        run(scenario())

    def test_session_count_returns_to_zero_on_normal_exit(self):
        async def scenario():
            async def body(session):
                await asyncio.sleep(0.01)
            wrapped = self.fake._with_session_tracking(body)
            await wrapped("s1")
            self.assertEqual(self.fake.sessions, 0)
        run(scenario())

    def test_a_failing_session_still_detaches(self):
        async def scenario():
            async def body(session):
                raise RuntimeError("handler blew up")
            wrapped = self.fake._with_session_tracking(body)
            await wrapped("s1")          # must not propagate
            self.assertEqual(self.fake.sessions, 0)
        run(scenario())

    def test_cancelling_the_wrapper_does_not_orphan_the_body(self):
        async def scenario():
            wrapped = self.fake._with_session_tracking(self._body())
            task = asyncio.ensure_future(wrapped("s1"))
            await asyncio.sleep(0.05)
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            await asyncio.sleep(0.05)
            self.assertEqual(self.cancelled, ["s1"],
                             "body kept upserting after the session ended")
            self.assertEqual(self.fake.sessions, 0)
        run(scenario())


if __name__ == "__main__":
    unittest.main(verbosity=2)
