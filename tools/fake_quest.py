#!/usr/bin/env python3
"""A Quest 3 stand-in that speaks XrLink.

Lets the whole host stack -- safety gate, align gate, dashboard -- be exercised
with no headset and no robot. Also the reference the real C# client is checked
against: if a scenario behaves differently here and on device, the device is
wrong.

    # host side
    python teleop/teleop_hand_and_arm.py --ipc --xr-source xrlink --sim
    # then
    python tools/fake_quest.py --scenario steady
    python tools/fake_quest.py --scenario doff        # the primary hazard
    python tools/fake_quest.py --scenario silent-doff # OS suspended first
    python tools/fake_quest.py --list

Scenarios map one-to-one onto the failure modes in
docs/xr_automation_and_safety_plan.md §4.2, so "does the robot actually stop"
can be answered before anyone puts a headset on.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "teleop"))

from xr.codec import INPUT_FIELDS, TrackingFrame, encode_tracking  # noqa: E402

RATE_HZ = 72.0

SCENARIOS = {
    "steady": "Stream normally, worn=true. The robot should follow.",
    "doff": "Stream, then send worn=false and keep streaming. Fast doff path.",
    "silent-doff": "Stream, then stop sending entirely, connection still open. "
                   "The OS-suspended-before-transmit case (docs §10.3).",
    "disconnect": "Stream, then drop the websocket. Terminal link loss.",
    "dropout": "Stream, go silent ~400ms, resume. Should HOLD then recover.",
    "freeze": "Keep streaming bit-identical payloads. Should trip FROZEN.",
    "jump": "Inject a 0.5m head transient. Should trip HEAD_JUMP.",
    "untracked": "Report both hands untracked while still streaming.",
    "estop": "Stream, then send an estop control message.",
    "confirm": "Stream with the confirm gesture held, for the align gate.",
    "align-skip": "Stream with the confirm gesture AND both face buttons held, "
                  "which is the operator waiving the position check. Should "
                  "reach 'accepted' after AlignConfig.hold_s. This is the exact "
                  "path that was dead in build 13 (docs §16.1) -- the app asked "
                  "for the grips, which the host cannot see, so the skip could "
                  "never engage.",
    "buttons": "Stream, then hold both thumbstick clicks (damp), release, "
               "then press right A (quit). Exercises the control-channel "
               "button path -- only visible with --motion on the host.",
}


def pose(x=0.0, y=0.0, z=0.0):
    """Raw OpenXR: y up, z back, x right (protocol v2 sends untransformed)."""
    m = np.eye(4)
    m[0:3, 3] = (x, y, z)
    return m


class FakeQuest:
    def __init__(self, url, hand_mode=False, confirm=False):
        self.url = url
        self.hand_mode = hand_mode
        self.confirm = confirm
        self.seq = 0
        self.worn = True
        self.tracked = True
        self.t0 = time.monotonic()
        self.head_offset = 0.0
        self._frozen = None

    # -- payload ---------------------------------------------------------
    def frame(self):
        t = time.monotonic() - self.t0
        # A little idle sway, so the payload is never bit-identical unless the
        # 'freeze' scenario asks for it.
        sway = 0.01 * math.sin(t * 1.5)
        head = pose(sway, 1.60 + self.head_offset, 0.0)
        left = pose(0.20 + sway, 1.50 + self.head_offset, -0.30)
        right = pose(-0.20 + sway, 1.50 + self.head_offset, -0.30)

        # Confirm gesture: both analog inputs driven to "fully pressed" (0.0),
        # matching televuer's inverted convention.
        pressed = 0.0 if self.confirm else 10.0
        inputs = {k: 0.0 for k in INPUT_FIELDS}
        inputs["left_trigger_value"] = pressed
        inputs["right_trigger_value"] = pressed
        inputs["left_pinch_value"] = pressed
        inputs["right_pinch_value"] = pressed

        self.seq += 1
        return TrackingFrame(
            seq=self.seq, t_device=t, worn=self.worn,
            left_tracked=self.tracked, right_tracked=self.tracked,
            hand_mode=self.hand_mode,
            head=head, left_wrist=left, right_wrist=right, inputs=inputs,
            left_joints=np.zeros((25, 3)) if self.hand_mode else None,
            right_joints=np.zeros((25, 3)) if self.hand_mode else None,
        )

    def payload(self):
        if self._frozen is not None:
            return self._frozen
        return encode_tracking(self.frame())

    def freeze(self):
        """Latch one payload and resend it verbatim."""
        self._frozen = encode_tracking(self.frame())

    # -- run -------------------------------------------------------------
    async def run(self, scenario, warmup=3.0, duration=15.0):
        import websockets
        print(f"[fake-quest] connecting to {self.url}")
        async with websockets.connect(self.url) as ws:
            await ws.send(json.dumps({
                "t": "hello", "proto": 2, "dev": "fake-quest",
                "session": f"sim-{int(time.time())}"}))
            await ws.send(json.dumps({"t": "presence", "worn": True,
                                      "focus": True, "ts": time.time()}))
            asyncio.ensure_future(self._log_incoming(ws))
            print(f"[fake-quest] streaming ({scenario}) — ctrl-c to stop")

            await self._stream(ws, warmup, "warmup")
            await self._apply(ws, scenario, duration)

    async def _apply(self, ws, scenario, duration):
        if scenario == "steady" or scenario == "confirm":
            await self._stream(ws, duration, scenario)

        elif scenario == "doff":
            print("[fake-quest] >>> DOFF: worn=false (still streaming)")
            await ws.send(json.dumps({"t": "presence", "worn": False,
                                      "ts": time.time()}))
            self.worn = False
            await self._stream(ws, duration, "doffed")

        elif scenario == "silent-doff":
            print("[fake-quest] >>> DOFF: going silent with no warning")
            await asyncio.sleep(duration)

        elif scenario == "disconnect":
            print("[fake-quest] >>> dropping the websocket")
            await ws.close()

        elif scenario == "dropout":
            print("[fake-quest] >>> 400ms silence")
            await asyncio.sleep(0.4)
            print("[fake-quest] >>> resuming")
            await self._stream(ws, duration, "resumed")

        elif scenario == "freeze":
            print("[fake-quest] >>> freezing the payload")
            self.freeze()
            await self._stream(ws, duration, "frozen")

        elif scenario == "jump":
            print("[fake-quest] >>> 0.5m head transient")
            self.head_offset = -0.5
            await self._stream(ws, duration, "jumped")

        elif scenario == "untracked":
            print("[fake-quest] >>> both hands untracked")
            self.tracked = False
            await self._stream(ws, duration, "untracked")

        elif scenario == "estop":
            print("[fake-quest] >>> sending estop")
            await ws.send(json.dumps({"t": "estop"}))
            await self._stream(ws, duration, "post-estop")

        elif scenario == "align-skip":
            # Both together, level-triggered, held continuously: the host tests
            # `left_ctrl_aButton and right_ctrl_aButton` fresh every cycle and
            # the confirm gesture alongside it, and lets go of either resets
            # the hold. --confirm is implied; without it this proves nothing,
            # because skip waives the position check and never the gesture.
            self.confirm = True
            await self._buttons(ws, ["left_a", "right_a"], "skip held")
            await self._stream(ws, duration, "skip+confirm")
            await self._buttons(ws, [], "release")

        elif scenario == "buttons":
            # Level-triggered, exactly as the app sends it: each message is the
            # complete held set, and releasing means sending an empty one.
            await self._buttons(ws, ["left_thumb", "right_thumb"], "damp")
            await self._stream(ws, 2.0, "damping")
            await self._buttons(ws, [], "release")
            await self._stream(ws, 1.0, "released")
            await self._buttons(ws, ["right_a"], "quit")
            await self._stream(ws, duration, "post-quit")

        else:
            raise SystemExit(f"unknown scenario: {scenario}")

    async def _buttons(self, ws, pressed, label):
        print(f"[fake-quest] >>> buttons {label}: {pressed or '(none)'}")
        await ws.send(json.dumps({"t": "buttons", "pressed": pressed}))

    async def _stream(self, ws, seconds, label):
        """Stream at RATE_HZ on an absolute schedule.

        Two Windows-specific traps are avoided here, and both of them silently
        turn this tool into a flood rather than a headset:

        `time.monotonic()` has 15.6ms granularity on Windows (it is
        GetTickCount64 before Python 3.13). asyncio fires any timer due within
        one clock resolution, so `await asyncio.sleep(1/72)` returns
        *immediately* once the event loop has other work -- which it does, as
        soon as a websocket is attached. Measured: ~10,000 frames a second
        instead of 72. `perf_counter` is high-resolution everywhere, so the
        deadline is computed from it and the sleep is re-checked against it.

        Pacing is absolute rather than `sleep(period)` per frame, so a slow send
        does not push every later frame back and quietly lower the rate.
        """
        period = 1.0 / RATE_HZ
        start = time.perf_counter()
        end = start + seconds
        sent = 0
        while time.perf_counter() < end:
            try:
                await ws.send(self.payload())
            except Exception as e:
                print(f"[fake-quest] send failed ({e!r}) — link is down")
                return
            sent += 1

            target = start + sent * period
            while True:
                remaining = target - time.perf_counter()
                if remaining <= 0:
                    break
                # Long waits can use a real timer; short ones become a yield
                # loop, because on Windows asyncio cannot represent them.
                await asyncio.sleep(remaining if remaining > 0.05 else 0)

        elapsed = time.perf_counter() - start
        print(f"[fake-quest] {label}: sent {sent} frames "
              f"({sent / max(elapsed, 1e-6):.0f} Hz)")

    async def _log_incoming(self, ws):
        try:
            async for raw in ws:
                msg = json.loads(raw)
                if msg.get("t") == "state":
                    align = msg.get("align") or {}
                    extra = ""
                    if align:
                        extra = (f" align={align.get('progress', 0):.0%}"
                                 f" {align.get('reason', '')}")
                    print(f"[host] {msg.get('session')}: {msg.get('reason', '')}{extra}")
                else:
                    print(f"[host] {msg}")
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8443)
    ap.add_argument("--tls", action="store_true", help="use wss:// instead of ws://")
    ap.add_argument("--scenario", default="steady", choices=sorted(SCENARIOS))
    ap.add_argument("--hand-mode", action="store_true",
                    help="send hand-tracking frames (25 joints per side)")
    ap.add_argument("--confirm", action="store_true",
                    help="hold the two-handed confirm gesture (for the align gate)")
    ap.add_argument("--warmup", type=float, default=3.0,
                    help="seconds of healthy streaming before the scenario fires")
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--list", action="store_true", help="list scenarios and exit")
    args = ap.parse_args()

    if args.list:
        width = max(len(k) for k in SCENARIOS)
        for name, desc in sorted(SCENARIOS.items()):
            print(f"  {name:<{width}}  {desc}")
        return

    scheme = "wss" if args.tls else "ws"
    quest = FakeQuest(f"{scheme}://{args.host}:{args.port}",
                      hand_mode=args.hand_mode,
                      confirm=args.confirm or args.scenario == "confirm")
    try:
        asyncio.get_event_loop().run_until_complete(
            quest.run(args.scenario, warmup=args.warmup, duration=args.duration))
    except KeyboardInterrupt:
        print("\n[fake-quest] stopped")


if __name__ == "__main__":
    main()
