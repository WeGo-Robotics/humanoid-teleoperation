"""Machine-readable record of what the device actually sent.

Every defect in docs sections 14.2 and 15.2 was found by measuring the wire, not by
reading the app: the head pose that never moved, the tracked flags that stayed
false while poses streamed, the button messages that never arrived. Each of
those was a separate ad-hoc probe, written after the session had already been
lost. This module makes the probe permanent, so the evidence exists the first
time rather than the second.

The output is JSONL -- one self-describing object per line -- because the reader
is a program, not a person. Three record kinds:

  {"rec": "event",   ...}   connect, hello, buttons, estop, decode errors
  {"rec": "frame",   ...}   a decimated tracking frame, poses included
  {"rec": "summary", ...}   a window's diagnostics, with verdicts

`summary` is the one to read first. It carries the measurements that separate a
working device from each known failure -- head motion spread, tracked-flag
rates, button counts -- and names the failure when one is present, so a reader
does not have to rediscover which numbers matter.

Poses are logged as they arrive: raw OpenXR (y up, z back, x right), before
`transforms.openxr_to_robot`. That is deliberate. A bug on the device shows up
in the device's own frame; running the host transform first would fold host bugs
into the same numbers.

Recording is decimated (default 10 Hz of the device's ~72) because the question
is "what is this device doing", not "reproduce every frame". Set `hz=0` for all
of them.

Writes happen on the server thread. They are line-buffered appends of a few
hundred bytes; if that ever shows up in the control loop's jitter, the fix is a
queue, not a smaller record.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any, Dict, Iterable, List, Optional

import numpy as np

try:
    import logging_mp
    logger_mp = logging_mp.get_logger(__name__)
except ImportError:      # pragma: no cover - robot-side dependency
    import logging
    logger_mp = logging.getLogger(__name__)

#: A pose this close to the identity matrix is the "never driven" signature from
#: section 14.2 defect 1, not a person standing very still.
IDENTITY_TOL = 1e-6

#: Below this, a channel did not move over the whole window. A seated operator
#: still drifts centimetres; 1 mm of total spread is a constant.
STATIC_SPREAD_M = 0.001


def _pos(mat) -> List[float]:
    return [float(mat[0, 3]), float(mat[1, 3]), float(mat[2, 3])]


def _quat(mat) -> List[float]:
    """Rotation part as (x, y, z, w). Kept here rather than pulled from scipy so
    the recorder can run in the bare `websockets`+`numpy` probe environment of
    section 15.3, where the robot stack is not installed."""
    m = np.asarray(mat, dtype=np.float64)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return [round(x, 6), round(y, 6), round(z, 6), round(w, 6)]


def _is_identity(mat) -> bool:
    return bool(np.allclose(np.asarray(mat, dtype=np.float64), np.eye(4),
                            atol=IDENTITY_TOL))


class _Spread:
    """Per-axis min/max of a position channel. Spread, not variance: the
    question is "did this thing move at all", and a range answers it in metres
    the reader can picture."""

    def __init__(self):
        self.lo = [math.inf] * 3
        self.hi = [-math.inf] * 3
        self.n = 0

    def add(self, p: Iterable[float]):
        for i, v in enumerate(p):
            if v < self.lo[i]:
                self.lo[i] = v
            if v > self.hi[i]:
                self.hi[i] = v
        self.n += 1

    def spread(self) -> List[float]:
        if not self.n:
            return [0.0, 0.0, 0.0]
        return [round(self.hi[i] - self.lo[i], 6) for i in range(3)]

    def max_spread(self) -> float:
        return max(self.spread()) if self.n else 0.0


class _Window:
    """Everything measured between two summary records."""

    def __init__(self, t0: float):
        self.t0 = t0
        self.frames = 0
        self.head = _Spread()
        self.left = _Spread()
        self.right = _Spread()
        self.head_identity = 0
        self.left_tracked = 0
        self.right_tracked = 0
        self.hand_mode = 0
        self.worn_true = 0
        self.worn_seen = 0
        self.button_msgs = 0
        self.buttons: set = set()
        self.estops = 0
        self.decode_errors = 0
        self.gap_max_ms = 0.0
        self.seq_first: Optional[int] = None
        self.seq_last: Optional[int] = None


class LinkRecorder:
    """Append-only JSONL sink. Safe to call from the server thread."""

    def __init__(self, path: str, hz: float = 10.0,
                 summary_period: float = 5.0):
        self.path = path
        self._min_dt = (1.0 / hz) if hz and hz > 0 else 0.0
        self._summary_period = summary_period
        self._lock = threading.Lock()
        self._last_write = 0.0
        self._last_rx = 0.0
        self._win = _Window(time.monotonic())
        self._closed = False

        d = os.path.dirname(os.path.abspath(path))
        if d:
            os.makedirs(d, exist_ok=True)
        # Line buffered: a session that dies mid-flight -- which is how every
        # defect so far has ended -- must still leave its evidence on disk.
        self._fh = open(path, "a", buffering=1, encoding="utf-8")
        self.event("recording_started", path=os.path.abspath(path),
                   hz=hz, summary_period=summary_period)
        logger_mp.info(f"[XrLink] recording device telemetry to {path}")

    # ------------------------------------------------------------------
    # writing
    # ------------------------------------------------------------------
    def _write(self, obj: Dict[str, Any]):
        obj["t"] = round(time.time(), 3)
        obj["mono"] = round(time.monotonic(), 4)
        try:
            self._fh.write(json.dumps(obj, ensure_ascii=False) + "\n")
        except Exception as e:      # pragma: no cover - disk full, fd closed
            logger_mp.warning(f"[XrLink] recorder write failed: {e!r}")

    def event(self, kind: str, **fields):
        """Log a control-channel or connection event. Never decimated: these are
        rare, and their absence is itself the finding (defect 3 was 'no button
        message ever reached the host')."""
        with self._lock:
            if self._closed:
                return
            w = self._win
            if kind == "buttons":
                w.button_msgs += 1
                for b in fields.get("pressed") or ():
                    w.buttons.add(b)
            elif kind == "estop":
                w.estops += 1
            elif kind == "decode_error":
                w.decode_errors += 1
            self._write(dict(rec="event", kind=kind, **fields))

    def tracking(self, frame, rx_monotonic: float,
                 dropped: int = 0, decode_errors: int = 0):
        """Accumulate one tracking frame, and write a decimated sample of it."""
        with self._lock:
            if self._closed:
                return
            w = self._win
            w.frames += 1

            head_p = _pos(frame.head)
            left_p = _pos(frame.left_wrist)
            right_p = _pos(frame.right_wrist)
            w.head.add(head_p)
            w.left.add(left_p)
            w.right.add(right_p)
            if _is_identity(frame.head):
                w.head_identity += 1
            if frame.left_tracked:
                w.left_tracked += 1
            if frame.right_tracked:
                w.right_tracked += 1
            if frame.hand_mode:
                w.hand_mode += 1
            w.worn_seen += 1
            if frame.worn:
                w.worn_true += 1
            if w.seq_first is None:
                w.seq_first = frame.seq
            w.seq_last = frame.seq

            if self._last_rx:
                gap_ms = (rx_monotonic - self._last_rx) * 1000.0
                w.gap_max_ms = max(w.gap_max_ms, gap_ms)
            self._last_rx = rx_monotonic

            if rx_monotonic - self._last_write >= self._min_dt:
                self._last_write = rx_monotonic
                self._write({
                    "rec": "frame",
                    "seq": int(frame.seq),
                    "t_device": round(float(frame.t_device), 4),
                    "worn": bool(frame.worn),
                    "left_tracked": bool(frame.left_tracked),
                    "right_tracked": bool(frame.right_tracked),
                    "hand_mode": bool(frame.hand_mode),
                    "head_pos": [round(v, 5) for v in head_p],
                    "head_quat": _quat(frame.head),
                    "left_pos": [round(v, 5) for v in left_p],
                    "left_quat": _quat(frame.left_wrist),
                    "right_pos": [round(v, 5) for v in right_p],
                    "right_quat": _quat(frame.right_wrist),
                    # The chain the safety layer actually consumes. If head is
                    # stuck at the origin these equal the absolute wrists, which
                    # is exactly how defect 1 hid: both numbers looked sane on
                    # their own.
                    "left_rel_head": [round(left_p[i] - head_p[i], 5)
                                      for i in range(3)],
                    "right_rel_head": [round(right_p[i] - head_p[i], 5)
                                       for i in range(3)],
                    "inputs": {k: round(float(v), 4)
                               for k, v in (frame.inputs or {}).items()},
                    "dropped": int(dropped),
                    "decode_errors": int(decode_errors),
                })

            if rx_monotonic - w.t0 >= self._summary_period:
                self._flush_summary(rx_monotonic, dropped)

    # ------------------------------------------------------------------
    # summaries
    # ------------------------------------------------------------------
    def _flush_summary(self, now: float, dropped: int = 0):
        """Caller holds the lock."""
        w = self._win
        dur = max(now - w.t0, 1e-6)
        summary = summarize_window(w, dur, dropped)
        self._write(dict(rec="summary", **summary))
        self._win = _Window(now)

    def flush_summary(self):
        """Force a summary out -- used at disconnect, so a short session that
        never reached the period boundary still produces a verdict."""
        with self._lock:
            if self._closed or not self._win.frames:
                return
            self._flush_summary(time.monotonic())

    def close(self):
        with self._lock:
            if self._closed:
                return
            if self._win.frames:
                self._flush_summary(time.monotonic())
            self._write(dict(rec="event", kind="recording_stopped"))
            self._closed = True
            try:
                self._fh.close()
            except Exception:       # pragma: no cover
                pass


def summarize_window(w: _Window, dur: float, dropped: int = 0) -> Dict[str, Any]:
    """Turn a window of measurements into numbers plus named verdicts.

    Kept a free function so the same rules produce the live summary and the
    offline digest -- a reader comparing the two must never have to wonder
    whether they were computed differently."""
    n = max(w.frames, 1)
    head_spread = w.head.max_spread()
    left_spread = w.left.max_spread()
    right_spread = w.right.max_spread()
    left_pct = 100.0 * w.left_tracked / n
    right_pct = 100.0 * w.right_tracked / n
    identity_pct = 100.0 * w.head_identity / n

    findings: List[str] = []
    if w.frames == 0:
        findings.append("no_frames")
    else:
        # Defect 1: the rig anchor is never driven, so the head pose is a
        # constant -- usually the identity, but any frozen value breaks the
        # p_wrist - p_head chain the same way.
        if identity_pct > 99.0:
            findings.append("head_pose_identity")
        elif head_spread < STATIC_SPREAD_M:
            findings.append("head_pose_static")
        # Defect 2: poses stream while the connected/tracked flag says no, so
        # the host latches tracking_lost and never leaves idle.
        moving = max(left_spread, right_spread) >= STATIC_SPREAD_M
        if left_pct < 1.0 and moving:
            findings.append("left_untracked_while_moving")
        if right_pct < 1.0 and moving:
            findings.append("right_untracked_while_moving")
        if left_spread < STATIC_SPREAD_M and right_spread < STATIC_SPREAD_M:
            findings.append("wrists_static")
        # Two channels of one device sampled identically is the section 14.3
        # signature: the wrists move, the head does not.
        if head_spread < STATIC_SPREAD_M and moving:
            findings.append("head_static_while_wrists_move")
    # Defect 3: no button message ever reaches the host, so the device-side
    # emergency stop does not exist. Only meaningful once frames prove the
    # device is alive and talking.
    if w.button_msgs == 0 and w.frames > 0:
        findings.append("no_button_messages")
    if w.decode_errors:
        findings.append("decode_errors")

    return {
        "window_s": round(dur, 2),
        "frames": w.frames,
        "fps": round(w.frames / dur, 1),
        "seq_first": w.seq_first,
        "seq_last": w.seq_last,
        "dropped_total": int(dropped),
        "gap_max_ms": round(w.gap_max_ms, 1),
        "head_spread_m": w.head.spread(),
        "head_spread_max_m": round(head_spread, 6),
        "head_identity_pct": round(identity_pct, 1),
        "left_spread_m": w.left.spread(),
        "right_spread_m": w.right.spread(),
        "left_tracked_pct": round(left_pct, 1),
        "right_tracked_pct": round(right_pct, 1),
        "hand_mode_pct": round(100.0 * w.hand_mode / n, 1),
        "worn_pct": round(100.0 * w.worn_true / max(w.worn_seen, 1), 1),
        "button_msgs": w.button_msgs,
        "buttons_seen": sorted(w.buttons),
        "estops": w.estops,
        "decode_errors": w.decode_errors,
        "findings": findings,
    }


#: What each finding means, so the digest explains itself to a reader who has
#: not read the plan document.
FINDING_NOTES = {
    "no_frames": "No tracking frame arrived. The device never connected, or it "
                 "connected and sent nothing.",
    "head_pose_identity": "Head pose is the identity matrix (docs 14.2 defect 1). "
                          "The rig anchor is not being driven, so p_wrist - p_head "
                          "leaves the wrists in absolute tracking space and the "
                          "head-jump detector cannot fire.",
    "head_pose_static": "Head pose never moved, though it is not the identity. "
                        "Same consequence as defect 1: a constant head breaks the "
                        "relative chain and disables the jump guard.",
    "head_static_while_wrists_move": "The wrists moved and the head did not, on the "
                                     "same device over the same window. This is the "
                                     "docs 14.3 measurement that settles it: the "
                                     "head channel is not being sampled.",
    "left_untracked_while_moving": "Left controller streams moving poses while its "
                                   "tracked flag stays false (docs 14.2 defect 2). "
                                   "The host will latch tracking_lost and stay idle.",
    "right_untracked_while_moving": "Right controller streams moving poses while its "
                                    "tracked flag stays false (docs 14.2 defect 2).",
    "wrists_static": "Neither wrist moved. Either nobody was holding the controllers, "
                     "or the pose source is frozen.",
    "no_button_messages": "Frames arrived but no button message ever did (docs 14.2 "
                          "defect 3). The device-side emergency stop is not reaching "
                          "the host. This finding is expected if nothing was pressed.",
    "decode_errors": "Frames failed to decode. Usually a protocol version mismatch "
                     "between the app and the host.",
}


def digest(path: str) -> Dict[str, Any]:
    """Fold a whole recording into one verdict object.

    This is the entry point for a reader that wants the answer rather than the
    data: it merges every window, keeps the events that matter, and reports each
    finding with the measurement that produced it."""
    total = _Window(0.0)
    windows = 0
    dur = 0.0
    events: Dict[str, int] = {}
    devices: set = set()
    frames_seen = 0
    dropped = 0

    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            rec = obj.get("rec")
            if rec == "event":
                kind = str(obj.get("kind"))
                events[kind] = events.get(kind, 0) + 1
                if kind == "hello" and obj.get("dev"):
                    devices.add(str(obj["dev"]))
            elif rec == "summary":
                windows += 1
                dur += float(obj.get("window_s") or 0.0)
                frames_seen += int(obj.get("frames") or 0)
                dropped = max(dropped, int(obj.get("dropped_total") or 0))
                total.frames += int(obj.get("frames") or 0)
                total.button_msgs += int(obj.get("button_msgs") or 0)
                total.buttons.update(obj.get("buttons_seen") or ())
                total.estops += int(obj.get("estops") or 0)
                total.decode_errors += int(obj.get("decode_errors") or 0)
                total.gap_max_ms = max(total.gap_max_ms,
                                       float(obj.get("gap_max_ms") or 0.0))
                # Spreads merge by taking the widest window: a device that moved
                # in any window is not static, and summing ranges would inflate
                # a slow drift into apparent motion.
                _merge_spread(total.head, obj.get("head_spread_m"))
                _merge_spread(total.left, obj.get("left_spread_m"))
                _merge_spread(total.right, obj.get("right_spread_m"))
                f = int(obj.get("frames") or 0)
                total.head_identity += round(
                    f * float(obj.get("head_identity_pct") or 0.0) / 100.0)
                total.left_tracked += round(
                    f * float(obj.get("left_tracked_pct") or 0.0) / 100.0)
                total.right_tracked += round(
                    f * float(obj.get("right_tracked_pct") or 0.0) / 100.0)
                total.hand_mode += round(
                    f * float(obj.get("hand_mode_pct") or 0.0) / 100.0)
                total.worn_seen += f
                total.worn_true += round(
                    f * float(obj.get("worn_pct") or 0.0) / 100.0)
                if total.seq_first is None:
                    total.seq_first = obj.get("seq_first")
                total.seq_last = obj.get("seq_last") or total.seq_last

    out = summarize_window(total, max(dur, 1e-6), dropped)
    out["windows"] = windows
    out["events"] = events
    out["devices"] = sorted(devices)
    out["notes"] = {f: FINDING_NOTES.get(f, "") for f in out["findings"]}
    out["source"] = os.path.abspath(path)
    if frames_seen == 0:
        out["verdict"] = "no device telemetry in this recording"
    elif out["findings"]:
        out["verdict"] = "device telemetry is defective: " + ", ".join(out["findings"])
    else:
        out["verdict"] = "device telemetry looks healthy"
    return out


def _merge_spread(sp: _Spread, spread_list):
    """Fold a reported per-axis spread back into an accumulator by widening it."""
    if not spread_list:
        return
    sp.n += 1
    for i, v in enumerate(spread_list[:3]):
        v = float(v)
        # Represent the window as a range centred on zero: only the width is
        # meaningful once the absolute positions are gone.
        sp.lo[i] = min(sp.lo[i], -v / 2.0)
        sp.hi[i] = max(sp.hi[i], v / 2.0)


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser(
        description="Summarize an XrLink telemetry recording into verdicts.")
    p.add_argument("path", help="JSONL file written by --xr-log")
    p.add_argument("--json", action="store_true",
                   help="Emit the digest as JSON rather than text")
    a = p.parse_args(argv)

    d = digest(a.path)
    if a.json:
        print(json.dumps(d, indent=2, ensure_ascii=False))
        return 0

    print(f"source   : {d['source']}")
    print(f"verdict  : {d['verdict']}")
    print(f"frames   : {d['frames']} over {d['window_s']}s "
          f"({d['fps']} fps, gap max {d['gap_max_ms']}ms, "
          f"dropped {d['dropped_total']})")
    print(f"devices  : {', '.join(d['devices']) or '(none)'}")
    print(f"head     : spread {d['head_spread_m']} m, "
          f"identity {d['head_identity_pct']}%")
    print(f"wrists   : left spread {d['left_spread_m']} m, "
          f"right spread {d['right_spread_m']} m")
    print(f"tracked  : left {d['left_tracked_pct']}%, right {d['right_tracked_pct']}%")
    print(f"buttons  : {d['button_msgs']} messages {d['buttons_seen']}, "
          f"{d['estops']} estop")
    print(f"events   : {d['events']}")
    for f in d["findings"]:
        print(f"  - {f}: {FINDING_NOTES.get(f, '')}")
    return 0


if __name__ == "__main__":       # pragma: no cover
    raise SystemExit(main())
