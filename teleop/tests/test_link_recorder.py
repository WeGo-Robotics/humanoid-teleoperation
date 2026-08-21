"""The recorder exists to catch known device defects, so the tests are those
defects: a frozen head, tracked flags that lie, buttons that never arrive."""
import json
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xr.codec import TrackingFrame                     # noqa: E402
from xr.link_recorder import LinkRecorder, digest      # noqa: E402


def _pose(x=0.0, y=0.0, z=0.0):
    m = np.eye(4)
    m[0, 3], m[1, 3], m[2, 3] = x, y, z
    return m


def _frame(seq=0, head=None, left=None, right=None,
           left_tracked=True, right_tracked=True, worn=True):
    return TrackingFrame(
        seq=seq, t_device=float(seq) / 72.0, worn=worn,
        left_tracked=left_tracked, right_tracked=right_tracked,
        hand_mode=False,
        head=head if head is not None else _pose(0, 1.6, 0),
        left_wrist=left if left is not None else _pose(0.2, 1.5, -0.3),
        right_wrist=right if right is not None else _pose(-0.2, 1.5, -0.3),
        inputs={"left_trigger_value": 0.0},
    )


class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, "rec.jsonl")

    def _read(self):
        with open(self.path, encoding="utf-8") as fh:
            return [json.loads(l) for l in fh if l.strip()]

    def _feed(self, frames, t0=0.0, dt=1.0 / 72.0, **kw):
        rec = LinkRecorder(self.path, hz=0.0, summary_period=1e9, **kw)
        for i, f in enumerate(frames):
            rec.tracking(f, t0 + i * dt)
        return rec

    def test_healthy_session_has_no_findings(self):
        frames = [_frame(seq=i,
                         head=_pose(0.01 * i, 1.6, 0),
                         left=_pose(0.2 + 0.01 * i, 1.5, -0.3),
                         right=_pose(-0.2 - 0.01 * i, 1.5, -0.3))
                  for i in range(20)]
        rec = self._feed(frames)
        rec.event("buttons", pressed=["left_a"])
        rec.close()
        d = digest(self.path)
        self.assertEqual(d["frames"], 20)
        self.assertEqual(d["findings"], [])
        self.assertEqual(d["verdict"], "device telemetry looks healthy")

    def test_identity_head_is_named(self):
        """Defect 1: the rig anchor is never driven, so the head never leaves
        the origin while the wrists move around it."""
        frames = [_frame(seq=i, head=np.eye(4),
                         left=_pose(0.2 + 0.01 * i, 1.5, -0.3),
                         right=_pose(-0.2 - 0.01 * i, 1.5, -0.3))
                  for i in range(20)]
        rec = self._feed(frames)
        rec.event("buttons", pressed=["left_a"])
        rec.close()
        d = digest(self.path)
        self.assertIn("head_pose_identity", d["findings"])
        self.assertIn("head_static_while_wrists_move", d["findings"])
        self.assertEqual(d["head_identity_pct"], 100.0)
        self.assertLess(d["head_spread_max_m"], 1e-6)
        self.assertTrue(d["notes"]["head_pose_identity"])

    def test_static_but_non_identity_head_still_flagged(self):
        frames = [_frame(seq=i, head=_pose(0.0, 1.6, 0.0),
                         left=_pose(0.2 + 0.01 * i, 1.5, -0.3))
                  for i in range(20)]
        rec = self._feed(frames)
        rec.close()
        d = digest(self.path)
        self.assertIn("head_pose_static", d["findings"])
        self.assertNotIn("head_pose_identity", d["findings"])

    def test_tracked_flag_false_while_poses_move(self):
        """Defect 2: poses stream, the flag says the controller is not there."""
        frames = [_frame(seq=i,
                         head=_pose(0.01 * i, 1.6, 0),
                         left=_pose(0.2 + 0.01 * i, 1.5, -0.3),
                         right=_pose(-0.2 - 0.01 * i, 1.5, -0.3),
                         left_tracked=False, right_tracked=False)
                  for i in range(20)]
        rec = self._feed(frames)
        rec.close()
        d = digest(self.path)
        self.assertIn("left_untracked_while_moving", d["findings"])
        self.assertIn("right_untracked_while_moving", d["findings"])
        self.assertEqual(d["left_tracked_pct"], 0.0)

    def test_absent_buttons_are_a_finding(self):
        """Defect 3: an absence only counts as evidence because a press would
        have been recorded."""
        frames = [_frame(seq=i, head=_pose(0.01 * i, 1.6, 0),
                         left=_pose(0.2 + 0.01 * i, 1.5, -0.3))
                  for i in range(10)]
        rec = self._feed(frames)
        rec.close()
        self.assertIn("no_button_messages", digest(self.path)["findings"])

    def test_button_message_clears_the_finding(self):
        frames = [_frame(seq=i, head=_pose(0.01 * i, 1.6, 0),
                         left=_pose(0.2 + 0.01 * i, 1.5, -0.3))
                  for i in range(10)]
        rec = self._feed(frames)
        rec.event("buttons", pressed=["right_b"])
        rec.close()
        d = digest(self.path)
        self.assertNotIn("no_button_messages", d["findings"])
        self.assertIn("right_b", d["buttons_seen"])

    def test_empty_recording_says_so(self):
        LinkRecorder(self.path, hz=0.0).close()
        d = digest(self.path)
        self.assertEqual(d["verdict"], "no device telemetry in this recording")
        self.assertIn("no_frames", d["findings"])

    def test_decimation_keeps_summary_over_every_frame(self):
        """The pose stream is sampled; the diagnostics are not. A 10 Hz sample
        of a 72 Hz link must still count all 72."""
        frames = [_frame(seq=i, head=_pose(0.01 * i, 1.6, 0)) for i in range(72)]
        rec = LinkRecorder(self.path, hz=10.0, summary_period=1e9)
        for i, f in enumerate(frames):
            rec.tracking(f, i / 72.0)
        rec.close()
        rows = self._read()
        n_frames = sum(1 for r in rows if r.get("rec") == "frame")
        summary = [r for r in rows if r.get("rec") == "summary"][-1]
        self.assertLessEqual(n_frames, 12)
        self.assertEqual(summary["frames"], 72)

    def test_frame_record_carries_the_relative_chain(self):
        rec = self._feed([_frame(seq=1, head=_pose(0, 1.6, 0),
                                 left=_pose(0.2, 1.5, -0.3))])
        rec.close()
        row = [r for r in self._read() if r.get("rec") == "frame"][0]
        self.assertEqual(row["head_pos"], [0.0, 1.6, 0.0])
        self.assertEqual(row["left_rel_head"], [0.2, -0.1, -0.3])
        self.assertEqual(row["head_quat"], [0.0, 0.0, 0.0, 1.0])

    def test_decode_errors_are_recorded_and_flagged(self):
        rec = self._feed([_frame(seq=i, head=_pose(0.01 * i, 1.6, 0))
                          for i in range(5)])
        rec.event("decode_error", error="proto mismatch")
        rec.close()
        d = digest(self.path)
        self.assertIn("decode_errors", d["findings"])
        self.assertEqual(d["events"]["decode_error"], 1)

    def test_disconnect_flush_leaves_a_verdict_for_a_short_session(self):
        """A session that dies before the first period boundary is exactly the
        session worth reading, so it must not be the one without a summary."""
        rec = LinkRecorder(self.path, hz=0.0, summary_period=1e9)
        for i in range(5):
            rec.tracking(_frame(seq=i, head=np.eye(4),
                                left=_pose(0.2 + 0.01 * i, 1.5, -0.3)),
                         i / 72.0)
        rec.event("disconnected", peer="1.2.3.4")
        rec.flush_summary()
        self.assertTrue(any(r.get("rec") == "summary" for r in self._read()))
        rec.close()
        self.assertIn("head_pose_identity", digest(self.path)["findings"])


if __name__ == "__main__":
    unittest.main()
