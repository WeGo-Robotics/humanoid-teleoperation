"""XrLink wire codec tests.

This is the contract a separate team builds the Quest app against, so the tests
are deliberately exhaustive about sizes, byte order and rejection behaviour --
a decode that silently half-succeeds would put wrong poses on the control path.

    python -m unittest discover -s teleop/tests -v
"""
from __future__ import annotations

import json
import os
import struct
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from xr.codec import (  # noqa: E402
    BASE_SIZE, CodecError, FLAG_HAND_MODE, FLAG_WORN, HAND_SIZE, HEADER_SIZE,
    INPUT_FIELDS, MAGIC, PROTO_VERSION, TrackingFrame, decode_control,
    decode_tracking, encode_control, encode_tracking,
)


def pose(x=0.0, y=0.0, z=0.0):
    m = np.eye(4)
    m[0:3, 3] = (x, y, z)
    return m


def make_frame(hand_mode=True, **kw):
    defaults = dict(
        seq=42, t_device=123.456, worn=True, left_tracked=True,
        right_tracked=True, hand_mode=hand_mode,
        head=pose(0, 0, 1.6), left_wrist=pose(0.2, 0.3, -0.1),
        right_wrist=pose(0.2, -0.3, -0.1),
        inputs={k: 0.25 * (i + 1) for i, k in enumerate(INPUT_FIELDS)},
        left_joints=np.arange(75, dtype=float).reshape(25, 3),
        right_joints=np.arange(75, 150, dtype=float).reshape(25, 3),
    )
    defaults.update(kw)
    if not defaults["hand_mode"]:
        defaults["left_joints"] = defaults["right_joints"] = None
    return TrackingFrame(**defaults)


class TestTrackingRoundTrip(unittest.TestCase):
    def test_hand_mode_round_trip(self):
        original = make_frame(hand_mode=True)
        decoded = decode_tracking(encode_tracking(original))

        self.assertEqual(decoded.seq, original.seq)
        self.assertAlmostEqual(decoded.t_device, original.t_device, places=9)
        self.assertTrue(decoded.worn)
        self.assertTrue(decoded.hand_mode)
        np.testing.assert_allclose(decoded.head, original.head, atol=1e-6)
        np.testing.assert_allclose(decoded.left_wrist, original.left_wrist, atol=1e-6)
        np.testing.assert_allclose(decoded.right_wrist, original.right_wrist, atol=1e-6)
        np.testing.assert_allclose(decoded.left_joints, original.left_joints, atol=1e-4)
        np.testing.assert_allclose(decoded.right_joints, original.right_joints, atol=1e-4)
        for k in INPUT_FIELDS:
            self.assertAlmostEqual(decoded.inputs[k], original.inputs[k], places=5)

    def test_controller_mode_round_trip(self):
        original = make_frame(hand_mode=False)
        decoded = decode_tracking(encode_tracking(original))
        self.assertFalse(decoded.hand_mode)
        self.assertIsNone(decoded.left_joints)
        self.assertIsNone(decoded.right_joints)
        np.testing.assert_allclose(decoded.left_wrist, original.left_wrist, atol=1e-6)

    def test_flags_round_trip_independently(self):
        for worn in (True, False):
            for lt in (True, False):
                for rt in (True, False):
                    f = make_frame(worn=worn, left_tracked=lt, right_tracked=rt)
                    d = decode_tracking(encode_tracking(f))
                    self.assertEqual((d.worn, d.left_tracked, d.right_tracked),
                                     (worn, lt, rt))

    def test_t_device_keeps_double_precision(self):
        """Timestamps are float64 on the wire on purpose -- float32 would
        quantise a monotonic clock to ~10ms after a few days of uptime, which
        is the same order as the staleness deadline."""
        t = 987654.321098765
        d = decode_tracking(encode_tracking(make_frame(t_device=t)))
        self.assertAlmostEqual(d.t_device, t, places=9)

    def test_sequence_wraps_without_error(self):
        d = decode_tracking(encode_tracking(make_frame(seq=2 ** 32 + 5)))
        self.assertEqual(d.seq, 5)


class TestFrameSizes(unittest.TestCase):
    """Sizes are part of the published contract."""

    def test_documented_sizes(self):
        self.assertEqual(HEADER_SIZE, 16)
        self.assertEqual(BASE_SIZE, 240)
        self.assertEqual(HAND_SIZE, 840)

    def test_encoded_lengths_match(self):
        self.assertEqual(len(encode_tracking(make_frame(hand_mode=False))), BASE_SIZE)
        self.assertEqual(len(encode_tracking(make_frame(hand_mode=True))), HAND_SIZE)

    def test_header_is_little_endian_as_documented(self):
        raw = encode_tracking(make_frame(seq=0x01020304, hand_mode=False))
        magic, proto, flags, seq, _t = struct.unpack_from("<HBBId", raw, 0)
        self.assertEqual(magic, MAGIC)
        self.assertEqual(proto, PROTO_VERSION)
        self.assertEqual(seq, 0x01020304)
        self.assertTrue(flags & FLAG_WORN)
        self.assertFalse(flags & FLAG_HAND_MODE)


class TestTrackingRejection(unittest.TestCase):
    """Every malformed input must raise, never half-decode."""

    def test_truncated_header(self):
        with self.assertRaises(CodecError):
            decode_tracking(b"\x52\x58\x01")

    def test_empty(self):
        with self.assertRaises(CodecError):
            decode_tracking(b"")

    def test_bad_magic(self):
        raw = bytearray(encode_tracking(make_frame()))
        raw[0:2] = b"\x00\x00"
        with self.assertRaises(CodecError) as cm:
            decode_tracking(bytes(raw))
        self.assertIn("magic", str(cm.exception))

    def test_future_protocol_version_is_refused(self):
        raw = bytearray(encode_tracking(make_frame()))
        raw[2] = PROTO_VERSION + 1
        with self.assertRaises(CodecError) as cm:
            decode_tracking(bytes(raw))
        self.assertIn("protocol version", str(cm.exception))

    def test_truncated_body(self):
        raw = encode_tracking(make_frame())
        with self.assertRaises(CodecError):
            decode_tracking(raw[:-4])

    def test_trailing_garbage(self):
        raw = encode_tracking(make_frame()) + b"\x00\x00"
        with self.assertRaises(CodecError):
            decode_tracking(raw)

    def test_hand_flag_without_joint_payload(self):
        """A controller-size frame that claims hand mode must not be read as
        hand data -- it would alias the input block as joint positions."""
        raw = bytearray(encode_tracking(make_frame(hand_mode=False)))
        raw[3] |= FLAG_HAND_MODE
        with self.assertRaises(CodecError):
            decode_tracking(bytes(raw))

    def test_encoder_rejects_wrong_pose_shape(self):
        with self.assertRaises(CodecError):
            encode_tracking(make_frame(head=np.eye(3)))

    def test_encoder_rejects_wrong_joint_shape(self):
        with self.assertRaises(CodecError):
            encode_tracking(make_frame(left_joints=np.zeros((21, 3))))


class TestControlChannel(unittest.TestCase):
    def test_host_message_round_trip(self):
        text = encode_control("prompt_align", target={"left": [1, 2]},
                              tol={"pos_m": 0.08})
        msg = json.loads(text)
        self.assertEqual(msg["t"], "prompt_align")
        self.assertEqual(msg["tol"]["pos_m"], 0.08)

    def test_host_cannot_send_a_device_message(self):
        with self.assertRaises(CodecError):
            encode_control("presence", worn=True)

    def test_device_message_accepted(self):
        msg = decode_control(json.dumps({"t": "presence", "worn": False}))
        self.assertEqual(msg["t"], "presence")
        self.assertFalse(msg["worn"])

    def test_device_message_accepts_bytes(self):
        msg = decode_control(json.dumps({"t": "estop"}).encode())
        self.assertEqual(msg["t"], "estop")

    def test_unknown_message_is_refused(self):
        with self.assertRaises(CodecError):
            decode_control(json.dumps({"t": "definitely_not_a_thing"}))

    def test_malformed_json_is_refused(self):
        for bad in ("{", "[1,2,3]", "null", "not json"):
            with self.assertRaises(CodecError):
                decode_control(bad)

    def test_non_utf8_is_refused(self):
        with self.assertRaises(CodecError):
            decode_control(b"\xff\xfe\x00")


if __name__ == "__main__":
    unittest.main(verbosity=2)
