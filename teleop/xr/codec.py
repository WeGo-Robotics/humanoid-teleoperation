"""XrLink wire format.

One WebSocket carries two multiplexed channels:

* **Control** -- JSON text frames, ~10 Hz plus events. Session state, presence,
  align prompts, acknowledgements.
* **Tracking** -- binary frames, 72-90 Hz, device to host only.

Tracking frame layout (little-endian, packed):

    offset  type        field
    0       uint16      magic  (0x5852 = 'XR')
    2       uint8       proto  (PROTO_VERSION)
    3       uint8       flags  (bit0 worn, bit1 left_tracked,
                                bit2 right_tracked, bit3 hand_mode)
    4       uint32      seq            monotonic, wraps at 2^32
    8       float64     t_device       device monotonic clock, seconds
    16      float32[16] head           4x4 row-major
    80      float32[16] left_wrist
    144     float32[16] right_wrist
    208     float32[8]  inputs         see INPUT_FIELDS
    240     float32[75] left_joints    (25,3), hand mode only
    540     float32[75] right_joints   hand mode only
    840                                end (hand mode); 240 in controller mode

Why binary and not JSON: 25 joints x 3 axes x 2 hands x 90 Hz is ~13.5k floats
a second per hand set. JSON-encoding that is the difference between a few
hundred KB/s and several MB/s over Wi-Fi, and it is on the control path.

`worn` is carried in the *frame header*, redundantly with the control-channel
presence message, precisely because the OS may suspend the app on doff before
the control message can be sent (see docs §10.3). The last tracking frames
before suspension still carry the bit.

**Poses are raw OpenXR** (y up, z back, x right), straight off the device's XR
nodes. The device applies no convention change of its own -- the host does the
whole OpenXR->robot chain in `transforms.py`. That keeps the safety-relevant
geometry in one reviewed implementation instead of one per client, and makes
the device side very nearly a marshalling exercise. Protocol v1 carried
pre-transformed poses; it was changed before any client existed.
"""
from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

MAGIC = 0x5852
PROTO_VERSION = 2

FLAG_WORN = 1 << 0
FLAG_LEFT_TRACKED = 1 << 1
FLAG_RIGHT_TRACKED = 1 << 2
FLAG_HAND_MODE = 1 << 3

_HEADER = struct.Struct("<HBBId")          # magic, proto, flags, seq, t_device
_POSE_FLOATS = 16
_N_JOINTS = 25
_JOINT_FLOATS = _N_JOINTS * 3

#: Order of the packed input block. Kept explicit so the app and the host cannot
#: drift: adding a field means bumping PROTO_VERSION.
INPUT_FIELDS = (
    "left_pinch_value", "right_pinch_value",
    "left_trigger_value", "right_trigger_value",
    "left_thumb_x", "left_thumb_y",
    "right_thumb_x", "right_thumb_y",
)
_N_INPUTS = len(INPUT_FIELDS)

HEADER_SIZE = _HEADER.size
BASE_SIZE = HEADER_SIZE + (3 * _POSE_FLOATS + _N_INPUTS) * 4
HAND_SIZE = BASE_SIZE + 2 * _JOINT_FLOATS * 4


class CodecError(ValueError):
    """Malformed frame. Always raised rather than returning a partial decode --
    a half-understood tracking frame must never reach the control loop."""


@dataclass(frozen=True)
class TrackingFrame:
    seq: int
    t_device: float
    worn: bool
    left_tracked: bool
    right_tracked: bool
    hand_mode: bool
    head: np.ndarray                       # (4,4)
    left_wrist: np.ndarray                 # (4,4)
    right_wrist: np.ndarray                # (4,4)
    inputs: dict                           # INPUT_FIELDS -> float
    left_joints: Optional[np.ndarray] = None    # (25,3)
    right_joints: Optional[np.ndarray] = None

    # Buttons are deliberately *not* here. They ride on the control channel
    # because they are low-rate, and they live on `LinkSnapshot.buttons` rather
    # than on the frame so there is exactly one place to read them from. An
    # earlier version carried an always-empty tuple on this dataclass, which
    # read like the source of truth and quietly disabled every button.


def _pack_pose(mat: np.ndarray) -> bytes:
    arr = np.asarray(mat, dtype=np.float32)
    if arr.shape != (4, 4):
        raise CodecError(f"pose must be 4x4, got {arr.shape}")
    return arr.reshape(16).tobytes()


def _unpack_pose(buf: memoryview, offset: int) -> Tuple[np.ndarray, int]:
    end = offset + _POSE_FLOATS * 4
    mat = np.frombuffer(buf[offset:end], dtype=np.float32).reshape(4, 4)
    return np.array(mat, dtype=np.float64), end


def encode_tracking(frame: TrackingFrame) -> bytes:
    flags = 0
    if frame.worn:
        flags |= FLAG_WORN
    if frame.left_tracked:
        flags |= FLAG_LEFT_TRACKED
    if frame.right_tracked:
        flags |= FLAG_RIGHT_TRACKED
    if frame.hand_mode:
        flags |= FLAG_HAND_MODE

    out = bytearray()
    out += _HEADER.pack(MAGIC, PROTO_VERSION, flags,
                        int(frame.seq) & 0xFFFFFFFF, float(frame.t_device))
    out += _pack_pose(frame.head)
    out += _pack_pose(frame.left_wrist)
    out += _pack_pose(frame.right_wrist)
    out += np.asarray([float(frame.inputs.get(k, 0.0)) for k in INPUT_FIELDS],
                      dtype=np.float32).tobytes()
    if frame.hand_mode:
        for joints, side in ((frame.left_joints, "left"),
                             (frame.right_joints, "right")):
            arr = np.asarray(
                np.zeros((_N_JOINTS, 3)) if joints is None else joints,
                dtype=np.float32)
            if arr.shape != (_N_JOINTS, 3):
                raise CodecError(f"{side} joints must be (25,3), got {arr.shape}")
            out += arr.reshape(_JOINT_FLOATS).tobytes()
    return bytes(out)


def decode_tracking(payload: bytes) -> TrackingFrame:
    if len(payload) < HEADER_SIZE:
        raise CodecError(f"frame too short: {len(payload)} bytes")

    magic, proto, flags, seq, t_device = _HEADER.unpack_from(payload, 0)
    if magic != MAGIC:
        raise CodecError(f"bad magic 0x{magic:04x}")
    if proto != PROTO_VERSION:
        # Refused, not best-effort decoded. A field appended by a newer app
        # would otherwise be silently misread as an existing one.
        raise CodecError(f"unsupported protocol version {proto} "
                         f"(this host speaks {PROTO_VERSION})")

    hand_mode = bool(flags & FLAG_HAND_MODE)
    expected = HAND_SIZE if hand_mode else BASE_SIZE
    if len(payload) != expected:
        raise CodecError(
            f"frame size {len(payload)} != expected {expected} "
            f"({'hand' if hand_mode else 'controller'} mode)")

    buf = memoryview(payload)
    off = HEADER_SIZE
    head, off = _unpack_pose(buf, off)
    left_wrist, off = _unpack_pose(buf, off)
    right_wrist, off = _unpack_pose(buf, off)

    raw_inputs = np.frombuffer(buf[off:off + _N_INPUTS * 4], dtype=np.float32)
    off += _N_INPUTS * 4
    inputs = {k: float(v) for k, v in zip(INPUT_FIELDS, raw_inputs)}

    left_joints = right_joints = None
    if hand_mode:
        left_joints = np.array(
            np.frombuffer(buf[off:off + _JOINT_FLOATS * 4],
                          dtype=np.float32).reshape(_N_JOINTS, 3), dtype=np.float64)
        off += _JOINT_FLOATS * 4
        right_joints = np.array(
            np.frombuffer(buf[off:off + _JOINT_FLOATS * 4],
                          dtype=np.float32).reshape(_N_JOINTS, 3), dtype=np.float64)

    return TrackingFrame(
        seq=seq, t_device=t_device,
        worn=bool(flags & FLAG_WORN),
        left_tracked=bool(flags & FLAG_LEFT_TRACKED),
        right_tracked=bool(flags & FLAG_RIGHT_TRACKED),
        hand_mode=hand_mode,
        head=head, left_wrist=left_wrist, right_wrist=right_wrist,
        inputs=inputs, left_joints=left_joints, right_joints=right_joints,
    )


# ----------------------------------------------------------------------------
# control channel
# ----------------------------------------------------------------------------
DEVICE_MESSAGES = frozenset({
    "hello",       # {proto, dev, session, resume}
    "presence",    # {worn, focus, ts}      <- also mirrored in tracking flags
    "align_ack",   # {accepted, held_ms}
    "status",      # {battery, fps, tracking:{left,right}}
    "buttons",     # {pressed: [...]}
    "estop",       # {}
    "pong",        # {t}
})

#: The full button vocabulary, shared by every device implementation. A name
#: outside this set is dropped rather than passed through: the host maps these
#: onto real actions (`right_a` quits, both thumbstick clicks damp the robot),
#: so an app that invents a name must not be able to reach those actions by
#: accident, and a typo must fail loudly at review rather than silently at
#: runtime. Adding a name is a protocol change -- change both sides together.
BUTTON_NAMES = frozenset({
    "left_a", "right_a",          # X on the left controller, A on the right
    "left_b", "right_b",          # Y on the left controller, B on the right
    "left_thumb", "right_thumb",  # thumbstick clicks
})


HOST_MESSAGES = frozenset({
    "state",         # {session, reason}
    "prompt_align",  # {target:{left,right}, tol:{pos_m, rot_deg}}
    "abort",         # {reason}
    "video",         # {mode, url}
    "ping",          # {t}
})


def encode_control(kind: str, **payload) -> str:
    if kind not in HOST_MESSAGES:
        raise CodecError(f"not a host->device message: {kind}")
    return json.dumps({"t": kind, **payload}, separators=(",", ":"))


def decode_control(text) -> dict:
    """Parse a device->host control message. Raises CodecError on anything
    unexpected -- an unrecognised message is a protocol mismatch, not
    something to guess at."""
    if isinstance(text, (bytes, bytearray)):
        try:
            text = bytes(text).decode("utf-8")
        except UnicodeDecodeError as e:
            raise CodecError(f"control frame is not utf-8: {e}")
    try:
        msg = json.loads(text)
    except Exception as e:
        raise CodecError(f"malformed control json: {e}")
    if not isinstance(msg, dict):
        raise CodecError("control message must be a JSON object")
    kind = msg.get("t")
    if kind not in DEVICE_MESSAGES:
        raise CodecError(f"unknown control message: {kind!r}")
    return msg
