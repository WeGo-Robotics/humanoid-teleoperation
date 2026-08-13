"""Cross-module contract tests.

These guard the seams where a rename would not fail until the robot was already
moving. They are parsed from source with `ast` rather than imported, because the
modules involved pull in vuer / DDS / PyQt.
"""
from __future__ import annotations

import ast
import os
import sys
import unittest

TELEOP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TELEOP)

from safety import SafetyFSM  # noqa: E402


def dataclass_fields(path, class_name):
    """Field names of a dataclass, read from source without importing it."""
    with open(path, encoding="utf-8") as fh:
        tree = ast.parse(fh.read())
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return [n.target.id for n in node.body if isinstance(n, ast.AnnAssign)]
    raise AssertionError(f"{class_name} not found in {path}")


class TestLivenessContract(unittest.TestCase):
    """teleop_hand_and_arm.py converts one to the other with
    `XRLiveness(**asdict(link_status))`, so the field names must match exactly.
    A rename on either side would raise a TypeError mid-control-loop."""

    def test_xrlinkstatus_matches_xrliveness(self):
        producer = dataclass_fields(
            os.path.join(TELEOP, "televuer", "src", "televuer", "tv_wrapper.py"),
            "XRLinkStatus")
        consumer = dataclass_fields(
            os.path.join(TELEOP, "safety", "types.py"), "XRLiveness")
        self.assertEqual(producer, consumer)

    def test_conversion_is_actually_used_that_way(self):
        """The conversion lives in VuerXRSource now -- the control loop only
        ever sees an XRFrame."""
        with open(os.path.join(TELEOP, "xr", "vuer_source.py"), encoding="utf-8") as fh:
            src = fh.read()
        self.assertIn("XRLiveness(**asdict(self._tv.get_link_status()))", src)


class TestXRFrameCoversTheControlLoop(unittest.TestCase):
    """Every attribute the control loop reads off the XR payload must exist on
    XRFrame. This is the static guard for the TeleData -> XRFrame migration: a
    missed field would otherwise surface as an AttributeError mid-teleop rather
    than at import time."""

    def _attrs_read_from(self, var_name):
        with open(os.path.join(TELEOP, "teleop_hand_and_arm.py"), encoding="utf-8") as fh:
            tree = ast.parse(fh.read())
        found = set()
        for node in ast.walk(tree):
            if (isinstance(node, ast.Attribute)
                    and isinstance(node.value, ast.Name)
                    and node.value.id == var_name):
                found.add(node.attr)
        return found

    def test_every_frame_attribute_exists(self):
        sys.path.insert(0, TELEOP)
        from xr.types import XRFrame
        known = XRFrame.field_names() | {
            p for p in dir(XRFrame) if not p.startswith("_")}
        for var in ("frame", "_idle"):
            for attr in self._attrs_read_from(var):
                self.assertIn(attr, known,
                              f"{var}.{attr} is read by the control loop but is "
                              f"not on XRFrame")

    def test_the_loop_actually_migrated(self):
        with open(os.path.join(TELEOP, "teleop_hand_and_arm.py"), encoding="utf-8") as fh:
            src = fh.read()
        self.assertNotIn("tele_data", src, "leftover TeleData usage in the loop")
        self.assertIn("frame = xr.read()", src)


class TestHeartbeatContract(unittest.TestCase):
    """The dashboard's headset panel reads these keys off the heartbeat."""

    DASHBOARD_KEYS = {"state", "reason", "stale_ms", "link_up", "worn", "latched"}

    def test_snapshot_supplies_every_key_the_dashboard_reads(self):
        snap = SafetyFSM().snapshot()
        self.assertTrue(self.DASHBOARD_KEYS.issubset(snap.keys()),
                        f"missing: {self.DASHBOARD_KEYS - set(snap.keys())}")

    def test_snapshot_is_json_serialisable(self):
        import json
        json.dumps(SafetyFSM().snapshot())   # heartbeat goes over ZMQ send_json


class TestIPCCommandContract(unittest.TestCase):
    """Dashboard buttons send these; the teleop key handler must accept them."""

    def _cmd_map(self):
        with open(os.path.join(TELEOP, "utils", "ipc.py"), encoding="utf-8") as fh:
            tree = ast.parse(fh.read())
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and getattr(
                    node.targets[0], "id", None) == "cmd_map":
                return {k.value: v.value for k, v in
                        zip(node.value.keys, node.value.values)}
        raise AssertionError("cmd_map not found")

    def test_safety_commands_are_mapped(self):
        cmd_map = self._cmd_map()
        self.assertEqual(cmd_map.get("CMD_ACK_FAULT"), "a")
        self.assertEqual(cmd_map.get("CMD_ESTOP"), "e")

    def test_teleop_handles_every_mapped_key(self):
        with open(os.path.join(TELEOP, "teleop_hand_and_arm.py"), encoding="utf-8") as fh:
            tree = ast.parse(fh.read())
        handled = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == "on_press":
                for cmp_node in ast.walk(node):
                    if isinstance(cmp_node, ast.Compare):
                        for c in cmp_node.comparators:
                            if isinstance(c, ast.Constant) and isinstance(c.value, str):
                                handled.add(c.value)
        for cmd, key in self._cmd_map().items():
            self.assertIn(key, handled, f"{cmd} maps to '{key}', unhandled in on_press")


class TestQuestAppMatchesTheWireFormat(unittest.TestCase):
    """The only automated link between the C# client and the Python host.

    Nothing else checks them against each other: the app is built by Unity, the
    host by pytest, and a mismatch surfaces as a decode error on a robot with a
    person standing in front of it. These are crude string checks on purpose --
    a crude check that runs is worth more than a precise one that needs a Unity
    licence in CI.
    """

    QUEST = os.path.join(os.path.dirname(TELEOP), "quest_app", "Assets", "Scripts")

    def source(self, name):
        path = os.path.join(self.QUEST, name)
        if not os.path.exists(path):
            self.skipTest(f"{name} not present")
        with open(path, encoding="utf-8") as fh:
            return fh.read()

    def test_protocol_version_matches(self):
        from xr.codec import PROTO_VERSION
        src = self.source("XrLinkClient.cs")
        self.assertIn(f"ProtoVersion = {PROTO_VERSION};", src,
                      "the app and the host disagree about the protocol version")

    def test_magic_matches(self):
        from xr.codec import MAGIC
        src = self.source("XrLinkClient.cs")
        self.assertIn(f"Magic = 0x{MAGIC:04X}".lower(), src.lower())

    def test_frame_buffer_sizes_match(self):
        from xr.codec import BASE_SIZE, HAND_SIZE
        src = self.source("XrLinkClient.cs")
        self.assertIn(f"_controllerBuffer = new byte[{BASE_SIZE}]", src)
        self.assertIn(f"_handBuffer = new byte[{HAND_SIZE}]", src)

    def test_every_button_the_app_sends_is_one_the_host_knows(self):
        from xr.codec import BUTTON_NAMES
        import re
        src = self.source("TeleopSession.cs")
        sent = {m for m in re.findall(r'"([a-z]+_[a-z]+)"', src)
                if m.split("_")[0] in ("left", "right")}
        self.assertTrue(sent, "found no button names in TeleopSession.cs")
        unknown = sent - BUTTON_NAMES
        self.assertFalse(unknown,
                         f"the app sends buttons the host drops: {sorted(unknown)}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
