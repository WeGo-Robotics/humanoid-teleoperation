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

    def test_the_hud_names_every_state_the_host_can_send(self):
        """A state with no case falls through to the neutral colour, so a new
        SafetyState would render as 'nothing in particular' on the headset --
        including a new one that means the robot has stopped."""
        from safety.types import SafetyState
        src = self.source("TeleopHud.cs")
        expected = {s.value.upper() for s in SafetyState} | {"ALIGN"}
        missing = {s for s in expected if f'case "{s}":' not in src}
        self.assertFalse(missing,
                         f"TeleopHud has no colour case for: {sorted(missing)}")

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


    def test_the_apps_confirm_gesture_is_one_the_host_can_see(self):
        """Build 13's console asked for the grips.

        Grip is not a button name, not an input field, and not a bit anywhere
        on the wire -- so the host could never see it, XRFrame.confirm_gesture
        was false for the whole session, and the align gate could not be
        passed or skipped. The console meanwhile lit its own HOLD CONFIRM bar
        off the local grip state, so the app looked right and the host looked
        broken. Nothing in the 182 host tests could catch that, because the
        gesture is chosen on the device.
        """
        src = self.source("TeleopSession.cs")
        confirm = [ln for ln in src.splitlines() if "ConfirmHeld =" in ln]
        self.assertTrue(confirm, "TeleopSession no longer assigns ConfirmHeld")
        joined = " ".join(confirm)
        self.assertNotIn("gripButton", joined,
                         "the app confirms on the grips, which never reach the "
                         "host -- see XRFrame.confirm_gesture")
        self.assertIn("Trigger", joined,
                      "the host's confirm_gesture is both triggers (or both "
                      "pinches) and nothing else can satisfy it")

    def test_the_confirm_threshold_matches_the_hosts(self):
        """The host tests `trigger_value < 1.0` on the inverted 10.0-open

        scale, which is a raw pull above 0.9. If the app picks a different
        number the two disagree near the edge, and the operator gets a console
        that ticks confirm while the hold does not accumulate."""
        src = self.source("TeleopSession.cs")
        self.assertIn("ConfirmTriggerRaw = 0.9f;", src)
        from xr.native_source import _IDENTITY  # noqa: F401  (module imports)
        import inspect
        from xr import native_source
        host = inspect.getsource(native_source)
        self.assertIn('inputs.get("left_trigger_value", 10.0) < 1.0', host)

    def test_the_console_tells_the_operator_the_gesture_it_sends(self):
        """The prompt and the binding have to name the same finger. They did

        not in build 13, and an operator following the console exactly could
        not pass the gate."""
        hud = self.source("TeleopHud.cs")
        self.assertNotIn("grips to confirm", hud)
        self.assertIn("triggers to confirm", hud)

    def test_the_stage_mirror_is_not_gated_on_the_texture_changing(self):
        """The flip has to be re-asserted every frame, not set once when the

        stage texture is swapped in.

        Build 13's version lived inside `if (_stageImage.texture != wanted)`.
        BuildStageColumn already assigns Stage.Output at construction time, so
        on the ALIGN path the texture never changes, the branch never ran, and
        the panel stayed unmirrored for the whole of the one state the mirror
        exists to serve -- the operator's left hand on the right of the panel,
        every correction reversed. The fix that shipped in the commit before
        this one was real and was dead code.

        Checked by indentation, which is crude, but the alternative is a Unity
        licence in CI (see this class's docstring). Twelve spaces is method
        body; sixteen is inside a branch.
        """
        src = self.source("TeleopHud.cs")
        hits = [ln for ln in src.splitlines()
                if ln.strip().startswith("_stageImage.uvRect")]
        self.assertTrue(hits, "TeleopHud never sets the stage uvRect")
        for line in hits:
            indent = len(line) - len(line.lstrip())
            self.assertEqual(
                indent, 12,
                "the stage mirror is nested inside a branch; it must run every "
                f"frame. Offending line: {line.strip()!r}")

    def test_the_console_does_not_hold_a_tolerance_of_its_own(self):
        """TeleopHud used to tick each hand's checklist row by testing the

        reported error against a private `PosTolerance = 0.10f`. That number
        was the absolute gate's, and once the host moved to a scale-free check
        the console was applying a rule the host was not. The verdict now comes
        down the wire per wrist (AlignReport.left_ok) and the console renders
        it. This guards against the constant creeping back."""
        hud = self.source("TeleopHud.cs")
        self.assertNotIn("PosTolerance", hud)
        self.assertIn("Session.LeftInPosition", hud)
        self.assertIn("Session.RightInPosition", hud)

    def test_passthrough_waits_for_the_sdk_before_clearing_transparent(self):
        """A camera clearing to transparent with nothing composited behind it

        renders black, so the app must not do it on the strength of having
        asked for passthrough. Initialisation is asynchronous -- OVRManager
        parks at PassthroughInitializationState.Pending and a later frame moves
        it to Initialized -- and it can fail outright. Build 13 set the clear
        in the same frame as the request and treated "OVRManager exists" as
        "passthrough is on", which is why the operator got a console floating
        in a void instead of the room and the instructor standing in it.
        """
        pt = self.source("TeleopPassthrough.cs")
        self.assertIn("IsInsightPassthroughInitialized", pt,
                      "passthrough must be confirmed up, not assumed")
        self.assertIn("HasInsightPassthroughInitFailed", pt,
                      "a failed initialisation has to be distinguishable from "
                      "a slow one, or the log cannot say which happened")

        boot = self.source("TeleopBootstrap.cs")
        self.assertNotIn("OVRPassthroughLayer", boot,
                         "the passthrough layer belongs to TeleopPassthrough, "
                         "which knows when it is safe to create one")

if __name__ == "__main__":
    unittest.main(verbosity=2)
