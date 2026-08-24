#!/usr/bin/env python3
"""A host for the Meta XR Simulator: the align gate, and nothing else.

`teleop_hand_and_arm.py` cannot run on a development PC. It wants `pinocchio`
for inverse kinematics -- which has no Windows wheel -- plus DDS, plus a robot
on the other end of it. So the one thing the simulator is most useful for,
walking the align gate end to end while wearing the console, was not reachable.

This is the smallest host that makes it reachable. It runs:

  * the real `XrLinkServer`, decoding the real wire format
  * the real `openxr_to_robot` transform chain, via `NativeXRSource`
  * the real `AlignGate`, with the real config

against forward kinematics read straight out of `assets/g1/g1_body29_hand14.urdf`
by walking joint origins with numpy. At zero joint angles that is exactly what
`ArmFKMixin.forward_kinematics(q)` returns for a standing G1, so the gate sees
what it would see with a real robot powered on and not yet moving.

What it deliberately is not: no IK, no `robot_arm.py`, no velocity limiter, no
DDS, no gripper. **Nothing here commands a robot and nothing here can.** It
answers one question -- would alignment have passed -- and the answer is real
because the gate deciding it is the shipped one.

    python tools/sim_host.py
    python tools/sim_host.py --pose ready   # arms out, to check the gate refuses

Then in Unity: Meta > Meta XR Simulator > Activate, and press Play.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "teleop"))

from teleop.safety.align import AlignConfig, AlignGate          # noqa: E402
from teleop.xr.link_server import XrLinkServer                  # noqa: E402
from teleop.xr.native_source import NativeXRSource              # noqa: E402

URDF = os.path.join(REPO, "assets", "g1", "g1_body29_hand14.urdf")

#: The IK model's end-effector frame: 5 cm beyond the wrist yaw joint, matching
#: the `L_ee` / `R_ee` frames robot_arm_ik.py adds to the reduced model.
EE_OFFSET = np.array([0.05, 0.0, 0.0])


def _rotation(rpy):
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def wrist_fk(urdf_path=URDF):
    """(left, right) 4x4 wrist poses in the pelvis frame, at zero joint angles.

    Chains the URDF's joint origins rather than importing pinocchio. Only valid
    for q = 0, which is all this host needs: it stands in for a robot that is
    powered and holding its initial pose, which is the state alignment happens
    against.
    """
    root = ET.parse(urdf_path).getroot()
    joints, child_of = {}, {}
    for j in root.findall("joint"):
        origin = j.find("origin")
        xyz = np.array([float(v) for v in (origin.get("xyz", "0 0 0").split())]) \
            if origin is not None else np.zeros(3)
        rpy = np.array([float(v) for v in (origin.get("rpy", "0 0 0").split())]) \
            if origin is not None else np.zeros(3)
        name = j.get("name")
        joints[name] = (j.find("parent").get("link"), xyz, rpy)
        child_of[j.find("child").get("link")] = name

    def chain(link):
        path, cur = [], link
        while cur in child_of:
            jn = child_of[cur]
            path.append(jn)
            cur = joints[jn][0]
        T = np.eye(4)
        for jn in reversed(path):
            _, xyz, rpy = joints[jn]
            M = np.eye(4)
            M[:3, :3] = _rotation(rpy)
            M[:3, 3] = xyz
            T = T @ M
        return T

    out = []
    for side in ("left", "right"):
        T = chain(f"{side}_wrist_yaw_link")
        T = T.copy()
        T[:3, 3] = (T @ np.append(EE_OFFSET, 1.0))[:3]
        out.append(T)
    return out[0], out[1]


#: Alternative "robot poses" to align against, for checking that the gate still
#: refuses things. `initial` is the URDF's own rest pose and the only one a real
#: robot would be in.
POSES = {
    "initial": None,                                   # straight from the URDF
    "ready": np.array([0.45, 0.20, 0.10]),             # arms further out
    "high": np.array([0.30, 0.20, 0.45]),              # hands up
}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address (default: loopback, which is what the "
                         "simulator needs and the only safe default for a "
                         "link with no TLS)")
    ap.add_argument("--port", type=int, default=8443)
    ap.add_argument("--pose", choices=sorted(POSES), default="initial",
                    help="robot arm pose to align against")
    ap.add_argument("--rate", type=float, default=50.0, help="control rate, Hz")
    args = ap.parse_args()

    left_fk, right_fk = wrist_fk()
    if POSES[args.pose] is not None:
        offset = POSES[args.pose]
        left_fk, right_fk = left_fk.copy(), right_fk.copy()
        left_fk[:3, 3] = offset * np.array([1, 1, 1])
        right_fk[:3, 3] = offset * np.array([1, -1, 1])

    cfg = AlignConfig()
    gate = AlignGate(cfg)
    gate.reset(time.monotonic())

    print(f"robot wrists (pelvis frame, {args.pose}): "
          f"L {np.round(left_fk[:3, 3], 3)}  R {np.round(right_fk[:3, 3], 3)}")
    print(f"gate: scale_free={cfg.scale_free} dir_tol={cfg.dir_tol_deg}deg "
          f"rot_tol={cfg.rot_tol_deg}deg hold={cfg.hold_s}s")

    server = XrLinkServer(host=args.host, port=args.port)
    if not server.start():
        raise SystemExit(f"could not bind {args.host}:{args.port}")
    source = NativeXRSource(server)
    print(f"listening on ws://{args.host}:{args.port} -- press Play in Unity")

    period = 1.0 / max(args.rate, 1.0)
    accepted_at = None
    last_print = 0.0
    try:
        while True:
            now = time.monotonic()
            frame = source.read()

            if not frame.liveness.session_up:
                _status(now, last_print, "waiting for the headset to connect")
                if now - last_print >= 1.0:
                    last_print = now
                time.sleep(period)
                continue

            if accepted_at is None:
                skip = frame.left_ctrl_aButton and frame.right_ctrl_aButton
                report = gate.update(now, left_fk, right_fk,
                                     frame.left_wrist_pose, frame.right_wrist_pose,
                                     frame.confirm_gesture, skip,
                                     head_height_m=float(frame.head_pose[2, 3]))
                server.send("state", session="ALIGN", reason=report.reason,
                            align=report.as_dict())
                if report.accepted:
                    accepted_at = now
                    print("\nALIGN ACCEPTED. Following would begin here; this "
                          "host stops instead -- it has no IK and no robot.")
                elif now - last_print >= 0.25:
                    last_print = now
                    print(f"\r  dir L {_deg(report.left_dir_err)} "
                          f"R {_deg(report.right_dir_err)}  "
                          f"ok L{int(report.left_ok)} R{int(report.right_ok)}  "
                          f"confirm {int(report.operator_confirming)} "
                          f"skip {int(skip)}  "
                          f"k={report.operator_scale:.2f}  "
                          f"hold {report.progress * 100:3.0f}%  "
                          f"{report.reason[:60]:<60}", end="", flush=True)
            else:
                server.send("state", session="IDLE",
                            reason="aligned — sim host does not drive a robot")
                if now - last_print >= 2.0:
                    last_print = now
                    print(f"  aligned {now - accepted_at:.0f}s ago; "
                          "ctrl-c to stop, or restart to align again")

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        server.stop()


def _deg(v):
    return " inf " if not np.isfinite(v) else f"{v:5.1f}"


def _status(now, last, msg):
    if now - last >= 1.0:
        print(f"\r  {msg}...{' ' * 40}", end="", flush=True)


if __name__ == "__main__":
    main()
