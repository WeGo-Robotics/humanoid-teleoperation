"""
Humanoid Teleoperation Dashboard (PyQt5)

Port of teleop/static/dashboard.html into a native Qt app, with a live MuJoCo
offscreen render of the G1 wired into the "simulation" view.

Data flow (mirrors g1_ws .../gui/frames/robot_viewer.py):
  DDS rt/lowstate  ->  motor_state[i].q  ->  mujoco qpos[7 + i]  ->  offscreen render

Run alongside `python teleop_hand_and_arm.py`:
  # simulation teleop (teleop uses DDS domain 1 with --sim)
  conda activate vtv
  python dashboard.py --domain 1
  # real robot (domain 0)
  python dashboard.py --domain 0 --img-server-ip 192.168.123.164

Buttons talk to the teleop process over its IPC channel (only when teleop is
started with `--ipc`):
  시작 -> CMD_START (== keyboard 'r'),  종료 -> CMD_STOP (== 'q').
'정지' pauses the local elapsed-time / view only; the status tag is driven by
the teleop heartbeat.
"""

import os
import signal
os.environ.setdefault("MUJOCO_GL", "egl")  # headless offscreen GL

PR_SET_PDEATHSIG = 1


def _die_with_parent():
    """Ask the kernel to SIGTERM this process when its parent dies.

    The last line of defence against orphans. Every softer mechanism --
    closeEvent, the SIGTERM handler, the respawn supervisor -- needs the
    dashboard to still be running to do its job, so none of them survive
    `kill -9` on the dashboard. This does, because it is the kernel that
    delivers it. The teleop child turns that SIGTERM into its normal safe
    shutdown, so the arms still go home on the way out.

    Used two ways: as Popen's `preexec_fn` for the teleop child, and as a
    register_at_fork hook for everything else that forks. Failure here must
    never stop a launch, so it is deliberately swallowed.
    """
    try:
        import ctypes
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGTERM)
    except Exception:
        pass


# Installed before every other import, deliberately. The teleop child is not the
# only child: logging_mp forks a listener process when it is imported, which
# happens further down this file via teleop.utils.ipc. PR_SET_PDEATHSIG is
# cleared across fork, so that listener outlived a `kill -9` on the dashboard and
# sat there reparented to init. Registering the hook after those imports was too
# late to catch it -- the fork had already happened.
try:
    os.register_at_fork(after_in_child=_die_with_parent)
except AttributeError:      # Python < 3.7
    pass

import sys
import time
import json
import argparse
import threading
from datetime import datetime

import numpy as np

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QFrame, QSizePolicy, QTextEdit, QComboBox, QLineEdit, QToolButton,
    QButtonGroup, QDialog, QScrollArea,
)

# ----------------------------------------------------------------------------
# constants
# ----------------------------------------------------------------------------
DEFAULT_MODEL = "/home/wego/GMR/assets/unitree_g1/g1_mocap_29dof.xml"
FALLBACK_MODEL = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "assets", "g1", "g1_body29_hand14.xml"
)
RENDER_W, RENDER_H = 960, 720          # initial size
MAX_RENDER_W, MAX_RENDER_H = 2560, 1440  # offscreen framebuffer cap
MIN_RENDER_W, MIN_RENDER_H = 320, 240
RESIZE_THRESHOLD = 8                    # px change before recreating renderer
FPS = 30
G1_NUM_MOTOR = 29
QPOS_OFFSET = 7        # qpos[0:7] = pelvis free joint; qpos[7:36] = 29 motors
STAND_Z = 0.79

# The IPC client is shared with the teleop process rather than reimplemented
# here: the previous local copy lacked heartbeat liveness and reqid correlation,
# so the UI could not tell a slow reply from a dead peer.
try:
    from teleop.utils.ipc import IPC_Client
except ImportError:      # launched as `python teleop/dashboard.py`
    from utils.ipc import IPC_Client

# design tokens (from static/dashboard.html, light theme)
C = {
    "bg": "#f4f4f2", "text": "#1a1a1a", "divider": "#e2e2dd",
    "neutral700": "#6b6b66", "neutral900": "#1a1a1a", "accent": "#2f6df6",
    "card": "#ffffff",
}


def now_str():
    return datetime.now().strftime("%H:%M:%S")


# The teleop child needs the `vtv` conda env (pinocchio, unitree_sdk2py, ...);
# the dashboard itself needs far less and will happily start under the system
# python3. Launching the child with plain sys.executable made that mismatch
# silent: the child died on `ModuleNotFoundError: No module named 'pinocchio'`
# within a second, one line in the log box, and the XrLink server never came up
# -- which reads to the operator as "the headset won't connect".
TELEOP_ENV_PYTHON = "/home/wego/miniconda3/envs/vtv/bin/python"


def _teleop_python():
    """Interpreter for the teleop subprocess: the env that has its deps."""
    if os.path.exists(TELEOP_ENV_PYTHON):
        return TELEOP_ENV_PYTHON
    return sys.executable


def list_net_ifaces():
    """Return [(iface, ip), ...] for up IPv4 interfaces, excluding lo/docker.
    DDS-capable ones (192.168.123.x subnet) are listed first."""
    import subprocess
    out = []
    try:
        raw = subprocess.check_output(["ip", "-o", "-4", "addr", "show"],
                                      text=True, timeout=3)
        for line in raw.splitlines():
            parts = line.split()
            if len(parts) < 4:
                continue
            iface = parts[1]
            ip = parts[3].split("/")[0]
            if iface == "lo" or iface.startswith(("docker", "br-", "veth")):
                continue
            out.append((iface, ip))
    except Exception:
        pass
    # DDS subnet (robot link) first
    out.sort(key=lambda t: (not t[1].startswith("192.168.123."), t[0]))
    return out


def is_dds_ip(ip):
    return ip.startswith("192.168.123.")


class Segmented(QWidget):
    """Two-or-more segment toggle. .value() returns the selected payload."""
    changed = pyqtSignal()

    def __init__(self, options, index=0):
        # options: [(label, value), ...]
        super().__init__()
        self._values = [v for _, v in options]
        self._btns = []
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self._grp = QButtonGroup(self)
        self._grp.setExclusive(True)
        for i, (label, _val) in enumerate(options):
            b = QPushButton(label)
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setChecked(i == index)
            b.setFixedHeight(34)
            self._grp.addButton(b, i)
            self._btns.append(b)
            lay.addWidget(b, 1)
        self._grp.buttonClicked.connect(self._on_click)
        self._restyle()

    def _on_click(self, _btn):
        self._restyle()
        self.changed.emit()

    def _restyle(self):
        n = len(self._btns)
        for i, b in enumerate(self._btns):
            left = "8px" if i == 0 else "0"
            right = "8px" if i == n - 1 else "0"
            if b.isChecked():
                bg, fg, weight = C["accent"], "#fff", 700
            else:
                bg, fg, weight = C["divider"], C["neutral700"], 600
            b.setStyleSheet(
                f"QPushButton{{background:{bg};color:{fg};border:none;"
                f"border-top-left-radius:{left};border-bottom-left-radius:{left};"
                f"border-top-right-radius:{right};border-bottom-right-radius:{right};"
                f"font-size:12px;font-weight:{weight};padding:0 6px;}}"
                f"QPushButton:disabled{{color:#b6b6b0;}}")

    def value(self):
        return self._values[self._grp.checkedId()]

    def set_value(self, val):
        if val in self._values:
            i = self._values.index(val)
            self._btns[i].setChecked(True)
            self._restyle()


class ClickRow(QFrame):
    """A clickable row (used as an accordion header)."""
    clicked = pyqtSignal()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(e)


class MotionModeChecker:
    """Query the robot's motion-service mode via MotionSwitcherClient.CheckMode().
    Walking/Regular mode => result['name'] is non-empty. Debug mode => ''."""
    # G1 loco FSM ids (verified on the real robot)
    FSM_ZERO_TORQUE = 0
    FSM_DAMP = 1
    FSM_SIT = 3
    FSM_STAND = 4
    FSM_WALK = 501
    FSM_RUN = 802
    GET_FSM_ID_API = 7001

    def __init__(self):
        self._msc = None
        self._loco = None

    def _client(self):
        if self._msc is None:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
                MotionSwitcherClient)
            c = MotionSwitcherClient()
            c.SetTimeout(0.4)
            c.Init()
            self._msc = c
        return self._msc

    def _loco_client(self):
        if self._loco is None:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            c = LocoClient()
            c.SetTimeout(0.4)
            c.Init()
            self._loco = c
        return self._loco

    # motion-service mode name that means the robot is in walking (Regular) mode
    WALK_MODE = "ai"

    def status(self):
        """Return (ok, walking, name). ok=False => could not query the robot.
        walking is True only for the walking-mode name ('ai')."""
        try:
            st, result = self._client().CheckMode()
            if st != 0 or not isinstance(result, dict):
                return False, False, None
            name = result.get("name", "") or ""
            return True, name == self.WALK_MODE, name
        except Exception:
            return False, False, None

    def fsm_id(self):
        """Return (ok, fsm_id). ok=False => could not query (e.g. debug mode, the
        loco service is off and the call times out)."""
        try:
            code, data = self._loco_client()._Call(self.GET_FSM_ID_API, "")
            if code != 0 or not data:
                return False, None
            import json
            return True, int(json.loads(data).get("data"))
        except Exception:
            return False, None


class WalkModeDialog(QDialog):
    """Modal overlay shown when '전신' is selected: guides the operator to put the
    robot into walking (Regular) mode; [계속] enables only once it is detected."""
    def __init__(self, parent, checker):
        super().__init__(parent)
        self._checker = checker
        self.setModal(True)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        dim = QWidget()
        dim.setStyleSheet("background:rgba(20,20,22,0.48);")
        outer.addWidget(dim)

        dl = QVBoxLayout(dim)
        dl.setContentsMargins(0, 0, 0, 0)
        dl.addStretch(1)
        crow = QHBoxLayout()
        crow.addStretch(1)

        card = QFrame()
        card.setFixedWidth(380)
        card.setStyleSheet(f"QFrame{{background:{C['card']};border-radius:16px;}}")
        cv = QVBoxLayout(card)
        cv.setContentsMargins(28, 28, 28, 24)
        cv.setSpacing(14)

        icon = QLabel("🚶")
        icon.setAlignment(Qt.AlignCenter)
        icon.setStyleSheet("font-size:40px;")
        cv.addWidget(icon)

        title = QLabel("걷기 모드로 전환해 주세요")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            f"font-size:17px;font-weight:700;color:{C['text']};")
        cv.addWidget(title)

        body = QLabel("전신(이동) 제어는 로봇이 <b>걷기 모드</b>일 때만 동작합니다.<br>"
                      "아래 순서대로 리모컨으로 모드를 올려 주세요.")
        body.setWordWrap(True)
        body.setAlignment(Qt.AlignCenter)
        body.setStyleSheet(
            f"font-size:13px;line-height:1.5;color:{C['neutral700']};")
        cv.addWidget(body)

        # FSM step list: highlights the robot's current stage on the way to walking
        self.STEPS = [
            (MotionModeChecker.FSM_ZERO_TORQUE, "제로토크"),
            (MotionModeChecker.FSM_DAMP, "댐핑"),
            (MotionModeChecker.FSM_STAND, "서기"),
            (MotionModeChecker.FSM_WALK, "걷기"),
        ]
        steps = QFrame()
        steps.setStyleSheet(f"QFrame{{background:{C['bg']};border-radius:10px;}}")
        sv = QVBoxLayout(steps)
        sv.setContentsMargins(16, 12, 16, 12)
        sv.setSpacing(8)
        self._step_labels = {}
        for i, (fid, name) in enumerate(self.STEPS):
            row = QLabel()
            row.setStyleSheet("font-size:13px;")
            sv.addWidget(row)
            self._step_labels[fid] = row
        cv.addWidget(steps)

        # off-sequence / unknown-state hint
        self._hint = QLabel()
        self._hint.setWordWrap(True)
        self._hint.setAlignment(Qt.AlignCenter)
        self._hint.setStyleSheet("font-size:12px;font-weight:600;color:#d64545;")
        self._hint.setVisible(False)
        cv.addWidget(self._hint)

        cv.addSpacing(4)
        brow = QHBoxLayout()
        brow.setSpacing(10)
        self._btn_cancel = QPushButton("취소")
        self._btn_cancel.setCursor(Qt.PointingHandCursor)
        self._btn_cancel.setFixedHeight(40)
        self._btn_cancel.setStyleSheet(
            f"QPushButton{{background:{C['divider']};color:{C['text']};border:none;"
            f"border-radius:8px;font-size:13px;font-weight:600;}}")
        self._btn_ok = QPushButton("계속")
        self._btn_ok.setCursor(Qt.PointingHandCursor)
        self._btn_ok.setFixedHeight(40)
        self._btn_ok.setEnabled(False)
        self._btn_ok.setStyleSheet(
            f"QPushButton{{background:{C['accent']};color:#fff;border:none;"
            f"border-radius:8px;font-size:13px;font-weight:700;}}"
            f"QPushButton:disabled{{background:{C['divider']};color:#aaa;}}")
        self._btn_cancel.clicked.connect(self.reject)
        self._btn_ok.clicked.connect(self.accept)
        brow.addWidget(self._btn_cancel, 1)
        brow.addWidget(self._btn_ok, 1)
        cv.addLayout(brow)

        crow.addWidget(card)
        crow.addStretch(1)
        dl.addLayout(crow)
        dl.addStretch(1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._render_steps(None)

    def _render_steps(self, current):
        # highlight only the step matching the robot's current fsm id
        for fid, name in self.STEPS:
            active = (fid == current)
            col = "#1f9d55" if active else "#b6b6b0"
            weight = 700 if active else 500
            check = "●" if active else "○"
            self._step_labels[fid].setText(
                f'<span style="color:{col}">{check}</span>'
                f'<span style="color:{col};font-weight:{weight}">&nbsp;&nbsp;{name}</span>')

    def _poll(self):
        ok, fid = self._checker.fsm_id()
        step_ids = [s[0] for s in self.STEPS]
        if not ok or fid is None:
            # loco service off (e.g. debug mode) or query failed
            self._render_steps(None)
            self._hint.setText("로봇 상태 확인 불가 — 리모컨으로 모드를 켜 주세요")
            self._hint.setVisible(True)
            self._btn_ok.setEnabled(False)
        elif fid in step_ids:
            self._render_steps(fid)
            self._hint.setVisible(False)
            self._btn_ok.setEnabled(fid == MotionModeChecker.FSM_WALK)
        else:
            # off the zero-torque -> damp -> stand -> walk path (e.g. sit, run)
            self._render_steps(None)
            other = {MotionModeChecker.FSM_SIT: "앉기",
                     MotionModeChecker.FSM_RUN: "러닝"}.get(fid, f"기타(id {fid})")
            self._hint.setText(f"현재 '{other}' 상태 — 걷기 순서를 벗어남. 걷기 모드로 맞춰 주세요")
            self._hint.setVisible(True)
            self._btn_ok.setEnabled(False)

    def showEvent(self, e):
        p = self.parent()
        if p is not None:
            tl = p.mapToGlobal(p.rect().topLeft())
            self.setGeometry(tl.x(), tl.y(), p.width(), p.height())
        self._poll()
        self._timer.start(500)
        super().showEvent(e)

    def hideEvent(self, e):
        self._timer.stop()
        super().hideEvent(e)


# fsm id -> human label
FSM_NAMES = {0: "제로토크", 1: "댐핑", 3: "앉기", 4: "서기", 501: "걷기", 802: "러닝"}
# states where the robot bears its own weight -> entering debug drops it
FSM_WEIGHT_BEARING = {4, 501, 802}


class DebugWarnDialog(QDialog):
    """Modal shown before launching 상체(debug) control: warns that entering debug
    mode releases the legs (fall risk). Acknowledgement only ([확인] always on);
    severity is raised live when the robot is currently weight-bearing."""
    def __init__(self, parent, checker):
        super().__init__(parent)
        self._checker = checker
        self.setModal(True)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        dim = QWidget()
        dim.setStyleSheet("background:rgba(20,20,22,0.48);")
        outer.addWidget(dim)
        dl = QVBoxLayout(dim)
        dl.setContentsMargins(0, 0, 0, 0)
        dl.addStretch(1)
        crow = QHBoxLayout()
        crow.addStretch(1)

        card = QFrame()
        card.setFixedWidth(380)
        card.setStyleSheet(f"QFrame{{background:{C['card']};border-radius:16px;}}")
        cv = QVBoxLayout(card)
        cv.setContentsMargins(28, 28, 28, 24)
        cv.setSpacing(14)

        icon = QLabel("⚠️")
        icon.setAlignment(Qt.AlignCenter)
        icon.setStyleSheet("font-size:40px;")
        cv.addWidget(icon)

        title = QLabel("디버그 모드 — 하체 힘 풀림")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size:17px;font-weight:700;color:{C['text']};")
        cv.addWidget(title)

        body = QLabel("상체(팔만) 제어는 <b>디버그 모드</b>로 진입해 다리 힘이 풀립니다.<br>"
                      "로봇이 주저앉을 수 있으니 지지 상태를 확인 후 진행하세요.")
        body.setWordWrap(True)
        body.setAlignment(Qt.AlignCenter)
        body.setStyleSheet(f"font-size:13px;line-height:1.5;color:{C['neutral700']};")
        cv.addWidget(body)

        # live severity line (depends on current fsm)
        self._sev = QLabel()
        self._sev.setWordWrap(True)
        self._sev.setAlignment(Qt.AlignCenter)
        self._sev.setStyleSheet("font-size:12px;font-weight:700;")
        cv.addWidget(self._sev)

        cv.addSpacing(4)
        brow = QHBoxLayout()
        brow.setSpacing(10)
        btn_cancel = QPushButton("취소")
        btn_cancel.setCursor(Qt.PointingHandCursor)
        btn_cancel.setFixedHeight(40)
        btn_cancel.setStyleSheet(
            f"QPushButton{{background:{C['divider']};color:{C['text']};border:none;"
            f"border-radius:8px;font-size:13px;font-weight:600;}}")
        btn_ok = QPushButton("확인, 진행")
        btn_ok.setCursor(Qt.PointingHandCursor)
        btn_ok.setFixedHeight(40)
        btn_ok.setStyleSheet(
            f"QPushButton{{background:{C['accent']};color:#fff;border:none;"
            f"border-radius:8px;font-size:13px;font-weight:700;}}")
        btn_cancel.clicked.connect(self.reject)
        btn_ok.clicked.connect(self.accept)
        brow.addWidget(btn_cancel, 1)
        brow.addWidget(btn_ok, 1)
        cv.addLayout(brow)

        crow.addWidget(card)
        crow.addStretch(1)
        dl.addLayout(crow)
        dl.addStretch(1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)

    def _poll(self):
        ok, fid = self._checker.fsm_id()
        if ok and fid in FSM_WEIGHT_BEARING:
            self._sev.setStyleSheet("font-size:12px;font-weight:700;color:#d64545;")
            self._sev.setText(f"⚠️ 현재 '{FSM_NAMES.get(fid, fid)}' — 진입 즉시 주저앉습니다!")
        elif ok and fid is not None:
            self._sev.setStyleSheet("font-size:12px;font-weight:700;color:#1f9d55;")
            self._sev.setText(f"현재 '{FSM_NAMES.get(fid, fid)}' — 지지 상태, 비교적 안전")
        else:
            self._sev.setStyleSheet(f"font-size:12px;font-weight:700;color:{C['neutral700']};")
            self._sev.setText("로봇 상태 확인 불가 — 지지 상태를 직접 확인하세요")

    def showEvent(self, e):
        p = self.parent()
        if p is not None:
            tl = p.mapToGlobal(p.rect().topLeft())
            self.setGeometry(tl.x(), tl.y(), p.width(), p.height())
        self._poll()
        self._timer.start(500)
        super().showEvent(e)

    def hideEvent(self, e):
        self._timer.stop()
        super().hideEvent(e)


# ----------------------------------------------------------------------------
# MuJoCo render worker (own thread => keeps EGL context local, like reference)
# ----------------------------------------------------------------------------
class MujocoWorker(QObject):
    frame_ready = pyqtSignal(QImage)
    status = pyqtSignal(str)

    def __init__(self, model_path, state_source):
        super().__init__()
        self._model_path = model_path
        self._src = state_source           # LowStateSource
        self._running = False
        self.cam_azimuth = 180.0
        self.cam_elevation = -15.0
        self.cam_distance = 2.8
        self._target_w = RENDER_W
        self._target_h = RENDER_H

    def set_target_size(self, w, h):
        """Requested render size (px). Renderer is recreated in the render thread."""
        self._target_w = int(max(MIN_RENDER_W, min(MAX_RENDER_W, w)))
        self._target_h = int(max(MIN_RENDER_H, min(MAX_RENDER_H, h)))

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _loop(self):
        try:
            import mujoco as mj
        except Exception as e:
            self.status.emit(f"MuJoCo import 실패: {e}")
            return
        try:
            model = mj.MjModel.from_xml_path(self._model_path)
            # bump offscreen framebuffer cap so we can recreate up to MAX_RENDER
            model.vis.global_.offwidth = MAX_RENDER_W
            model.vis.global_.offheight = MAX_RENDER_H
            data = mj.MjData(model)
            data.qpos[2] = STAND_Z
            data.qpos[3] = 1.0             # quat w = 1
            # robot geoms only (exclude worldbody floor) for floor-grounding
            robot_geom = np.asarray(model.geom_bodyid) > 0
            mj.mj_forward(model, data)
            cur_w, cur_h = self._target_w, self._target_h
            renderer = mj.Renderer(model, height=cur_h, width=cur_w)
            cam = mj.MjvCamera()
        except Exception as e:
            self.status.emit(f"MuJoCo 로드 실패: {e}")
            return

        self.status.emit("MuJoCo 로드 완료")
        interval = 1.0 / FPS
        while self._running:
            t0 = time.time()
            try:
                # recreate renderer if the target size changed meaningfully
                tw, th = self._target_w, self._target_h
                if abs(tw - cur_w) > RESIZE_THRESHOLD or abs(th - cur_h) > RESIZE_THRESHOLD:
                    try:
                        renderer.close()
                    except Exception:
                        pass
                    renderer = mj.Renderer(model, height=th, width=tw)
                    cur_w, cur_h = tw, th

                q = self._src.get_motor_q()
                if q is not None:
                    n = min(G1_NUM_MOTOR, len(q))
                    data.qpos[QPOS_OFFSET:QPOS_OFFSET + n] = q[:n]
                # tilt the pelvis to match the real robot's IMU orientation
                quat = self._src.get_pelvis_quat()
                if quat is not None and np.linalg.norm(quat) > 1e-6:
                    data.qpos[3:7] = quat / np.linalg.norm(quat)
                # the real robot's lowstate has no floating-base height, so pin the
                # pelvis then drop the model until its lowest geom rests on the floor.
                # keeps feet grounded for any leg pose (STAND_Z alone floats/sinks).
                data.qpos[2] = STAND_Z
                mj.mj_forward(model, data)
                zmin = float(data.geom_xpos[robot_geom, 2].min())
                if abs(zmin) > 1e-4:
                    data.qpos[2] -= zmin
                    mj.mj_forward(model, data)

                cam.type = mj.mjtCamera.mjCAMERA_FREE
                cam.azimuth = self.cam_azimuth
                cam.elevation = self.cam_elevation
                cam.distance = self.cam_distance
                cam.lookat[:] = [0.0, 0.0, 0.7]

                renderer.update_scene(data, camera=cam)
                img = renderer.render()          # (H, W, 3) uint8 RGB
                h, w, _ = img.shape
                qimg = QImage(img.data, w, h, 3 * w, QImage.Format_RGB888).copy()
                self.frame_ready.emit(qimg)
            except Exception:
                pass
            rem = interval - (time.time() - t0)
            if rem > 0:
                time.sleep(rem)
        try:
            renderer.close()
        except Exception:
            pass


# ----------------------------------------------------------------------------
# DDS low-state subscriber -> latest 29 motor angles
# ----------------------------------------------------------------------------
class LowStateSource:
    def __init__(self):
        self._q = None
        self._quat = None                  # pelvis IMU orientation [w,x,y,z]
        self._lock = threading.Lock()
        self._ok = False

    def start(self, domain, net):
        try:
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize, ChannelSubscriber)
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            ChannelFactoryInitialize(domain, net) if net else ChannelFactoryInitialize(domain)
            self._sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._sub.Init(self._on_msg, 10)
            self._ok = True
        except Exception as e:
            print(f"[LowStateSource] DDS init 실패: {e}", file=sys.stderr)
            self._ok = False

    def _on_msg(self, msg):
        try:
            q = np.array([msg.motor_state[i].q for i in range(G1_NUM_MOTOR)])
            quat = np.array(msg.imu_state.quaternion, dtype=float)   # [w,x,y,z]
            with self._lock:
                self._q = q
                self._quat = quat
        except Exception:
            pass

    def get_motor_q(self):
        with self._lock:
            return None if self._q is None else self._q.copy()

    def get_pelvis_quat(self):
        with self._lock:
            return None if self._quat is None else self._quat.copy()


# ----------------------------------------------------------------------------
# IPC bridge to the teleop process (commands + heartbeat)
# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
# camera view (head camera via ImageClient) — best effort
# ----------------------------------------------------------------------------
class CameraSource(QObject):
    frame_ready = pyqtSignal(QImage)
    status = pyqtSignal(str)

    def __init__(self, host):
        super().__init__()
        self._host = host
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False

    RECONNECT_WAIT = 3.0        # seconds between (re)connect attempts
    MAX_EMPTY_FRAMES = 90       # ~3s of no frames -> assume dropped, reconnect

    def _loop(self):
        try:
            from teleimager.image_client import ImageClient
        except Exception as e:
            self.status.emit(f"카메라 모듈 로드 실패: {e}")
            return
        # outer loop: keep (re)connecting until stopped. handles the robot/server
        # not being up at launch and server restarts mid-session.
        while self._running:
            client = None
            try:
                client = ImageClient(host=self._host)
            except Exception:
                self.status.emit("카메라 연결 시도 중… (서버 대기)")
                self._sleep(self.RECONNECT_WAIT)
                continue
            self.status.emit("카메라 연결됨")
            empty = 0
            while self._running:
                try:
                    img, _ = client.get_head_frame()
                    if img is None:
                        empty += 1
                        if empty >= self.MAX_EMPTY_FRAMES:
                            self.status.emit("카메라 끊김 — 재연결")
                            break            # drop client, reconnect from scratch
                    else:
                        empty = 0
                        if img.ndim == 2:
                            img = np.stack([img] * 3, axis=-1)
                        img = np.ascontiguousarray(img[:, :, ::-1])  # BGR -> RGB
                        h, w, _ = img.shape
                        qimg = QImage(img.data, w, h, 3 * w, QImage.Format_RGB888).copy()
                        self.frame_ready.emit(qimg)
                except Exception:
                    self.status.emit("카메라 오류 — 재연결")
                    break
                time.sleep(1.0 / FPS)
            try:
                if client is not None:
                    client.close()
            except Exception:
                pass

    def _sleep(self, seconds):
        # interruptible sleep so stop() is honored promptly
        end = time.time() + seconds
        while self._running and time.time() < end:
            time.sleep(0.1)


# ----------------------------------------------------------------------------
# stage widget: main feed + PiP feed, click PiP to swap
# ----------------------------------------------------------------------------
class VideoLabel(QLabel):
    clicked = pyqtSignal()               # press+release without drag
    dragged = pyqtSignal(int, int)       # (dx, dy) while left button held
    wheel_scrolled = pyqtSignal(int)     # +1 up / -1 down

    def __init__(self, placeholder, parent=None):
        super().__init__(parent)
        self._placeholder = placeholder
        self._pix = None
        self._press = None
        self._moved = False
        self.setAlignment(Qt.AlignCenter)
        self.setText(placeholder)
        self.setStyleSheet(
            "color:#8a8a90;font-size:13px;letter-spacing:.03em;"
            "background:#232327;")

    def set_frame(self, qimg):
        self._pix = QPixmap.fromImage(qimg)
        self._update_scaled()

    def _update_scaled(self):
        if self._pix is None:
            return
        self.setPixmap(self._pix.scaled(
            self.size(), Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation))

    def resizeEvent(self, e):
        self._update_scaled()
        super().resizeEvent(e)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._press = e.pos()
            self._moved = False
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if self._press is not None and (e.buttons() & Qt.LeftButton):
            dx = e.pos().x() - self._press.x()
            dy = e.pos().y() - self._press.y()
            if abs(dx) + abs(dy) > 2:
                self._moved = True
            self._press = e.pos()
            self.dragged.emit(dx, dy)
        super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton and self._press is not None and not self._moved:
            self.clicked.emit()
        self._press = None
        super().mouseReleaseEvent(e)

    def wheelEvent(self, e):
        self.wheel_scrolled.emit(1 if e.angleDelta().y() > 0 else -1)


class Stage(QWidget):
    """Holds two VideoLabels; one is main (fills), other is PiP bottom-right."""
    def __init__(self):
        super().__init__()
        self.setStyleSheet(f"background:{C['neutral900']};")
        self.sim = VideoLabel("시뮬레이션 화면", self)
        self.cam = VideoLabel("로봇 카메라 시점", self)
        self.sim_label = self._badge("시뮬레이션")
        self.cam_label = self._badge("카메라 뷰")
        self.main_view = "sim"
        self._mj = None                 # MujocoWorker, set via set_mujoco()
        for lbl in (self.sim, self.cam):
            lbl.clicked.connect(lambda l=lbl: self._on_click(l))
            lbl.dragged.connect(lambda dx, dy, l=lbl: self._on_drag(l, dx, dy))
            lbl.wheel_scrolled.connect(lambda d, l=lbl: self._on_wheel(l, d))
        self._relayout()

    def set_mujoco(self, worker):
        self._mj = worker
        self._relayout()

    def _is_pip(self, lbl):
        main = self.sim if self.main_view == "sim" else self.cam
        return lbl is not main

    def _sim_is_main(self, lbl):
        return lbl is self.sim and self.main_view == "sim"

    # PiP click -> swap. Main click -> ignored (main sim uses drag/scroll for cam).
    def _on_click(self, lbl):
        if self._is_pip(lbl):
            self._swap()

    def _on_drag(self, lbl, dx, dy):
        if self._mj is None or not self._sim_is_main(lbl):
            return
        self._mj.cam_azimuth = (self._mj.cam_azimuth - dx * 0.4) % 360
        self._mj.cam_elevation = max(-89.0, min(0.0, self._mj.cam_elevation - dy * 0.3))

    def _on_wheel(self, lbl, direction):
        if self._mj is None or not self._sim_is_main(lbl):
            return
        # scroll up (dir +1) -> zoom in (decrease distance)
        self._mj.cam_distance = max(0.5, min(6.0, self._mj.cam_distance - direction * 0.25))

    def _badge(self, text):
        lb = QLabel(text, self)
        lb.setStyleSheet(
            f"background:{C['neutral900']};color:#fff;font-weight:600;"
            "letter-spacing:.04em;padding:5px 10px;font-size:11px;")
        return lb

    def _swap(self):
        self.main_view = "camera" if self.main_view == "sim" else "sim"
        self._relayout()

    def _relayout(self):
        w, h = self.width(), self.height()
        pip_w, pip_h = 300, 168
        sim_is_main = self.main_view == "sim"
        main, pip = (self.sim, self.cam) if sim_is_main else (self.cam, self.sim)
        main_lb, pip_lb = ((self.sim_label, self.cam_label) if sim_is_main
                           else (self.cam_label, self.sim_label))

        main.setGeometry(0, 0, w, h)
        main.lower()
        pip.setGeometry(w - pip_w - 24, h - pip_h - 24, pip_w, pip_h)
        pip.raise_()
        pip.setStyleSheet(pip.styleSheet() + "border:2px solid #fff;")
        main.setStyleSheet("background:#232327;color:#8a8a90;font-size:13px;")

        pip.setCursor(Qt.PointingHandCursor)
        main.setCursor(Qt.SizeAllCursor if sim_is_main else Qt.ArrowCursor)

        main_lb.setGeometry(16, 16, main_lb.sizeHint().width(), 24)
        main_lb.raise_()
        pip_lb.setGeometry(pip.x() + 8, pip.y() + 8, pip_lb.sizeHint().width(), 22)
        pip_lb.raise_()

        # match MuJoCo render resolution to the sim label's current pixel size
        if self._mj is not None:
            dpr = self.devicePixelRatioF() if hasattr(self, "devicePixelRatioF") else 1.0
            self._mj.set_target_size(max(1, int(self.sim.width() * dpr)),
                                     max(1, int(self.sim.height() * dpr)))

    def resizeEvent(self, e):
        self._relayout()
        super().resizeEvent(e)

    # routing frames regardless of which is main
    def set_sim_frame(self, qimg):
        self.sim.set_frame(qimg)

    def set_cam_frame(self, qimg):
        self.cam.set_frame(qimg)


# ----------------------------------------------------------------------------
# main window
# ----------------------------------------------------------------------------
class Dashboard(QWidget):
    _log_signal = pyqtSignal(str)      # thread-safe logging (from proc pipe thread)

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.setWindowTitle("Humanoid Teleoperation")
        self.resize(1280, 800)
        self.setStyleSheet(f"background:{C['bg']};")

        self._phase = "off"            # off | starting | ready | running | paused
        self._elapsed = 0
        self._xr = {}                  # latest safety snapshot from the heartbeat
        self._align = None             # latest AlignReport dict, or None
        self.ipc = None                # set below; _build_ui may query it early
        self.proc = None
        self._stop_deadline = None   # time by which the proc group must exit after CMD_STOP
        self._kill_deadline = None   # time by which it must be gone after the SIGTERM escalation
        # --- teleop supervision -------------------------------------------
        # The XrLink websocket server that the headset connects to lives inside
        # the teleop process, so every teleop exit -- 비상 정지, a crash, a signal
        # -- drops the headset's link. Respawning keeps the device connected for as
        # long as this window is open; the operator should not have to re-pair
        # from inside the headset after every session.
        self._supervise = True       # cleared on window close so we stop respawning
        self._respawn_timer = None   # pending QTimer for the next respawn
        self._term_requested = False # set by the SIGTERM/SIGINT/SIGHUP handler
        self._respawn_backoff = 0.0  # grows while the child keeps dying young
        self._launch_time = 0.0      # monotonic time of the last spawn
        # A respawn starts a process whose SafetyFSM has never seen the e-stop,
        # so the latch has to live out here to survive it. Cleared only by
        # [안전정지 해제].
        self._estop_latched = False
        self.mode_checker = MotionModeChecker()   # robot walking-mode probe (real robot)
        self._log_signal.connect(self._log)

        self._build_ui()

        # timers
        self._sec_timer = QTimer(self)
        self._sec_timer.timeout.connect(self._tick)
        self._proc_timer = QTimer(self)
        self._proc_timer.timeout.connect(self._poll_proc)
        # Always-on: decays the XR panel when heartbeats stop, so the operator
        # never sees a stale "all good" after the teleop process has died.
        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh_xr)
        self._ui_timer.start(250)

        # SIGTERM/SIGINT/SIGHUP -> the same teardown as closing the window.
        # Qt's event loop sits in C code, so a Python signal handler does not run
        # until the interpreter gets control back; the handler therefore only
        # sets a flag and this timer -- which is already ticking -- acts on it.
        # Without this, `pkill dashboard.py` or a Ctrl-C in the launching shell
        # killed the dashboard outright and left the teleop child running: an
        # orphan still holding port 8443, so the next dashboard's own child could
        # not bind it.
        self._sig_timer = QTimer(self)
        self._sig_timer.timeout.connect(self._check_term)
        self._sig_timer.start(200)
        self._install_signal_handlers()

        # --- data sources ---
        self.state_src = LowStateSource()
        self.state_src.start(args.domain, args.net)

        self.mj = MujocoWorker(self._resolve_model(), self.state_src)
        self.stage.set_mujoco(self.mj)
        self.mj.frame_ready.connect(self.stage.set_sim_frame)
        self.mj.status.connect(lambda s: self._log(s))
        self.mj.start()

        self.cam = CameraSource(args.img_server_ip)
        self.cam.frame_ready.connect(self.stage.set_cam_frame)
        self.cam.status.connect(lambda s: self._log(s))
        if args.camera:
            self.cam.start()

        try:
            self.ipc = IPC_Client()
        except Exception as e:
            self.ipc = None
            print(f"[dashboard] IPC disabled: {e}", file=sys.stderr)
        # Heartbeats are polled rather than pushed: IPC_Client already keeps the
        # newest state and its own liveness, so a timer avoids a second thread
        # marshalling into the Qt event loop.
        self._hb_timer = QTimer(self)
        self._hb_timer.timeout.connect(self._poll_heartbeat)
        self._hb_timer.start(100)

        self._log("대시보드 준비 완료" + ("" if self.ipc else " (IPC 미연결)"))

        # Launch the teleop subprocess as soon as the window is up, instead of
        # waiting on a manual 실행 click. Deferred via singleShot(0, ...) rather
        # than called directly here: _on_launch can pop a modal safety dialog
        # (real-robot walk-mode / debug-mode warning), which needs the main
        # window to already be shown, and show() has not run yet at this point
        # in __init__.
        QTimer.singleShot(0, self._on_launch)

    # --- ui -----------------------------------------------------------------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # topbar
        top = QWidget()
        top.setStyleSheet(f"border-bottom:2px solid {C['divider']};")
        tl = QHBoxLayout(top)
        tl.setContentsMargins(32, 18, 32, 18)
        h1 = QLabel("Humanoid Teleoperation")
        h1.setStyleSheet(f"font-size:22px;font-weight:700;color:{C['text']};")
        sub = QLabel("Simulation & Camera Streaming")
        sub.setStyleSheet(f"font-size:13px;color:{C['neutral700']};")
        tl.addWidget(h1)
        tl.addStretch(1)
        tl.addWidget(sub)
        top.setFixedHeight(60)
        root.addWidget(top)

        # body: stage | side
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)

        self.stage = Stage()
        self.stage.setStyleSheet(
            self.stage.styleSheet() + f"border-right:2px solid {C['divider']};")
        self.stage.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        body.addWidget(self.stage, 1)

        # The side column lives inside a scroll area so that expanding the
        # settings accordion scrolls instead of growing the window: a plain
        # child widget propagates its minimum height up to the window, which
        # forces a resize the moment the accordion opens.
        side = QWidget()
        side.setStyleSheet(f"background:{C['bg']};")
        sl = QVBoxLayout(side)
        sl.setContentsMargins(24, 24, 24, 24)
        sl.setSpacing(20)

        sl.addWidget(self._settings_card())
        sl.addWidget(self._xr_card())
        sl.addWidget(self._status_card())
        sl.addWidget(self._log_card(), 1)

        side_scroll = QScrollArea()
        side_scroll.setWidget(side)
        side_scroll.setWidgetResizable(True)
        side_scroll.setFixedWidth(340)
        side_scroll.setFrameShape(QFrame.NoFrame)
        side_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        side_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        side_scroll.setStyleSheet(
            f"QScrollArea{{background:{C['bg']};border:none;}}"
            "QScrollBar:vertical{background:transparent;width:8px;margin:6px 2px 6px 0;}"
            "QScrollBar::handle:vertical{background:#c9c9c4;min-height:28px;border-radius:4px;}"
            "QScrollBar::handle:vertical:hover{background:#a9a9a2;}"
            "QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}"
            "QScrollBar::add-page:vertical,QScrollBar::sub-page:vertical{background:transparent;}")

        body.addWidget(side_scroll)
        root.addLayout(body, 1)

    def _card(self):
        f = QFrame()
        f.setStyleSheet(
            f"QFrame{{background:{C['card']};border-radius:10px;}}")
        return f

    def _caption(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet(
            f"font-size:11px;font-weight:700;letter-spacing:.04em;"
            f"color:{C['neutral700']};")
        return lbl

    def _settings_card(self):
        card = self._card()
        self.settings_card = card
        outer = QVBoxLayout(card)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # --- clickable accordion header (collapsed by default) ---
        header = ClickRow()
        header.setCursor(Qt.PointingHandCursor)
        # transparent so it shows the parent card's white bg instead of drawing
        # its own rounded box (which would inherit the QFrame card style)
        header.setStyleSheet("QFrame{background:transparent;border:none;border-radius:0px;}")
        header.clicked.connect(self._toggle_settings)
        hl = QHBoxLayout(header)
        hl.setContentsMargins(20, 16, 20, 16)
        htitle = QLabel("⚙  설정")
        htitle.setStyleSheet(
            f"font-size:11px;font-weight:700;letter-spacing:.08em;color:{C['neutral700']};")
        self._settings_chevron = QLabel("▸")
        self._settings_chevron.setStyleSheet(f"font-size:11px;color:{C['neutral700']};")
        hl.addWidget(htitle)
        hl.addStretch(1)
        hl.addWidget(self._settings_chevron)
        outer.addWidget(header)

        # --- collapsible body ---
        self.settings_body = QWidget()
        # transparent (scoped) so the parent card's white bg shows through; a plain
        # child QWidget otherwise paints the page gray over the card area
        self.settings_body.setObjectName("settingsBody")
        self.settings_body.setStyleSheet("QWidget#settingsBody{background:transparent;}")
        self.settings_body.setVisible(False)   # hidden by default
        v = QVBoxLayout(self.settings_body)
        v.setContentsMargins(20, 0, 20, 20)
        v.setSpacing(14)
        outer.addWidget(self.settings_body)

        # 1) VR입력 : --input-mode  (controller | hand)
        v.addWidget(self._caption("VR입력"))
        self.set_inputmode = Segmented(
            [("컨트롤러", "controller"), ("손 추적", "hand")],
            index=(0 if self.args.input_mode != "hand" else 1))
        v.addWidget(self.set_inputmode)

        # 2) 제어범위 : --motion  (상체=no motion / 전신=motion)
        v.addWidget(self._caption("제어범위"))
        self.set_motion = Segmented(
            [("상체 (팔만)", False), ("전신 (이동)", True)],
            index=(1 if self.args.motion else 0))
        self.set_motion.changed.connect(self._on_motion_changed)
        v.addWidget(self.set_motion)

        # 3) 네트워크 : --network-interface  (dropdown of live ifaces)
        v.addWidget(self._caption("네트워크"))
        self.cmb_net = QComboBox()
        self.cmb_net.setFixedHeight(34)
        self.cmb_net.setCursor(Qt.PointingHandCursor)
        self.cmb_net.setStyleSheet(
            f"QComboBox{{background:{C['divider']};color:{C['text']};border:none;"
            f"border-radius:8px;padding:0 12px;font-size:12px;font-weight:600;}}"
            f"QComboBox::drop-down{{border:none;width:22px;}}"
            f"QComboBox QAbstractItemView{{background:{C['card']};color:{C['text']};"
            f"selection-background-color:{C['accent']};selection-color:#fff;"
            f"border:1px solid {C['divider']};outline:none;}}")
        self._populate_net()
        v.addWidget(self.cmb_net)

        # 4) 카메라서버 : --img-server-ip  (read-only + edit toggle)
        v.addWidget(self._caption("카메라서버"))
        camrow = QHBoxLayout()
        camrow.setSpacing(8)
        self.ed_camip = QLineEdit(self.args.img_server_ip)
        self.ed_camip.setReadOnly(True)
        self.ed_camip.setFixedHeight(34)
        self._style_camip(False)
        self.btn_camedit = QToolButton()
        self.btn_camedit.setText("✎")
        self.btn_camedit.setCursor(Qt.PointingHandCursor)
        self.btn_camedit.setFixedSize(34, 34)
        self.btn_camedit.setToolTip("카메라서버 IP 편집")
        self.btn_camedit.setStyleSheet(
            f"QToolButton{{background:{C['divider']};color:{C['neutral700']};"
            f"border:none;border-radius:8px;font-size:14px;}}"
            f"QToolButton:hover{{background:#d6d6d0;}}"
            f"QToolButton:disabled{{color:#b6b6b0;}}")
        self.btn_camedit.clicked.connect(self._toggle_camip_edit)
        camrow.addWidget(self.ed_camip, 1)
        camrow.addWidget(self.btn_camedit)
        v.addLayout(camrow)

        return card

    def _populate_net(self):
        self.cmb_net.clear()
        ifaces = list_net_ifaces()
        preferred = getattr(self.args, "net", None)
        sel = 0
        for i, (iface, ip) in enumerate(ifaces):
            mark = "  ✓" if is_dds_ip(ip) else ""
            self.cmb_net.addItem(f"{iface}  ({ip}){mark}", iface)
            # preselect: explicit --net wins, else first DDS-subnet iface
            if preferred and iface == preferred:
                sel = i
            elif not preferred and is_dds_ip(ip) and sel == 0:
                sel = i
        if ifaces:
            self.cmb_net.setCurrentIndex(sel)
        else:
            self.cmb_net.addItem("(인터페이스 없음)", None)

    def _style_camip(self, editable):
        if editable:
            self.ed_camip.setStyleSheet(
                f"QLineEdit{{background:{C['card']};color:{C['text']};"
                f"border:2px solid {C['accent']};border-radius:8px;padding:0 10px;"
                f"font-size:12px;font-weight:600;}}")
        else:
            self.ed_camip.setStyleSheet(
                f"QLineEdit{{background:{C['divider']};color:{C['neutral700']};"
                f"border:none;border-radius:8px;padding:0 10px;"
                f"font-size:12px;font-weight:600;}}")

    def _toggle_camip_edit(self):
        editable = self.ed_camip.isReadOnly()   # currently RO -> switch to editable
        self.ed_camip.setReadOnly(not editable)
        self._style_camip(editable)
        self.btn_camedit.setText("✓" if editable else "✎")
        if editable:
            self.ed_camip.setFocus()
            self.ed_camip.selectAll()

    def _toggle_settings(self):
        show = not self.settings_body.isVisible()
        self.settings_body.setVisible(show)
        self._settings_chevron.setText("▾" if show else "▸")

    def _on_motion_changed(self):
        # selecting 전신(이동) on a real robot -> guide operator into walking mode.
        # informational at toggle time: cancel/continue both keep the 전신 choice
        # (never revert to 상체, which could imply switching the robot to debug mode).
        if self.set_motion.value() and self.args.domain == 0:
            self._open_walk_dialog()

    def _open_walk_dialog(self):
        """Show the walking-mode gate. Returns True if [계속] pressed."""
        dlg = WalkModeDialog(self, self.mode_checker)
        return dlg.exec_() == QDialog.Accepted

    def _open_debug_warn_dialog(self):
        """Show the debug-mode fall-risk warning. Returns True if [확인] pressed."""
        dlg = DebugWarnDialog(self, self.mode_checker)
        return dlg.exec_() == QDialog.Accepted

    # --- XR headset / safety panel -------------------------------------------
    # Mirrors the teleop process's SafetyFSM snapshot. This is what tells the
    # operator *why* 시작 is unavailable, instead of leaving them to guess.
    XR_SAFETY_LABELS = {
        "idle": "대기", "following": "추종 중", "hold": "일시 정지(홀드)",
        "safe_stop": "안전 정지",
    }

    def _xr_card(self):
        card = self._card()
        v = QVBoxLayout(card)
        v.setContentsMargins(20, 18, 20, 18)
        v.setSpacing(12)

        row = QHBoxLayout()
        kicker = QLabel("XR 헤드셋")
        kicker.setStyleSheet(
            f"font-size:11px;font-weight:700;letter-spacing:.08em;color:{C['neutral700']};")
        self.xr_tag = QLabel("—")
        row.addWidget(kicker)
        row.addStretch(1)
        row.addWidget(self.xr_tag)
        v.addLayout(row)

        self.xr_rows = {}
        for key, label in (("link", "링크"), ("worn", "착용"),
                           ("stale", "지연"), ("safety", "안전")):
            r = QHBoxLayout()
            lb = QLabel(label)
            lb.setStyleSheet(f"font-size:12px;color:{C['neutral700']};")
            val = QLabel("—")
            val.setStyleSheet(f"font-size:12px;font-weight:600;color:{C['text']};")
            r.addWidget(lb)
            r.addStretch(1)
            r.addWidget(val)
            v.addLayout(r)
            self.xr_rows[key] = val

        self.xr_reason = QLabel()
        self.xr_reason.setWordWrap(True)
        self.xr_reason.setStyleSheet("font-size:11px;font-weight:600;color:#d64545;")
        self.xr_reason.setVisible(False)
        v.addWidget(self.xr_reason)

        # --- start-alignment progress (visible only while aligning) ----------
        self.align_box = QWidget()
        self.align_box.setVisible(False)
        av = QVBoxLayout(self.align_box)
        av.setContentsMargins(0, 6, 0, 0)
        av.setSpacing(6)
        self.align_title = QLabel("정렬 중")
        self.align_title.setStyleSheet(
            f"font-size:12px;font-weight:700;color:{C['accent']};")
        av.addWidget(self.align_title)
        self.align_reason = QLabel()
        self.align_reason.setWordWrap(True)
        self.align_reason.setStyleSheet(f"font-size:11px;color:{C['neutral700']};")
        av.addWidget(self.align_reason)
        self.align_bar = QFrame()
        self.align_bar.setFixedHeight(6)
        self.align_bar.setStyleSheet(
            f"QFrame{{background:{C['divider']};border-radius:3px;}}")
        self.align_fill = QFrame(self.align_bar)
        self.align_fill.setGeometry(0, 0, 0, 6)
        self.align_fill.setStyleSheet(
            f"QFrame{{background:{C['accent']};border-radius:3px;}}")
        av.addWidget(self.align_bar)
        self.align_err = QLabel()
        self.align_err.setStyleSheet(
            "font-size:11px;font-family:monospace;color:#6b6b66;")
        av.addWidget(self.align_err)
        self.btn_align_cancel = self._btn("정렬 취소")
        self.btn_align_cancel.clicked.connect(self._on_cancel_align)
        av.addWidget(self.btn_align_cancel)
        v.addWidget(self.align_box)

        self.btn_ack = self._btn("안전정지 해제")
        self.btn_ack.setEnabled(False)
        self.btn_ack.setToolTip("래치된 안전 정지를 해제합니다 (해제 후 [시작] 가능)")
        self.btn_ack.clicked.connect(self._on_ack)
        v.addWidget(self.btn_ack)

        self._set_xr_tag(None)
        return card

    def _set_xr_tag(self, ok, text=None):
        if text is None:
            text = {True: "정상", False: "주의", None: "미연결"}[ok]
        bg = {True: "#1f9d55", False: "#d64545", None: C["divider"]}[ok]
        fg = C["neutral700"] if ok is None else "#fff"
        self.xr_tag.setText(text)
        self.xr_tag.setStyleSheet(
            f"font-size:11px;font-weight:700;padding:4px 10px;border-radius:10px;"
            f"background:{bg};color:{fg};")

    def _send_cmd(self, cmd, require_online=True):
        """Send an IPC command. Returns (ok, message)."""
        if self.ipc is None:
            return False, "IPC 미연결"
        rep = self.ipc.send_data(cmd, require_online=require_online)
        return rep.get("status") == "ok", rep.get("msg", "")

    def _poll_heartbeat(self):
        if self.ipc is None:
            return
        state = self.ipc.latest_state()
        if state:
            self._on_heartbeat(state)
        else:
            self._refresh_xr()   # nothing arriving -> let the panel decay

    def _hb_fresh(self):
        """Heartbeats arrive at 10Hz; treat a 1.5s gap as telemetry lost.

        Without this the panel would keep displaying the last known-good XR
        state after the teleop process dies -- the exact moment it must not be
        trusted."""
        return self.ipc is not None and self.ipc.heartbeat_age() < 1.5

    def _refresh_xr(self):
        xr = self._xr
        if not xr or not self._hb_fresh():
            self._refresh_align(None)
            for val in self.xr_rows.values():
                val.setText("—")
            self.xr_reason.setVisible(False)
            # stays clickable with no heartbeat: after 비상 정지 the teleop process
            # is exiting/respawning, and the latch to clear is the local one.
            self.btn_ack.setEnabled(self._estop_latched)
            self._set_xr_tag(None)
            self._apply_button_state()
            return

        link_up = bool(xr.get("link_up"))
        worn = xr.get("worn")
        state = xr.get("state") or "—"
        latched = bool(xr.get("latched"))

        self.xr_rows["link"].setText("연결됨" if link_up else "끊김")
        self.xr_rows["worn"].setText(
            {True: "착용 중", False: "벗음", None: "알 수 없음"}.get(worn, "알 수 없음"))
        self.xr_rows["stale"].setText(f"{int(xr.get('stale_ms', 0))} ms")
        self.xr_rows["safety"].setText(self.XR_SAFETY_LABELS.get(state, state))

        reason = xr.get("reason") or ""
        self.xr_reason.setText(reason)
        self.xr_reason.setVisible(bool(reason))
        self.btn_ack.setEnabled(latched or self._estop_latched)

        if self._estop_latched:
            self._set_xr_tag(False, "비상 정지")
        elif latched:
            self._set_xr_tag(False, "안전 정지")
        elif not link_up:
            self._set_xr_tag(False, "링크 끊김")
        elif worn is False:
            self._set_xr_tag(False, "미착용")
        elif state == "hold":
            self._set_xr_tag(False, "홀드")
        else:
            self._set_xr_tag(True)
        self._apply_button_state()

    def _xr_block_reason(self):
        """Why 시작 is unavailable, or None if it is fine to start."""
        # Checked before the heartbeat: an e-stop survives the respawn that
        # follows it, so it must still block 시작 while the new process is
        # starting up and no heartbeat has arrived yet.
        if self._estop_latched:
            return "비상 정지됨 — [안전정지 해제] 후 시작"
        if not self._hb_fresh():
            return "텔레옵 상태 수신 없음"
        xr = self._xr
        if not xr:
            return "XR 상태 미수신"
        if xr.get("latched"):
            return "안전 정지 래치됨 — [안전정지 해제] 후 시작"
        if not xr.get("link_up"):
            return "헤드셋 링크 끊김"
        if xr.get("worn") is False:
            return "헤드셋 미착용"
        return None

    def _refresh_align(self, align):
        """Render the start-alignment gate. `align` is None when not aligning."""
        if not align or not self._hb_fresh():
            self.align_box.setVisible(False)
            return
        self.align_box.setVisible(True)
        self.align_reason.setText(align.get("reason", ""))

        progress = float(align.get("progress", 0.0) or 0.0)
        self.align_fill.setGeometry(
            0, 0, int(self.align_bar.width() * max(0.0, min(1.0, progress))), 6)

        def fmt(pos, rot):
            if pos is None or rot is None:
                return "  --"
            return f"{pos * 100:5.1f}cm {rot:5.1f}°"
        self.align_err.setText(
            f"L {fmt(align.get('left_pos_err'), align.get('left_rot_err'))}\n"
            f"R {fmt(align.get('right_pos_err'), align.get('right_rot_err'))}")

        # `within_tolerance` gates acceptance on the guided path, same as
        # before -- unless the operator is also holding the in-VR skip (both
        # A/X), which this summary line doesn't distinguish from "still
        # aligning". See teleop/safety/align.py.
        ok = bool(align.get("within_tolerance"))
        self.align_title.setText("정렬 확인됨 — 확정 대기" if ok else "정렬 중")
        self.align_title.setStyleSheet(
            f"font-size:12px;font-weight:700;"
            f"color:{'#1f9d55' if ok else C['accent']};")

    def _on_cancel_align(self):
        ok, msg = self._send_cmd("CMD_CANCEL_ALIGN")
        self._log(f"[IPC] CMD_CANCEL_ALIGN -> {'ok' if ok else msg}")

    def _on_ack(self):
        # Clears both latches: the teleop-side SafetyFSM one (if that process is
        # still the one that latched) and the dashboard-side e-stop one, which is
        # the only record left after a respawn.
        was_estop = self._estop_latched
        self._estop_latched = False
        ok, msg = self._send_cmd("CMD_ACK_FAULT", require_online=False)
        self._log(f"[IPC] CMD_ACK_FAULT -> {'ok' if ok else msg}"
                  + (" (비상 정지 해제됨)" if was_estop else ""))
        self._apply_button_state()

    def _on_estop(self):
        """The one stop: arms home under the safe ceiling, teleop exits, a fresh
        process comes back. Absorbs what [종료] used to do -- see _status_card."""
        if self._phase == "off":
            return
        # Latched here, not only in the teleop process: the supervisor brings a
        # fresh process back to keep the headset link up, and that process has a
        # new SafetyFSM which never saw the e-stop. Without a latch on this side
        # the operator could be offered [시작] again mid-shutdown.
        self._estop_latched = True
        ok, msg = self._send_cmd("CMD_ESTOP", require_online=False)
        self._log(f"[IPC] CMD_ESTOP -> {'ok' if ok else msg}")
        self._sec_timer.stop()
        self._elapsed = 0
        self.time_lbl.setText("00:00")
        self._set_tag(False, "비상 정지")
        self._log("비상 정지 — 팔 홈 복귀 후 종료, 이후 자동 재기동")
        # If the shutdown wedges, _poll_proc SIGTERMs the group then SIGKILLs it.
        # STOP_GRACE has to cover the deliberate slow homing move; killing during
        # it would strand the arms mid-swing, the thing this path exists to avoid.
        #
        # Only arm the deadline on the first press: a pressed-again-because-
        # nothing-seems-to-happen click must not push the escalation further out.
        # It used to reset unconditionally, so an operator repeatedly pressing
        # stop while the subprocess was stuck (e.g. an unbounded SDK call in its
        # shutdown path -- see motion_switcher.Exit_Debug_Mode) kept deferring the
        # one-shot SIGTERM instead of getting closer to it.
        if self._stop_deadline is None:
            self._stop_deadline = time.time() + self.STOP_GRACE
        self._apply_button_state()

    def _status_card(self):
        card = self._card()
        v = QVBoxLayout(card)
        v.setContentsMargins(20, 20, 20, 20)
        v.setSpacing(16)

        row = QHBoxLayout()
        kicker = QLabel("상태")
        kicker.setStyleSheet(
            f"font-size:11px;font-weight:700;letter-spacing:.08em;color:{C['neutral700']};")
        self.status_tag = QLabel("정지됨")
        self._set_tag(False)
        row.addWidget(kicker)
        row.addStretch(1)
        row.addWidget(self.status_tag)
        v.addLayout(row)

        trow = QHBoxLayout()
        tl = QLabel("경과 시간")
        tl.setStyleSheet(f"font-size:12px;color:{C['neutral700']};")
        self.time_lbl = QLabel("00:00")
        self.time_lbl.setStyleSheet(
            f"font-size:28px;font-weight:700;color:{C['text']};")
        trow.addWidget(tl)
        trow.addStretch(1)
        trow.addWidget(self.time_lbl)
        v.addLayout(trow)

        # No manual "실행" button: the teleop subprocess launches itself once,
        # right after the window is shown (see __init__'s deferred _on_launch
        # call). 시작 stays gated the same way it always was -- phase must reach
        # "ready" *and* _xr_block_reason() must clear, which requires a live XR
        # link -- so auto-launching does not skip the "headset must actually be
        # connected" check, it just removes the busywork step in front of it.
        self.btn_start = self._btn("시작", primary=True)
        self.btn_pause = self._btn("정지")
        self.btn_start.clicked.connect(self._on_start)
        self.btn_pause.clicked.connect(self._on_pause)

        row1 = QHBoxLayout(); row1.setSpacing(10)
        row1.addWidget(self.btn_start); row1.addWidget(self.btn_pause)
        v.addLayout(row1)

        # One stop, not two. [종료] and [비상 정지] used to differ -- 종료 exited,
        # 비상 정지 only latched following off -- but both now run the same path:
        # freeze, drop to SAFE_ARM_VELOCITY, walk the arms home, exit, and let the
        # supervisor bring a fresh process back. Two buttons doing the identical
        # thing is worse than one: in the moment you need this, picking between
        # them is hesitation. The remaining ladder is 정지 (홈 복귀 후 홀드,
        # resumable) and this.
        self.btn_estop = QPushButton("비상 정지")
        self.btn_estop.setCursor(Qt.PointingHandCursor)
        self.btn_estop.setStyleSheet(
            "QPushButton{background:#d64545;color:#fff;border:none;border-radius:8px;"
            "padding:11px 12px;font-size:13px;font-weight:700;}"
            "QPushButton:hover{background:#bf3a3a;}"
            f"QPushButton:disabled{{color:#aaa;background:{C['divider']};}}")
        self.btn_estop.clicked.connect(self._on_estop)
        v.addWidget(self.btn_estop)

        self._apply_button_state()
        return card

    def _log_card(self):
        card = self._card()
        v = QVBoxLayout(card)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        head = QLabel("로그")
        head.setStyleSheet(
            f"font-size:11px;font-weight:700;letter-spacing:.08em;color:{C['neutral700']};"
            f"padding:14px 20px;border-bottom:2px solid {C['divider']};")
        v.addWidget(head)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFrameShape(QFrame.NoFrame)
        self.log_box.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.log_box.setStyleSheet(
            f"QTextEdit{{font-size:12px;color:{C['text']};background:{C['card']};"
            f"border:none;padding:12px 14px 12px 20px;}}"
            "QScrollBar:vertical{background:transparent;width:8px;margin:6px 2px 6px 0;}"
            "QScrollBar::handle:vertical{background:#c9c9c4;min-height:28px;border-radius:4px;}"
            "QScrollBar::handle:vertical:hover{background:#a9a9a2;}"
            "QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}"
            "QScrollBar::add-page:vertical,QScrollBar::sub-page:vertical{background:transparent;}")
        self.log_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # keeps the log readable when the settings accordion is open: without
        # a floor the side column squashes it before the scroll area scrolls
        self.log_box.setMinimumHeight(140)
        v.addWidget(self.log_box, 1)
        self._log_lines = []
        return card

    def _btn(self, text, primary=False):
        b = QPushButton(text)
        b.setCursor(Qt.PointingHandCursor)
        bg = C["accent"] if primary else C["divider"]
        fg = "#fff" if primary else C["text"]
        b.setStyleSheet(
            f"QPushButton{{background:{bg};color:{fg};border:none;border-radius:8px;"
            f"padding:11px 12px;font-size:13px;font-weight:600;}}"
            f"QPushButton:disabled{{color:#aaa;background:{C['divider']};}}")
        return b

    def _set_tag(self, running, text=None):
        if text is None:
            text = "실행 중" if running else "정지됨"
        self.status_tag.setText(text)
        bg = C["accent"] if running else C["divider"]
        fg = "#fff" if running else C["neutral700"]
        self.status_tag.setStyleSheet(
            f"font-size:11px;font-weight:700;padding:4px 10px;border-radius:10px;"
            f"background:{bg};color:{fg};")

    # --- logging ------------------------------------------------------------
    def _log(self, text):
        self._log_lines.insert(0, f'<span style="color:{C["neutral700"]};'
                                  f'font-family:monospace">{now_str()}</span>&nbsp;&nbsp;{text}')
        self._log_lines = self._log_lines[:200]
        self.log_box.setHtml("<br>".join(self._log_lines))
        self.log_box.verticalScrollBar().setValue(0)  # newest first -> stay at top

    # --- button state machine ----------------------------------------------
    # phases: "off" (no teleop process) -> "starting" (spawned, waiting READY)
    #         -> "ready" (idle, can 시작) -> "running" (following) -> "paused"
    def _apply_button_state(self):
        p = self._phase
        # 시작 additionally requires a live headset link with the operator wearing
        # it and no latched fault -- the operator should never be able to start
        # following into a headset that is not there.
        blocked = self._xr_block_reason()
        phase_ok = p in ("ready", "paused")
        # Nothing is operable until the teleop process reports READY. The window
        # appears within a second of launch but the process behind it needs ~10s
        # (DDS, IK, cameras, the XR server), and every control in here is a
        # message to a process that is not listening yet: 시작 would be refused,
        # 비상 정지 would arm a shutdown deadline against a process still coming
        # up. Disabling them says "not yet" instead of failing per-button.
        # There is nothing to emergency-stop during this window either -- the
        # arms cannot move before alignment, which cannot happen before ready --
        # and closing the window still aborts a boot that wedges.
        booting = p in ("off", "starting")
        self.btn_start.setEnabled(phase_ok and blocked is None)
        self.btn_start.setToolTip(
            "텔레옵 준비 중…" if booting else (blocked if (phase_ok and blocked) else ""))
        self.btn_pause.setEnabled(p == "running")
        if hasattr(self, "btn_estop"):
            self.btn_estop.setEnabled(not booting)
            self.btn_estop.setToolTip("텔레옵 준비 중…" if booting else "")
        # settings are launch-time args -> body editable only before launch.
        # header stays clickable so the panel can still be expanded to view them.
        if hasattr(self, "settings_body"):
            editable = (p == "off")
            self.settings_body.setEnabled(editable)
            if not editable and not self.ed_camip.isReadOnly():
                self._toggle_camip_edit()  # collapse camera-ip edit on lock

    def _set_phase(self, phase):
        self._phase = phase
        self._apply_button_state()

    def _on_launch(self):
        if self.proc and self.proc.poll() is None:
            self._log("이미 실행 중")
            return
        self._cancel_respawn()
        # safety gates on the real robot, evaluated right before launch
        if self.args.domain == 0:
            if self.set_motion.value():
                # 전신(이동): requires walking mode (re-verify; may have dropped)
                ok, fid = self.mode_checker.fsm_id()
                if not (ok and fid == MotionModeChecker.FSM_WALK):
                    if not self._open_walk_dialog():
                        self._log("실행 취소 — 걷기 모드 미확인 (전신 제어)")
                        return
            else:
                # 상체(팔만): entering debug mode releases the legs -> fall warning
                if not self._open_debug_warn_dialog():
                    self._log("실행 취소 — 디버그 경고 미확인 (상체 제어)")
                    return
        cmd = self._build_teleop_cmd()
        self._log("텔레옵 프로세스 실행: " + " ".join(cmd))
        try:
            import subprocess
            env = os.environ.copy()
            env["COLUMNS"] = "200"          # widen rich/logging_mp console -> less soft-wrap
            env["PYTHONUNBUFFERED"] = "1"    # flush logs promptly to the pipe
            self.proc = subprocess.Popen(
                cmd, cwd=os.path.dirname(os.path.abspath(__file__)),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, env=env,
                start_new_session=True,      # own process group -> killpg on exit kills children
                preexec_fn=_die_with_parent)  # ...and no orphan if we are killed outright
        except Exception as e:
            self._log(f"실행 실패: {e}")
            self._schedule_respawn(rc="spawn-failed")
            return
        self._launch_time = time.monotonic()
        threading.Thread(target=self._pipe_proc_output, daemon=True).start()
        self._set_phase("starting")
        self._set_tag(False, "준비 중…")   # matches the disabled-buttons window
        self._proc_timer.start(1000)

    def _on_start(self):
        if self._phase not in ("ready", "paused"):
            return
        resuming = self._phase == "paused"
        ok, msg = self._send_cmd("CMD_START")
        self._log(f"[IPC] CMD_START -> {'ok' if ok else msg}")
        if not ok:
            return
        self._set_phase("running")
        self._set_tag(True)
        if not self._sec_timer.isActive():
            self._sec_timer.start(1000)
        self._log("텔레옵 재개됨" if resuming else "텔레옵 시작됨")

    def _on_pause(self):
        if self._phase != "running":
            return
        ok, msg = self._send_cmd("CMD_PAUSE")
        self._log(f"[IPC] CMD_PAUSE -> {'ok' if ok else msg}")
        if not ok:
            return
        self._set_phase("paused")
        self._sec_timer.stop()
        self._set_tag(False, "정지됨(홈 복귀)")
        self._log("텔레옵 정지 — 팔 홈 복귀 후 홀드")


    def _tick(self):
        self._elapsed += 1
        self.time_lbl.setText(f"{self._elapsed // 60:02d}:{self._elapsed % 60:02d}")

    def _on_heartbeat(self, hb):
        # XR telemetry first: it stays meaningful even in phases where the
        # phase machine below bails out early.
        xr = hb.get("XR")
        if isinstance(xr, dict):
            if xr.get("latched") and not self._xr.get("latched"):
                self._log(f"🛑 안전 정지 — {xr.get('reason', '')}")
            elif xr.get("state") == "hold" and self._xr.get("state") != "hold":
                self._log(f"⏸️ 홀드 — {xr.get('reason', '')}")
            self._xr = xr
            self._refresh_xr()
        self._align = hb.get("ALIGN")
        self._refresh_align(self._align)

        # teleop heartbeat is authoritative for readiness/following
        if self._phase == "off":
            return
        following = bool(hb.get("START"))
        ready = bool(hb.get("READY"))
        rec = bool(hb.get("RECORD_RUNNING"))
        if hb.get("STOP"):
            return  # let _poll_proc handle exit
        if self._phase == "starting" and ready and not following:
            # A respawn that gets this far is a fresh process whose arms are
            # already home (the exit path homed them) and which cannot follow
            # anything until the operator passes the alignment gate. That is the
            # re-arm ceremony, so the e-stop latch has done its job and clears
            # here -- [비상 정지] returns the system to 준비 완료 by itself.
            if self._estop_latched:
                self._estop_latched = False
                self._log("비상 정지 복구 — 팔 홈 복귀 완료, 텔레옵 재기동됨")
            self._set_phase("ready")
            self._set_tag(False, "준비 완료")
            self._log("텔레옵 준비 완료 — [시작] 가능")
        # keep status tag in sync with actual following state
        if self._phase == "running":
            self._set_tag(True, "기록 중" if rec else "실행 중")

    # --- teleop subprocess ---------------------------------------------------
    def _build_teleop_cmd(self):
        a = self.args
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "teleop_hand_and_arm.py")
        # live settings-panel values (fall back to CLI args)
        input_mode = self.set_inputmode.value()
        motion = self.set_motion.value()
        net = self.cmb_net.currentData()
        img_ip = self.ed_camip.text().strip() or a.img_server_ip
        cmd = [_teleop_python(), script, "--ipc",
               "--xr-source", "xrlink",
               # Device telemetry is recorded on every launch. The app-side
               # defects so far were all found after the session ended, by
               # which point nothing had been captured; a session that is not
               # recorded is a session that has to be run again.
               "--xr-log",
               "--input-mode", input_mode,
               "--arm", a.arm,
               "--img-server-ip", img_ip]
        if motion:
            cmd.append("--motion")
        if a.domain == 1:
            cmd.append("--sim")
        if a.ee:
            cmd += ["--ee", a.ee]
        if net:
            cmd += ["--network-interface", net]
        if a.teleop_extra:
            cmd += a.teleop_extra.split()
        return cmd

    def _pipe_proc_output(self):
        try:
            for line in self.proc.stdout:
                line = line.rstrip()
                if line:
                    self._log_signal.emit(line)
        except Exception:
            pass

    def _poll_proc(self):
        if self.proc is None:
            return
        rc = self.proc.poll()
        if rc is not None:
            self._proc_timer.stop()
            self.proc = None
            self._stop_deadline = None
            self._kill_deadline = None
            self._set_phase("off")
            self._sec_timer.stop()
            self._elapsed = 0
            self.time_lbl.setText("00:00")
            self._set_tag(False)
            self._log(f"텔레옵 프로세스 종료 (rc={rc})")
            self._schedule_respawn(rc)
            return
        # still alive past the stop deadline -> SIGTERM the group, and arm a second,
        # shorter deadline to SIGKILL it if that doesn't work either. SIGTERM alone
        # used to be a one-shot: a subprocess stuck in an uninterruptible or slow
        # blocking call (an SDK RPC to a robot that already vanished, say) could
        # ignore it and just sit there with no further escalation, leaving [종료]
        # looking permanently ignored.
        if self._stop_deadline and time.time() > self._stop_deadline:
            self._stop_deadline = None
            self._log("종료 지연 — 프로세스 그룹 강제 종료 (SIGTERM)")
            self._kill_proc_group(signal.SIGTERM)
            self._kill_deadline = time.time() + 5.0
            return
        if self._kill_deadline and time.time() > self._kill_deadline:
            self._kill_deadline = None
            self._log("SIGTERM 무반응 — 강제 종료 (SIGKILL)")
            self._kill_proc_group(signal.SIGKILL)

    # How long [종료]/[비상 정지] may take before the dashboard stops waiting and
    # starts signalling. Both now end with a slow homing move.
    STOP_GRACE = 20.0

    # --- respawn supervision -------------------------------------------------
    RESPAWN_MIN = 2.0     # normal gap: let the old process group's ports free up
    RESPAWN_MAX = 15.0    # ceiling once the child is clearly failing to come up
    RESPAWN_YOUNG = 20.0  # a child that dies sooner than this never got running

    def _schedule_respawn(self, rc):
        """Bring the teleop process back so the headset link does not stay down.

        The XrLink server the headset talks to is hosted by the teleop process.
        Whether it exited because the operator pressed 종료, because 비상 정지
        homed the arms and quit, or because it crashed, leaving it dead leaves
        the device disconnected -- and reconnecting means putting the headset
        back on and re-pairing. So the dashboard owns keeping it alive.

        Backoff exists for the crash case only: a child that cannot start at all
        (missing deps, port already bound) would otherwise respawn in a tight
        loop and bury the log. A child that ran a real session resets it.
        """
        if not self._supervise:
            return
        ran_for = time.monotonic() - self._launch_time
        if ran_for >= self.RESPAWN_YOUNG:
            self._respawn_backoff = self.RESPAWN_MIN
        else:
            # died young -> it never became usable; back off before trying again
            self._respawn_backoff = min(self.RESPAWN_MAX,
                                        max(self.RESPAWN_MIN, self._respawn_backoff * 2))
            self._log(f"기동 직후 종료 (rc={rc}, {ran_for:.1f}s) — "
                      f"{self._respawn_backoff:.0f}초 후 재시도")
        delay = self._respawn_backoff or self.RESPAWN_MIN
        if ran_for >= self.RESPAWN_YOUNG:
            self._log(f"XR 링크 유지를 위해 {delay:.0f}초 후 텔레옵 재기동")
        self._respawn_timer = QTimer(self)
        self._respawn_timer.setSingleShot(True)
        self._respawn_timer.timeout.connect(self._respawn_now)
        self._respawn_timer.start(int(delay * 1000))

    def _respawn_now(self):
        self._respawn_timer = None
        if not self._supervise or (self.proc and self.proc.poll() is None):
            return
        self._on_launch()

    # --- signalled shutdown --------------------------------------------------
    def _install_signal_handlers(self):
        def handler(signum, _frame):
            self._term_requested = True
        for sig in (signal.SIGTERM, signal.SIGINT, getattr(signal, "SIGHUP", None)):
            if sig is None:
                continue
            try:
                signal.signal(sig, handler)
            except (ValueError, OSError):
                pass

    def _check_term(self):
        if not self._term_requested:
            return
        self._term_requested = False
        self._log("종료 신호 수신 — 텔레옵 정리 후 종료")
        self.close()                       # runs closeEvent: stops supervising,
        QApplication.instance().quit()     # stops the child, then tears down IPC

    def _cancel_respawn(self):
        if self._respawn_timer is not None:
            self._respawn_timer.stop()
            self._respawn_timer = None

    def _kill_proc_group(self, sig=signal.SIGTERM):
        """Signal the whole teleop process group (parent + multiprocessing children)."""
        if not self.proc:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    # --- misc ---------------------------------------------------------------
    def _resolve_model(self):
        if self.args.model and os.path.exists(self.args.model):
            return self.args.model
        if os.path.exists(DEFAULT_MODEL):
            return DEFAULT_MODEL
        return os.path.abspath(FALLBACK_MODEL)

    def closeEvent(self, e):
        # First, before anything can exit: stop respawning. The teleop process is
        # kept alive only for as long as this window is open, and _poll_proc must
        # not resurrect it while we are shutting it down.
        self._supervise = False
        self._cancel_respawn()
        try:
            self._proc_timer.stop()
        except Exception:
            pass
        try:
            self.mj.stop()
            self.cam.stop()
        except Exception:
            pass
        # shut down teleop process group if we launched it. The wait is
        # STOP_GRACE, not the 5s it used to be: the exit path now walks the arms
        # home at SAFE_ARM_VELOCITY before it tears anything down, and the old
        # timeout expired mid-move -- so closing the window SIGTERMed the child
        # partway through the very homing move that exists to avoid that.
        if self.proc and self.proc.poll() is None:
            try:
                self._send_cmd("CMD_STOP", require_online=False)
                self.proc.wait(timeout=self.STOP_GRACE)
            except Exception:
                # graceful exit failed -> SIGTERM the group, then SIGKILL
                try:
                    self._kill_proc_group(signal.SIGTERM)
                    self.proc.wait(timeout=5)
                except Exception:
                    self._kill_proc_group(signal.SIGKILL)
                    try:
                        self.proc.wait(timeout=3)
                    except Exception:
                        pass
        # Stop polling before tearing the client down, so a timer callback
        # cannot land on a half-closed socket.
        for timer in ("_hb_timer", "_ui_timer", "_sec_timer", "_proc_timer"):
            try:
                getattr(self, timer).stop()
            except Exception:
                pass
        if self.ipc is not None:
            try:
                self.ipc.stop()
            except Exception:
                pass
            self.ipc = None
        super().closeEvent(e)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--domain", type=int, default=1,
                   help="DDS domain id. teleop --sim uses 1, real robot uses 0.")
    p.add_argument("--net", type=str, default=None, help="network interface (e.g. eth0)")
    p.add_argument("--model", type=str, default=None, help="MuJoCo XML path override")
    p.add_argument("--img-server-ip", type=str, default="192.168.123.164")
    p.add_argument("--camera", action="store_true", default=True,
                   help="enable head-camera PiP via ImageClient (default on)")
    p.add_argument("--no-camera", dest="camera", action="store_false",
                   help="disable the head-camera PiP relay")
    # teleop subprocess launch parameters (used by the 실행 button)
    p.add_argument("--input-mode", type=str, default="controller",
                   choices=["hand", "controller"])
    p.add_argument("--arm", type=str, default="G1_29",
                   choices=["G1_29", "G1_23", "H1_2", "H1", "R1"])
    p.add_argument("--ee", type=str, default=None,
                   choices=["dex1", "dex3", "inspire_ftp", "inspire_dfx", "brainco"])
    p.add_argument("--motion", action="store_true", default=False,
                   help="default control range: 전신(이동). off => 상체(팔만, safe default)")
    p.add_argument("--no-motion", dest="motion", action="store_false")
    p.add_argument("--teleop-extra", type=str, default=None,
                   help="extra args appended to the teleop command")
    args = p.parse_args()

    app = QApplication(sys.argv)
    app.setFont(QFont("Sans Serif", 10))
    win = Dashboard(args)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
