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
os.environ.setdefault("MUJOCO_GL", "egl")  # headless offscreen GL

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
    QFrame, QSizePolicy, QTextEdit,
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

IPC_DATA_ADDR = "ipc://@xr_teleoperate_data.ipc"
IPC_HB_ADDR = "ipc://@xr_teleoperate_hb.ipc"

# design tokens (from static/dashboard.html, light theme)
C = {
    "bg": "#f4f4f2", "text": "#1a1a1a", "divider": "#e2e2dd",
    "neutral700": "#6b6b66", "neutral900": "#1a1a1a", "accent": "#2f6df6",
    "card": "#ffffff",
}


def now_str():
    return datetime.now().strftime("%H:%M:%S")


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
            with self._lock:
                self._q = q
        except Exception:
            pass

    def get_motor_q(self):
        with self._lock:
            return None if self._q is None else self._q.copy()


# ----------------------------------------------------------------------------
# IPC bridge to the teleop process (commands + heartbeat)
# ----------------------------------------------------------------------------
class IPCBridge(QObject):
    heartbeat = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._ctx = None
        self._running = False
        self.available = False

    def start(self):
        try:
            import zmq
            self._zmq = zmq
            self._ctx = zmq.Context.instance()
            self._req = self._ctx.socket(zmq.REQ)
            self._req.setsockopt(zmq.RCVTIMEO, 500)
            self._req.setsockopt(zmq.SNDTIMEO, 500)
            self._req.setsockopt(zmq.LINGER, 0)
            self._req.connect(IPC_DATA_ADDR)
            self.available = True
            self._running = True
            threading.Thread(target=self._hb_loop, daemon=True).start()
        except Exception as e:
            print(f"[IPCBridge] disabled: {e}", file=sys.stderr)
            self.available = False

    def send_cmd(self, cmd):
        """Returns (ok, msg). Best-effort; REQ/REP is recreated on timeout."""
        if not self.available:
            return False, "IPC 미연결"
        try:
            self._req.send_json({"reqid": int(time.time() * 1000) & 0x7fffffff, "cmd": cmd})
            rep = self._req.recv_json()
            return rep.get("status") == "ok", rep.get("msg", "")
        except Exception as e:
            # REQ socket is now in a bad state after a timeout — rebuild it
            try:
                self._req.close(0)
                self._req = self._ctx.socket(self._zmq.REQ)
                self._req.setsockopt(self._zmq.RCVTIMEO, 500)
                self._req.setsockopt(self._zmq.SNDTIMEO, 500)
                self._req.setsockopt(self._zmq.LINGER, 0)
                self._req.connect(IPC_DATA_ADDR)
            except Exception:
                pass
            return False, f"응답 없음 ({e})"

    def _hb_loop(self):
        try:
            sub = self._ctx.socket(self._zmq.SUB)
            sub.setsockopt(self._zmq.RCVTIMEO, 500)
            sub.setsockopt_string(self._zmq.SUBSCRIBE, "")
            sub.connect(IPC_HB_ADDR)
        except Exception:
            return
        while self._running:
            try:
                msg = sub.recv_json()
                self.heartbeat.emit(msg)
            except Exception:
                continue


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

    def _loop(self):
        try:
            from teleimager.image_client import ImageClient
            client = ImageClient(host=self._host)
        except Exception as e:
            self.status.emit(f"카메라 미연결: {e}")
            return
        self.status.emit("카메라 연결됨")
        while self._running:
            try:
                img, _ = client.get_head_frame()
                if img is not None:
                    if img.ndim == 2:
                        img = np.stack([img] * 3, axis=-1)
                    img = np.ascontiguousarray(img[:, :, ::-1])  # BGR -> RGB
                    h, w, _ = img.shape
                    qimg = QImage(img.data, w, h, 3 * w, QImage.Format_RGB888).copy()
                    self.frame_ready.emit(qimg)
            except Exception:
                pass
            time.sleep(1.0 / FPS)
        try:
            client.close()
        except Exception:
            pass


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
        self.sim = VideoLabel("무조코 시뮬레이션 화면", self)
        self.cam = VideoLabel("로봇 카메라 시점", self)
        self.sim_label = self._badge("무조코 시뮬레이션")
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
        self.proc = None
        self._log_signal.connect(self._log)

        self._build_ui()

        # timers
        self._sec_timer = QTimer(self)
        self._sec_timer.timeout.connect(self._tick)
        self._proc_timer = QTimer(self)
        self._proc_timer.timeout.connect(self._poll_proc)

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

        self.ipc = IPCBridge()
        self.ipc.heartbeat.connect(self._on_heartbeat)
        self.ipc.start()

        self._log("대시보드 준비 완료" + ("" if self.ipc.available else " (IPC 미연결)"))

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

        side = QWidget()
        side.setFixedWidth(340)
        side.setStyleSheet(f"background:{C['bg']};")
        sl = QVBoxLayout(side)
        sl.setContentsMargins(24, 24, 24, 24)
        sl.setSpacing(20)

        sl.addWidget(self._status_card())
        sl.addWidget(self._log_card(), 1)

        body.addWidget(side)
        root.addLayout(body, 1)

    def _card(self):
        f = QFrame()
        f.setStyleSheet(
            f"QFrame{{background:{C['card']};border-radius:10px;}}")
        return f

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

        self.btn_launch = self._btn("실행", primary=True)
        self.btn_start = self._btn("시작", primary=True)
        self.btn_pause = self._btn("정지")
        self.btn_stop = self._btn("종료")
        self.btn_launch.clicked.connect(self._on_launch)
        self.btn_start.clicked.connect(self._on_start)
        self.btn_pause.clicked.connect(self._on_pause)
        self.btn_stop.clicked.connect(self._on_stop)

        row1 = QHBoxLayout(); row1.setSpacing(10)
        row1.addWidget(self.btn_launch); row1.addWidget(self.btn_stop)
        row2 = QHBoxLayout(); row2.setSpacing(10)
        row2.addWidget(self.btn_start); row2.addWidget(self.btn_pause)
        v.addLayout(row1)
        v.addLayout(row2)
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
        self.btn_launch.setEnabled(p == "off")
        self.btn_start.setEnabled(p in ("ready", "paused"))
        self.btn_pause.setEnabled(p == "running")
        self.btn_stop.setEnabled(p != "off")

    def _set_phase(self, phase):
        self._phase = phase
        self._apply_button_state()

    def _on_launch(self):
        if self.proc and self.proc.poll() is None:
            self._log("이미 실행 중")
            return
        cmd = self._build_teleop_cmd()
        self._log("텔레옵 프로세스 실행: " + " ".join(cmd))
        try:
            import subprocess
            self.proc = subprocess.Popen(
                cmd, cwd=os.path.dirname(os.path.abspath(__file__)),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, env=os.environ.copy())
        except Exception as e:
            self._log(f"실행 실패: {e}")
            return
        threading.Thread(target=self._pipe_proc_output, daemon=True).start()
        self._set_phase("starting")
        self._set_tag(False, "실행 중…")
        self._proc_timer.start(1000)

    def _on_start(self):
        if self._phase not in ("ready", "paused"):
            return
        resuming = self._phase == "paused"
        ok, msg = self.ipc.send_cmd("CMD_START")
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
        ok, msg = self.ipc.send_cmd("CMD_PAUSE")
        self._log(f"[IPC] CMD_PAUSE -> {'ok' if ok else msg}")
        if not ok:
            return
        self._set_phase("paused")
        self._sec_timer.stop()
        self._set_tag(False, "정지됨(홈 복귀)")
        self._log("텔레옵 정지 — 팔 홈 복귀 후 홀드")

    def _on_stop(self):
        if self._phase == "off":
            return
        ok, msg = self.ipc.send_cmd("CMD_STOP")
        self._log(f"[IPC] CMD_STOP -> {'ok' if ok else msg}")
        self._sec_timer.stop()
        self._elapsed = 0
        self.time_lbl.setText("00:00")
        self._set_tag(False, "종료됨")
        self._log("텔레옵 종료 요청")
        # teleop exits on its own; _poll_proc will flip phase to "off"

    def _tick(self):
        self._elapsed += 1
        self.time_lbl.setText(f"{self._elapsed // 60:02d}:{self._elapsed % 60:02d}")

    def _on_heartbeat(self, hb):
        # teleop heartbeat is authoritative for readiness/following
        if self._phase == "off":
            return
        following = bool(hb.get("START"))
        ready = bool(hb.get("READY"))
        rec = bool(hb.get("RECORD_RUNNING"))
        if hb.get("STOP"):
            return  # let _poll_proc handle exit
        if self._phase == "starting" and ready and not following:
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
        cmd = [sys.executable, script, "--ipc",
               "--input-mode", a.input_mode,
               "--arm", a.arm,
               "--img-server-ip", a.img_server_ip]
        if a.motion:
            cmd.append("--motion")
        if a.domain == 1:
            cmd.append("--sim")
        if a.ee:
            cmd += ["--ee", a.ee]
        if a.net:
            cmd += ["--network-interface", a.net]
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
            self._set_phase("off")
            self._sec_timer.stop()
            self._elapsed = 0
            self.time_lbl.setText("00:00")
            self._set_tag(False)
            self._log(f"텔레옵 프로세스 종료 (rc={rc})")

    # --- misc ---------------------------------------------------------------
    def _resolve_model(self):
        if self.args.model and os.path.exists(self.args.model):
            return self.args.model
        if os.path.exists(DEFAULT_MODEL):
            return DEFAULT_MODEL
        return os.path.abspath(FALLBACK_MODEL)

    def closeEvent(self, e):
        try:
            self.mj.stop()
            self.cam.stop()
        except Exception:
            pass
        # shut down teleop process if we launched it
        if self.proc and self.proc.poll() is None:
            try:
                self.ipc.send_cmd("CMD_STOP")
                self.proc.wait(timeout=5)
            except Exception:
                try:
                    self.proc.terminate()
                    self.proc.wait(timeout=3)
                except Exception:
                    try:
                        self.proc.kill()
                    except Exception:
                        pass
        super().closeEvent(e)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--domain", type=int, default=1,
                   help="DDS domain id. teleop --sim uses 1, real robot uses 0.")
    p.add_argument("--net", type=str, default=None, help="network interface (e.g. eth0)")
    p.add_argument("--model", type=str, default=None, help="MuJoCo XML path override")
    p.add_argument("--img-server-ip", type=str, default="192.168.123.164")
    p.add_argument("--camera", action="store_true",
                   help="enable head-camera PiP via ImageClient")
    # teleop subprocess launch parameters (used by the 실행 button)
    p.add_argument("--input-mode", type=str, default="controller",
                   choices=["hand", "controller"])
    p.add_argument("--arm", type=str, default="G1_29",
                   choices=["G1_29", "G1_23", "H1_2", "H1", "R1"])
    p.add_argument("--ee", type=str, default=None,
                   choices=["dex1", "dex3", "inspire_ftp", "inspire_dfx", "brainco"])
    p.add_argument("--motion", action="store_true", default=True,
                   help="pass --motion to teleop (default on)")
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
