import json
import os
import zmq
import time
import threading
import logging_mp
logger_mp = logging_mp.get_logger(__name__, level=logging_mp.INFO)

"""
# Client → Server (Request)
    {
        "reqid": unique id,
        "cmd": one of IPC_Server.cmd_map
    }

  Supported commands:
    CMD_START          start / resume following
    CMD_PAUSE          pause following (arms go home, then hold), resumable
    CMD_STOP           exit the teleop program
    CMD_RECORD_TOGGLE  start / stop recording (toggle)
    CMD_ESTOP          emergency stop: latch following off immediately
    CMD_ACK_FAULT      acknowledge a latched safety fault

# Server → Client (Reply)
1) if ok
    {
        "repid": same as reqid,
        "status": "ok",
        "msg": "ok"
    }
2) if error
    {
        "repid": same as reqid | 0 | 1,   # 0: bad/absent reqid, 1: internal error
        "status": "error",
        "msg": "reqid not provided"
             | "cmd not provided"
             | "cmd not supported: {cmd}"
             | "malformed request: {detail}"
             | "internal error msg"
    }

# Heartbeat (PUB)
    {
        "START": True | False,          # whether robot follows vr
        "STOP" : True | False,          # whether exit program
        "READY": True | False,          # whether ready to start
        "RECORD_RUNNING": True | False, # whether is recording
        "XR": {...} | None,             # SafetyFSM.snapshot(), see teleop/safety
    }
"""

DEFAULT_DATA_ADDR = "ipc://@xr_teleoperate_data.ipc"
DEFAULT_HB_ADDR = "ipc://@xr_teleoperate_hb.ipc"


class IPC_Server:
    """
    Inter - Process Communication Server:
    - Handle data via REP
    - Publish heartbeat via PUB, Heartbeat state is provided by external callback get_state()
    """
    # Mapping table for on_press keys
    cmd_map = {
        "CMD_START": "r",          # start / resume following
        "CMD_PAUSE": "p",          # pause following (go home + hold), resumable
        "CMD_STOP": "q",           # exit
        "CMD_RECORD_TOGGLE": "s",  # start & stop (toggle record)
        "CMD_ESTOP": "e",          # emergency stop: latch following off now
        "CMD_ACK_FAULT": "a",      # acknowledge a latched safety fault
        "CMD_CANCEL_ALIGN": "c",   # abandon an in-progress start alignment
    }

    def __init__(self, on_press=None, get_state=None, hb_fps=10.0,
                 data_addr=DEFAULT_DATA_ADDR, hb_addr=DEFAULT_HB_ADDR):
        """
        Args:
            on_press  : callback(cmd:str), called for every command
            get_state : callback() -> dict, provides current heartbeat state
            hb_fps    : heartbeat publish frequency
            data_addr : REP bind address
            hb_addr   : PUB bind address
        """
        if callable(on_press):
            self.on_press = on_press
        else:
            raise ValueError("[IPC_Server] on_press callback function must be provided")

        if callable(get_state):
            self.get_state = get_state
        else:
            raise ValueError("[IPC_Server] get_state callback function must be provided")
        self._hb_interval = 1.0 / float(hb_fps)
        self._running = True
        self._data_loop_thread = None
        self._hb_loop_thread = None
        self._data_addr = data_addr
        self._hb_addr = hb_addr

        # A private context, not zmq.Context.instance(). term() on the shared
        # singleton would tear down every other ZMQ user in this process (the
        # image client among them) and leave it unable to open new sockets.
        self.ctx = zmq.Context()

        self.rep_socket = self._bind_rep()
        logger_mp.info(f"[IPC_Server] Listening to Data at {self._data_addr}")

        # heartbeat IPC (PUB/SUB)
        self.pub_socket = self.ctx.socket(zmq.PUB)
        self.pub_socket.setsockopt(zmq.LINGER, 0)
        self.pub_socket.bind(self._hb_addr)
        logger_mp.info(f"[IPC_Server] Publishing HeartBeat at {self._hb_addr}")

    def _bind_rep(self):
        sock = self.ctx.socket(zmq.REP)
        sock.setsockopt(zmq.LINGER, 0)
        sock.bind(self._data_addr)
        return sock

    def _reset_rep(self, poller):
        """Rebuild the REP socket after its request/reply alternation is broken."""
        logger_mp.warning("[IPC_Server] resetting REP socket")
        try:
            poller.unregister(self.rep_socket)
        except Exception:
            pass
        try:
            self.rep_socket.close(0)
        except Exception:
            pass
        self.rep_socket = self._bind_rep()
        poller.register(self.rep_socket, zmq.POLLIN)

    def _data_loop(self):
        """
        Listen for REQ/REP commands and optional info.

        A REP socket is a strict recv->send alternation: once a request has been
        received it will not deliver another until a reply is sent. Every path
        below therefore ends in exactly one send. Receiving raw bytes rather than
        recv_json() is what makes that possible -- recv_json() consumes the
        message and *then* raises on bad JSON, leaving the socket owing a reply
        it never sends, which deafens the command channel permanently (including
        to CMD_STOP).
        """
        poller = zmq.Poller()
        poller.register(self.rep_socket, zmq.POLLIN)
        while self._running:
            try:
                socks = dict(poller.poll(20))
            except zmq.error.ContextTerminated:
                break
            except zmq.ZMQError as e:
                if not self._running:
                    break
                logger_mp.error(f"[IPC_Server] Data poll error: {e}")
                time.sleep(0.05)
                continue

            if self.rep_socket not in socks:
                continue

            try:
                raw = self.rep_socket.recv()
            except zmq.error.ContextTerminated:
                break
            except Exception as e:
                logger_mp.error(f"[IPC_Server] Failed to receive request: {e}")
                continue

            reply = self._build_reply(raw)
            try:
                self.rep_socket.send_json(reply)
                logger_mp.debug(f"[IPC_Server] DATA recv: {raw!r} -> rep: {reply}")
            except Exception as e:
                logger_mp.error(f"[IPC_Server] Failed to send reply: {e}")
                self._reset_rep(poller)

    def _build_reply(self, raw):
        """Turn a raw request frame into a reply. Never raises."""
        try:
            msg = json.loads(raw.decode("utf-8"))
        except Exception as e:
            return {"repid": 0, "status": "error", "msg": f"malformed request: {e}"}
        if not isinstance(msg, dict):
            return {"repid": 0, "status": "error",
                    "msg": "malformed request: expected a JSON object"}
        return self._handle_message(msg)

    def _hb_loop(self):
        """Publish heartbeat periodically"""
        while self._running:
            start_time = time.monotonic()
            try:
                state = dict(self.get_state() or {})
                self.pub_socket.send_json(state)
                logger_mp.debug(f"[IPC_Server] HB pub: {state}")
            except zmq.error.ContextTerminated:
                break
            except Exception as e:
                logger_mp.error(f"[IPC_Server] HeartBeat loop exception: {e}")
            elapsed = time.monotonic() - start_time
            if elapsed < self._hb_interval:
                time.sleep(self._hb_interval - elapsed)

    def _handle_message(self, msg: dict) -> dict:
        """Process message and return reply"""
        try:
            # validate reqid
            reqid = msg.get("reqid", None)
            if not reqid:
                return {"repid": 0, "status": "error", "msg": "reqid not provided"}

            # validate cmd
            cmd = msg.get("cmd", None)
            if not cmd:
                return {"repid": reqid, "status": "error", "msg": "cmd not provided"}

            # unsupported cmd
            if cmd not in self.cmd_map:
                return {"repid": reqid, "status": "error", "msg": f"cmd not supported: {cmd}"}

            # supported cmd path
            self.on_press(self.cmd_map[cmd])
            return {"repid": reqid, "status": "ok", "msg": "ok"}

        except Exception as e:
            return {"repid": 1, "status": "error", "msg": str(e)}

    # ---------------------------
    # Public API
    # ---------------------------
    def start(self):
        """Start both data loop and heartbeat loop"""
        self._data_loop_thread = threading.Thread(target=self._data_loop, daemon=True)
        self._data_loop_thread.start()
        self._hb_loop_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_loop_thread.start()

    def stop(self):
        """Stop server"""
        self._running = False
        if self._data_loop_thread:
            self._data_loop_thread.join(timeout=1.0)
        if self._hb_loop_thread:
            self._hb_loop_thread.join(timeout=1.0)
        for sock in (self.rep_socket, self.pub_socket):
            try:
                sock.setsockopt(zmq.LINGER, 0)
                sock.close()
            except Exception:
                pass
        try:
            self.ctx.term()   # safe: this context is ours alone
        except Exception:
            pass


class IPC_Client:
    """
    Inter - Process Communication Client:
    - Send command via REQ
    - Subscribe heartbeat via SUB
    """
    def __init__(self, hb_fps=10.0, data_addr=DEFAULT_DATA_ADDR,
                 hb_addr=DEFAULT_HB_ADDR, req_timeout_ms=1000):
        """hb_fps: heartbeat subscribe frequency, should match server side."""
        self.ctx = zmq.Context()          # private; see IPC_Server.__init__
        self._data_addr = data_addr
        self._hb_addr = hb_addr
        self._req_timeout_ms = int(req_timeout_ms)

        # heartbeat IPC (PUB/SUB)
        self._hb_running = True
        self._hb_last_time = 0           # timestamp of last heartbeat received
        self._hb_latest_state = {}       # latest heartbeat state
        self._hb_online = False          # whether heartbeat is online
        self._hb_interval = 1.0 / float(hb_fps)     # expected heartbeat interval
        self._hb_lock = threading.Lock()            # lock for heartbeat state
        self._hb_timeout = 5.0 * self._hb_interval  # timeout to consider offline

        self.sub_socket = self.ctx.socket(zmq.SUB)
        self.sub_socket.setsockopt(zmq.RCVHWM, 1)
        self.sub_socket.setsockopt(zmq.LINGER, 0)
        self.sub_socket.connect(self._hb_addr)
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        logger_mp.info(f"[IPC_Client] Subscribed to HeartBeat at {self._hb_addr}")

        self._hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_thread.start()

        # data IPC (REQ/REP)
        self._req_lock = threading.Lock()
        self.req_socket = self._make_req()
        logger_mp.info(f"[IPC_Client] Connected to Data at {self._data_addr}")

    def _make_req(self):
        sock = self.ctx.socket(zmq.REQ)
        # Without these a single timed-out reply leaves the REQ socket stuck in
        # "must recv" and every later send raises EFSM -- one slow reply would
        # silently disable the command channel for the rest of the session.
        # RELAXED permits the next send regardless; CORRELATE makes libzmq drop
        # the stale reply when it eventually arrives.
        sock.setsockopt(zmq.REQ_RELAXED, 1)
        sock.setsockopt(zmq.REQ_CORRELATE, 1)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self._data_addr)
        return sock

    def _reset_req(self):
        try:
            self.req_socket.close(0)
        except Exception:
            pass
        self.req_socket = self._make_req()

    def _make_reqid(self) -> str:
        import uuid
        return str(uuid.uuid4())

    # ---------------------------
    # Heartbeat handling
    # ---------------------------
    def _hb_loop(self):
        poller = zmq.Poller()
        poller.register(self.sub_socket, zmq.POLLIN)
        consecutive = 0
        while self._hb_running:
            try:
                socks = dict(poller.poll(int(self._hb_interval * 1000)))
            except zmq.error.ContextTerminated:
                break
            except zmq.ZMQError:
                if not self._hb_running:
                    break
                continue

            now = time.monotonic()
            if self.sub_socket in socks:
                try:
                    msg = self.sub_socket.recv_json()
                except Exception as e:
                    logger_mp.error(f"[IPC_Client] HB decode failed: {e}")
                    continue
                with self._hb_lock:
                    self._hb_latest_state = msg
                    self._hb_last_time = now
                    consecutive += 1
                    # require 3 consecutive heartbeats before trusting the link
                    if consecutive >= 3 and not self._hb_online:
                        self._hb_online = True
                        logger_mp.info("[IPC_Client] HeartBeat -> ONLINE")
            else:
                with self._hb_lock:
                    if self._hb_last_time > 0 and (now - self._hb_last_time) > self._hb_timeout:
                        was_online = self._hb_online
                        self._hb_latest_state = {}
                        self._hb_last_time = 0
                        self._hb_online = False
                        consecutive = 0
                        if was_online:
                            logger_mp.warning("[IPC_Client] HeartBeat timeout -> OFFLINE")

    # ---------------------------
    # Public API
    # ---------------------------
    def send_data(self, cmd: str, require_online: bool = True) -> dict:
        """Send command to server and wait reply.

        require_online=False lets a shutdown command through even when the
        heartbeat has already stopped -- refusing to try would be worse than a
        wasted request.
        """
        reqid = self._make_reqid()
        if require_online and not self.is_online():
            logger_mp.warning(f"[IPC_Client] Cannot send {cmd}, server offline (no heartbeat)")
            return {"repid": reqid, "status": "error", "msg": "server offline (no heartbeat)"}

        msg = {"reqid": reqid, "cmd": cmd}
        with self._req_lock:
            try:
                self.req_socket.send_json(msg)
                if self.req_socket.poll(self._req_timeout_ms):
                    reply = self.req_socket.recv_json()
                else:
                    return {"repid": reqid, "status": "error",
                            "msg": "timeout waiting for server reply"}
            except Exception as e:
                logger_mp.error(f"[IPC_Client] send_data failed: {e}")
                self._reset_req()
                return {"repid": reqid, "status": "error", "msg": str(e)}

        if not isinstance(reply, dict):
            return {"repid": reqid, "status": "error", "msg": "malformed reply"}
        if reply.get("status") != "ok":
            return reply
        if reply.get("repid") != reqid:
            return {"repid": reqid, "status": "error",
                    "msg": f"reply id mismatch: expected {reqid}, got {reply.get('repid')}"}
        return reply

    def is_online(self) -> bool:
        with self._hb_lock:
            return self._hb_online

    def heartbeat_age(self) -> float:
        """Seconds since the last heartbeat, or inf if none has arrived."""
        with self._hb_lock:
            if self._hb_last_time <= 0:
                return float("inf")
            return time.monotonic() - self._hb_last_time

    def latest_state(self) -> dict:
        with self._hb_lock:
            return dict(self._hb_latest_state)

    def stop(self):
        self._hb_running = False
        if self._hb_thread:
            self._hb_thread.join(timeout=1.0)
        for sock in (self.req_socket, self.sub_socket):
            try:
                sock.setsockopt(zmq.LINGER, 0)
                sock.close()
            except Exception:
                pass
        try:
            self.ctx.term()   # safe: this context is ours alone
        except Exception:
            pass


# ---------------------------
# Client Example usage
# ---------------------------
if __name__ == "__main__":
    from sshkeyboard import listen_keyboard, stop_listening
    client = None

    def on_press(key: str):
        global client
        if client is None:
            logger_mp.warning("⚠️ Client not initialized, ignoring key press")
            return

        if key == "r":
            logger_mp.info("▶️ Sending launch command...")
            rep = client.send_data("CMD_START")
            logger_mp.info("Reply: %s", rep)

        elif key == "s":
            logger_mp.info("⏺️ Sending record toggle command...")
            rep = client.send_data("CMD_RECORD_TOGGLE")
            logger_mp.info("Reply: %s", rep)

        elif key == "e":
            logger_mp.info("🛑 Sending emergency stop...")
            rep = client.send_data("CMD_ESTOP", require_online=False)
            logger_mp.info("Reply: %s", rep)

        elif key == "a":
            logger_mp.info("✅ Acknowledging safety fault...")
            rep = client.send_data("CMD_ACK_FAULT")
            logger_mp.info("Reply: %s", rep)

        elif key == "q":
            logger_mp.info("⏹️ Sending exit command...")
            rep = client.send_data("CMD_STOP", require_online=False)
            logger_mp.info("Reply: %s", rep)

        elif key == "b":
            if client.is_online():
                state = client.latest_state()
                logger_mp.info(f"[HEARTBEAT] Current heartbeat: {state}")
            else:
                logger_mp.warning("[HEARTBEAT] No heartbeat received (OFFLINE)")

        else:
            logger_mp.warning(f"⚠️ Undefined key: {key}")

    # Initialize client
    client = IPC_Client(hb_fps=10.0)

    # Start keyboard listening thread
    listen_keyboard_thread = threading.Thread(target=listen_keyboard, kwargs={"on_press": on_press, "until": None, "sequential": False}, daemon=True)
    listen_keyboard_thread.start()

    logger_mp.info("✅ Client started, waiting for keyboard input:\n [r] launch, [s] record, [e] estop, [a] ack, [b] heartbeat, [q] exit")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger_mp.info("⏹️ User interrupt, preparing to exit...")
    finally:
        stop_listening()
        client.stop()
        logger_mp.info("✅ Client exited")
