import time
import argparse
from multiprocessing import Value, Array, Lock
import threading
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds 
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController, R1_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, R1_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper
from teleop.safety import (Action, SafetyConfig, SafetyFSM, XRLiveness)
from teleop.safety.align import AlignConfig, AlignGate
from teleop.xr import XRFrame
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
PAUSE_REQUEST  = False  # One-shot: on pause, return arms to home then hold (following stays off)
SAFETY         = None   # SafetyFSM, built during startup; gates every arm command
ALIGN_STATE    = None   # latest AlignReport.as_dict(), or None when not aligning
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.

def on_press(key):
    global STOP, START, RECORD_TOGGLE, PAUSE_REQUEST, SAFETY
    if key == 'r':
        START = True
    elif key == 'p':
        # pause following without exiting; robot returns home then holds
        if START:
            START = False
            PAUSE_REQUEST = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    elif key == 'a':
        # acknowledge a latched safety stop. This is the only way out of
        # SAFE_STOP -- deliberately a separate, explicit operator action.
        if SAFETY is not None and SAFETY.acknowledge(time.monotonic()):
            logger_mp.info("✅ safety fault acknowledged — [r] to re-arm")
        else:
            logger_mp.warning("[on_press] nothing to acknowledge")
    elif key == 'e':
        # operator emergency stop
        if SAFETY is not None:
            SAFETY.estop(time.monotonic(), "keyboard")
        START = False
    elif key == 'c':
        # cancel an alignment in progress (same effect as never having started)
        if START:
            START = False
            logger_mp.info("alignment cancelled")
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY, SAFETY, ALIGN_STATE
    state = {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
    }
    # XR link + safety telemetry, rendered by the dashboard's headset panel.
    state["XR"] = SAFETY.snapshot() if SAFETY is not None else None
    state["ALIGN"] = ALIGN_STATE
    return state

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device input tracking source')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default='immersive', help='Select XR device display mode')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'R1'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire_ftp', 'inspire_dfx', 'brainco'], help='Select end effector controller')
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity mode')
    # xr safety gate (see docs/xr_automation_and_safety_plan.md)
    parser.add_argument('--xr-stale-ms', type = float, default = 200.0,
                        help = 'No XR pose event for this long -> hold the arms')
    parser.add_argument('--xr-dead-ms', type = float, default = 1000.0,
                        help = 'No XR pose event for this long -> latched safe stop')
    parser.add_argument('--safe-stop-home', action = 'store_true', default = True,
                        help = 'On a safe stop, return the arms home slowly (default on)')
    parser.add_argument('--no-safe-stop-home', dest = 'safe_stop_home', action = 'store_false',
                        help = 'On a safe stop, freeze the arms in place and do not home them')
    parser.add_argument('--xr-source', type = str, choices = ['vuer', 'xrlink'], default = 'vuer',
                        help = 'XR transport: vuer (browser, default) or xrlink (native app)')
    parser.add_argument('--xrlink-port', type = int, default = 8443,
                        help = 'XrLink websocket port when --xr-source=xrlink')
    # start-alignment gate
    parser.add_argument('--align-pos-tol', type = float, default = 0.10,
                        help = 'Per-wrist position tolerance for the start-alignment gate (m)')
    parser.add_argument('--align-rot-tol', type = float, default = 25.0,
                        help = 'Per-wrist orientation tolerance for the start-alignment gate (deg)')
    parser.add_argument('--align-hold', type = float, default = 2.0,
                        help = 'Seconds the operator must hold the confirm gesture in position')
    parser.add_argument('--skip-align', action = 'store_true',
                        help = 'DANGEROUS. Skip the start-alignment gate; following begins '
                               'from whatever pose the operator is in.')
    parser.add_argument('--disable-xr-safety', action = 'store_true',
                        help = 'DANGEROUS. Bench/bringup only: run without the XR safety '
                               'gate, so losing the headset will NOT stop the robot.')
    # record mode and task info
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording mode')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')

    args = parser.parse_args()
    logger_mp.info(f"args: {args}")

    motion_switcher = None   # set in debug mode; used to restore ai mode on exit

    # XR safety gate. Nothing reaches the arms without its verdict.
    safety_cfg = SafetyConfig()
    safety_cfg.watchdog.stale_s = args.xr_stale_ms / 1000.0
    safety_cfg.watchdog.dead_s = args.xr_dead_ms / 1000.0
    SAFETY = SafetyFSM(safety_cfg,
                       on_event=lambda level, msg: getattr(logger_mp, level, logger_mp.info)(msg))
    if args.disable_xr_safety:
        logger_mp.error("=" * 70)
        logger_mp.error("⚠️  XR SAFETY GATE DISABLED (--disable-xr-safety)")
        logger_mp.error("⚠️  Removing the headset will NOT stop the robot. Bench use only.")
        logger_mp.error("=" * 70)

    # Start-alignment gate: the operator's wrists must match the robot's own,
    # verified from robot state, before following can begin.
    ALIGN = AlignGate(AlignConfig(pos_tol_m=args.align_pos_tol,
                                  rot_tol_deg=args.align_rot_tol,
                                  hold_s=args.align_hold))
    if args.skip_align:
        logger_mp.error("=" * 70)
        logger_mp.error("⚠️  START-ALIGNMENT GATE SKIPPED (--skip-align)")
        logger_mp.error("⚠️  The arms will jump to your pose the instant you start.")
        logger_mp.error("=" * 70)

    try:
        # setup dds communication domains id
        if args.sim:
            ChannelFactoryInitialize(1, networkInterface=args.network_interface)
        else:
            ChannelFactoryInitialize(0, networkInterface=args.network_interface)

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press,get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # image client
        img_client = ImageClient(host=args.img_server_ip)
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])

        # XR source. Everything below this point talks to `xr`, never to a
        # specific device -- see teleop/xr. Swapping the browser transport for
        # the native app is this one branch.
        if args.xr_source == "xrlink":
            from teleop.xr.link_server import XrLinkServer
            from teleop.xr.native_source import NativeXRSource
            link = XrLinkServer(port=args.xrlink_port,
                                on_estop=lambda: SAFETY.estop(time.monotonic(), "device"))
            if not link.start():
                raise RuntimeError("XrLink server failed to start")
            xr = NativeXRSource(link)
            tv_wrapper = None
        else:
            from teleop.xr.vuer_source import VuerXRSource
            # televuer_wrapper: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
            tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.input_mode == "hand",
                                         binocular=camera_config['head_camera']['binocular'],
                                         img_shape=camera_config['head_camera']['image_shape'],
                                         # maybe should decrease fps for better performance?
                                         # https://github.com/unitreerobotics/xr_teleoperate/issues/172
                                         # display_fps=camera_config['head_camera']['fps'] ? args.frequency? 30.0?
                                         display_mode=args.display_mode,
                                         zmq=camera_config['head_camera']['enable_zmq'],
                                         webrtc=camera_config['head_camera']['enable_webrtc'],
                                         webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
                                         )
            xr = VuerXRSource(tv_wrapper, use_hand_tracking=args.input_mode == "hand")
        logger_mp.info(f"XR source: {xr.name}")

        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller":
                loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        elif args.arm == "R1":
            arm_ik = R1_ArmIK()
            arm_ctrl = R1_ArmController(motion_mode=args.motion, simulation_mode=args.sim)

        # end-effector
        if args.ee == "dex3":
            from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                          dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "dex1":
            from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, 
                                                     dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim)
        elif args.ee == "inspire_dfx":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_DFX(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "inspire_ftp":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_FTP(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "brainco":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                           dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        else:
            pass
        
        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil
            p = psutil.Process(os.getpid())
            p.cpu_affinity([0,1,2,3]) # Set CPU affinity to cores 0-3
            try:
                p.nice(-20)           # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")
                
            for child in p.children(recursive=True):
                try:
                    logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5,6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record:
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless)

        logger_mp.info("----------------------------------------------------------------")
        logger_mp.info("🟢  Press [r] to start syncing the robot with your movements.")
        if args.record:
            logger_mp.info("🟡  Press [s] to START or SAVE recording (toggle cycle).")
        else:
            logger_mp.info("🔵  Recording is DISABLED (run with --record to enable).")
        logger_mp.info("🔴  Press [q] to stop and exit the program.")
        logger_mp.info("🟠  Press [e] for an emergency stop, [a] to acknowledge a safety fault.")
        logger_mp.info("⚠️  IMPORTANT: Please keep your distance and stay safe.")
        READY = True                  # now ready to (1) enter START state
        while not START and not STOP: # wait for start or stop signal.
            time.sleep(0.033)
            # Keep the safety telemetry live while idle so the dashboard can show
            # headset link state and refuse to arm until the operator is present.
            _idle = xr.read()
            SAFETY.update(time.monotonic(), _idle.liveness, _idle.head_pose,
                          _idle.left_wrist_pose, _idle.right_wrist_pose)
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img, _ = img_client.get_head_frame()
                xr.render_to_xr(head_img)

        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")
        was_following = False
        aligning = False
        _last_unsafe_warn = 0.0
        _fk_warned = False

        _last_state_push = 0.0
        _last_state_key = None

        def _begin_following():
            """Common entry into FOLLOWING: fresh baselines and a fresh ramp."""
            arm_ctrl.release_hold()
            arm_ctrl.restore_velocity_limit()
            arm_ctrl.speed_gradual_max()
            logger_mp.info("▶️  following armed")

        def _session_label():
            """(session, reason) as the headset should display it."""
            snap = SAFETY.snapshot()
            if snap["latched"]:
                return "SAFE_STOP", snap["reason"]
            if not was_following:
                return "IDLE", "" if snap["link_up"] else snap["reason"]
            return snap["state"].upper(), snap["reason"]

        def _push_device_state(now, session, reason="", align=None,
                               min_interval=0.5):
            """Mirror the host's view of the session into the headset.

            The operator in passthrough can see the robot but not *why* it
            stopped -- the terminal log is on the host and they are wearing a
            headset. This is the only channel that tells them, so it covers the
            whole session and not just alignment.

            Sends immediately on change, then repeats at `min_interval` as a
            keepalive so a headset that reconnects mid-session does not sit on a
            blank HUD until the next transition. No-ops on the Vuer path, where
            the transport is receive-only.
            """
            global _last_state_push, _last_state_key
            key = (session, reason)
            if key == _last_state_key and (now - _last_state_push) < min_interval:
                return
            _last_state_push, _last_state_key = now, key
            msg = {"t": "state", "session": session, "reason": reason}
            if align is not None:
                msg["align"] = align
            xr.send(msg)

        # main loop. robot start to follow VR user's motion
        while not STOP:
            start_time = time.time()

            # --- START edge: enter ALIGNMENT, not following -------------------
            # Every entry into following goes through alignment, so a resume
            # after a pause or a fault is re-verified exactly like a cold start.
            # There is no path that silently resumes motion.
            if START and not was_following and not aligning:
                if SAFETY.latched and not args.disable_xr_safety:
                    START = False
                    logger_mp.error("⛔ cannot start — safety fault latched. Press [a] to acknowledge.")
                elif args.skip_align:
                    if SAFETY.arm(time.monotonic()) or args.disable_xr_safety:
                        _begin_following()
                        was_following = True
                    else:
                        START = False
                else:
                    aligning = True
                    ALIGN.reset(time.monotonic())
                    logger_mp.info("🧍 alignment — match the robot's arm pose, then "
                                   "hold both pinches/triggers. [c] to cancel.")
            elif not START and (was_following or aligning):
                SAFETY.disarm(time.monotonic())
                was_following = False
                if aligning:
                    aligning = False
                    ALIGN_STATE = None

            # paused: stop following, return arms home once, then hold (loop stays alive)
            if not START:
                if PAUSE_REQUEST:
                    PAUSE_REQUEST = False
                    logger_mp.info("⏸️  paused — returning arms to home position")
                    try:
                        arm_ctrl.ctrl_dual_arm_go_home()
                    except Exception as e:
                        logger_mp.error(f"Failed to go home on pause: {e}")
                    logger_mp.info("⏸️  paused — hold. Press [r]/시작 to resume following.")
                # keep robot base still while paused
                if args.input_mode == "controller" and args.motion:
                    loco_wrapper.Move(0, 0, 0)
                # keep XR telemetry live while paused so the dashboard can show
                # headset presence and decide whether 시작 may be offered
                _idle = xr.read()
                SAFETY.update(time.monotonic(), _idle.liveness, _idle.head_pose,
                              _idle.left_wrist_pose, _idle.right_wrist_pose)
                # keep feeding the XR head image while paused
                if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                    head_img, _ = img_client.get_head_frame()
                    xr.render_to_xr(head_img)
                time.sleep(max(0, (1 / args.frequency) - (time.time() - start_time)))
                continue

            # get image
            if camera_config['head_camera']['enable_zmq']:
                if args.record or xr_need_local_img:
                    head_img, head_img_fps = img_client.get_head_frame()
                if xr_need_local_img:
                    xr.render_to_xr(head_img)
            #if camera_config['left_wrist_camera']['enable_zmq']:
            #    if args.record:
            #        left_wrist_img, _ = img_client.get_left_wrist_frame()
            #if camera_config['right_wrist_camera']['enable_zmq']:
            #    if args.record:
            #        right_wrist_img, _ = img_client.get_right_wrist_frame()

            # record mode
            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

            # get xr's tele data (device-neutral; see teleop/xr)
            frame = xr.read()
            left_target, right_target = frame.left_wrist_pose, frame.right_wrist_pose

            # Alignment pushes its own richer message below, including progress.
            if not aligning:
                _push_device_state(start_time, *_session_label())

            # -------------------- START-ALIGNMENT GATE ------------------------
            # The arms stay frozen here. Following begins only once the host
            # agrees (FK of the robot's own joints matches the operator's wrists)
            # AND the operator agrees (two-handed gesture), both held together.
            if aligning:
                arm_ctrl.hold()
                robot_l, robot_r = arm_ik.forward_kinematics(
                    arm_ctrl.get_current_dual_arm_q())
                if robot_l is None and not _fk_warned:
                    # Without FK the gate can never pass, and the operator would
                    # otherwise just see alignment hang until it times out.
                    _fk_warned = True
                    logger_mp.error("⛔ forward kinematics unavailable — alignment "
                                    "cannot be verified and will not pass. Fix the "
                                    "IK model, or use --skip-align at your own risk.")
                report = ALIGN.update(time.monotonic(), robot_l, robot_r,
                                      frame.left_wrist_pose, frame.right_wrist_pose,
                                      frame.confirm_gesture)
                ALIGN_STATE = report.as_dict()

                # Mirror progress into the headset where the transport allows it
                # (native only; the browser transport is receive-only). Faster
                # than the ordinary keepalive because the operator is watching a
                # progress bar and adjusting their stance against it.
                _push_device_state(start_time, "ALIGN", report.reason,
                                   align=ALIGN_STATE, min_interval=0.1)

                if report.accepted:
                    aligning = False
                    ALIGN_STATE = None
                    if SAFETY.arm(time.monotonic()) or args.disable_xr_safety:
                        _begin_following()
                        was_following = True
                        logger_mp.info("✅ aligned — following")
                    else:
                        START = False
                elif report.timed_out:
                    aligning = False
                    ALIGN_STATE = None
                    START = False
                    logger_mp.warning("⌛ alignment timed out — not started")
                else:
                    if args.input_mode == "controller" and args.motion:
                        loco_wrapper.Move(0, 0, 0)
                    time.sleep(max(0, (1 / args.frequency) - (time.time() - start_time)))
                    continue
            # ------------------------------------------------------------------

            # ---------------------- XR SAFETY GATE ----------------------------
            # Placed before the end-effector arrays, the locomotion commands and
            # the IK, so a fault freezes the hands and the base too -- not just
            # the arms. Nothing below this point runs on untrusted XR data.
            verdict = SAFETY.update(time.monotonic(), frame.liveness,
                                    frame.head_pose,
                                    frame.left_wrist_pose,
                                    frame.right_wrist_pose)
            if args.disable_xr_safety:
                if start_time - _last_unsafe_warn >= 10.0:
                    _last_unsafe_warn = start_time
                    logger_mp.warning("⚠️  XR safety gate is DISABLED "
                                      f"(would be: {verdict.action.value} {verdict.reason})")
            elif verdict.action is Action.SAFE_STOP:
                logger_mp.error(f"🛑 SAFE STOP — {verdict.reason}")
                START = False
                was_following = False
                if args.input_mode == "controller" and args.motion:
                    loco_wrapper.Move(0, 0, 0)
                arm_ctrl.safe_stop(go_home=args.safe_stop_home)
                if RECORD_RUNNING:
                    # Deliberately not auto-saved: an episode that ends in a
                    # fault is usually not a demonstration worth keeping, and
                    # that is the operator's call, not ours.
                    logger_mp.warning("🛑 recording is still open — [s] to save, or discard it")
                logger_mp.error("🛑 following latched off. Press [a] to acknowledge, then [r].")
                continue
            elif verdict.action is Action.HOLD:
                # The head image was already pushed to the headset earlier this
                # cycle, so the operator keeps seeing a live feed while held.
                arm_ctrl.hold()
                if args.input_mode == "controller" and args.motion:
                    loco_wrapper.Move(0, 0, 0)
                time.sleep(max(0, (1 / args.frequency) - (time.time() - start_time)))
                continue
            else:
                # PASS: use the rate-limited targets, never the raw XR poses.
                arm_ctrl.release_hold()
                left_target, right_target = verdict.left_wrist, verdict.right_wrist
            # ------------------------------------------------------------------

            if (args.ee == "dex3" or args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = frame.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = frame.right_hand_pos.flatten()
            elif args.ee == "dex1" and args.input_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = frame.left_ctrl_triggerValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = frame.right_ctrl_triggerValue
            elif args.ee == "dex1" and args.input_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = frame.left_hand_pinchValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = frame.right_hand_pinchValue
            else:
                pass
            
            # high level control
            if args.input_mode == "controller" and args.motion:
                # quit teleoperate
                if frame.right_ctrl_aButton:
                    START = False
                    STOP = True
                # command robot to enter damping mode. soft emergency stop function
                if frame.left_ctrl_thumbstick and frame.right_ctrl_thumbstick:
                    loco_wrapper.Damp()
                # https://github.com/unitreerobotics/xr_teleoperate/issues/135, control, limit velocity to within 0.3
                loco_wrapper.Move(-frame.left_ctrl_thumbstickValue[1] * 0.3,
                                  -frame.left_ctrl_thumbstickValue[0] * 0.3,
                                  -frame.right_ctrl_thumbstickValue[0]* 0.3)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff  = arm_ik.solve_ik(left_target, right_target, current_lr_arm_q, current_lr_arm_dq)
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            # record data
            if args.record:
                READY = recorder.is_ready() # now ready to (2) enter RECORD_RUNNING state
                # dex hand or gripper
                if args.ee == "dex3" and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-frame.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -frame.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -frame.right_ctrl_thumbstickValue[0] * 0.3]
                elif (args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []

                # arm state and action
                left_arm_state  = current_lr_arm_q[:7]
                right_arm_state = current_lr_arm_q[-7:]
                left_arm_action = sol_q[:7]
                right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}
                    if camera_config['head_camera']['binocular']:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img[:, :camera_config['head_camera']['image_shape'][1]//2]
                            colors[f"color_{1}"] = head_img[:, camera_config['head_camera']['image_shape'][1]//2:]
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{2}"] = left_wrist_img
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{3}"] = right_wrist_img
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    else:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img
                        else:
                            logger_mp.warning("Head image is None!")
                        #if camera_config['left_wrist_camera']['enable_zmq']:
                        #    if left_wrist_img is not None:
                        #        colors[f"color_{1}"] = left_wrist_img
                        #    else:
                        #        logger_mp.warning("Left wrist image is None!")
                        #if camera_config['right_wrist_camera']['enable_zmq']:
                        #    if right_wrist_img is not None:
                        #        colors[f"color_{2}"] = right_wrist_img
                        #    else:
                        #        logger_mp.warning("Right wrist image is None!")
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": {
                            "qpos": current_body_state,
                        }, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": {
                            "qpos": current_body_action,
                        }, 
                    }
                    if args.sim:
                        sim_state = sim_state_subscriber.read_data()            
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                    else:
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
    finally:
        try:
            # A prior safe stop leaves the velocity ceiling at SAFE_ARM_VELOCITY.
            # ctrl_dual_arm_go_home() gives up after ~5s, so leaving it there
            # would strand the arms part-way through the exit move.
            arm_ctrl.release_hold()
            arm_ctrl.restore_velocity_limit()
        except Exception as e:
            logger_mp.error(f"Failed to restore arm velocity limit: {e}")

        try:
            arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")
        
        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                listen_keyboard_thread.join()
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")
        
        try:
            img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            xr.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            # debug(상체) mode: restore ai mode on exit so the remote controller
            # regains control (remote only works in ai mode). motion(전신) mode
            # never entered debug, so nothing to restore.
            if not args.motion and motion_switcher is not None:
                ok, _name = motion_switcher.Exit_Debug_Mode()
                logger_mp.info(f"Restore ai mode: {'Success' if ok else 'Failed'}")
        except Exception as e:
            logger_mp.error(f"Failed to restore ai mode: {e}")

        try:
            if args.sim:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")
        
        try:
            if args.record:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")
        logger_mp.info("✅ Finally, exiting program.")
        exit(0)
