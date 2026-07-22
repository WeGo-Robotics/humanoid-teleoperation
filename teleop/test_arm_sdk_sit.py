"""
Test whether the robot's CURRENT FSM (e.g. a seated / sit posture) accepts
arm_sdk overrides on topic rt/arm_sdk.

What it does:
  1. Subscribes rt/lowstate, reads current arm joint angles.
  2. Publishes rt/arm_sdk with the arm_sdk weight (motor 29 = kNotUsedJoint0)
     ramped 0 -> 1, holding every arm joint at its current angle EXCEPT one
     test joint (default: left elbow) which is nudged by a small delta.
  3. Reads back rt/lowstate and measures how far the test joint actually moved.
  4. Verdict:
        moved close to target      -> PASS  (this FSM accepts arm_sdk)
        did not move               -> FAIL  (this FSM ignores arm_sdk)

SAFETY
  - Moves ONLY ONE arm joint by a SMALL amount (default 0.30 rad, low kp).
  - Keep E-STOP in hand. Make sure arm path is clear.
  - Real robot: domain id 0. Simulation: domain id 1.

Usage
  python test_arm_sdk_sit.py --net eth0                 # real robot
  python test_arm_sdk_sit.py --sim                      # sim (domain 1)
  python test_arm_sdk_sit.py --net eth0 --joint 18 --delta 0.3
"""
import sys, time, argparse
import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd, LowState_ as hg_LowState
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

kTopicLowCmd_ArmSdk = "rt/arm_sdk"
kTopicLowState      = "rt/lowstate"

# G1_29 indices
ARM_IDS   = list(range(15, 29))          # 15..28 both arms
WEIGHT_ID = 29                           # kNotUsedJoint0 -> arm_sdk weight
WEAK_ARM  = {15,16,17,18, 22,23,24,25}   # shoulders + elbow (weak motors)
WRIST     = {19,20,21, 26,27,28}

CONTROL_DT = 1.0 / 250.0
VEL_LIMIT  = 0.6                          # rad/s cap on the moving joint (gentle)


def kp_kd(mid):
    if mid in WRIST: return 40.0, 1.5
    if mid in WEAK_ARM: return 80.0, 3.0
    return 80.0, 3.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--net", type=str, default="", help="network interface, e.g. eth0 (real robot)")
    ap.add_argument("--sim", action="store_true", help="simulation mode (domain id 1)")
    ap.add_argument("--joint", type=int, default=18, help="test joint index (default 18 = left elbow)")
    ap.add_argument("--delta", type=float, default=0.30, help="target offset rad (default 0.30)")
    ap.add_argument("--ramp", type=float, default=3.0, help="seconds to ramp weight 0->1")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds to hold at target and measure")
    ap.add_argument("--pass-thresh", type=float, default=0.5, help="fraction of delta that counts as PASS")
    args = ap.parse_args()

    assert args.joint in ARM_IDS, f"--joint must be one of {ARM_IDS}"

    domain = 1 if args.sim else 0
    if args.net:
        ChannelFactoryInitialize(domain, args.net)
    else:
        ChannelFactoryInitialize(domain)

    # subscriber
    sub = ChannelSubscriber(kTopicLowState, hg_LowState)
    sub.Init()
    print("[*] waiting for rt/lowstate ...")
    low = None
    for _ in range(500):
        low = sub.Read()
        if low is not None:
            break
        time.sleep(0.01)
    assert low is not None, "no rt/lowstate received. check net/domain/robot."

    def read_q(mid):
        return sub.Read().motor_state[mid].q

    q_start = {mid: low.motor_state[mid].q for mid in ARM_IDS}
    j = args.joint
    q0 = q_start[j]
    q_target_final = q0 + args.delta
    print(f"[*] test joint {j}: start q = {q0:+.4f}, target = {q_target_final:+.4f} (delta {args.delta:+.3f})")

    # publisher
    pub = ChannelPublisher(kTopicLowCmd_ArmSdk, hg_LowCmd)
    pub.Init()
    crc = CRC()
    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 0
    msg.mode_machine = low.mode_machine

    for mid in ARM_IDS:
        kp, kd = kp_kd(mid)
        msg.motor_cmd[mid].mode = 1
        msg.motor_cmd[mid].kp = kp
        msg.motor_cmd[mid].kd = kd
        msg.motor_cmd[mid].dq = 0.0
        msg.motor_cmd[mid].tau = 0.0
        msg.motor_cmd[mid].q = q_start[mid]

    cur_cmd = q0                      # velocity-clipped commanded pos for test joint
    t_ramp = args.ramp
    t_total = args.ramp + args.hold
    print("[*] publishing rt/arm_sdk ... (ramp weight 0->1, then hold)")
    t_begin = time.time()
    max_reached = q0
    try:
        while True:
            now = time.time() - t_begin
            if now > t_total:
                break
            weight = min(1.0, now / t_ramp) if t_ramp > 0 else 1.0
            msg.motor_cmd[WEIGHT_ID].q = weight

            # velocity-clipped ramp of the test joint toward target
            step = VEL_LIMIT * CONTROL_DT
            if cur_cmd < q_target_final:
                cur_cmd = min(q_target_final, cur_cmd + step)
            else:
                cur_cmd = max(q_target_final, cur_cmd - step)
            msg.motor_cmd[j].q = cur_cmd

            msg.crc = crc.Crc(msg)
            pub.Write(msg)

            actual = read_q(j)
            if abs(actual - q0) > abs(max_reached - q0):
                max_reached = actual
            if int(now * 5) != int((now - CONTROL_DT) * 5):
                print(f"  t={now:4.1f}s w={weight:3.2f} cmd={cur_cmd:+.3f} actual={actual:+.3f} moved={actual-q0:+.3f}")
            time.sleep(CONTROL_DT)

        # measure at end
        time.sleep(0.2)
        q_end = read_q(j)
        moved = q_end - q0
        frac = moved / args.delta if args.delta != 0 else 0.0
        print("\n==================== RESULT ====================")
        print(f" start   q : {q0:+.4f}")
        print(f" target  q : {q_target_final:+.4f}  (delta {args.delta:+.3f})")
        print(f" end     q : {q_end:+.4f}")
        print(f" max move  : {max_reached-q0:+.4f}")
        print(f" moved     : {moved:+.4f}  ({frac*100:5.1f}% of target)")
        if abs(frac) >= args.pass_thresh:
            print(" VERDICT   : PASS  -> this FSM ACCEPTS arm_sdk (teleop --motion will work here)")
        elif abs(moved) < 0.02:
            print(" VERDICT   : FAIL  -> joint did not move. This FSM IGNORES arm_sdk.")
        else:
            print(" VERDICT   : PARTIAL -> some motion but weak. FSM may fight arm_sdk. Inspect.")
        print("================================================")
    finally:
        # release: ramp weight back to 0 so the internal controller retakes arms
        print("[*] releasing arm_sdk (weight -> 0) ...")
        for w in np.linspace(msg.motor_cmd[WEIGHT_ID].q, 0.0, 50):
            msg.motor_cmd[WEIGHT_ID].q = float(w)
            for mid in ARM_IDS:
                msg.motor_cmd[mid].q = read_q(mid)
            msg.crc = crc.Crc(msg)
            pub.Write(msg)
            time.sleep(0.02)
        print("[*] done.")


if __name__ == "__main__":
    main()
