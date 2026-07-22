# `--motion` 인자 동작 분석

명령 예시:
```
python teleop_hand_and_arm.py --input-mode=controller --motion --img-server-ip=192.168.123.164
```

`--motion`을 켜면 **하체는 고수준(LocoClient) 제어**, **상체는 텔레오퍼레이션(IK)** 로 분리되어 동시에 동작한다.

---

## 1) 인자 정의 — `teleop/teleop_hand_and_arm.py:84`
```python
parser.add_argument('--motion', action='store_true', help='Enable motion control mode')
```

## 2) 모드 분기 (하체 측) — `teleop/teleop_hand_and_arm.py:137-144`
```python
# motion mode (G1: Regular mode R1+X, not Running mode R2+A)
if args.motion:
    if args.input_mode == "controller":
        loco_wrapper = LocoClientWrapper()   # 하체 고수준 제어 클라이언트
else:
    motion_switcher = MotionSwitcher()
    status, result = motion_switcher.Enter_Debug_Mode()  # 디버그(저수준) 모드
```
`--motion`을 켜면 `Enter_Debug_Mode()`를 호출하지 않고 로봇을 **Regular(고수준) 모드**에 둔 채 `LocoClientWrapper`로 하체를 제어한다. 끄면 디버그 모드로 들어가 전신 저수준 제어가 된다.

## 3) 상체(팔) 측에 motion_mode 전달 — `teleop_hand_and_arm.py:147-155`
```python
arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
```
`motion_mode=True`면 `ArmController`가 고수준 모드와 공존하는 형태로 팔만 명령을 보내도록 동작한다 (`teleop/robot_control/robot_arm.py`의 `motion_mode` 분기 참고).

## 4) 메인 루프에서 하체 고수준 명령 — `teleop_hand_and_arm.py:309-321`
```python
# high level control
if args.input_mode == "controller" and args.motion:
    if tele_data.right_ctrl_aButton:           # 종료
        START = False; STOP = True
    if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
        loco_wrapper.Damp()                    # 소프트 비상정지
    loco_wrapper.Move(-tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                      -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                      -tele_data.right_ctrl_thumbstickValue[0] * 0.3)  # vx, vy, vyaw
```
컨트롤러 썸스틱 → `loco_wrapper.Move(vx, vy, vyaw)` 로 하체 보행/회전을 고수준 API로 호출한다.

## 5) 같은 루프에서 상체는 IK 텔레옵 — `teleop_hand_and_arm.py:323-332`
```python
current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()
sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_wrist_pose,
                                    tele_data.right_wrist_pose,
                                    current_lr_arm_q, current_lr_arm_dq)
arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)       # 팔만 저수준으로 추종
```

## 6) `LocoClientWrapper` / `MotionSwitcher` 정의
`teleop/utils/motion_switcher.py` (import는 `teleop_hand_and_arm.py:22`). Unitree SDK의 `LocoClient`(고수준 보행 클라이언트)를 감싼 래퍼이고, `MotionSwitcher`는 Regular ↔ Debug 모드 전환을 담당한다.

---

## 요약
`--motion`이 켜지면:
1. `Enter_Debug_Mode()`를 건너뛰어 로봇을 **고수준 모드**로 유지
2. 하체는 `LocoClientWrapper.Move/Damp` (309-321행)
3. 상체는 `ArmController(motion_mode=True)` + IK (323-332행)

→ 하체 고수준 제어와 상체 텔레오퍼레이션이 동시에 동작한다.
