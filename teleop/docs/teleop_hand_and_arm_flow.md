# `teleop_hand_and_arm.py` 코드 흐름 문서

## 개요

이 파일은 **XR(VR) 디바이스를 통해 로봇 팔과 손을 원격 조작(Teleoperation)하는 메인 스크립트**입니다.

---

## 진입점 (Entry Point)

```python
if __name__ == '__main__':  # Line 73
```

---

## 전체 흐름도

```
┌─────────────────────────────────────────────────────────────────┐
│                    1. Argument Parsing (74-98)                  │
│         주요 옵션: --arm, --ee, --record, --sim, --motion       │
└─────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                2. 초기화 단계 (100-240)                          │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ • DDS 통신 초기화 (ChannelFactoryInitialize)             │   │
│  │ • 입력 핸들러 (IPC Server 또는 SSH Keyboard)             │   │
│  │ • ImageClient (카메라 이미지 수신)                       │   │
│  │ • TeleVuerWrapper (XR 디바이스 연동)                     │   │
│  │ • MotionSwitcher (로봇 모드 전환)                        │   │
│  │ • Arm Controller + IK Solver (팔 제어)                   │   │
│  │ • Hand/Gripper Controller (손/그리퍼 제어)               │   │
│  │ • EpisodeWriter (데이터 기록 - optional)                 │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│              3. 대기 루프 (251-255) - [r] 키 대기                │
│                   START=True 될 때까지 대기                      │
└─────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                   4. 메인 제어 루프 (260-479)                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  while not STOP:                                         │   │
│  │    ① 이미지 수신 (img_client)                            │   │
│  │    ② XR로 이미지 전송 (tv_wrapper.render_to_xr)          │   │
│  │    ③ 녹화 토글 처리 (RECORD_TOGGLE)                      │   │
│  │    ④ XR에서 tele_data 수신 (손/컨트롤러 위치)            │   │
│  │    ⑤ 손/그리퍼에 데이터 전달 (multiprocessing Array)     │   │
│  │    ⑥ 현재 로봇 팔 상태 조회                              │   │
│  │    ⑦ IK 계산 (arm_ik.solve_ik)                           │   │
│  │    ⑧ 팔 제어 명령 전송 (arm_ctrl.ctrl_dual_arm)          │   │
│  │    ⑨ 데이터 기록 (recorder.add_item) - optional          │   │
│  │    ⑩ 주파수 유지를 위한 sleep                            │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                  5. 종료 처리 (486-531)                          │
│         팔 홈 위치 복귀, 리소스 정리, 프로세스 종료               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 주요 컴포넌트

### 1. 상태 전이 (State Transition)

**위치**: Line 34-49

| 전역 변수 | 설명 |
|-----------|------|
| `START` | 로봇이 VR 사용자 동작을 따라하기 시작 |
| `STOP` | 시스템 종료 절차 시작 |
| `READY` | START 또는 RECORD_RUNNING 상태 진입 가능 |
| `RECORD_RUNNING` | 현재 녹화 중인지 여부 |
| `RECORD_TOGGLE` | 녹화 상태 토글 트리거 |

**상태 전이 다이어그램**:

```
  -------        ---------                -----------                -----------            ---------
   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
  -------        ---------      |         -----------      |         -----------      |     ---------
   START           True         |manual      True          |manual      True          |        True
   READY           True         |set         False         |set         False         |auto    True
   RECORD_RUNNING  False        |to          True          |to          False         |        False
                                ∨                          ∨                          ∨
   RECORD_TOGGLE   False       True          False        True          False                  False
```

**키보드 입력** (`on_press` 함수, Line 51-61):

| 키 | 동작 |
|----|------|
| `r` | START = True (로봇 추적 시작) |
| `q` | STOP = True (프로그램 종료) |
| `s` | RECORD_TOGGLE = True (녹화 시작/저장 토글) |

---

### 2. Arm Controller & IK

**위치**: Line 147-158

| 로봇 타입 | IK Solver | Controller |
|-----------|-----------|------------|
| G1_29 | `G1_29_ArmIK` | `G1_29_ArmController` |
| G1_23 | `G1_23_ArmIK` | `G1_23_ArmController` |
| H1_2 | `H1_2_ArmIK` | `H1_2_ArmController` |
| H1 | `H1_ArmIK` | `H1_ArmController` |

**주요 메서드**:

| 메서드 | 설명 |
|--------|------|
| `arm_ik.solve_ik(left_wrist, right_wrist, q, dq)` | 손목 포즈 → 관절 각도 계산 |
| `arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)` | 양팔 제어 명령 전송 |
| `arm_ctrl.get_current_dual_arm_q()` | 현재 관절 각도 조회 |
| `arm_ctrl.get_current_dual_arm_dq()` | 현재 관절 속도 조회 |
| `arm_ctrl.ctrl_dual_arm_go_home()` | 팔을 홈 위치로 복귀 |

---

### 3. End-Effector Controller

**위치**: Line 161-205

| --ee 옵션 | Controller 클래스 | 입력 방식 |
|-----------|-------------------|-----------|
| `dex3` | `Dex3_1_Controller` | 손 추적 (75차원 포즈) |
| `dex1` | `Dex1_1_Gripper_Controller` | 컨트롤러 트리거 또는 핀치 |
| `inspire_dfx` | `Inspire_Controller_DFX` | 손 추적 |
| `inspire_ftp` | `Inspire_Controller_FTP` | 손 추적 |
| `brainco` | `Brainco_Controller` | 손 추적 |

**Multiprocessing 공유 메모리**:

| 변수 | 크기 | 설명 |
|------|------|------|
| `left_hand_pos_array` | 75 | XR 왼손 위치 데이터 (입력) |
| `right_hand_pos_array` | 75 | XR 오른손 위치 데이터 (입력) |
| `dual_hand_state_array` | 12-14 | 양손 상태 데이터 (출력) |
| `dual_hand_action_array` | 12-14 | 양손 액션 데이터 (출력) |

---

### 4. TeleVuerWrapper

**위치**: Line 125-135

XR 디바이스와의 통신을 담당합니다.

| 메서드 | 설명 |
|--------|------|
| `get_tele_data()` | 손목 포즈, 손가락 위치, 컨트롤러 입력 수신 |
| `render_to_xr(image)` | 로봇 카메라 이미지를 XR 디스플레이로 전송 |
| `close()` | 연결 종료 |

**`tele_data` 구조체 주요 필드**:

| 필드 | 설명 |
|------|------|
| `left_wrist_pose` | 왼쪽 손목 포즈 |
| `right_wrist_pose` | 오른쪽 손목 포즈 |
| `left_hand_pos` | 왼손 관절 위치 (5x5x3) |
| `right_hand_pos` | 오른손 관절 위치 (5x5x3) |
| `left_hand_pinchValue` | 왼손 핀치 값 |
| `right_hand_pinchValue` | 오른손 핀치 값 |
| `left_ctrl_triggerValue` | 왼쪽 컨트롤러 트리거 값 |
| `right_ctrl_triggerValue` | 오른쪽 컨트롤러 트리거 값 |
| `left_ctrl_thumbstickValue` | 왼쪽 조이스틱 값 |
| `right_ctrl_thumbstickValue` | 오른쪽 조이스틱 값 |

---

### 5. ImageClient

**위치**: Line 119-121

로봇의 카메라 이미지를 수신합니다.

| 메서드 | 설명 |
|--------|------|
| `get_cam_config()` | 카메라 설정 조회 |
| `get_head_frame()` | 머리 카메라 이미지 수신 |
| `get_left_wrist_frame()` | 왼쪽 손목 카메라 이미지 (주석 처리됨) |
| `get_right_wrist_frame()` | 오른쪽 손목 카메라 이미지 (주석 처리됨) |
| `close()` | 연결 종료 |

---

### 6. EpisodeWriter

**위치**: Line 234-240

데이터 녹화를 담당합니다 (`--record` 옵션 사용 시).

| 메서드 | 설명 |
|--------|------|
| `create_episode()` | 새 에피소드 생성 |
| `add_item(colors, depths, states, actions)` | 프레임 데이터 추가 |
| `save_episode()` | 에피소드 저장 |
| `is_ready()` | 녹화 준비 상태 확인 |
| `close()` | 리소스 정리 |

**녹화 데이터 구조**:

```python
states = {
    "left_arm":  {"qpos": [...], "qvel": [], "torque": []},
    "right_arm": {"qpos": [...], "qvel": [], "torque": []},
    "left_ee":   {"qpos": [...], "qvel": [], "torque": []},
    "right_ee":  {"qpos": [...], "qvel": [], "torque": []},
    "body":      {"qpos": [...]},
}

actions = {
    "left_arm":  {"qpos": [...], "qvel": [], "torque": []},
    "right_arm": {"qpos": [...], "qvel": [], "torque": []},
    "left_ee":   {"qpos": [...], "qvel": [], "torque": []},
    "right_ee":  {"qpos": [...], "qvel": [], "torque": []},
    "body":      {"qpos": [...]},
}
```

---

## 의존성 모듈

| 모듈 | 파일 경로 | 역할 |
|------|-----------|------|
| `televuer` | `../televuer/` | XR 디바이스 통신 |
| `teleimager` | `../teleimager/` | 카메라 이미지 클라이언트 |
| `robot_arm` | `robot_control/robot_arm.py` | 팔 제어 |
| `robot_arm_ik` | `robot_control/robot_arm_ik.py` | IK 솔버 |
| `episode_writer` | `utils/episode_writer.py` | 데이터 기록 |
| `ipc` | `utils/ipc.py` | IPC 서버 |
| `motion_switcher` | `utils/motion_switcher.py` | 로봇 모드 전환 |
| `sim_state_topic` | `utils/sim_state_topic.py` | 시뮬레이션 상태 구독 |

---

## CLI 인자 (Arguments)

### 기본 제어 파라미터

| 인자 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `--frequency` | float | 30.0 | 제어 및 녹화 주파수 (Hz) |
| `--input-mode` | str | `hand` | XR 입력 소스 (`hand` / `controller`) |
| `--display-mode` | str | `immersive` | XR 디스플레이 모드 (`immersive` / `ego` / `pass-through`) |
| `--arm` | str | `G1_29` | 팔 컨트롤러 타입 (`G1_29` / `G1_23` / `H1_2` / `H1`) |
| `--ee` | str | - | End-effector 타입 (`dex1` / `dex3` / `inspire_ftp` / `inspire_dfx` / `brainco`) |
| `--img-server-ip` | str | `192.168.123.164` | 이미지 서버 IP |
| `--network-interface` | str | None | DDS 통신용 네트워크 인터페이스 |

### 모드 플래그

| 인자 | 설명 |
|------|------|
| `--motion` | 모션 제어 모드 활성화 |
| `--headless` | 헤드리스 모드 (디스플레이 없음) |
| `--sim` | Isaac 시뮬레이션 모드 |
| `--ipc` | IPC 서버 활성화 (sshkeyboard 대신) |
| `--affinity` | CPU 친화성 및 높은 우선순위 설정 |

### 녹화 관련

| 인자 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `--record` | flag | - | 데이터 녹화 모드 활성화 |
| `--task-dir` | str | `./utils/data/` | 데이터 저장 경로 |
| `--task-name` | str | `pick cube` | 태스크 이름 |
| `--task-goal` | str | `pick up cube.` | 태스크 목표 |
| `--task-desc` | str | `task description` | 태스크 설명 |
| `--task-steps` | str | `step1: do this; step2: do that;` | 태스크 단계 |

---

## 실행 예시

```bash
# 기본 실행 (G1_29 로봇, 손 추적, dex3 핸드)
python teleop_hand_and_arm.py --arm G1_29 --ee dex3

# 시뮬레이션 + 녹화 모드
python teleop_hand_and_arm.py --arm G1_29 --ee dex3 --sim --record --task-name "pick_cube"

# 컨트롤러 입력 + 이동 제어 (dex1 그리퍼)
python teleop_hand_and_arm.py --arm G1_29 --ee dex1 --input-mode controller --motion

# 헤드리스 녹화 모드
python teleop_hand_and_arm.py --arm G1_29 --ee dex3 --record --headless

# IPC 서버 모드 (외부 프로그램에서 제어)
python teleop_hand_and_arm.py --arm G1_29 --ee dex3 --ipc
```

---

## 시퀀스 다이어그램

```
┌─────┐     ┌──────────┐     ┌─────────┐     ┌────────┐     ┌─────────┐
│ XR  │     │TeleVuer  │     │ ArmIK   │     │ArmCtrl │     │ Robot   │
└──┬──┘     └────┬─────┘     └────┬────┘     └───┬────┘     └────┬────┘
   │             │                │              │               │
   │ hand pose   │                │              │               │
   │────────────>│                │              │               │
   │             │                │              │               │
   │             │ tele_data      │              │               │
   │             │───────────────>│              │               │
   │             │                │              │               │
   │             │                │ solve_ik()   │               │
   │             │                │─────────────>│               │
   │             │                │              │               │
   │             │                │              │ ctrl_dual_arm │
   │             │                │              │──────────────>│
   │             │                │              │               │
   │             │                │              │  current_q    │
   │             │                │              │<──────────────│
   │             │                │              │               │
   │  image      │                │              │               │
   │<────────────│                │              │               │
   │             │                │              │               │
```
