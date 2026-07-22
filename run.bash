#!/bin/bash
set -x   # 디버깅용 (나중에 지워도 됨)

# conda 초기화 (bashrc 말고 직접)
source ~/miniconda3/etc/profile.d/conda.sh

# conda env 활성화
conda activate tv

# ROS humble
source /opt/ros/humble/setup.bash

# 작업 디렉토리 이동
cd ~/xr_teleoperate/teleop || exit 1

# 실행 (python3 권장)
python teleop_hand_and_arm.py --input-mode=controller --motion --img-server-ip=192.168.123.164