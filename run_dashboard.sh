#!/usr/bin/env bash
# Launch the G1 humanoid teleop dashboard inside the `vtv` conda env.
#
# Usage:
#   ./run_dashboard.sh              # real robot  (DDS domain 0, iface enp4s0)
#   ./run_dashboard.sh sim          # simulation  (DDS domain 1)
#   ./run_dashboard.sh --camera     # extra flags pass straight through
#   ./run_dashboard.sh sim --camera # mode + extra flags
#
# Everything is preconfigured for the wired robot link
# (host 192.168.123.2/24 -> robot 192.168.123.164 on enp4s0).
set -euo pipefail

# ---- config -----------------------------------------------------------------
CONDA_HOME="/home/wego/miniconda3"
ENV_NAME="vtv"
NET_IFACE="enp4s0"          # wired interface on the 192.168.123.x subnet
ROBOT_IP="192.168.123.164"  # image server / robot host

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- pick mode --------------------------------------------------------------
MODE="real"
if [[ "${1:-}" == "sim" || "${1:-}" == "real" ]]; then
  MODE="$1"; shift
fi

if [[ "$MODE" == "sim" ]]; then
  ARGS=(--domain 1)
else
  ARGS=(--domain 0 --net "$NET_IFACE" --img-server-ip "$ROBOT_IP")
fi

# ---- activate env -----------------------------------------------------------
# shellcheck disable=SC1091
source "$CONDA_HOME/etc/profile.d/conda.sh"
conda activate "$ENV_NAME"

# ---- run --------------------------------------------------------------------
cd "$SCRIPT_DIR"
echo "[run_dashboard] mode=$MODE env=$ENV_NAME  args: ${ARGS[*]} $*"
exec python teleop/dashboard.py "${ARGS[@]}" "$@"
