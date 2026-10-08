#!/usr/bin/env bash
set -eo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/scripts/workspace_env.sh"

if [[ ! -f "$HOME/hunter_ros2/install/setup.bash" ]]; then
  echo "Hunter ROS 2 overlay is missing: $HOME/hunter_ros2/install/setup.bash" >&2
  exit 2
fi
if [[ ! -f "$BUILD_ROOT/install/setup.bash" ]]; then
  echo "CSSC workspace overlay is not built; run: $ROOT_DIR/scripts/build.sh all" >&2
  exit 2
fi
source "$HOME/hunter_ros2/install/setup.bash"
source "$BUILD_ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

HUNTER_BIN="$HOME/hunter_ros2/install/hunter_base/lib/hunter_base/hunter_base_node"
HUNTER_PARAMS="$ROOT_DIR/config/hunter_base.yaml"
LOG_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/cssc_loc/pure_pursuit"
mkdir -p "$LOG_DIR"
hunter_driver_pid=""

find_hunter_driver_pid() {
  local proc exe
  for proc in /proc/[0-9]*; do
    [[ -e "$proc/exe" ]] || continue
    exe="$(readlink -f "$proc/exe" 2>/dev/null || true)"
    if [[ "$(basename "$exe")" == "hunter_base_node" ]]; then
      echo "${proc##*/}"
      return 0
    fi
  done
  return 1
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "$hunter_driver_pid" ]] && kill -0 "$hunter_driver_pid" 2>/dev/null; then
    echo "[tracking] stopping Hunter driver started by this script (pid $hunter_driver_pid)"
    kill -TERM "$hunter_driver_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$hunter_driver_pid" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "$hunter_driver_pid" 2>/dev/null; then
      kill -KILL "$hunter_driver_pid" 2>/dev/null || true
    fi
    wait "$hunter_driver_pid" 2>/dev/null || true
  fi
  exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if existing_pid="$(find_hunter_driver_pid)"; then
  echo "[tracking] Hunter chassis driver already running (pid $existing_pid); reusing it."
else
  if [[ ! -x "$HUNTER_BIN" ]]; then
    echo "[tracking] Hunter driver binary not found: $HUNTER_BIN" >&2
    exit 2
  fi
  if ! ip -br link show can0 2>/dev/null | awk '{print $2}' | grep -qx UP; then
    echo "[tracking] bringing up can0 at 500000 bit/s"
    sudo modprobe gs_usb
    sudo ip link set can0 down 2>/dev/null || true
    sudo ip link set can0 up type can bitrate 500000
  fi
  echo "[tracking] starting Hunter chassis driver with $HUNTER_PARAMS"
  nohup "$HUNTER_BIN" --ros-args --params-file "$HUNTER_PARAMS" \
    >>"$LOG_DIR/hunter_base.log" 2>&1 &
  hunter_driver_pid=$!
  sleep 1
  if ! kill -0 "$hunter_driver_pid" 2>/dev/null; then
    echo "[tracking] Hunter driver exited during startup; see $LOG_DIR/hunter_base.log" >&2
    tail -30 "$LOG_DIR/hunter_base.log" >&2 || true
    exit 1
  fi
fi

echo "[tracking] waiting for Hunter /hunter_odom"
if ! timeout 20s ros2 topic echo /hunter_odom --once >/dev/null 2>"$LOG_DIR/odom_wait.log"; then
  echo "[tracking] no /hunter_odom message received; refusing to start tracking." >&2
  if [[ -s "$LOG_DIR/odom_wait.log" ]]; then
    tail -20 "$LOG_DIR/odom_wait.log" >&2 || true
  fi
  exit 1
fi

echo "[tracking] starting Pure Pursuit; localization must report FUSED before motion."
ros2 launch hunter_pure_pursuit pure_pursuit.launch.py
