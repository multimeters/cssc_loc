#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [output-directory]" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$HOME/hunter_ros2/install/setup.bash"
source "$ROOT_DIR/scripts/workspace_env.sh"
if [[ ! -f "$BUILD_ROOT/install/setup.bash" ]]; then
  echo "CSSC workspace overlay is missing; build the workspace first." >&2
  exit 2
fi
source "$BUILD_ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${1:-$ROOT_DIR/logs/waypoint4_diagnostics/$RUN_ID}"
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"
BAG_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    echo "[record] finalizing ROS bag..."
    kill -INT "$BAG_PID" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "$BAG_PID" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "$BAG_PID" 2>/dev/null; then
      kill -TERM "$BAG_PID" 2>/dev/null || true
    fi
    wait "$BAG_PID" 2>/dev/null || true
  fi
  echo "[record] files saved under $OUTPUT_DIR"
  exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

TOPICS=(
  /localization/kinematic_state
  /localization/fusion_status
  /localization/pose_estimator/pose_with_covariance
  /localization/pose_estimator/transform_probability
  /cmd_vel
  /hunter_odom
  /hunter_status
  /initialpose
  /localization/initialpose
  /rosout
  /tf
  /tf_static
)

printf '[record] Output directory: %s\n' "$OUTPUT_DIR"
ros2 bag record --storage sqlite3 -o "$OUTPUT_DIR/rosbag" "${TOPICS[@]}" \
  >"$OUTPUT_DIR/rosbag_record.log" 2>&1 &
BAG_PID=$!
sleep 1
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "[record] ROS bag recorder did not start; see $OUTPUT_DIR/rosbag_record.log" >&2
else
  echo "[record] ROS bag is capturing the diagnostic topics."
fi

python3 "$ROOT_DIR/scripts/waypoint4_diagnostics_monitor.py" \
  --output-dir "$OUTPUT_DIR" --sample-rate-hz 20
