#!/usr/bin/env bash
# Linux / WSL single entry point. Settings and extrinsics live in YAML.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/workspace_env.sh"
cd "$ROOT_DIR"
CHECK_ONLY=false
INSTALL_DEPS=false
BUILD=true
BRINGUP_SENSORS=true
START_RVIZ=true
RUN_ARGS=()
MODE_HINT=""
RVIZ_PID=""
for arg in "$@"; do
  case "$arg" in
    --install-deps) INSTALL_DEPS=true ;;
    --no-build) BUILD=false ;;
    --check) CHECK_ONLY=true ;;
    --no-sensors) BRINGUP_SENSORS=false ;;
    --no-rviz) START_RVIZ=false ;;
    --mode=live) MODE_HINT=live; RUN_ARGS+=(--mode live) ;;
    --mode=replay) MODE_HINT=replay; RUN_ARGS+=(--mode replay) ;;
    -h|--help)
      cat <<'HELP'
中文说明：默认实时定位；自动拉起底盘+雷达，并打开 RViz 显示地图以便点初值。
  bash start.sh                       一键：传感器 → 定位 → RViz
  bash start.sh --no-rviz             不自动开 RViz
  bash start.sh --no-sensors          不自动拉起硬件
  bash start.sh --mode replay         回放录包
  bash start.sh --no-build
  bash start.sh --check
HELP
      exit 0 ;;
    *) RUN_ARGS+=("$arg") ;;
  esac
done

for ((i=0; i<${#RUN_ARGS[@]}; i++)); do
  if [[ "${RUN_ARGS[$i]}" == "--mode" && $((i+1)) -lt ${#RUN_ARGS[@]} ]]; then
    MODE_HINT="${RUN_ARGS[$((i+1))]}"
  fi
done
if [[ -z "$MODE_HINT" ]]; then
  MODE_HINT="$(python3 - <<'PY'
import yaml
from pathlib import Path
print(yaml.safe_load(Path('config/localization.yaml').read_text()).get('runtime',{}).get('mode','live'))
PY
)"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '未找到 ROS 2 Humble。请先准备 Ubuntu 22.04 + ROS 2 Humble，再运行此入口。' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
if "$INSTALL_DEPS"; then
  bash "$ROOT_DIR/scripts/install_dependencies.sh"
fi
if ! python3 -c 'import yaml; import rclpy' >/dev/null 2>&1; then
  echo '缺少 PyYAML 或 ROS Python 依赖，请运行 bash start.sh --install-deps。' >&2
  exit 2
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo '未找到 colcon，请运行 bash start.sh --install-deps。' >&2
  exit 2
fi

python3 "$ROOT_DIR/scripts/run_localization.py" "${RUN_ARGS[@]}" --check
if "$CHECK_ONLY"; then
  echo "检查通过。构建缓存：$BUILD_ROOT"
  exit 0
fi
if "$BUILD"; then
  echo '开始增量编译定位工作区……'
  bash "$ROOT_DIR/scripts/build.sh" all
fi
if [[ ! -f "$BUILD_ROOT/install/setup.bash" ]]; then
  echo '当前目录尚未编译成功，请去掉 --no-build 再运行。' >&2
  exit 2
fi
source "$BUILD_ROOT/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

RVIZ_CFG="$ROOT_DIR/config/cssc_live.rviz"

cat <<EOF

========== CSSC 定位必要话题 ==========
模式: $MODE_HINT    DOMAIN=$ROS_DOMAIN_ID
【输入】 /livox/lidar  /livox/imu  /hunter_odom  /initialpose
【输出】 /localization/kinematic_state
        /localization/pose_with_covariance
        /localization/fusion_status
        TF: map → base_footprint
【RViz】 Fixed Frame=map
        地图: /map/output/debug/downsampled_pointcloud_map
        工具: 「2D Pose Estimate」→ /initialpose
======================================

EOF

cleanup() {
  echo
  if [[ -n "${RVIZ_PID:-}" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "[start] 关闭 RViz (pid $RVIZ_PID)"
    kill "$RVIZ_PID" 2>/dev/null || true
  fi
  if [[ "$MODE_HINT" == "live" ]] && "$BRINGUP_SENSORS"; then
    echo '[start] 停止本次自动拉起的 hunter/livox…'
    bash "$ROOT_DIR/scripts/bringup_sensors_chassis.sh" stop || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$MODE_HINT" == "live" ]] && "$BRINGUP_SENSORS"; then
  echo '拉起传感器与底盘（话题已有发布者则跳过）…'
  bash "$ROOT_DIR/scripts/bringup_sensors_chassis.sh" start || {
    echo '传感器/底盘 bringup 有警告，仍继续启动定位' >&2
  }
fi

start_rviz() {
  if [[ "$MODE_HINT" != "live" ]] || ! "$START_RVIZ"; then
    return 0
  fi
  if [[ ! -f "$RVIZ_CFG" ]]; then
    echo "[start] 未找到 $RVIZ_CFG，跳过 RViz" >&2
    return 0
  fi
  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    # 常见本机桌面 :0/:1；无图形会话则跳过
    for _d in :1 :0; do
      if [[ -S "/tmp/.X11-unix/X${_d#:}" ]] && DISPLAY="$_d" xdpyinfo >/dev/null 2>&1; then
        export DISPLAY="$_d"
        echo "[start] 自动设置 DISPLAY=$DISPLAY"
        break
      fi
    done
  fi
  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo '[start] 无 DISPLAY，无法开 RViz（请在桌面终端运行）' >&2
    return 0
  fi
  # 稍等地图 loader 发布 Transient Local 点云后再开，避免空白
  (
    sleep 4
    echo "[start] 打开 RViz: $RVIZ_CFG"
    rviz2 -d "$RVIZ_CFG" >/tmp/cssc_loc_rviz.log 2>&1
  ) &
  RVIZ_PID=$!
  echo "[start] RViz 将在约 4s 后打开 (pid $RVIZ_PID)；日志 /tmp/cssc_loc_rviz.log"
  echo "[start] 看到灰色地图后，点工具栏「2D Pose Estimate」在车上位置拖一下朝向"
}

start_rviz

echo '开始定位；按 Ctrl+C 可停止。'
python3 "$ROOT_DIR/scripts/run_localization.py" "${RUN_ARGS[@]}"
