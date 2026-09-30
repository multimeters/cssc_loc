#!/usr/bin/env bash
# 为 cssc_loc 实时定位拉起：can0 + hunter_base + Livox Mid-360
# 注意：禁止 pkill -f 匹配路径字符串（会误杀当前启动脚本自身）。
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/workspace_env.sh"

PID_DIR="${CSSC_SENSOR_PID_DIR:-/tmp/cssc_loc_sensors}"
mkdir -p "$PID_DIR"
LOG_DIR="${CSSC_SENSOR_LOG_DIR:-$ROOT_DIR/artifacts/bringup_logs}"
mkdir -p "$LOG_DIR"

CAN_IFACE="${CAN_IFACE:-can0}"
BRINGUP_CAN="${CSSC_BRINGUP_CAN:-1}"
BRINGUP_HUNTER="${CSSC_BRINGUP_HUNTER:-1}"
BRINGUP_LIVOX="${CSSC_BRINGUP_LIVOX:-1}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

set +u
source /opt/ros/humble/setup.bash
[[ -f "$HOME/hunter_ros2/install/setup.bash" ]] && source "$HOME/hunter_ros2/install/setup.bash"
[[ -f "$BUILD_ROOT/install/setup.bash" ]] && source "$BUILD_ROOT/install/setup.bash"
[[ -f "$HOME/ws_livox/install/setup.bash" ]] && source "$HOME/ws_livox/install/setup.bash"
set -u

_write_pid() {
  echo "$2" >"$PID_DIR/$1.pid"
  echo "$1 pid=$2" >>"$PID_DIR/started.txt"
}

# 用 /proc/*/exe 匹配完整可执行文件名（pgrep -x 只会看 15 字符 comm，长名字会漏检）
exe_running() {
  # $1 = 进程名，如 hunter_base_node / livox_ros_driver2_node
  local name="$1" p base
  for p in /proc/[0-9]*; do
    base="$(basename "$(readlink -f "$p/exe" 2>/dev/null)" 2>/dev/null || true)"
    if [[ "$base" == "$name" ]]; then
      return 0
    fi
  done
  return 1
}

# 等话题上真正有消息（比 ros2 topic info 更可靠）
wait_msgs() {
  # wait_msgs <kind> <topic> <label> <max_sec>
  # kind: odom|imu|livox
  local kind="$1" topic="$2" label="$3" max_sec="${4:-15}"
  python3 - "$kind" "$topic" "$max_sec" <<'PY'
import sys, time
kind, topic, max_sec = sys.argv[1], sys.argv[2], float(sys.argv[3])
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

rclpy.init()
node = Node('cssc_bringup_wait')
count = [0]
qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)
qos_rel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)

def cb(_msg):
    count[0] += 1

if kind == 'odom':
    from nav_msgs.msg import Odometry
    node.create_subscription(Odometry, topic, cb, qos)
    node.create_subscription(Odometry, topic, cb, qos_rel)
elif kind == 'imu':
    from sensor_msgs.msg import Imu
    node.create_subscription(Imu, topic, cb, qos)
    node.create_subscription(Imu, topic, cb, qos_rel)
elif kind == 'livox':
    from livox_ros_driver2.msg import CustomMsg
    node.create_subscription(CustomMsg, topic, cb, qos_rel)
    node.create_subscription(CustomMsg, topic, cb, qos)
else:
    raise SystemExit('bad kind')

t0 = time.time()
last_print = -1
ok = False
while time.time() - t0 < max_sec:
    rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = int(time.time() - t0)
    if elapsed != last_print:
        last_print = elapsed
        print(f'[bringup] 等待 {topic} 消息 ... {elapsed}s / {int(max_sec)}s (got={count[0]})', flush=True)
    if count[0] > 0:
        ok = True
        break
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
PY
}

bringup_can() {
  [[ "$BRINGUP_CAN" == "1" ]] || return 0
  if ip -br link show "$CAN_IFACE" 2>/dev/null | awk '{print $2}' | grep -qx UP; then
    echo "[bringup] $CAN_IFACE 已 UP"
    return 0
  fi
  local script="$HOME/ws_nav_fastlio/scripts/bringup_can.sh"
  if [[ -f "$script" ]]; then
    echo "[bringup] 拉起 CAN"
    bash "$script" "$CAN_IFACE" >/dev/null
  else
    sudo modprobe gs_usb || true
    sudo ip link set "$CAN_IFACE" down 2>/dev/null || true
    sudo ip link set "$CAN_IFACE" up type can bitrate 500000
  fi
  echo "[bringup] $CAN_IFACE 完成"
}

bringup_hunter() {
  [[ "$BRINGUP_HUNTER" == "1" ]] || return 0
  if exe_running hunter_base_node; then
    local existing
    existing="$(python3 - <<'PY'
import os
for pid in os.listdir("/proc"):
    if not pid.isdigit():
        continue
    try:
        if os.path.basename(os.path.realpath(f"/proc/{pid}/exe")) == "hunter_base_node":
            print(pid); break
    except Exception:
        pass
PY
)"
    echo "[bringup] hunter_base_node 已在运行，跳过 (pid ${existing:-?})"
    [[ -n "$existing" ]] && _write_pid hunter_base "$existing"
    return 0
  fi
  local bin="$HOME/hunter_ros2/install/hunter_base/lib/hunter_base/hunter_base_node"
  if [[ ! -x "$bin" ]]; then
    echo "[bringup] 错误: 找不到 hunter_base_node" >&2
    return 1
  fi
  local params="$ROOT_DIR/config/hunter_base.yaml"
  echo "[bringup] 启动 hunter_base -> /hunter_odom"
  # 使用 nohup；avoid setsid（部分环境会触发 hunter_base 异常退出）
  nohup "$bin" --ros-args --params-file "$params" \
    >>"$LOG_DIR/hunter_base.log" 2>&1 &
  local pid=$!
  disown "$pid" 2>/dev/null || true
  _write_pid hunter_base "$pid"
  sleep 0.8
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[bringup] 错误: hunter_base 启动即退出，见 $LOG_DIR/hunter_base.log" >&2
    tail -20 "$LOG_DIR/hunter_base.log" >&2 || true
    return 1
  fi
  if wait_msgs odom /hunter_odom hunter_base 12; then
    echo "[bringup] hunter_base OK (pid $pid)"
    return 0
  fi
  if kill -0 "$pid" 2>/dev/null; then
    echo "[bringup] 警告: 进程在跑但 12s 内未收到 /hunter_odom，继续" >&2
    return 0
  fi
  echo "[bringup] 错误: hunter_base 已退出" >&2
  return 1
}

bringup_livox() {
  [[ "$BRINGUP_LIVOX" == "1" ]] || return 0
  if exe_running livox_ros_driver2_node; then
    local existing
    existing="$(python3 - <<'PY'
import os
for pid in os.listdir("/proc"):
    if not pid.isdigit():
        continue
    try:
        if os.path.basename(os.path.realpath(f"/proc/{pid}/exe")) == "livox_ros_driver2_node":
            print(pid); break
    except Exception:
        pass
PY
)"
    echo "[bringup] livox_ros_driver2_node 已在运行，跳过 (pid ${existing:-?})"
    [[ -n "$existing" ]] && _write_pid livox "$existing"
    return 0
  fi
  local cfg="$HOME/ws_livox/src/livox_ros_driver2/config/MID360_config.json"
  local bin="$HOME/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"
  local sdk_lib="$HOME/ws_nav_fastlio/third_party/livox_sdk2_install/lib"
  local livox_lib="$HOME/ws_livox/install/livox_ros_driver2/lib"
  if [[ ! -f "$cfg" || ! -x "$bin" ]]; then
    echo "[bringup] 错误: 缺少 Livox 配置或二进制 ($cfg / $bin)" >&2
    return 1
  fi
  echo "[bringup] 启动 Livox -> /livox/lidar /livox/imu"
  nohup env \
    ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY" \
    LD_LIBRARY_PATH="${sdk_lib}:${livox_lib}:${LD_LIBRARY_PATH:-}" \
    "$bin" --ros-args \
      -r __node:=livox_lidar_publisher \
      -p xfer_format:=1 \
      -p multi_topic:=0 \
      -p data_src:=0 \
      -p publish_freq:=10.0 \
      -p output_data_type:=0 \
      -p frame_id:=livox_frame \
      -p "user_config_path:=$cfg" \
      -p cmdline_input_bd_code:=livox0000000001 \
    >>"$LOG_DIR/livox.log" 2>&1 &
  local pid=$!
  disown "$pid" 2>/dev/null || true
  _write_pid livox "$pid"
  sleep 1.0
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[bringup] 错误: Livox 启动即退出，见 $LOG_DIR/livox.log" >&2
    tail -30 "$LOG_DIR/livox.log" >&2 || true
    return 1
  fi
  # 雷达握手通常 3s+，给足时间
  if wait_msgs livox /livox/lidar Livox 20; then
    echo "[bringup] Livox OK (pid $pid)"
    return 0
  fi
  if kill -0 "$pid" 2>/dev/null; then
    echo "[bringup] 警告: Livox 进程在跑但 20s 内未收到点云，继续" >&2
    return 0
  fi
  echo "[bringup] 错误: Livox 已退出" >&2
  return 1
}

stop_brought_up() {
  [[ -d "$PID_DIR" ]] || return 0
  local f name pid
  for f in "$PID_DIR"/*.pid; do
    [[ -f "$f" ]] || continue
    name="$(basename "$f" .pid)"
    pid="$(cat "$f" 2>/dev/null || true)"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "[bringup] 停止 $name (pid $pid)"
      kill "$pid" 2>/dev/null || true
      sleep 0.4
      kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$f"
  done
}

case "${1:-start}" in
  start)
    : >"$PID_DIR/started.txt"
    echo "[bringup] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
    bringup_can
    bringup_hunter
    bringup_livox
    echo "[bringup] 完成"
    ;;
  stop)
    stop_brought_up
    ;;
  *)
    echo "用法: $0 {start|stop}" >&2
    exit 2
    ;;
esac
