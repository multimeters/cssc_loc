#!/usr/bin/env bash
# Linux / WSL single entry point. Settings and extrinsics live in YAML.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/workspace_env.sh"
cd "$ROOT_DIR"
CHECK_ONLY=false
INSTALL_DEPS=false
BUILD=true
RUN_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --install-deps) INSTALL_DEPS=true ;;
    --no-build) BUILD=false ;;
    --check) CHECK_ONLY=true; RUN_ARGS+=("$arg") ;;
    -h|--help)
      cat <<'HELP'
中文说明：默认读取 config/localization.yaml，检查数据，增量编译，启动定位并回放。
  bash start.sh                       一键启动
  bash start.sh --install-deps        首次安装依赖后启动（需要 sudo）
  bash start.sh --check               仅检查配置、数据和运行环境
  bash start.sh --config /path/x.yaml 使用指定配置（相对数据路径以 YAML 所在目录为准）
  bash start.sh --max-bag-seconds 30   仅回放前 30 秒
  bash start.sh --no-build            使用当前目录已经编译的版本
  bash start.sh --output /new/result  指定新的结果目录
外参只从 YAML 读取；不要回放原包的旧定位 TF。
HELP
      exit 0 ;;
    *) RUN_ARGS+=("$arg") ;;
  esac
done
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
# Validate data before spending time compiling. This does not launch any node.
python3 "$ROOT_DIR/scripts/run_fusion_replay.py" "${RUN_ARGS[@]}" --check
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
echo '开始定位回放；按 Ctrl+C 可停止，结果保存到 YAML 中指定的 output_root。'
exec python3 "$ROOT_DIR/scripts/run_fusion_replay.py" "${RUN_ARGS[@]}"
