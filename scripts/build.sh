#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/workspace_env.sh"
PROFILE="${1:-all}"
source /opt/ros/humble/setup.bash
mkdir -p "$BUILD_ROOT"
selection=()
if [[ "$PROFILE" != all ]]; then
  mapfile -t packages < <(python3 -c 'import json,sys; print("\n".join(json.load(open(sys.argv[1]))["profiles"][sys.argv[2]]))' "$ROOT_DIR/sources.lock.json" "$PROFILE")
  if [[ "${#packages[@]}" -eq 0 ]]; then
    echo "Unknown or empty profile: $PROFILE" >&2
    exit 2
  fi
  case "$PROFILE" in
    ndt) packages+=(aps_ndt_localization) ;;
    dead_reckoning) packages+=(aps_bag_localization) ;;
  esac
  # colcon also orders source test-dependencies even with BUILD_TESTING=OFF.
  # Let it include those if present instead of requiring an unbuilt overlay.
  selection=(--packages-up-to "${packages[@]}")
fi
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
export MAKEFLAGS="${MAKEFLAGS:--j4}"
colcon --log-base "$BUILD_ROOT/log" build \
  --base-paths "$ROOT_DIR/src" \
  --build-base "$BUILD_ROOT/build" --install-base "$BUILD_ROOT/install" \
  --parallel-workers "${APS_BUILD_WORKERS:-4}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  "${selection[@]}"
printf 'Source this overlay: %s/install/setup.bash\n' "$BUILD_ROOT"
