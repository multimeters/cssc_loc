#!/usr/bin/env bash
# One cache per checkout: moving or cloning the repository cannot reuse a
# CMake cache that points at a different source directory.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CSSC_WORKSPACE_ID="$(printf '%s' "$ROOT_DIR" | sha256sum | cut -c1-12)"
BUILD_ROOT="${APS_BUILD_ROOT:-$HOME/.cache/cssc_loc/$CSSC_WORKSPACE_ID}"
export APS_BUILD_ROOT="$BUILD_ROOT"
