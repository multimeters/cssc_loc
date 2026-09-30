#!/usr/bin/env bash
# Backward-compatible entry. All experiment settings now come from YAML.
set -eo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT_DIR/start.sh" "$@"
