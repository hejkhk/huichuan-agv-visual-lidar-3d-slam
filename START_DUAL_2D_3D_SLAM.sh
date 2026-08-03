#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="$ROOT_DIR/visual_laser_slam/run_dual_resolution_3d_slam.sh"

if [ ! -f "$RUNNER" ]; then
  echo "[ERROR] Missing launcher: $RUNNER" >&2
  exit 1
fi

# Keep execution independent of the terminal's current directory.
cd "$ROOT_DIR"
export USE_RVIZ=true
export DUAL_3D_ENABLE_NAVIGATION=false
export DUAL_3D_CARTOGRAPHER_CONFIG=cartographer_2d_v9_tightened.lua
exec bash "$RUNNER"
