#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"
export DUAL_3D_ENABLE_NAVIGATION=false
export DUAL_3D_STACK_MODE=mapping
export USE_RVIZ="${USE_RVIZ:-true}"
# Mapping-only keeps the finalized V9 profile. This explicit assignment also
# prevents an inherited shell variable from selecting the navigation guard.
export DUAL_3D_CARTOGRAPHER_CONFIG=cartographer_2d_v9_tightened.lua
exec bash "$ROOT_DIR/visual_laser_slam/run_dual_resolution_3d_slam.sh"
