#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"
export DUAL_3D_ENABLE_NAVIGATION=false
export DUAL_3D_STACK_MODE=mapping
export USE_RVIZ="${USE_RVIZ:-true}"
# Mapping uses a conservative refinement of the finalized V9 profile. An
# explicit environment override provides an immediate rollback/A-B path.
export DUAL_3D_CARTOGRAPHER_CONFIG="${DUAL_3D_CARTOGRAPHER_CONFIG:-cartographer_2d_v9_mapping_balanced.lua}"
exec bash "$ROOT_DIR/visual_laser_slam/run_dual_resolution_3d_slam.sh"
