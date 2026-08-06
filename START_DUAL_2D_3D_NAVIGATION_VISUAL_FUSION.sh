#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"
export DUAL_3D_ENABLE_NAVIGATION=true
export DUAL_3D_ENABLE_VISUAL_FUSION=true
export DUAL_3D_STACK_MODE=navigation_visual_fusion
export USE_RVIZ="${USE_RVIZ:-true}"
# Use exactly the same guarded Cartographer/Nav2 profile as the stable
# navigation launcher. The only A/B variable is the odom prediction source.
export DUAL_3D_CARTOGRAPHER_CONFIG=cartographer_2d_v9_nav_guarded.lua
export DUAL_3D_DATABASE="maps/rtabmap_3d/rtabmap_nav_visual_fusion.db"
# This dedicated database is intentionally persistent across runs so RTAB-Map
# can recognize previously seen RGB-D places and optimize its internal 3D graph.
export DUAL_3D_RESET_GLOBAL_MAP=false
export DUAL_3D_REQUIRE_DEPTH_BASELINE_FOR_PS2=false
export DUAL_3D_RTABMAP_RATE=1.0
exec bash "$ROOT_DIR/visual_laser_slam/run_dual_resolution_3d_slam.sh"
