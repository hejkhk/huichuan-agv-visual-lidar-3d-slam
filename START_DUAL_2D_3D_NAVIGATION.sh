#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"
export DUAL_3D_ENABLE_NAVIGATION=true
export USE_RVIZ=true
# Navigation-specific pose-graph guard. Mapping-only startup keeps the
# finalized V9 Cartographer profile unchanged.
export DUAL_3D_CARTOGRAPHER_CONFIG=cartographer_2d_v9_nav_guarded.lua
# A new Cartographer map cannot share coordinates with an old RTAB-Map graph.
# Keep navigation runs isolated so historical 3D cubes cannot overlap a new 2D map.
export DUAL_3D_DATABASE="maps/rtabmap_3d/rtabmap_nav_live.db"
export DUAL_3D_RESET_GLOBAL_MAP=true
# This profile uses the live 3D point-cloud watchdog instead of the legacy
# image-baseline process, which is not launched alongside the ROS camera driver.
export DUAL_3D_REQUIRE_DEPTH_BASELINE_FOR_PS2=false
# The RTX 3060 host can sustain two optimized 3D keyframes per second. This
# keeps the persistent OctoMap visibly current without touching the 15 Hz
# local collision cloud used by Regulated Pure Pursuit and the C++ safety gate.
export DUAL_3D_RTABMAP_RATE=2.0
exec bash "$ROOT_DIR/visual_laser_slam/run_dual_resolution_3d_slam.sh"
