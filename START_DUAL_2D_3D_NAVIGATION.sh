#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"
export DUAL_3D_ENABLE_NAVIGATION=true
export DUAL_3D_STACK_MODE=navigation
export USE_RVIZ="${USE_RVIZ:-true}"
# Navigation uses the same proven online loop closure as mapping. The profile
# already constrains corridor matches to 1.5 m / 3 degrees; the runtime
# correction guard pauses motion while Nav2 replaces a pre-correction path.
export DUAL_3D_CARTOGRAPHER_CONFIG=cartographer_2d_v9_mapping_balanced.lua
# A new Cartographer map cannot share coordinates with an old RTAB-Map graph.
# Keep navigation runs isolated so historical 3D cubes cannot overlap a new 2D map.
export DUAL_3D_DATABASE="maps/rtabmap_3d/rtabmap_nav_live.db"
export DUAL_3D_RESET_GLOBAL_MAP=true
# This profile uses the live 3D point-cloud watchdog instead of the legacy
# image-baseline process, which is not launched alongside the ROS camera driver.
export DUAL_3D_REQUIRE_DEPTH_BASELINE_FOR_PS2=false
# The shared runner selects 1 Hz / one OMP thread on Jetson and 2 Hz / two
# threads on an x86 PC. Do not override that platform-aware default here.
# The 15 Hz local collision cloud is independent from this RTAB-Map rate.
exec bash "$ROOT_DIR/visual_laser_slam/run_dual_resolution_3d_slam.sh"
