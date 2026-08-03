#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export USE_RVIZ=false
exec "$ROOT_DIR/visual_laser_slam/run_visual_slam_step.sh" visual_odom_baseline
