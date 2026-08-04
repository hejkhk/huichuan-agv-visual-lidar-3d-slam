#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$ROOT_DIR/visual_laser_slam/run_visual_slam_step.sh" filtered_cloud
