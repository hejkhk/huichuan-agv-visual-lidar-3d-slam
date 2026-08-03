#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$ROOT_DIR/visual_laser_slam/run_isolated_perception.sh" visual_odom_clean
