#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CACHE_WS="${DUAL_3D_BUILD_ROOT:-$HOME/.cache/huichuan_agv_dual_3d_humble_ws}"
ENV_FILE="$ROOT_DIR/visual_laser_slam/dual_resolution_3d.env"

if [ ! -f "$ENV_FILE" ]; then
  echo "[ERROR] Calibration settings not found: $ENV_FILE" >&2
  exit 1
fi
# Use the same ROS domain and middleware as the running mapping stack.
# shellcheck disable=SC1090
source "$ENV_FILE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ROS setup files legitimately probe unset AMENT variables. Temporarily
# disable nounset while sourcing, then restore strict mode for this script.
set +u
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] ROS 2 Humble setup not found: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [ ! -f "$CACHE_WS/install/setup.bash" ]; then
  echo "[ERROR] Built workspace not found: $CACHE_WS/install/setup.bash" >&2
  echo "        Run ./START_DUAL_2D_3D_MAPPING.sh once first." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$CACHE_WS/install/setup.bash"
set -u

echo "Keep the AGV stationary on a flat floor. Calibration takes about 2 seconds."
echo "A passing result is backed up and written to: $ROOT_DIR/visual_laser_slam/dual_resolution_3d.env"
echo "Run this in a SECOND terminal. Do not press Ctrl+C in the mapping terminal."
echo "ROS graph: domain=$ROS_DOMAIN_ID middleware=$RMW_IMPLEMENTATION"
echo "The calibrator now validates actual depth frames instead of relying on the"
echo "ROS CLI topic-list cache. It will print a precise timeout diagnosis."
command -v setsid >/dev/null 2>&1 || {
  echo "[ERROR] Required command is missing: setsid" >&2
  exit 1
}
setsid --wait ros2 run lidar_py camera_ground_calibrator --ros-args \
  -p auto_write:=true \
  -p "env_file:=$ENV_FILE"
