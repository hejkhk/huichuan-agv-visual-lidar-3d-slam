#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
ENV_FILE="$ROOT_DIR/visual_laser_slam/visual_laser_slam.env"
[ -f "$ENV_FILE" ] && source "$ENV_FILE"
if [ -n "${ORBBEC_SETUP:-}" ] && [ -f "$ORBBEC_SETUP" ]; then source "$ORBBEC_SETUP"; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
OUT_DIR="$ROOT_DIR/maps_3d"
mkdir -p "$OUT_DIR"
OUT_PATH="${1:-$OUT_DIR/octomap_$(date +%Y%m%d_%H%M%S).bt}"
case "$OUT_PATH" in *.bt|*.ot) ;; *) OUT_PATH="${OUT_PATH}.bt" ;; esac
if ! ros2 topic list 2>/dev/null | grep -Fxq /octomap_binary; then
  echo "[错误] 未发现 /octomap_binary。请先运行 STEP9_2D_3D_DUAL_MAP_TEST.sh。" >&2
  exit 1
fi
echo "[保存] $OUT_PATH"
ros2 run octomap_server octomap_saver_node --ros-args -p "octomap_path:=$OUT_PATH"
echo "[完成] 三维地图已保存：$OUT_PATH"
