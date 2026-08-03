#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CACHE_WS="${DUAL_3D_BUILD_ROOT:-$HOME/.cache/huichuan_agv_dual_3d_humble_ws}"
ENV_FILE="$ROOT_DIR/visual_laser_slam/dual_resolution_3d.env"
RESTART_MARKER="$ROOT_DIR/visual_laser_slam/.camera_calibration_restart_required"

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
if [ ! -f "$ENV_FILE" ]; then
  echo "[ERROR] Calibration settings not found: $ENV_FILE" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$CACHE_WS/install/setup.bash"
source "$ENV_FILE"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [ "${CAMERA_GROUND_CALIBRATED:-false}" != "true" ]; then
  echo "[ERROR] Ground calibration has not been completed for this parameter set." >&2
  echo "        Run CALIBRATE_CAMERA_EXTRINSIC.sh, restart mapping, then retry YAW." >&2
  exit 1
fi

if [ -f "$RESTART_MARKER" ]; then
  echo "[ERROR] Camera parameters changed, but the running static TF is still old." >&2
  echo "        Stop mapping, restart START_DUAL_2D_3D_MAPPING.sh, then run YAW." >&2
  echo "        Pending stage: $(head -n 1 "$RESTART_MARKER" 2>/dev/null || echo unknown)" >&2
  exit 1
fi

cloud_ready=false
for _ in $(seq 1 20); do
  if ros2 topic list --no-daemon 2>/dev/null | grep -Fxq /local_highres_cloud_v21; then
    cloud_ready=true
    break
  fi
  sleep 0.5
done
if [ "$cloud_ready" != true ]; then
  echo "[ERROR] Missing /local_highres_cloud_v21." >&2
  echo "        Keep ./START_DUAL_2D_3D_MAPPING.sh running in another terminal." >&2
  echo "        Calibration is using ROS_DOMAIN_ID=$ROS_DOMAIN_ID and $RMW_IMPLEMENTATION." >&2
  exit 1
fi

echo "Park 1-3 m from one large flat wall seen by both sensors."
echo "Keep the vehicle still; the wall does not need to be square to the AGV."
echo "A passing result is backed up and written to: $ENV_FILE"
echo "Run this in a SECOND terminal. Do not press Ctrl+C in the mapping terminal."
command -v setsid >/dev/null 2>&1 || {
  echo "[ERROR] Required command is missing: setsid" >&2
  exit 1
}
setsid --wait ros2 run lidar_py camera_lidar_yaw_calibrator --ros-args \
  -p "current_camera_yaw_deg:=${CAMERA_YAW_DEG:-0.0}" \
  -p auto_write:=true \
  -p "env_file:=$ENV_FILE"
