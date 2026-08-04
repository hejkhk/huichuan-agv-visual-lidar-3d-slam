#!/usr/bin/env bash
set -Eeo pipefail

PROFILE="${1:-}"
case "$PROFILE" in
  camera|visual_odom|wheel_imu|fusion|slam|slam_clean|pointcloud|filtered_cloud|octomap_odom|dual_map|local_highres|local_highres_v2|local_highres_v21|rgbd_sync_test|visual_odom_sync|visual_odom_sync_lite|visual_odom_baseline) ;;
  *)
    echo "用法: $0 {camera|visual_odom|wheel_imu|fusion|slam|slam_clean|pointcloud|filtered_cloud|octomap_odom|dual_map|local_highres|local_highres_v2|local_highres_v21|rgbd_sync_test|visual_odom_sync}" >&2
    exit 2
    ;;
esac

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$ROOT_DIR/visual_laser_slam/visual_laser_slam.env"
LIDAR_WS="$ROOT_DIR/lidar/chapt1_ws"
CACHE_WS="${VISUAL_SLAM_BUILD_ROOT:-$HOME/.cache/huichuan_agv_visual_slam_ws}"
CACHE_SRC="$CACHE_WS/src"
BUILD_BASE="$CACHE_WS/build"
INSTALL_BASE="$CACHE_WS/install"
LOG_BASE="$CACHE_WS/log"
RUN_STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_DIR="$ROOT_DIR/SLAM_Log/visual_laser_${PROFILE}_${RUN_STAMP}"
RUNTIME_LOG="$RUN_DIR/runtime.log"
LAUNCH_PID=""
TAIL_PID=""

log() { printf '%s\n' "$*"; }
die() { log "[错误] $*"; [ -f "$RUNTIME_LOG" ] && tail -n 120 "$RUNTIME_LOG" || true; exit 1; }
is_true() { case "${1,,}" in 1|true|yes|on) return 0;; *) return 1;; esac; }

cleanup() {
  local code=$?
  trap - EXIT INT TERM HUP
  log ""
  log "[停止] 正在关闭本次视觉/激光/三维建图测试……"
  if [ -n "$TAIL_PID" ]; then kill "$TAIL_PID" 2>/dev/null || true; fi
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    sleep 0.5
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  fi
  wait "$LAUNCH_PID" 2>/dev/null || true
  log "[停止] 已关闭。日志：$RUNTIME_LOG"
  exit "$code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

[ -f "$ENV_FILE" ] || die "缺少配置文件：$ENV_FILE"
# shellcheck disable=SC1090
source "$ENV_FILE"

# STEP11 deliberately reuses the standalone clean visual-odometry parameters.
CLEAN_ENV_FILE="$ROOT_DIR/visual_laser_slam/visual_odom_clean.env"
if [ "$PROFILE" = "slam_clean" ]; then
  [ -f "$CLEAN_ENV_FILE" ] || die "缺少 STEP11 clean 配置：$CLEAN_ENV_FILE"
  # shellcheck disable=SC1090
  source "$CLEAN_ENV_FILE"
fi

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
USE_RVIZ="${USE_RVIZ:-true}"
COLOR_WIDTH="${COLOR_WIDTH:-640}"
COLOR_HEIGHT="${COLOR_HEIGHT:-480}"
COLOR_FPS="${COLOR_FPS:-15}"
DEPTH_WIDTH="${DEPTH_WIDTH:-640}"
DEPTH_HEIGHT="${DEPTH_HEIGHT:-400}"
DEPTH_FPS="${DEPTH_FPS:-10}"
CAMERA_X="${CAMERA_X:-0.0}"
CAMERA_Y="${CAMERA_Y:-0.0}"
CAMERA_Z="${CAMERA_Z:-0.0}"
CAMERA_ROLL="${CAMERA_ROLL:-0.0}"
CAMERA_PITCH="${CAMERA_PITCH:-0.0}"
CAMERA_YAW="${CAMERA_YAW:-0.0}"
CAMERA_TF_CONFIRMED="${CAMERA_TF_CONFIRMED:-false}"
LIDAR_BAUD="${LIDAR_BAUD:-115200}"
LASER_YAW_DEG="${LASER_YAW_DEG:-0.0}"
SCAN_ANGLE_SIGN="${SCAN_ANGLE_SIGN:--1.0}"
AUTO_BUILD="${AUTO_BUILD:-true}"

VO_COLOR_WIDTH="${VO_COLOR_WIDTH:-640}"
VO_COLOR_HEIGHT="${VO_COLOR_HEIGHT:-480}"
VO_COLOR_FPS="${VO_COLOR_FPS:-15}"
VO_DEPTH_WIDTH="${VO_DEPTH_WIDTH:-640}"
VO_DEPTH_HEIGHT="${VO_DEPTH_HEIGHT:-400}"
VO_DEPTH_FPS="${VO_DEPTH_FPS:-15}"
VO_APPROX_SYNC_MAX_INTERVAL="${VO_APPROX_SYNC_MAX_INTERVAL:-0.020}"
VO_TOPIC_QUEUE_SIZE="${VO_TOPIC_QUEUE_SIZE:-1}"
VO_SYNC_QUEUE_SIZE="${VO_SYNC_QUEUE_SIZE:-3}"
VO_WAIT_FOR_TRANSFORM="${VO_WAIT_FOR_TRANSFORM:-0.10}"
VO_ODOM_GUESS_MOTION="${VO_ODOM_GUESS_MOTION:-false}"
VO_ODOM_IMAGE_DECIMATION="${VO_ODOM_IMAGE_DECIMATION:-1}"

POINT_CLOUD_TOPIC="${POINT_CLOUD_TOPIC:-/camera/depth/points}"
FILTERED_CLOUD_TOPIC="${FILTERED_CLOUD_TOPIC:-/camera/depth/points_filtered}"
CLOUD_MAX_RATE_HZ="${CLOUD_MAX_RATE_HZ:-5.0}"
CLOUD_SAMPLE_STRIDE="${CLOUD_SAMPLE_STRIDE:-4}"
CLOUD_VOXEL_SIZE="${CLOUD_VOXEL_SIZE:-0.06}"
CLOUD_MIN_RANGE="${CLOUD_MIN_RANGE:-0.30}"
CLOUD_MAX_RANGE="${CLOUD_MAX_RANGE:-4.0}"
CLOUD_TRANSFORM_TIMEOUT="${CLOUD_TRANSFORM_TIMEOUT:-0.20}"
CLOUD_BASE_X_MIN="${CLOUD_BASE_X_MIN:--0.50}"
CLOUD_BASE_X_MAX="${CLOUD_BASE_X_MAX:-4.50}"
CLOUD_BASE_Y_MIN="${CLOUD_BASE_Y_MIN:--3.00}"
CLOUD_BASE_Y_MAX="${CLOUD_BASE_Y_MAX:-3.00}"
CLOUD_BASE_Z_MIN="${CLOUD_BASE_Z_MIN:--1.00}"
CLOUD_BASE_Z_MAX="${CLOUD_BASE_Z_MAX:-2.50}"
CLOUD_REMOVE_SELF="${CLOUD_REMOVE_SELF:-false}"
CLOUD_SELF_X_MIN="${CLOUD_SELF_X_MIN:--0.40}"
CLOUD_SELF_X_MAX="${CLOUD_SELF_X_MAX:-0.40}"
CLOUD_SELF_Y_MIN="${CLOUD_SELF_Y_MIN:--0.40}"
CLOUD_SELF_Y_MAX="${CLOUD_SELF_Y_MAX:-0.40}"
CLOUD_SELF_Z_MIN="${CLOUD_SELF_Z_MIN:--0.20}"
CLOUD_SELF_Z_MAX="${CLOUD_SELF_Z_MAX:-1.20}"
CLOUD_LOG_EVERY_N="${CLOUD_LOG_EVERY_N:-30}"

LOCAL_HIGHRES_CLOUD_TOPIC="${LOCAL_HIGHRES_CLOUD_TOPIC:-/local_highres_cloud}"
LOCAL_HIGHRES_STATS_TOPIC="${LOCAL_HIGHRES_STATS_TOPIC:-/local_highres_cloud/stats}"
LOCAL_HIGHRES_MARKER_TOPIC="${LOCAL_HIGHRES_MARKER_TOPIC:-/local_highres_cloud/crop_markers}"
LOCAL_MAX_RATE_HZ="${LOCAL_MAX_RATE_HZ:-12.0}"
LOCAL_SAMPLE_STRIDE="${LOCAL_SAMPLE_STRIDE:-1}"
LOCAL_VOXEL_SIZE="${LOCAL_VOXEL_SIZE:-0.025}"
LOCAL_MIN_RANGE="${LOCAL_MIN_RANGE:-0.20}"
LOCAL_MAX_RANGE="${LOCAL_MAX_RANGE:-4.0}"
LOCAL_TRANSFORM_TIMEOUT="${LOCAL_TRANSFORM_TIMEOUT:-0.03}"
LOCAL_X_MIN="${LOCAL_X_MIN:-0.20}"
LOCAL_X_MAX="${LOCAL_X_MAX:-4.00}"
LOCAL_Y_MIN="${LOCAL_Y_MIN:--2.50}"
LOCAL_Y_MAX="${LOCAL_Y_MAX:-2.50}"
LOCAL_Z_MIN="${LOCAL_Z_MIN:--0.50}"
LOCAL_Z_MAX="${LOCAL_Z_MAX:-2.00}"
LOCAL_REMOVE_SELF="${LOCAL_REMOVE_SELF:-true}"
LOCAL_SELF_X_MIN="${LOCAL_SELF_X_MIN:--0.36}"
LOCAL_SELF_X_MAX="${LOCAL_SELF_X_MAX:-0.36}"
LOCAL_SELF_Y_MIN="${LOCAL_SELF_Y_MIN:--0.36}"
LOCAL_SELF_Y_MAX="${LOCAL_SELF_Y_MAX:-0.36}"
LOCAL_SELF_Z_MIN="${LOCAL_SELF_Z_MIN:--0.10}"
LOCAL_SELF_Z_MAX="${LOCAL_SELF_Z_MAX:-0.90}"
LOCAL_GROUND_FILTER_ENABLED="${LOCAL_GROUND_FILTER_ENABLED:-false}"
LOCAL_GROUND_Z_MIN="${LOCAL_GROUND_Z_MIN:--0.06}"
LOCAL_GROUND_Z_MAX="${LOCAL_GROUND_Z_MAX:-0.08}"
LOCAL_STATS_PERIOD_SEC="${LOCAL_STATS_PERIOD_SEC:-1.0}"
LOCAL_PUBLISH_MARKERS="${LOCAL_PUBLISH_MARKERS:-true}"

# STEP10V2：直接读取深度图，由C++投影局部点云；不再生成/传输完整原始PointCloud2。
STEP10V2_ENABLE_COLOR="${STEP10V2_ENABLE_COLOR:-false}"
STEP10V2_COLOR_WIDTH="${STEP10V2_COLOR_WIDTH:-1280}"
STEP10V2_COLOR_HEIGHT="${STEP10V2_COLOR_HEIGHT:-720}"
STEP10V2_COLOR_FPS="${STEP10V2_COLOR_FPS:-30}"
STEP10V2_DEPTH_WIDTH="${STEP10V2_DEPTH_WIDTH:-1280}"
STEP10V2_DEPTH_HEIGHT="${STEP10V2_DEPTH_HEIGHT:-800}"
STEP10V2_DEPTH_FPS="${STEP10V2_DEPTH_FPS:-30}"
STEP10V2_DEPTH_REGISTRATION="${STEP10V2_DEPTH_REGISTRATION:-false}"
STEP10V2_ENABLE_FRAME_SYNC="${STEP10V2_ENABLE_FRAME_SYNC:-false}"
STEP10V2_ENABLE_NOISE_REMOVAL_FILTER="${STEP10V2_ENABLE_NOISE_REMOVAL_FILTER:-false}"
STEP10V2_DEPTH_TOPIC="${STEP10V2_DEPTH_TOPIC:-/camera/depth/image_raw}"
STEP10V2_CAMERA_INFO_TOPIC="${STEP10V2_CAMERA_INFO_TOPIC:-/camera/depth/camera_info}"
STEP10V2_CLOUD_TOPIC="${STEP10V2_CLOUD_TOPIC:-/local_highres_cloud_v2}"
STEP10V2_STATS_TOPIC="${STEP10V2_STATS_TOPIC:-/local_highres_cloud_v2/stats}"
STEP10V2_MARKER_TOPIC="${STEP10V2_MARKER_TOPIC:-/local_highres_cloud_v2/crop_markers}"
STEP10V2_MAX_RATE_HZ="${STEP10V2_MAX_RATE_HZ:-30.0}"
STEP10V2_PIXEL_STRIDE="${STEP10V2_PIXEL_STRIDE:-2}"
STEP10V2_DEPTH_UNIT_SCALE="${STEP10V2_DEPTH_UNIT_SCALE:-0.001}"
STEP10V2_VOXEL_SIZE="${STEP10V2_VOXEL_SIZE:-0.03}"
STEP10V2_MIN_RANGE="${STEP10V2_MIN_RANGE:-0.20}"
STEP10V2_MAX_RANGE="${STEP10V2_MAX_RANGE:-4.0}"
STEP10V2_TRANSFORM_TIMEOUT="${STEP10V2_TRANSFORM_TIMEOUT:-0.015}"
STEP10V2_ROI_U_MIN="${STEP10V2_ROI_U_MIN:-0}"
STEP10V2_ROI_U_MAX="${STEP10V2_ROI_U_MAX:--1}"
STEP10V2_ROI_V_MIN="${STEP10V2_ROI_V_MIN:-0}"
STEP10V2_ROI_V_MAX="${STEP10V2_ROI_V_MAX:--1}"
STEP10V2_X_MIN="${STEP10V2_X_MIN:-0.15}"
STEP10V2_X_MAX="${STEP10V2_X_MAX:-4.00}"
STEP10V2_Y_MIN="${STEP10V2_Y_MIN:--2.50}"
STEP10V2_Y_MAX="${STEP10V2_Y_MAX:-2.50}"
STEP10V2_Z_MIN="${STEP10V2_Z_MIN:--0.50}"
STEP10V2_Z_MAX="${STEP10V2_Z_MAX:-2.00}"
STEP10V2_REMOVE_SELF="${STEP10V2_REMOVE_SELF:-true}"
STEP10V2_SELF_X_MIN="${STEP10V2_SELF_X_MIN:--0.36}"
STEP10V2_SELF_X_MAX="${STEP10V2_SELF_X_MAX:-0.36}"
STEP10V2_SELF_Y_MIN="${STEP10V2_SELF_Y_MIN:--0.36}"
STEP10V2_SELF_Y_MAX="${STEP10V2_SELF_Y_MAX:-0.36}"
STEP10V2_SELF_Z_MIN="${STEP10V2_SELF_Z_MIN:--0.10}"
STEP10V2_SELF_Z_MAX="${STEP10V2_SELF_Z_MAX:-0.90}"
STEP10V2_GROUND_FILTER_ENABLED="${STEP10V2_GROUND_FILTER_ENABLED:-false}"
STEP10V2_GROUND_Z_MIN="${STEP10V2_GROUND_Z_MIN:--0.06}"
STEP10V2_GROUND_Z_MAX="${STEP10V2_GROUND_Z_MAX:-0.08}"
STEP10V2_STATS_PERIOD_SEC="${STEP10V2_STATS_PERIOD_SEC:-1.0}"
STEP10V2_PUBLISH_MARKERS="${STEP10V2_PUBLISH_MARKERS:-true}"

# STEP10V2.1：最新帧邮箱 + 独立线程 + 静态TF缓存 + 预分配体素表。
STEP10V21_ENABLE_COLOR="${STEP10V21_ENABLE_COLOR:-false}"
STEP10V21_COLOR_WIDTH="${STEP10V21_COLOR_WIDTH:-1280}"
STEP10V21_COLOR_HEIGHT="${STEP10V21_COLOR_HEIGHT:-720}"
STEP10V21_COLOR_FPS="${STEP10V21_COLOR_FPS:-30}"
STEP10V21_DEPTH_WIDTH="${STEP10V21_DEPTH_WIDTH:-1280}"
STEP10V21_DEPTH_HEIGHT="${STEP10V21_DEPTH_HEIGHT:-800}"
STEP10V21_DEPTH_FPS="${STEP10V21_DEPTH_FPS:-30}"
STEP10V21_DEPTH_REGISTRATION="${STEP10V21_DEPTH_REGISTRATION:-false}"
STEP10V21_ENABLE_FRAME_SYNC="${STEP10V21_ENABLE_FRAME_SYNC:-false}"
STEP10V21_ENABLE_NOISE_REMOVAL_FILTER="${STEP10V21_ENABLE_NOISE_REMOVAL_FILTER:-false}"
STEP10V21_ENABLE_DEPTH_AUTO_EXPOSURE_PRIORITY="${STEP10V21_ENABLE_DEPTH_AUTO_EXPOSURE_PRIORITY:-false}"
STEP10V21_ENABLE_SYNC_HOST_TIME="${STEP10V21_ENABLE_SYNC_HOST_TIME:-true}"
STEP10V21_TIME_DOMAIN="${STEP10V21_TIME_DOMAIN:-device}"
STEP10V21_TIME_SYNC_PERIOD="${STEP10V21_TIME_SYNC_PERIOD:-60}"
STEP10V21_ENABLE_FRAME_TIMESTAMP_CSV="${STEP10V21_ENABLE_FRAME_TIMESTAMP_CSV:-false}"
STEP10V21_FRAME_TIMESTAMP_CSV_FILE="${STEP10V21_FRAME_TIMESTAMP_CSV_FILE:-/tmp/orbbec_step10v21_timestamp.csv}"
STEP10V21_DEPTH_TOPIC="${STEP10V21_DEPTH_TOPIC:-/camera/depth/image_raw}"
STEP10V21_CAMERA_INFO_TOPIC="${STEP10V21_CAMERA_INFO_TOPIC:-/camera/depth/camera_info}"
STEP10V21_CLOUD_TOPIC="${STEP10V21_CLOUD_TOPIC:-/local_highres_cloud_v21}"
STEP10V21_STATS_TOPIC="${STEP10V21_STATS_TOPIC:-/local_highres_cloud_v21/stats}"
STEP10V21_MARKER_TOPIC="${STEP10V21_MARKER_TOPIC:-/local_highres_cloud_v21/crop_markers}"
STEP10V21_MAX_RATE_HZ="${STEP10V21_MAX_RATE_HZ:-30.0}"
STEP10V21_PIXEL_STRIDE="${STEP10V21_PIXEL_STRIDE:-2}"
STEP10V21_DEPTH_UNIT_SCALE="${STEP10V21_DEPTH_UNIT_SCALE:-0.001}"
STEP10V21_VOXEL_SIZE="${STEP10V21_VOXEL_SIZE:-0.03}"
STEP10V21_MIN_RANGE="${STEP10V21_MIN_RANGE:-0.20}"
STEP10V21_MAX_RANGE="${STEP10V21_MAX_RANGE:-4.0}"
STEP10V21_TRANSFORM_TIMEOUT="${STEP10V21_TRANSFORM_TIMEOUT:-0.50}"
STEP10V21_MAX_INPUT_AGE_MS="${STEP10V21_MAX_INPUT_AGE_MS:-150.0}"
STEP10V21_ROI_U_MIN="${STEP10V21_ROI_U_MIN:-0}"
STEP10V21_ROI_U_MAX="${STEP10V21_ROI_U_MAX:--1}"
STEP10V21_ROI_V_MIN="${STEP10V21_ROI_V_MIN:-0}"
STEP10V21_ROI_V_MAX="${STEP10V21_ROI_V_MAX:--1}"
STEP10V21_X_MIN="${STEP10V21_X_MIN:-0.15}"
STEP10V21_X_MAX="${STEP10V21_X_MAX:-4.00}"
STEP10V21_Y_MIN="${STEP10V21_Y_MIN:--2.50}"
STEP10V21_Y_MAX="${STEP10V21_Y_MAX:-2.50}"
STEP10V21_Z_MIN="${STEP10V21_Z_MIN:--0.50}"
STEP10V21_Z_MAX="${STEP10V21_Z_MAX:-2.00}"
STEP10V21_REMOVE_SELF="${STEP10V21_REMOVE_SELF:-true}"
STEP10V21_SELF_X_MIN="${STEP10V21_SELF_X_MIN:--0.36}"
STEP10V21_SELF_X_MAX="${STEP10V21_SELF_X_MAX:-0.36}"
STEP10V21_SELF_Y_MIN="${STEP10V21_SELF_Y_MIN:--0.36}"
STEP10V21_SELF_Y_MAX="${STEP10V21_SELF_Y_MAX:-0.36}"
STEP10V21_SELF_Z_MIN="${STEP10V21_SELF_Z_MIN:--0.10}"
STEP10V21_SELF_Z_MAX="${STEP10V21_SELF_Z_MAX:-0.90}"
STEP10V21_GROUND_FILTER_ENABLED="${STEP10V21_GROUND_FILTER_ENABLED:-false}"
STEP10V21_GROUND_Z_MIN="${STEP10V21_GROUND_Z_MIN:--0.06}"
STEP10V21_GROUND_Z_MAX="${STEP10V21_GROUND_Z_MAX:-0.08}"
STEP10V21_STATS_PERIOD_SEC="${STEP10V21_STATS_PERIOD_SEC:-1.0}"
STEP10V21_STATS_WINDOW_SIZE="${STEP10V21_STATS_WINDOW_SIZE:-300}"
STEP10V21_PROCESS_WARN_MS="${STEP10V21_PROCESS_WARN_MS:-50.0}"
STEP10V21_AGE_WARN_MS="${STEP10V21_AGE_WARN_MS:-120.0}"
STEP10V21_STALL_WARN_GAP_MS="${STEP10V21_STALL_WARN_GAP_MS:-120.0}"
STEP10V21_PUBLISH_MARKERS="${STEP10V21_PUBLISH_MARKERS:-true}"

# RGB-D 时间戳同步诊断。与STEP1-9分离，不直接改已验证的视觉里程计配置。
RGBD_SYNC_COLOR_WIDTH="${RGBD_SYNC_COLOR_WIDTH:-640}"
RGBD_SYNC_COLOR_HEIGHT="${RGBD_SYNC_COLOR_HEIGHT:-480}"
RGBD_SYNC_COLOR_FPS="${RGBD_SYNC_COLOR_FPS:-15}"
RGBD_SYNC_DEPTH_WIDTH="${RGBD_SYNC_DEPTH_WIDTH:-640}"
RGBD_SYNC_DEPTH_HEIGHT="${RGBD_SYNC_DEPTH_HEIGHT:-400}"
RGBD_SYNC_DEPTH_FPS="${RGBD_SYNC_DEPTH_FPS:-15}"
RGBD_SYNC_STATS_TOPIC="${RGBD_SYNC_STATS_TOPIC:-/rgbd_timestamp_sync/stats}"
RGBD_SYNC_MAX_PAIR_INTERVAL_MS="${RGBD_SYNC_MAX_PAIR_INTERVAL_MS:-40.0}"
RGBD_SYNC_WARN_P95_MS="${RGBD_SYNC_WARN_P95_MS:-25.0}"
RGBD_SYNC_WINDOW_SIZE="${RGBD_SYNC_WINDOW_SIZE:-300}"
RGBD_SYNC_ENABLE_FRAME_TIMESTAMP_CSV="${RGBD_SYNC_ENABLE_FRAME_TIMESTAMP_CSV:-true}"
RGBD_SYNC_FRAME_TIMESTAMP_CSV_FILE="${RGBD_SYNC_FRAME_TIMESTAMP_CSV_FILE:-/tmp/orbbec_rgbd_sync_timestamp.csv}"

RUN_COLOR_WIDTH="$COLOR_WIDTH"
RUN_COLOR_HEIGHT="$COLOR_HEIGHT"
RUN_COLOR_FPS="$COLOR_FPS"
RUN_DEPTH_WIDTH="$DEPTH_WIDTH"
RUN_DEPTH_HEIGHT="$DEPTH_HEIGHT"
RUN_DEPTH_FPS="$DEPTH_FPS"
if [ "$PROFILE" = "local_highres_v2" ]; then
  RUN_COLOR_WIDTH="$STEP10V2_COLOR_WIDTH"
  RUN_COLOR_HEIGHT="$STEP10V2_COLOR_HEIGHT"
  RUN_COLOR_FPS="$STEP10V2_COLOR_FPS"
  RUN_DEPTH_WIDTH="$STEP10V2_DEPTH_WIDTH"
  RUN_DEPTH_HEIGHT="$STEP10V2_DEPTH_HEIGHT"
  RUN_DEPTH_FPS="$STEP10V2_DEPTH_FPS"
fi
if [ "$PROFILE" = "local_highres_v21" ]; then
  RUN_COLOR_WIDTH="$STEP10V21_COLOR_WIDTH"
  RUN_COLOR_HEIGHT="$STEP10V21_COLOR_HEIGHT"
  RUN_COLOR_FPS="$STEP10V21_COLOR_FPS"
  RUN_DEPTH_WIDTH="$STEP10V21_DEPTH_WIDTH"
  RUN_DEPTH_HEIGHT="$STEP10V21_DEPTH_HEIGHT"
  RUN_DEPTH_FPS="$STEP10V21_DEPTH_FPS"
fi
if [ "$PROFILE" = "slam_clean" ]; then
  RUN_COLOR_WIDTH="$VO_COLOR_WIDTH"
  RUN_COLOR_HEIGHT="$VO_COLOR_HEIGHT"
  RUN_COLOR_FPS="$VO_COLOR_FPS"
  RUN_DEPTH_WIDTH="$VO_DEPTH_WIDTH"
  RUN_DEPTH_HEIGHT="$VO_DEPTH_HEIGHT"
  RUN_DEPTH_FPS="$VO_DEPTH_FPS"
fi
if [ "$PROFILE" = "rgbd_sync_test" ] || [ "$PROFILE" = "visual_odom_sync" ] || [ "$PROFILE" = "visual_odom_sync_lite" ]; then
  RUN_COLOR_WIDTH="$RGBD_SYNC_COLOR_WIDTH"
  RUN_COLOR_HEIGHT="$RGBD_SYNC_COLOR_HEIGHT"
  RUN_COLOR_FPS="$RGBD_SYNC_COLOR_FPS"
  RUN_DEPTH_WIDTH="$RGBD_SYNC_DEPTH_WIDTH"
  RUN_DEPTH_HEIGHT="$RGBD_SYNC_DEPTH_HEIGHT"
  RUN_DEPTH_FPS="$RGBD_SYNC_DEPTH_FPS"
fi

RUN_V2_ENABLE_COLOR="$STEP10V2_ENABLE_COLOR"
RUN_V2_DEPTH_REGISTRATION="$STEP10V2_DEPTH_REGISTRATION"
RUN_V2_ENABLE_FRAME_SYNC="$STEP10V2_ENABLE_FRAME_SYNC"
RUN_V2_ENABLE_NOISE_REMOVAL_FILTER="$STEP10V2_ENABLE_NOISE_REMOVAL_FILTER"
if [ "$PROFILE" = "local_highres_v21" ]; then
  RUN_V2_ENABLE_COLOR="$STEP10V21_ENABLE_COLOR"
  RUN_V2_DEPTH_REGISTRATION="$STEP10V21_DEPTH_REGISTRATION"
  RUN_V2_ENABLE_FRAME_SYNC="$STEP10V21_ENABLE_FRAME_SYNC"
  RUN_V2_ENABLE_NOISE_REMOVAL_FILTER="$STEP10V21_ENABLE_NOISE_REMOVAL_FILTER"
fi

OCTOMAP_RESOLUTION="${OCTOMAP_RESOLUTION:-0.08}"
OCTOMAP_MAX_RANGE="${OCTOMAP_MAX_RANGE:-4.0}"
OCTOMAP_POINT_MIN_Z="${OCTOMAP_POINT_MIN_Z:--2.0}"
OCTOMAP_POINT_MAX_Z="${OCTOMAP_POINT_MAX_Z:-3.0}"
OCTOMAP_OCCUPANCY_MIN_Z="${OCTOMAP_OCCUPANCY_MIN_Z:--1.0}"
OCTOMAP_OCCUPANCY_MAX_Z="${OCTOMAP_OCCUPANCY_MAX_Z:-2.5}"
OCTOMAP_FILTER_GROUND="${OCTOMAP_FILTER_GROUND:-false}"
OCTOMAP_LATCH="${OCTOMAP_LATCH:-false}"

usb_attr() {
  local dev="$1" attr="$2" path
  path="$(readlink -f "/sys/class/tty/${dev##*/}/device" 2>/dev/null || true)"
  while [ -n "$path" ] && [ "$path" != "/" ]; do
    if [ -r "$path/$attr" ]; then tr -d '\r\n' < "$path/$attr"; return 0; fi
    path="${path%/*}"
  done
  return 1
}

auto_detect_ports() {
  local dev pid
  if [ -z "${CHASSIS_PORT:-}" ] || [ -z "${LIDAR_PORT:-}" ]; then
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
      [ -e "$dev" ] || continue
      pid="$(usb_attr "$dev" idProduct 2>/dev/null || true)"
      case "${pid,,}" in
        7523) [ -n "${CHASSIS_PORT:-}" ] || CHASSIS_PORT="$dev" ;;
        55d4) [ -n "${LIDAR_PORT:-}" ] || LIDAR_PORT="$dev" ;;
      esac
    done
  fi
  [ -n "${CHASSIS_PORT:-}" ] || CHASSIS_PORT=/dev/ttyUSB0
  if [ -z "${LIDAR_PORT:-}" ]; then
    if [ -e /dev/ttyACM0 ]; then LIDAR_PORT=/dev/ttyACM0; else LIDAR_PORT=/dev/ttyUSB1; fi
  fi
}

need_chassis=false
need_lidar=false
need_camera=false
need_rtabmap=false
need_ekf=false
need_pointcloud=false
need_cloud_filter=false
need_octomap=false
need_local_highres=false
need_local_highres_v2=false
need_local_highres_v21=false
need_rgbd_sync_test=false
case "$PROFILE" in
  camera) need_camera=true ;;
  visual_odom) need_camera=true; need_rtabmap=true ;;
  visual_odom_sync|visual_odom_sync_lite|visual_odom_baseline) need_camera=true; need_rtabmap=true ;;
  wheel_imu) need_chassis=true; need_ekf=true ;;
  fusion) need_camera=true; need_rtabmap=true; need_chassis=true; need_ekf=true ;;
  slam) need_camera=true; need_rtabmap=true; need_chassis=true; need_ekf=true; need_lidar=true ;;
  slam_clean) need_camera=true; need_rtabmap=true; need_chassis=true; need_ekf=true; need_lidar=true ;;
  pointcloud) need_camera=true; need_pointcloud=true ;;
  filtered_cloud) need_camera=true; need_pointcloud=true; need_cloud_filter=true ;;
  octomap_odom)
    need_camera=true; need_rtabmap=true; need_chassis=true; need_ekf=true
    need_pointcloud=true; need_cloud_filter=true; need_octomap=true
    ;;
  dual_map)
    need_camera=true; need_rtabmap=true; need_chassis=true; need_ekf=true; need_lidar=true
    need_pointcloud=true; need_cloud_filter=true; need_octomap=true
    ;;
  local_highres)
    need_camera=true; need_pointcloud=true; need_local_highres=true
    ;;
  local_highres_v2)
    need_camera=true; need_local_highres_v2=true
    ;;
  local_highres_v21)
    need_camera=true; need_local_highres_v21=true
    ;;
  rgbd_sync_test)
    need_camera=true; need_rgbd_sync_test=true
    ;;
esac

auto_detect_ports
mkdir -p "$RUN_DIR"

[ -f /opt/ros/jazzy/setup.bash ] || die "未找到 /opt/ros/jazzy/setup.bash"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ -n "${ORBBEC_SETUP:-}" ]; then
  [ -f "$ORBBEC_SETUP" ] || die "ORBBEC_SETUP 不存在：$ORBBEC_SETUP"
  # shellcheck disable=SC1090
  source "$ORBBEC_SETUP"
fi
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

command -v ros2 >/dev/null 2>&1 || die "ros2 命令不可用"
command -v colcon >/dev/null 2>&1 || die "未安装 colcon"
command -v setsid >/dev/null 2>&1 || die "未安装 setsid"
[ -d "$LIDAR_WS/src/lidar_py" ] || die "找不到源码工作空间：$LIDAR_WS"

check_pkg() {
  ros2 pkg prefix "$1" >/dev/null 2>&1 || die "缺少 ROS 包 $1。先重新运行根目录 STEP0_INSTALL_VISUAL_SLAM_DEPS.sh"
}
$need_camera && check_pkg orbbec_camera
$need_rtabmap && check_pkg rtabmap_odom
$need_ekf && check_pkg robot_localization
if $need_lidar; then check_pkg cartographer_ros; check_pkg laser_filters; fi
$need_octomap && check_pkg octomap_server
check_pkg rmw_cyclonedds_cpp

if $need_chassis; then
  [ -e "$CHASSIS_PORT" ] || die "STM32 串口不存在：$CHASSIS_PORT"
  if [ ! -r "$CHASSIS_PORT" ] || [ ! -w "$CHASSIS_PORT" ]; then
    log "[权限] 尝试临时开放 STM32 串口：$CHASSIS_PORT"
    sudo chmod a+rw "$CHASSIS_PORT" || die "无法开放 STM32 串口权限"
  fi
fi
if $need_lidar; then
  [ -e "$LIDAR_PORT" ] || die "雷达串口不存在：$LIDAR_PORT"
  if [ ! -r "$LIDAR_PORT" ] || [ ! -w "$LIDAR_PORT" ]; then
    log "[权限] 尝试临时开放雷达串口：$LIDAR_PORT"
    sudo chmod a+rw "$LIDAR_PORT" || die "无法开放雷达串口权限"
  fi
fi

# 避免和完整 open_all 栈同时抢 TF/串口/相机。
if ros2 node list 2>/dev/null | grep -Eq '/(cartographer_node|controller_server|chassis_node|octomap_server_3d)$'; then
  die "检测到完整 SLAM/Nav2/OctoMap 栈仍在运行。请先 Ctrl+C 关闭 open_all.sh 或其他 STEP，再运行本步骤。"
fi

mkdir -p "$CACHE_WS"
if [ -L "$CACHE_SRC" ]; then rm -f "$CACHE_SRC"; elif [ -e "$CACHE_SRC" ]; then die "$CACHE_SRC 已存在且不是软链接"; fi
ln -s "$LIDAR_WS/src" "$CACHE_SRC"

if is_true "$AUTO_BUILD" || [ ! -f "$INSTALL_BASE/setup.bash" ] || $need_local_highres_v2 || $need_local_highres_v21 || $need_rgbd_sync_test || [ "$PROFILE" = "visual_odom_sync" ] || [ "$PROFILE" = "visual_odom_sync_lite" ] || [ "$PROFILE" = "slam_clean" ]; then
  log "[构建] 正在隔离构建 lidar_py（不覆盖原 open_all 构建）……"
  cd "$CACHE_WS"
  PYTHONNOUSERSITE=1 colcon --log-base "$LOG_BASE" build \
    --base-paths "$CACHE_SRC" \
    --build-base "$BUILD_BASE" \
    --install-base "$INSTALL_BASE" \
    --symlink-install \
    --packages-select local_depth_cloud_cpp lidar_py
fi
[ -f "$INSTALL_BASE/setup.bash" ] || die "构建结果不存在：$INSTALL_BASE/setup.bash"
# shellcheck disable=SC1090
source "$INSTALL_BASE/setup.bash"
check_pkg lidar_py
check_pkg local_depth_cloud_cpp

case "$PROFILE" in
  fusion|slam|slam_clean)
    if ! is_true "$CAMERA_TF_CONFIRMED"; then
      log ""
      log "[重要警告] CAMERA_TF_CONFIRMED=false"
      log "  当前会用配置文件中的相机外参：x=$CAMERA_X y=$CAMERA_Y z=$CAMERA_Z"
      log "  STEP4/STEP5/STEP11 的融合结果只有在 base_link→camera_link 外参正确时才可信。"
      log "  请修改：$ENV_FILE"
      log ""
    fi
    ;;
  filtered_cloud|octomap_odom|dual_map|local_highres|local_highres_v2|local_highres_v21)
    is_true "$CAMERA_TF_CONFIRMED" || die "STEP7/STEP8/STEP9/STEP10/STEP10V2/STEP10V2.1 必须先填写正确相机外参并设置 CAMERA_TF_CONFIRMED=true：$ENV_FILE"
    ;;
esac

log "============================================================"
log "  视觉 + 2D 雷达 + 三维体素地图分步测试"
log "  Profile      : $PROFILE"
log "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
log "  RViz         : $USE_RVIZ"
$need_chassis && log "  STM32        : $CHASSIS_PORT"
$need_lidar && log "  LiDAR        : $LIDAR_PORT @ $LIDAR_BAUD"
$need_camera && log "  RGB/Depth    : ${RUN_COLOR_WIDTH}x${RUN_COLOR_HEIGHT}@${RUN_COLOR_FPS} / ${RUN_DEPTH_WIDTH}x${RUN_DEPTH_HEIGHT}@${RUN_DEPTH_FPS}"
$need_pointcloud && log "  Raw cloud    : $POINT_CLOUD_TOPIC"
$need_cloud_filter && log "  Filter cloud : $FILTERED_CLOUD_TOPIC @ ${CLOUD_MAX_RATE_HZ}Hz, voxel=${CLOUD_VOXEL_SIZE}m"
$need_local_highres && log "  Local cloud  : $LOCAL_HIGHRES_CLOUD_TOPIC @ ${LOCAL_MAX_RATE_HZ}Hz, voxel=${LOCAL_VOXEL_SIZE}m"
$need_local_highres_v2 && log "  STEP10V2     : depth image -> $STEP10V2_CLOUD_TOPIC, stride=${STEP10V2_PIXEL_STRIDE}, voxel=${STEP10V2_VOXEL_SIZE}m"
$need_local_highres_v2 && log "  Raw PointCloud2 generation: DISABLED"
$need_local_highres_v21 && log "  STEP10V2.1   : latest-frame worker -> $STEP10V21_CLOUD_TOPIC, stride=${STEP10V21_PIXEL_STRIDE}, voxel=${STEP10V21_VOXEL_SIZE}m"
$need_local_highres_v21 && log "  Raw PointCloud2 generation: DISABLED; static TF cached once"
[ "$PROFILE" = "slam_clean" ] && log "  STEP11 VO     : /visual_odom_clean, GuessMotion=$VO_ODOM_GUESS_MOTION, Decimation=$VO_ODOM_IMAGE_DECIMATION"
[ "$PROFILE" = "slam_clean" ] && log "  TF authority  : EKF only publishes odom->base_link; RTAB-Map publish_tf=false"
$need_rgbd_sync_test && log "  RGB-D sync   : ${RGBD_SYNC_COLOR_FPS}Hz color + ${RGBD_SYNC_DEPTH_FPS}Hz depth -> $RGBD_SYNC_STATS_TOPIC"
[ "$PROFILE" = "visual_odom_sync" ] || [ "$PROFILE" = "visual_odom_sync_lite" ] && log "  Visual sync  : 15/15Hz + frame_sync + max interval 25ms"
$need_octomap && log "  OctoMap      : resolution=${OCTOMAP_RESOLUTION}m, max_range=${OCTOMAP_MAX_RANGE}m"
log "  Runtime log  : $RUNTIME_LOG"
log "============================================================"

cmd=(ros2 launch lidar_py visual_laser_slam.launch.py
  "profile:=$PROFILE"
  "launch_rviz:=$USE_RVIZ"
  "color_width:=$RUN_COLOR_WIDTH" "color_height:=$RUN_COLOR_HEIGHT" "color_fps:=$RUN_COLOR_FPS"
  "depth_width:=$RUN_DEPTH_WIDTH" "depth_height:=$RUN_DEPTH_HEIGHT" "depth_fps:=$RUN_DEPTH_FPS"
  "v2_enable_color:=$RUN_V2_ENABLE_COLOR"
  "v2_depth_registration:=$RUN_V2_DEPTH_REGISTRATION"
  "v2_enable_frame_sync:=$RUN_V2_ENABLE_FRAME_SYNC"
  "v2_enable_noise_removal_filter:=$RUN_V2_ENABLE_NOISE_REMOVAL_FILTER"
  "v21_enable_depth_auto_exposure_priority:=$STEP10V21_ENABLE_DEPTH_AUTO_EXPOSURE_PRIORITY"
  "camera_enable_sync_host_time:=${STEP10V21_ENABLE_SYNC_HOST_TIME:-true}"
  "camera_time_domain:=${STEP10V21_TIME_DOMAIN:-device}"
  "camera_time_sync_period:=${STEP10V21_TIME_SYNC_PERIOD:-60.0}"
  "camera_enable_frame_timestamp_csv:=$(if [ "$PROFILE" = "rgbd_sync_test" ] || [ "$PROFILE" = "visual_odom_sync" ] || [ "$PROFILE" = "visual_odom_sync_lite" ]; then printf %s "$RGBD_SYNC_ENABLE_FRAME_TIMESTAMP_CSV"; else printf %s "$STEP10V21_ENABLE_FRAME_TIMESTAMP_CSV"; fi)"
  "camera_frame_timestamp_csv_file:=$(if [ "$PROFILE" = "rgbd_sync_test" ] || [ "$PROFILE" = "visual_odom_sync" ] || [ "$PROFILE" = "visual_odom_sync_lite" ]; then printf %s "$RGBD_SYNC_FRAME_TIMESTAMP_CSV_FILE"; else printf %s "$STEP10V21_FRAME_TIMESTAMP_CSV_FILE"; fi)"
  "camera_x:=$CAMERA_X" "camera_y:=$CAMERA_Y" "camera_z:=$CAMERA_Z"
  "camera_roll:=$CAMERA_ROLL" "camera_pitch:=$CAMERA_PITCH" "camera_yaw:=$CAMERA_YAW"
  "chassis_serial_port:=$CHASSIS_PORT"
  "lidar_serial_port:=$LIDAR_PORT" "lidar_baudrate:=$LIDAR_BAUD"
  "laser_yaw_deg:=$LASER_YAW_DEG" "scan_angle_sign:=$SCAN_ANGLE_SIGN"
  "point_cloud_topic:=$POINT_CLOUD_TOPIC" "filtered_cloud_topic:=$FILTERED_CLOUD_TOPIC"
  "cloud_max_rate_hz:=$CLOUD_MAX_RATE_HZ" "cloud_sample_stride:=$CLOUD_SAMPLE_STRIDE"
  "cloud_voxel_size:=$CLOUD_VOXEL_SIZE" "cloud_min_range:=$CLOUD_MIN_RANGE" "cloud_max_range:=$CLOUD_MAX_RANGE"
  "cloud_transform_timeout:=$CLOUD_TRANSFORM_TIMEOUT"
  "cloud_base_x_min:=$CLOUD_BASE_X_MIN" "cloud_base_x_max:=$CLOUD_BASE_X_MAX"
  "cloud_base_y_min:=$CLOUD_BASE_Y_MIN" "cloud_base_y_max:=$CLOUD_BASE_Y_MAX"
  "cloud_base_z_min:=$CLOUD_BASE_Z_MIN" "cloud_base_z_max:=$CLOUD_BASE_Z_MAX"
  "cloud_remove_self:=$CLOUD_REMOVE_SELF"
  "cloud_self_x_min:=$CLOUD_SELF_X_MIN" "cloud_self_x_max:=$CLOUD_SELF_X_MAX"
  "cloud_self_y_min:=$CLOUD_SELF_Y_MIN" "cloud_self_y_max:=$CLOUD_SELF_Y_MAX"
  "cloud_self_z_min:=$CLOUD_SELF_Z_MIN" "cloud_self_z_max:=$CLOUD_SELF_Z_MAX"
  "cloud_log_every_n:=$CLOUD_LOG_EVERY_N"
  "local_cloud_topic:=$LOCAL_HIGHRES_CLOUD_TOPIC"
  "local_cloud_stats_topic:=$LOCAL_HIGHRES_STATS_TOPIC"
  "local_cloud_marker_topic:=$LOCAL_HIGHRES_MARKER_TOPIC"
  "local_max_rate_hz:=$LOCAL_MAX_RATE_HZ" "local_sample_stride:=$LOCAL_SAMPLE_STRIDE"
  "local_voxel_size:=$LOCAL_VOXEL_SIZE" "local_min_range:=$LOCAL_MIN_RANGE" "local_max_range:=$LOCAL_MAX_RANGE"
  "local_transform_timeout:=$LOCAL_TRANSFORM_TIMEOUT"
  "local_x_min:=$LOCAL_X_MIN" "local_x_max:=$LOCAL_X_MAX"
  "local_y_min:=$LOCAL_Y_MIN" "local_y_max:=$LOCAL_Y_MAX"
  "local_z_min:=$LOCAL_Z_MIN" "local_z_max:=$LOCAL_Z_MAX"
  "local_remove_self:=$LOCAL_REMOVE_SELF"
  "local_self_x_min:=$LOCAL_SELF_X_MIN" "local_self_x_max:=$LOCAL_SELF_X_MAX"
  "local_self_y_min:=$LOCAL_SELF_Y_MIN" "local_self_y_max:=$LOCAL_SELF_Y_MAX"
  "local_self_z_min:=$LOCAL_SELF_Z_MIN" "local_self_z_max:=$LOCAL_SELF_Z_MAX"
  "local_ground_filter_enabled:=$LOCAL_GROUND_FILTER_ENABLED"
  "local_ground_z_min:=$LOCAL_GROUND_Z_MIN" "local_ground_z_max:=$LOCAL_GROUND_Z_MAX"
  "local_stats_period_sec:=$LOCAL_STATS_PERIOD_SEC"
  "local_publish_markers:=$LOCAL_PUBLISH_MARKERS"
  "v2_depth_topic:=$STEP10V2_DEPTH_TOPIC"
  "v2_camera_info_topic:=$STEP10V2_CAMERA_INFO_TOPIC"
  "v2_cloud_topic:=$STEP10V2_CLOUD_TOPIC"
  "v2_stats_topic:=$STEP10V2_STATS_TOPIC"
  "v2_marker_topic:=$STEP10V2_MARKER_TOPIC"
  "v2_max_rate_hz:=$STEP10V2_MAX_RATE_HZ"
  "v2_pixel_stride:=$STEP10V2_PIXEL_STRIDE"
  "v2_depth_unit_scale:=$STEP10V2_DEPTH_UNIT_SCALE"
  "v2_voxel_size:=$STEP10V2_VOXEL_SIZE"
  "v2_min_range:=$STEP10V2_MIN_RANGE" "v2_max_range:=$STEP10V2_MAX_RANGE"
  "v2_transform_timeout:=$STEP10V2_TRANSFORM_TIMEOUT"
  "v2_roi_u_min:=$STEP10V2_ROI_U_MIN" "v2_roi_u_max:=$STEP10V2_ROI_U_MAX"
  "v2_roi_v_min:=$STEP10V2_ROI_V_MIN" "v2_roi_v_max:=$STEP10V2_ROI_V_MAX"
  "v2_x_min:=$STEP10V2_X_MIN" "v2_x_max:=$STEP10V2_X_MAX"
  "v2_y_min:=$STEP10V2_Y_MIN" "v2_y_max:=$STEP10V2_Y_MAX"
  "v2_z_min:=$STEP10V2_Z_MIN" "v2_z_max:=$STEP10V2_Z_MAX"
  "v2_remove_self:=$STEP10V2_REMOVE_SELF"
  "v2_self_x_min:=$STEP10V2_SELF_X_MIN" "v2_self_x_max:=$STEP10V2_SELF_X_MAX"
  "v2_self_y_min:=$STEP10V2_SELF_Y_MIN" "v2_self_y_max:=$STEP10V2_SELF_Y_MAX"
  "v2_self_z_min:=$STEP10V2_SELF_Z_MIN" "v2_self_z_max:=$STEP10V2_SELF_Z_MAX"
  "v2_ground_filter_enabled:=$STEP10V2_GROUND_FILTER_ENABLED"
  "v2_ground_z_min:=$STEP10V2_GROUND_Z_MIN" "v2_ground_z_max:=$STEP10V2_GROUND_Z_MAX"
  "v2_stats_period_sec:=$STEP10V2_STATS_PERIOD_SEC"
  "v2_publish_markers:=$STEP10V2_PUBLISH_MARKERS"
  "v21_depth_topic:=$STEP10V21_DEPTH_TOPIC"
  "v21_camera_info_topic:=$STEP10V21_CAMERA_INFO_TOPIC"
  "v21_cloud_topic:=$STEP10V21_CLOUD_TOPIC"
  "v21_stats_topic:=$STEP10V21_STATS_TOPIC"
  "v21_marker_topic:=$STEP10V21_MARKER_TOPIC"
  "v21_max_rate_hz:=$STEP10V21_MAX_RATE_HZ"
  "v21_pixel_stride:=$STEP10V21_PIXEL_STRIDE"
  "v21_depth_unit_scale:=$STEP10V21_DEPTH_UNIT_SCALE"
  "v21_voxel_size:=$STEP10V21_VOXEL_SIZE"
  "v21_min_range:=$STEP10V21_MIN_RANGE" "v21_max_range:=$STEP10V21_MAX_RANGE"
  "v21_transform_timeout:=$STEP10V21_TRANSFORM_TIMEOUT"
  "v21_max_input_age_ms:=$STEP10V21_MAX_INPUT_AGE_MS"
  "v21_roi_u_min:=$STEP10V21_ROI_U_MIN" "v21_roi_u_max:=$STEP10V21_ROI_U_MAX"
  "v21_roi_v_min:=$STEP10V21_ROI_V_MIN" "v21_roi_v_max:=$STEP10V21_ROI_V_MAX"
  "v21_x_min:=$STEP10V21_X_MIN" "v21_x_max:=$STEP10V21_X_MAX"
  "v21_y_min:=$STEP10V21_Y_MIN" "v21_y_max:=$STEP10V21_Y_MAX"
  "v21_z_min:=$STEP10V21_Z_MIN" "v21_z_max:=$STEP10V21_Z_MAX"
  "v21_remove_self:=$STEP10V21_REMOVE_SELF"
  "v21_self_x_min:=$STEP10V21_SELF_X_MIN" "v21_self_x_max:=$STEP10V21_SELF_X_MAX"
  "v21_self_y_min:=$STEP10V21_SELF_Y_MIN" "v21_self_y_max:=$STEP10V21_SELF_Y_MAX"
  "v21_self_z_min:=$STEP10V21_SELF_Z_MIN" "v21_self_z_max:=$STEP10V21_SELF_Z_MAX"
  "v21_ground_filter_enabled:=$STEP10V21_GROUND_FILTER_ENABLED"
  "v21_ground_z_min:=$STEP10V21_GROUND_Z_MIN" "v21_ground_z_max:=$STEP10V21_GROUND_Z_MAX"
  "v21_stats_period_sec:=$STEP10V21_STATS_PERIOD_SEC"
  "v21_stats_window_size:=$STEP10V21_STATS_WINDOW_SIZE"
  "v21_process_warn_ms:=$STEP10V21_PROCESS_WARN_MS"
  "v21_age_warn_ms:=$STEP10V21_AGE_WARN_MS"
  "v21_stall_warn_gap_ms:=$STEP10V21_STALL_WARN_GAP_MS"
  "v21_publish_markers:=$STEP10V21_PUBLISH_MARKERS"
  "sync_color_topic:=/camera/color/image_raw"
  "sync_depth_topic:=/camera/depth/image_raw"
  "sync_stats_topic:=$RGBD_SYNC_STATS_TOPIC"
  "sync_max_pair_interval_ms:=$RGBD_SYNC_MAX_PAIR_INTERVAL_MS"
  "sync_warn_p95_ms:=$RGBD_SYNC_WARN_P95_MS"
  "sync_window_size:=$RGBD_SYNC_WINDOW_SIZE"
  "visual_sync_max_interval:=0.025"
  "visual_clean_approx_sync_max_interval:=$VO_APPROX_SYNC_MAX_INTERVAL"
  "visual_clean_topic_queue_size:=$VO_TOPIC_QUEUE_SIZE"
  "visual_clean_sync_queue_size:=$VO_SYNC_QUEUE_SIZE"
  "visual_clean_wait_for_transform:=$VO_WAIT_FOR_TRANSFORM"
  "visual_clean_odom_guess_motion:=$VO_ODOM_GUESS_MOTION"
  "visual_clean_odom_image_decimation:=$VO_ODOM_IMAGE_DECIMATION"
  "octomap_resolution:=$OCTOMAP_RESOLUTION" "octomap_max_range:=$OCTOMAP_MAX_RANGE"
  "octomap_point_min_z:=$OCTOMAP_POINT_MIN_Z" "octomap_point_max_z:=$OCTOMAP_POINT_MAX_Z"
  "octomap_occupancy_min_z:=$OCTOMAP_OCCUPANCY_MIN_Z" "octomap_occupancy_max_z:=$OCTOMAP_OCCUPANCY_MAX_Z"
  "octomap_filter_ground:=$OCTOMAP_FILTER_GROUND" "octomap_latch:=$OCTOMAP_LATCH")

log "[启动] ${cmd[*]}"
setsid stdbuf -oL -eL "${cmd[@]}" >"$RUNTIME_LOG" 2>&1 &
LAUNCH_PID=$!

wait_topic() {
  local topic="$1" timeout_sec="${2:-35}" elapsed=0
  while [ "$elapsed" -lt "$timeout_sec" ]; do
    kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      log "[就绪] $topic"
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  log "[超时] 未发现话题：$topic"
  return 1
}

sleep 2
case "$PROFILE" in
  camera)
    wait_topic /camera/color/image_raw 45 || die "Gemini2 彩色流未启动"
    wait_topic /camera/depth/image_raw 20 || die "Gemini2 深度流未启动"
    log "[测试] 检查 RViz 中 RGB/Depth；静止运行至少30秒，确认无掉流。"
    ;;
  visual_odom)
    wait_topic /visual_odom 60 || die "RTAB-Map 未输出 /visual_odom"
    log "[测试] 手推小车：前进1米、后退、左转90°，观察 /visual_odom 连续性。"
    ;;
  visual_odom_sync)
    wait_topic /camera/color/image_raw 60 || die "同步彩色流未输出"
    wait_topic /camera/depth/image_raw 30 || die "同步深度流未输出"
    wait_topic /visual_odom 60 || die "同步版RTAB-Map未输出 /visual_odom"
    wait_topic "$RGBD_SYNC_STATS_TOPIC" 20 || die "RGB-D时间戳统计未输出"
    log "[统计] ros2 topic echo $RGBD_SYNC_STATS_TOPIC"
    log "[测试] RGB/Depth均为15Hz，frame_sync+host_time已开启，RTAB-Map最大配对间隔25ms。"
    log "[CSV] $RGBD_SYNC_FRAME_TIMESTAMP_CSV_FILE"
    log "[测试] 手推前进/转弯，确认不再出现约30ms以上的RGB/Depth差值警告。"
    ;;
  visual_odom_sync_lite)
    wait_topic /camera/color/image_raw 60 || die "同步彩色流未输出"
    wait_topic /camera/depth/image_raw 30 || die "同步深度流未输出"
    wait_topic /visual_odom 60 || die "RTAB-Map未输出 /visual_odom"
    log "[测试] 无同步监控，直接观察RTAB-Map日志中的delay和quality。"
    log "[验收] visual_odom>=10Hz, update_time_p95<80ms, delay_p95<150ms"
    ;;
  visual_odom_baseline)
    wait_topic /camera/color/image_raw 60 || die "彩色流未输出"
    wait_topic /camera/depth/image_raw 30 || die "深度流未输出"
    wait_topic /visual_odom 60 || die "RTAB-Map未输出 /visual_odom"
    log "[测试] 原STEP9视觉基线，无RViz，无同步监控。"
    log "[验收] 记录100帧以上的update_time和delay统计。"
    ;;
  wheel_imu)
    wait_topic /wheel/odom 30 || die "底盘未输出 /wheel/odom"
    wait_topic /imu_cartographer 30 || die "底盘未输出 /imu_cartographer"
    wait_topic /odometry/filtered 30 || die "EKF 未输出 /odometry/filtered"
    log "[测试] 可低速发送 /cmd_vel_visual_slam_test，检查直线和原地转向。"
    ;;
  fusion)
    wait_topic /visual_odom 60 || die "视觉里程计未输出"
    wait_topic /wheel/odom 30 || die "轮速里程计未输出"
    wait_topic /odometry/filtered 30 || die "融合里程计未输出"
    log "[测试] 对比 RViz 的 Visual/Wheel/EKF 三条里程计，EKF 应连续且不跳变。"
    ;;
  slam)
    wait_topic /odometry/filtered 60 || die "融合里程计未输出"
    wait_topic /scan_timed_v2_filtered 40 || die "过滤后雷达扫描未输出"
    wait_topic /map 90 || die "Cartographer 未输出 /map"
    log "[测试] RViz 已固定在 map。低速走闭环路线，观察 map→base_link 是否回到原点附近。"
    ;;
  slam_clean)
    wait_topic /visual_odom_clean 75 || die "clean视觉里程计未输出 /visual_odom_clean"
    wait_topic /wheel/odom 30 || die "轮速里程计未输出 /wheel/odom"
    wait_topic /imu_cartographer 30 || die "IMU未输出 /imu_cartographer"
    wait_topic /odometry/filtered 60 || die "STEP11 EKF未输出 /odometry/filtered"
    wait_topic /scan_timed_v2_filtered 40 || die "过滤后雷达扫描未输出"
    wait_topic /map 90 || die "Cartographer未输出 /map"
    log "[STEP11就绪] clean视觉里程计 + 轮速 + IMU -> EKF -> Cartographer 2D雷达SLAM。"
    log "[TF结构] map -> odom 由Cartographer发布；odom -> base_link只由EKF发布。"
    log "[测试] 先静止30秒，再低速直行、原地转向、走闭环；不要一上来高速跑。"
    log "[频率] ros2 topic hz /visual_odom_clean"
    log "[频率] ros2 topic hz /odometry/filtered"
    log "[TF] ros2 run tf2_ros tf2_echo map base_link"
    ;;
  pointcloud)
    wait_topic "$POINT_CLOUD_TOPIC" 60 || die "Gemini2 未输出原始深度点云：$POINT_CLOUD_TOPIC"
    log "[测试] 原地观察点云方向、距离、盲区和反光噪声；本步骤不需要开电机。"
    ;;
  filtered_cloud)
    wait_topic "$POINT_CLOUD_TOPIC" 60 || die "原始深度点云未输出"
    wait_topic "$FILTERED_CLOUD_TOPIC" 40 || die "过滤点云未输出"
    log "[测试] 检查过滤点云是否仍覆盖前方障碍，同时明显减少远处/无效点和点数。"
    log "[提示] 统计话题：/depth_point_cloud_filter/stats"
    ;;
  local_highres)
    wait_topic "$POINT_CLOUD_TOPIC" 60 || die "Gemini2 未输出原始深度点云"
    wait_topic "$LOCAL_HIGHRES_CLOUD_TOPIC" 45 || die "STEP10 局部高精度点云未输出"
    wait_topic "$LOCAL_HIGHRES_STATS_TOPIC" 10 || die "STEP10 统计话题未输出"
    log "[测试] 快速摆动物体并比较原始/局部点云延迟；连续运行5分钟确认延迟不增长。"
    log "[统计] ros2 topic echo $LOCAL_HIGHRES_STATS_TOPIC"
    ;;
  local_highres_v2)
    wait_topic "$STEP10V2_DEPTH_TOPIC" 60 || die "Gemini2 未输出深度图：$STEP10V2_DEPTH_TOPIC"
    wait_topic "$STEP10V2_CAMERA_INFO_TOPIC" 20 || die "Gemini2 未输出深度CameraInfo"
    wait_topic "$STEP10V2_CLOUD_TOPIC" 45 || die "STEP10V2 局部点云未输出"
    wait_topic "$STEP10V2_STATS_TOPIC" 10 || die "STEP10V2 统计话题未输出"
    log "[测试] 本步骤已关闭Orbbec完整PointCloud2。快速摆动物体，观察点云是否紧跟实物。"
    log "[统计] ros2 topic echo $STEP10V2_STATS_TOPIC"
    log "[频率] ros2 topic hz $STEP10V2_DEPTH_TOPIC"
    log "[频率] ros2 topic hz $STEP10V2_CLOUD_TOPIC"
    ;;
  local_highres_v21)
    wait_topic "$STEP10V21_DEPTH_TOPIC" 60 || die "Gemini2 未输出深度图：$STEP10V21_DEPTH_TOPIC"
    wait_topic "$STEP10V21_CAMERA_INFO_TOPIC" 20 || die "Gemini2 未输出深度CameraInfo"
    wait_topic "$STEP10V21_CLOUD_TOPIC" 45 || die "STEP10V2.1 局部点云未输出"
    wait_topic "$STEP10V21_STATS_TOPIC" 10 || die "STEP10V2.1 统计话题未输出"
    log "[测试] 快速左右摆动物体并连续运行5分钟，重点看P95/最大延迟和最长输出间隔。"
    log "[统计] ros2 topic echo $STEP10V21_STATS_TOPIC"
    log "[验收] process_p95<25ms、age_p95<100ms、output_gap_max<120ms、时间戳不倒退。"
    ;;
  rgbd_sync_test)
    wait_topic /camera/color/image_raw 60 || die "Gemini2 彩色图未输出"
    wait_topic /camera/depth/image_raw 30 || die "Gemini2 深度图未输出"
    wait_topic "$RGBD_SYNC_STATS_TOPIC" 20 || die "RGB-D时间戳统计未输出"
    log "[测试] 静止运行2分钟，再缓慢移动相机；观察RGB/Depth时间戳差。"
    log "[统计] ros2 topic echo $RGBD_SYNC_STATS_TOPIC"
    log "[CSV] $RGBD_SYNC_FRAME_TIMESTAMP_CSV_FILE"
    log "[验收] color/depth频率一致，abs_diff_p95_ms尽量<25ms，无重复/倒退时间戳。"
    ;;
  octomap_odom)
    wait_topic /odometry/filtered 60 || die "融合里程计未输出"
    wait_topic "$FILTERED_CLOUD_TOPIC" 60 || die "过滤点云未输出"
    wait_topic /occupied_cells_vis_array 60 || die "OctoMap 三维占据体素未输出"
    log "[测试] 这是 odom 坐标系三维建图隔离测试；低速走小范围，允许累计漂移。"
    ;;
  dual_map)
    wait_topic /odometry/filtered 60 || die "融合里程计未输出"
    wait_topic /scan_timed_v2_filtered 40 || die "过滤后2D雷达扫描未输出"
    wait_topic /map 90 || die "Cartographer 未输出二维 /map"
    wait_topic "$FILTERED_CLOUD_TOPIC" 60 || die "过滤点云未输出"
    wait_topic /occupied_cells_vis_array 60 || die "OctoMap 三维占据体素未输出"
    log "[测试] 最终双地图：2D雷达负责全局定位/二维地图，Gemini2负责三维体素地图。"
    log "[保存3D] 另开终端执行：./SAVE_STEP9_OCTOMAP.sh"
    ;;
esac

log "[运行] 按 Ctrl+C 关闭本步骤。实时日志如下："
tail -n 80 -F "$RUNTIME_LOG" &
TAIL_PID=$!
wait "$LAUNCH_PID"
