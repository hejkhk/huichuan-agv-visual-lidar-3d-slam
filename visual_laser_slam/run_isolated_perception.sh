#!/usr/bin/env bash
set -Eeo pipefail

PROFILE="${1:-}"
case "$PROFILE" in
  step10v21|visual_odom_clean) ;;
  *)
    echo "用法: $0 {step10v21|visual_odom_clean}" >&2
    exit 2
    ;;
esac

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_ENV="$ROOT_DIR/visual_laser_slam/visual_laser_slam.env"
PROFILE_ENV="$ROOT_DIR/visual_laser_slam/${PROFILE/step10v21/step10v21}.env"
if [ "$PROFILE" = "visual_odom_clean" ]; then
  PROFILE_ENV="$ROOT_DIR/visual_laser_slam/visual_odom_clean.env"
fi
LIDAR_WS="$ROOT_DIR/lidar/chapt1_ws"
CACHE_WS="${VISUAL_SLAM_BUILD_ROOT:-$HOME/.cache/huichuan_agv_visual_slam_humble_ws}"
CACHE_SRC="$CACHE_WS/src"
BUILD_BASE="$CACHE_WS/build"
INSTALL_BASE="$CACHE_WS/install"
LOG_BASE="$CACHE_WS/log"
RUN_STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_DIR="$ROOT_DIR/SLAM_Log/isolated_${PROFILE}_${RUN_STAMP}"
RUNTIME_LOG="$RUN_DIR/runtime.log"
LAUNCH_PID=""
TAIL_PID=""

log() { printf '%s\n' "$*"; }
die() { log "[错误] $*"; [ -f "$RUNTIME_LOG" ] && tail -n 160 "$RUNTIME_LOG" || true; exit 1; }
is_true() { case "${1,,}" in 1|true|yes|on) return 0;; *) return 1;; esac; }

cleanup() {
  local code=$?
  trap - EXIT INT TERM HUP
  log ""
  log "[停止] 正在关闭 $PROFILE ……"
  [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null || true
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    sleep 0.3
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  fi
  [ -n "$LAUNCH_PID" ] && wait "$LAUNCH_PID" 2>/dev/null || true
  log "[停止] 已关闭。日志：$RUNTIME_LOG"
  exit "$code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

[ -f "$BASE_ENV" ] || die "缺少原始配置：$BASE_ENV"
[ -f "$PROFILE_ENV" ] || die "缺少独立配置：$PROFILE_ENV"
# shellcheck disable=SC1090
source "$BASE_ENV"
# shellcheck disable=SC1090
source "$PROFILE_ENV"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
AUTO_BUILD="${AUTO_BUILD:-true}"
CAMERA_X="${CAMERA_X:-0.3}"
CAMERA_Y="${CAMERA_Y:-0.0}"
CAMERA_Z="${CAMERA_Z:-0.4}"
CAMERA_ROLL="${CAMERA_ROLL:-0.0}"
CAMERA_PITCH="${CAMERA_PITCH:-0.0}"
CAMERA_YAW="${CAMERA_YAW:-0.0}"

mkdir -p "$RUN_DIR" "$CACHE_WS"
[ -f /opt/ros/humble/setup.bash ] || die "未找到 /opt/ros/humble/setup.bash"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
[ "${ROS_DISTRO:-}" = "humble" ] || die "ROS_DISTRO 不是 humble：${ROS_DISTRO:-unset}"
if [ -n "${ORBBEC_SETUP:-}" ]; then
  [ -f "$ORBBEC_SETUP" ] || die "ORBBEC_SETUP 不存在：$ORBBEC_SETUP"
  # shellcheck disable=SC1090
  source "$ORBBEC_SETUP"
fi
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

command -v ros2 >/dev/null 2>&1 || die "ros2 不可用"
command -v colcon >/dev/null 2>&1 || die "colcon 不可用"
command -v setsid >/dev/null 2>&1 || die "setsid 不可用"
[ -d "$LIDAR_WS/src/lidar_py" ] || die "找不到 lidar_py 源码"

# 不允许和旧 STEP/open_all 同时抢相机或 TF。
if ros2 node list 2>/dev/null | grep -Eq '/(camera_container|camera/camera|rgbd_odometry|depth_image_to_local_cloud_v21|cartographer_node|octomap_server_3d)$'; then
  die "检测到相机/视觉/SLAM节点仍在运行。请先关闭其他STEP或open_all。"
fi

if [ -L "$CACHE_SRC" ]; then
  rm -f "$CACHE_SRC"
elif [ -e "$CACHE_SRC" ]; then
  die "$CACHE_SRC 已存在且不是软链接"
fi
ln -s "$LIDAR_WS/src" "$CACHE_SRC"

packages=(lidar_py)
if [ "$PROFILE" = "step10v21" ]; then
  packages=(local_depth_cloud_cpp lidar_py)
fi
if is_true "$AUTO_BUILD" || [ ! -f "$INSTALL_BASE/setup.bash" ]; then
  log "[构建] ${packages[*]}"
  cd "$CACHE_WS"
  PYTHONNOUSERSITE=1 colcon --log-base "$LOG_BASE" build \
    --base-paths "$CACHE_SRC" \
    --build-base "$BUILD_BASE" \
    --install-base "$INSTALL_BASE" \
    --symlink-install \
    --packages-select "${packages[@]}"
fi
[ -f "$INSTALL_BASE/setup.bash" ] || die "构建结果不存在"
# shellcheck disable=SC1090
source "$INSTALL_BASE/setup.bash"
ros2 pkg prefix lidar_py >/dev/null 2>&1 || die "未安装 lidar_py"
if [ "$PROFILE" = "step10v21" ]; then
  ros2 pkg prefix local_depth_cloud_cpp >/dev/null 2>&1 || die "未安装 local_depth_cloud_cpp"
fi

wait_topic() {
  local topic="$1" timeout_sec="${2:-60}" elapsed=0
  while [ "$elapsed" -lt "$timeout_sec" ]; do
    kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      log "[就绪] $topic"
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

wait_node() {
  local node="$1" timeout_sec="${2:-30}" elapsed=0
  while [ "$elapsed" -lt "$timeout_sec" ]; do
    kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
    if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
      log "[就绪] $node"
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

cmd=()
if [ "$PROFILE" = "step10v21" ]; then
  is_true "${CAMERA_TF_CONFIRMED:-false}" || die "请先在 visual_laser_slam.env 填写相机外参并设置 CAMERA_TF_CONFIRMED=true"
  cmd=(ros2 launch lidar_py step10v21_local_cloud.launch.py
    "launch_rviz:=${STEP10V21_USE_RVIZ:-true}"
    "depth_width:=${STEP10V21_DEPTH_WIDTH:-1280}"
    "depth_height:=${STEP10V21_DEPTH_HEIGHT:-800}"
    "depth_fps:=${STEP10V21_DEPTH_FPS:-30}"
    "camera_x:=$CAMERA_X" "camera_y:=$CAMERA_Y" "camera_z:=$CAMERA_Z"
    "camera_roll:=$CAMERA_ROLL" "camera_pitch:=$CAMERA_PITCH" "camera_yaw:=$CAMERA_YAW"
    "max_rate_hz:=${STEP10V21_MAX_RATE_HZ:-30.0}"
    "pixel_stride:=${STEP10V21_PIXEL_STRIDE:-2}"
    "voxel_size:=${STEP10V21_VOXEL_SIZE:-0.03}"
    "min_range:=${STEP10V21_MIN_RANGE:-0.20}"
    "max_range:=${STEP10V21_MAX_RANGE:-4.0}"
    "max_input_age_ms:=${STEP10V21_MAX_INPUT_AGE_MS:-150.0}"
    "x_min:=${STEP10V21_X_MIN:-0.15}" "x_max:=${STEP10V21_X_MAX:-4.0}"
    "y_min:=${STEP10V21_Y_MIN:--2.5}" "y_max:=${STEP10V21_Y_MAX:-2.5}"
    "z_min:=${STEP10V21_Z_MIN:--0.5}" "z_max:=${STEP10V21_Z_MAX:-2.0}"
    "remove_self:=${STEP10V21_REMOVE_SELF:-true}"
    "ground_filter_enabled:=${STEP10V21_GROUND_FILTER_ENABLED:-false}")
else
  cmd=(ros2 launch lidar_py visual_odom_clean.launch.py
    "launch_rviz:=${VISUAL_ODOM_CLEAN_USE_RVIZ:-false}"
    "color_width:=${VO_COLOR_WIDTH:-640}" "color_height:=${VO_COLOR_HEIGHT:-480}" "color_fps:=${VO_COLOR_FPS:-15}"
    "depth_width:=${VO_DEPTH_WIDTH:-640}" "depth_height:=${VO_DEPTH_HEIGHT:-400}" "depth_fps:=${VO_DEPTH_FPS:-15}"
    "camera_x:=$CAMERA_X" "camera_y:=$CAMERA_Y" "camera_z:=$CAMERA_Z"
    "camera_roll:=$CAMERA_ROLL" "camera_pitch:=$CAMERA_PITCH" "camera_yaw:=$CAMERA_YAW"
    "approx_sync_max_interval:=${VO_APPROX_SYNC_MAX_INTERVAL:-0.020}"
    "topic_queue_size:=${VO_TOPIC_QUEUE_SIZE:-2}"
    "sync_queue_size:=${VO_SYNC_QUEUE_SIZE:-5}"
    "wait_for_transform:=${VO_WAIT_FOR_TRANSFORM:-0.10}"
    "odom_guess_motion:=${VO_ODOM_GUESS_MOTION:-false}"
    "odom_image_decimation:=${VO_ODOM_IMAGE_DECIMATION:-1}")
fi

log "============================================================"
log "  独立感知测试：$PROFILE"
log "  原STEP1-STEP9核心文件未修改"
log "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
log "  Runtime log  : $RUNTIME_LOG"
log "============================================================"
log "[启动] ${cmd[*]}"
setsid stdbuf -oL -eL "${cmd[@]}" >"$RUNTIME_LOG" 2>&1 &
LAUNCH_PID=$!

sleep 2
if [ "$PROFILE" = "step10v21" ]; then
  wait_topic /camera/depth/image_raw 60 || die "深度流未启动"
  wait_topic /local_highres_cloud_v21 60 || die "局部点云未启动"
  wait_topic /local_highres_cloud_v21/stats 30 || die "统计话题未启动"
  log "[测试] 保持运行5分钟；统计：ros2 topic echo /local_highres_cloud_v21/stats --once --field data"
  log "[确认] /camera/depth/points 不应存在。"
else
  wait_topic /camera/color/image_raw 60 || die "彩色流未启动"
  wait_topic /camera/depth/image_raw 60 || die "深度流未启动"
  wait_node /rtabmap/rgbd_odometry_clean 30 || die "RTAB-Map节点启动失败，请查看 $RUNTIME_LOG"
  wait_topic /visual_odom_clean 90 || die "RTAB-Map未输出 /visual_odom_clean"
  sleep 1
  ros2 node list 2>/dev/null | grep -Fxq /rtabmap/rgbd_odometry_clean || \
    die "RTAB-Map节点已异常退出，请查看 $RUNTIME_LOG"
  log "[测试] 默认不开RViz和同步监控，避免大图额外订阅影响性能。"
  log "[统计] 停止后运行：./REPORT_STEP2C4_VISUAL_ODOM.sh"
  log "[QoS] 需要手动测图像频率时使用 --qos-reliability best_effort。"
fi

log "[运行] Ctrl+C关闭。实时日志如下："
tail -n 100 -F "$RUNTIME_LOG" &
TAIL_PID=$!
wait "$LAUNCH_PID"
