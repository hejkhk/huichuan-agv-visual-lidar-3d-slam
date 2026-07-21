#!/usr/bin/env bash
set -Eeo pipefail

PROFILE="${1:-}"
case "$PROFILE" in
  camera|visual_odom|wheel_imu|fusion|slam|pointcloud|filtered_cloud|octomap_odom|dual_map) ;;
  *)
    echo "用法: $0 {camera|visual_odom|wheel_imu|fusion|slam|pointcloud|filtered_cloud|octomap_odom|dual_map}" >&2
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
case "$PROFILE" in
  camera) need_camera=true ;;
  visual_odom) need_camera=true; need_rtabmap=true ;;
  wheel_imu) need_chassis=true; need_ekf=true ;;
  fusion) need_camera=true; need_rtabmap=true; need_chassis=true; need_ekf=true ;;
  slam) need_camera=true; need_rtabmap=true; need_chassis=true; need_ekf=true; need_lidar=true ;;
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

if is_true "$AUTO_BUILD" || [ ! -f "$INSTALL_BASE/setup.bash" ]; then
  log "[构建] 正在隔离构建 lidar_py（不覆盖原 open_all 构建）……"
  cd "$CACHE_WS"
  PYTHONNOUSERSITE=1 colcon --log-base "$LOG_BASE" build \
    --base-paths "$CACHE_SRC" \
    --build-base "$BUILD_BASE" \
    --install-base "$INSTALL_BASE" \
    --symlink-install \
    --packages-select lidar_py
fi
[ -f "$INSTALL_BASE/setup.bash" ] || die "构建结果不存在：$INSTALL_BASE/setup.bash"
# shellcheck disable=SC1090
source "$INSTALL_BASE/setup.bash"
check_pkg lidar_py

case "$PROFILE" in
  fusion|slam)
    if ! is_true "$CAMERA_TF_CONFIRMED"; then
      log ""
      log "[重要警告] CAMERA_TF_CONFIRMED=false"
      log "  当前会用配置文件中的相机外参：x=$CAMERA_X y=$CAMERA_Y z=$CAMERA_Z"
      log "  STEP4/STEP5 的融合结果只有在 base_link→camera_link 外参正确时才可信。"
      log "  请修改：$ENV_FILE"
      log ""
    fi
    ;;
  filtered_cloud|octomap_odom|dual_map)
    is_true "$CAMERA_TF_CONFIRMED" || die "STEP7/STEP8/STEP9 必须先填写正确相机外参并设置 CAMERA_TF_CONFIRMED=true：$ENV_FILE"
    ;;
esac

log "============================================================"
log "  视觉 + 2D 雷达 + 三维体素地图分步测试"
log "  Profile      : $PROFILE"
log "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
log "  RViz         : $USE_RVIZ"
$need_chassis && log "  STM32        : $CHASSIS_PORT"
$need_lidar && log "  LiDAR        : $LIDAR_PORT @ $LIDAR_BAUD"
$need_camera && log "  RGB/Depth    : ${COLOR_WIDTH}x${COLOR_HEIGHT}@${COLOR_FPS} / ${DEPTH_WIDTH}x${DEPTH_HEIGHT}@${DEPTH_FPS}"
$need_pointcloud && log "  Raw cloud    : $POINT_CLOUD_TOPIC"
$need_cloud_filter && log "  Filter cloud : $FILTERED_CLOUD_TOPIC @ ${CLOUD_MAX_RATE_HZ}Hz, voxel=${CLOUD_VOXEL_SIZE}m"
$need_octomap && log "  OctoMap      : resolution=${OCTOMAP_RESOLUTION}m, max_range=${OCTOMAP_MAX_RANGE}m"
log "  Runtime log  : $RUNTIME_LOG"
log "============================================================"

cmd=(ros2 launch lidar_py visual_laser_slam.launch.py
  "profile:=$PROFILE"
  "launch_rviz:=$USE_RVIZ"
  "color_width:=$COLOR_WIDTH" "color_height:=$COLOR_HEIGHT" "color_fps:=$COLOR_FPS"
  "depth_width:=$DEPTH_WIDTH" "depth_height:=$DEPTH_HEIGHT" "depth_fps:=$DEPTH_FPS"
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
