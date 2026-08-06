#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$ROOT_DIR/visual_laser_slam/dual_resolution_3d.env"
SOURCE_WS="$ROOT_DIR/lidar/chapt1_ws"
CACHE_WS="${DUAL_3D_BUILD_ROOT:-$HOME/.cache/huichuan_agv_dual_3d_humble_ws}"
BUILD_BASE="$CACHE_WS/build"
INSTALL_BASE="$CACHE_WS/install"
LOG_BASE="$CACHE_WS/log"
RUN_STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_DIR="$ROOT_DIR/SLAM_Log/dual_3d_$RUN_STAMP"
RUNTIME_LOG="$RUN_DIR/runtime.log"
LAUNCH_PID=""
TAIL_PID=""
RVIZ_PID=""
OCTOMAP_OUTPUT_PATH=""
STOP_REASON="unexpected_shell_exit"
SHUTDOWN_REQUESTED=false
LOCAL_CLOUD_PIPELINE_VERSION="v6.34"
STACK_MODE="${DUAL_3D_STACK_MODE:-unknown}"
STACK_STATE_DIR="$HOME/.cache/huichuan_agv"
STACK_PID_FILE="$STACK_STATE_DIR/launcher.pid"
STACK_MODE_FILE="$STACK_STATE_DIR/mode"
STACK_ROOT_FILE="$STACK_STATE_DIR/project_root"
STACK_RUN_DIR_FILE="$STACK_STATE_DIR/run_dir"

log() {
  printf '%s\n' "$*"
  if [ -d "$RUN_DIR" ]; then
    printf '[launcher] %s\n' "$*" >>"$RUNTIME_LOG"
  fi
}
is_true() { case "${1,,}" in 1|true|yes|on) return 0;; *) return 1;; esac; }
deg_to_rad() { awk -v degrees="$1" 'BEGIN { printf "%.9f", degrees * 3.141592653589793 / 180.0 }'; }

clear_stack_state() {
  local registered_pid=""
  [ -r "$STACK_PID_FILE" ] && registered_pid="$(cat "$STACK_PID_FILE" 2>/dev/null || true)"
  if [ "$registered_pid" = "$$" ]; then
    rm -f -- "$STACK_PID_FILE" "$STACK_MODE_FILE" \
      "$STACK_ROOT_FILE" "$STACK_RUN_DIR_FILE"
  fi
}

process_cmdline() {
  local pid="$1"
  [ -r "/proc/$pid/cmdline" ] || return 1
  tr '\0' ' ' <"/proc/$pid/cmdline"
}

wait_process_exit() {
  local pid="$1" timeout_tenths="${2:-100}"
  local _
  for _ in $(seq 1 "$timeout_tenths"); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  return 1
}

stop_registered_stack() {
  local old_pid="" old_root="" old_run_dir="" old_launch_pid="" cmdline=""
  [ -r "$STACK_PID_FILE" ] || return 0
  old_pid="$(cat "$STACK_PID_FILE" 2>/dev/null || true)"
  old_root="$(cat "$STACK_ROOT_FILE" 2>/dev/null || true)"
  old_run_dir="$(cat "$STACK_RUN_DIR_FILE" 2>/dev/null || true)"

  if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
    cmdline="$(process_cmdline "$old_pid" 2>/dev/null || true)"
    if [ "$old_root" != "$ROOT_DIR" ] || \
        [[ "$cmdline" != *run_dual_resolution_3d_slam.sh* ]]; then
      die "PID state belongs to another live process ($old_pid): ${cmdline:-unknown command}"
    fi
    log "[preflight] Stopping the previous project launcher (pid=$old_pid)..."
    kill -INT "$old_pid" 2>/dev/null || true
    if ! wait_process_exit "$old_pid" 150; then
      log "[preflight] Previous launcher did not stop gracefully; sending TERM."
      kill -TERM "$old_pid" 2>/dev/null || true
      wait_process_exit "$old_pid" 50 || \
        die "Previous project launcher is still alive (pid=$old_pid)"
    fi
  elif [ -n "$old_run_dir" ] && [ -r "$old_run_dir/ros_launch.pid" ]; then
    old_launch_pid="$(cat "$old_run_dir/ros_launch.pid" 2>/dev/null || true)"
    if [[ "$old_launch_pid" =~ ^[0-9]+$ ]] && \
        kill -0 "$old_launch_pid" 2>/dev/null; then
      cmdline="$(process_cmdline "$old_launch_pid" 2>/dev/null || true)"
      if [[ "$cmdline" == *"ros2 launch lidar_py dual_resolution_3d_slam.launch.py"* ]]; then
        log "[preflight] Removing orphaned ROS launch group (pid=$old_launch_pid)..."
        kill -INT -- "-$old_launch_pid" 2>/dev/null || \
          kill -INT "$old_launch_pid" 2>/dev/null || true
        if ! wait_process_exit "$old_launch_pid" 100; then
          kill -TERM -- "-$old_launch_pid" 2>/dev/null || true
          wait_process_exit "$old_launch_pid" 30 || \
            die "Orphaned ROS launch group is still alive (pid=$old_launch_pid)"
        fi
      fi
    fi
  fi

  rm -f -- "$STACK_PID_FILE" "$STACK_MODE_FILE" \
    "$STACK_ROOT_FILE" "$STACK_RUN_DIR_FILE"
}

orbbec_video_nodes() {
  local node="" props=""
  for node in /dev/video*; do
    [ -e "$node" ] || continue
    props="$(udevadm info --query=property --name="$node" 2>/dev/null || true)"
    if printf '%s\n' "$props" | grep -Eiq \
        'ID_VENDOR_ID=2bc5|ID_VENDOR=.*orbbec|ID_MODEL=.*gemini'; then
      printf '%s\n' "$node"
    fi
  done
}

camera_ownership_report() {
  local nodes="" node="" holders="" entry=""
  nodes="$(orbbec_video_nodes)"
  if [ -z "$nodes" ]; then
    log "[camera] No Orbbec /dev/video* interface was found (USB/udev/cable check required)."
  else
    log "[camera] Orbbec video interfaces: $(printf '%s' "$nodes" | tr '\n' ' ')"
    while IFS= read -r node; do
      [ -n "$node" ] || continue
      holders="$(fuser "$node" 2>/dev/null || true)"
      if [ -n "$holders" ]; then
        log "[camera] $node is held by PID(s):$holders"
        for holder in $holders; do
          log "[camera]   pid=$holder cmd=$(process_cmdline "$holder" 2>/dev/null || printf unknown)"
        done
      fi
    done <<<"$nodes"
  fi
  if command -v pgrep >/dev/null 2>&1; then
    holders="$(pgrep -af \
      '[/]rclcpp_components/component_container(_mt)?.*__node:=camera_container|[/]orbbec_camera_node( |$)|[/](OrbbecViewer|OrbbecSDKViewer|cheese|guvcview)( |$)' \
      2>/dev/null || true)"
    if [ -n "$holders" ]; then
      log "[camera] Candidate camera processes:"
      while IFS= read -r entry; do
        [ -z "$entry" ] || log "[camera]   $entry"
      done <<<"$holders"
    fi
  fi
}

stop_stale_project_camera_container() {
  local pids="" pid="" cmdline=""
  command -v pgrep >/dev/null 2>&1 || return 0
  pids="$(pgrep -f \
    '[/]rclcpp_components/component_container(_mt)?.*__node:=camera_container.*__ns:=/camera' \
    2>/dev/null || true)"
  [ -n "$pids" ] || return 0

  for pid in $pids; do
    cmdline="$(process_cmdline "$pid" 2>/dev/null || true)"
    log "[camera] Stopping stale project camera container pid=$pid: $cmdline"
    kill -TERM "$pid" 2>/dev/null || true
  done
  for pid in $pids; do
    if ! wait_process_exit "$pid" 50; then
      log "[camera] Stale camera container pid=$pid ignored TERM; sending KILL."
      kill -KILL "$pid" 2>/dev/null || true
      wait_process_exit "$pid" 20 || \
        die "Stale camera container could not be stopped (pid=$pid)"
    fi
  done
}

prepare_camera_ownership() {
  local holders="" camera_processes=""
  stop_stale_project_camera_container
  if command -v fuser >/dev/null 2>&1; then
    while IFS= read -r node; do
      [ -n "$node" ] || continue
      holders="$holders $(fuser "$node" 2>/dev/null || true)"
    done < <(orbbec_video_nodes)
  fi
  holders="$(printf '%s\n' "$holders" | xargs 2>/dev/null || true)"
  if [ -n "$holders" ]; then
    camera_ownership_report
    die "Gemini2 is already in use. Close the listed camera program before starting."
  fi

  if command -v pgrep >/dev/null 2>&1; then
    camera_processes="$(pgrep -af \
      '[/]rclcpp_components/component_container(_mt)?.*__node:=camera_container|[/]orbbec_camera_node( |$)|[/](OrbbecViewer|OrbbecSDKViewer|cheese|guvcview)( |$)' \
      2>/dev/null || true)"
  fi
  if [ -n "$camera_processes" ]; then
    camera_ownership_report
    die "A camera process is still running. Stop the listed process before starting."
  fi

  # The ROS driver creates this POSIX shared-memory mutex. A hard-killed old
  # process can leave the file behind even though no process owns the camera.
  if [ -e /dev/shm/orbbec_device_lock ]; then
    rm -f /dev/shm/orbbec_device_lock 2>/dev/null || true
    if [ -e /dev/shm/orbbec_device_lock ]; then
      log "[WARNING] Stale /dev/shm/orbbec_device_lock could not be removed."
    else
      log "[camera] Removed stale Orbbec shared-memory lock."
    fi
  fi
  # libuvc may need a short interval after a previous process releases the USB
  # interface, especially when that process reset the device during shutdown.
  sleep 1
}

rviz_supervisor() {
  set +e
  local child="" status=0 restart_count=0
  trap '
    if [ -n "$child" ]; then
      kill -TERM -- "-$child" 2>/dev/null || kill -TERM "$child" 2>/dev/null || true
      wait "$child" 2>/dev/null || true
    fi
    exit 0
  ' INT TERM HUP
  while true; do
    setsid rviz2 -d "$rviz_config" >>"$RUNTIME_LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?

    child=""

    if [ "$status" -eq 0 ]; then
      log "[rviz] Window closed; mapping/navigation continues headless."
      while true; do sleep 3600; done
    fi

    restart_count=$((restart_count + 1))
    log "[WARNING] RViz exited with status $status; core ROS stack is still running."
    if [ "$restart_count" -ge 3 ]; then
      log "[WARNING] RViz crashed 3 times; continuing headless."
      while true; do sleep 3600; done
    fi
    log "[rviz] Restarting RViz in 2 seconds ($restart_count/3)..."
    sleep 2
  done
}

die() {
  SHUTDOWN_REQUESTED=true
  STOP_REASON="startup_or_runtime_check_failed: $*"
  log "[ERROR] $*"
  if [ -f "$RUNTIME_LOG" ]; then
    log "[ERROR] Last 100 runtime lines:"
    tail -n 100 "$RUNTIME_LOG" || true
  fi
  exit 1
}

cleanup() {
  local code=$?
  trap - EXIT INT TERM HUP
  if ! is_true "$SHUTDOWN_REQUESTED" && [ -n "$LAUNCH_PID" ] && \
      process_is_running "$LAUNCH_PID"; then
    log "[FATAL] Launcher shell exited unexpectedly (status=$code); ROS process group remains alive as a fail-safe."
    log "[FATAL] Stop it explicitly with: kill -INT -- -$LAUNCH_PID"
    return 0
  fi
  log ""
  log "[stop] Trigger: $STOP_REASON (status=$code)"
  log "[stop] Gracefully stopping RTAB-Map database and ROS nodes..."
  [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null || true
  if is_true "${USE_OCTOMAP:-true}" && is_true "${SAVE_OCTOMAP_ON_EXIT:-true}" && \
      [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    mkdir -p "$(dirname "$OCTOMAP_OUTPUT_PATH")"
    log "[stop] Saving OctoMap: $OCTOMAP_OUTPUT_PATH"
    if timeout 20s ros2 run octomap_server octomap_saver_node --ros-args \
        -p "octomap_path:=$OCTOMAP_OUTPUT_PATH" \
        -r octomap_binary:=/rtabmap_3d/octomap_binary >/dev/null 2>&1; then
      log "[stop] OctoMap saved."
    else
      log "[WARNING] OctoMap save timed out or failed; ROS shutdown will continue."
    fi
  fi
  if [ -n "$RVIZ_PID" ]; then
    kill -TERM "$RVIZ_PID" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "$RVIZ_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      sleep 1
    fi
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  fi
  [ -n "$LAUNCH_PID" ] && wait "$LAUNCH_PID" 2>/dev/null || true
  clear_stack_state
  log "[stop] Complete. RTAB-Map database: $DATABASE_PATH"
  log "[stop] Runtime log: $RUNTIME_LOG"
  exit "$code"
}
trap cleanup EXIT
trap 'SHUTDOWN_REQUESTED=true; STOP_REASON="SIGINT_or_Ctrl+C"; exit 130' INT
trap 'SHUTDOWN_REQUESTED=true; STOP_REASON="SIGTERM"; exit 143' TERM
# Some terminal emulators transiently send HUP while RViz/Ogre restarts. The
# ROS launch process owns the safety chain, so a terminal HUP must not kill it.
trap 'log "[monitor] Ignored SIGHUP; use Ctrl+C for an intentional shutdown."' HUP

[ -f "$ENV_FILE" ] || die "Missing $ENV_FILE"
# shellcheck disable=SC1090
source "$ENV_FILE"

# Wrapper-only overrides are applied after sourcing the shared environment.
# This prevents Bash assignments in dual_resolution_3d.env from silently
# replacing the navigation profile's session and safety choices.
RTABMAP_DATABASE="${DUAL_3D_DATABASE:-${RTABMAP_DATABASE:-maps/rtabmap_3d/rtabmap_v4_color.db}}"
RESET_GLOBAL_3D_MAP="${DUAL_3D_RESET_GLOBAL_MAP:-${RESET_GLOBAL_3D_MAP:-false}}"
REQUIRE_DEPTH_BASELINE_FOR_PS2="${DUAL_3D_REQUIRE_DEPTH_BASELINE_FOR_PS2:-${REQUIRE_DEPTH_BASELINE_FOR_PS2:-true}}"
JETSON_PLATFORM=false
if [ -f /etc/nv_tegra_release ]; then
  JETSON_PLATFORM=true
elif [ -r /proc/device-tree/model ] && \
    tr -d '\0' </proc/device-tree/model 2>/dev/null | grep -qi 'NVIDIA.*Jetson'; then
  JETSON_PLATFORM=true
fi
if is_true "$JETSON_PLATFORM"; then
  DEFAULT_RTABMAP_RATE=1.0
  DEFAULT_RTABMAP_THREADS=1
else
  DEFAULT_RTABMAP_RATE=2.0
  DEFAULT_RTABMAP_THREADS=2
fi
RTABMAP_RATE="${DUAL_3D_RTABMAP_RATE:-${RTABMAP_RATE:-$DEFAULT_RTABMAP_RATE}}"
RTABMAP_THREADS="${DUAL_3D_RTABMAP_THREADS:-${RTABMAP_THREADS:-$DEFAULT_RTABMAP_THREADS}}"
ENABLE_VISUAL_FUSION="${DUAL_3D_ENABLE_VISUAL_FUSION:-${ENABLE_VISUAL_FUSION:-false}}"
ENABLE_STVL="${DUAL_3D_ENABLE_STVL:-${ENABLE_STVL:-true}}"
ENABLE_FIXED_SCAN_FILTER="${ENABLE_FIXED_SCAN_FILTER:-true}"
CARTOGRAPHER_CONFIG="${DUAL_3D_CARTOGRAPHER_CONFIG:-${CARTOGRAPHER_CONFIG:-cartographer_2d_v9_tightened.lua}}"
if is_true "$ENABLE_VISUAL_FUSION"; then
  CHASSIS_PUBLISH_TF=false
  CARTOGRAPHER_ODOM_TOPIC=/odometry/filtered
else
  CHASSIS_PUBLISH_TF=true
  CARTOGRAPHER_ODOM_TOPIC=/odom
fi
# RTAB-Map always consumes Cartographer's corrected map-frame pose. The EKF is
# only an odom-frame motion predictor for Cartographer and must not detach the
# persistent 3D graph from the corrected 2D map.
RTABMAP_ODOM_TOPIC=/cartographer_pose_odom
if is_true "$ENABLE_STVL"; then
  NAV_COSTMAP_OVERRIDE="$SOURCE_WS/src/lidar_py/config/nav2_dual_3d_stvl_override.yaml"
else
  # Perception fallback only: keep the same Smac/DWB controller chain and use
  # the base 2D obstacle layers. Never revive the retired MPPI controller just
  # because STVL was disabled for diagnosis.
  NAV_COSTMAP_OVERRIDE="$SOURCE_WS/src/lidar_py/config/nav2_auto_mapping_humble.yaml"
fi

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
# STEP11 is an interactive validation launcher: always open its RViz profile.
USE_RVIZ=true
USE_RTABMAP_VIZ="${USE_RTABMAP_VIZ:-false}"
ENABLE_RTABMAP="${ENABLE_RTABMAP:-true}"
ENABLE_NAVIGATION="${DUAL_3D_ENABLE_NAVIGATION:-${ENABLE_NAVIGATION:-false}}"
LOCALIZATION_MODE="${DUAL_3D_LOCALIZATION_MODE:-false}"
LOCALIZATION_MAP_YAML="${DUAL_3D_LOCALIZATION_MAP_YAML:-}"
LOCALIZATION_PBSTREAM="${DUAL_3D_LOCALIZATION_PBSTREAM:-}"
RVIZ_CONFIG_FILE="${DUAL_3D_RVIZ_CONFIG_FILE:-$SOURCE_WS/src/lidar_py/rviz/dual_resolution_3d_slam.rviz}"
# A direct call to this runner with navigation enabled must not silently use
# the mapping-only loop-closure threshold. An explicit DUAL_3D override remains
# available for controlled experiments.
if is_true "$ENABLE_NAVIGATION"; then
  if is_true "$LOCALIZATION_MODE"; then
    CARTOGRAPHER_CONFIG="cartographer_2d_localization.lua"
  else
    CARTOGRAPHER_CONFIG="${DUAL_3D_CARTOGRAPHER_CONFIG:-cartographer_2d_v9_nav_guarded.lua}"
  fi
  # Keep NavigateToPose inactive while host Twist output is gated. PS2 remains
  # available during this staged startup; Nav2 activates only after mapping,
  # perception and the hard collision input have produced data.
  NAV_AUTOSTART=false
else
  NAV_AUTOSTART=true
fi

if is_true "$LOCALIZATION_MODE"; then
  [ -f "$LOCALIZATION_MAP_YAML" ] || \
    die "Localization map YAML not found: ${LOCALIZATION_MAP_YAML:-unset}"
  [ -f "$LOCALIZATION_PBSTREAM" ] || \
    die "Localization pbstream not found: ${LOCALIZATION_PBSTREAM:-unset}"
fi
AUTO_BUILD="${AUTO_BUILD:-true}"
LIDAR_BAUD="${LIDAR_BAUD:-115200}"
DATABASE_SIGNATURE_PATH=""
CAMERA_ROLL="$(deg_to_rad "${CAMERA_ROLL_DEG:-0.0}")"
CAMERA_PITCH="$(deg_to_rad "${CAMERA_PITCH_DEG:-0.0}")"
CAMERA_YAW="$(deg_to_rad "${CAMERA_YAW_DEG:-0.0}")"
LOCAL_GROUND_FILTER_EFFECTIVE="${LOCAL_GROUND_FILTER:-true}"
if is_true "$LOCAL_GROUND_FILTER_EFFECTIVE" && \
    ! is_true "${CAMERA_GROUND_CALIBRATED:-false}"; then
  # A fixed z-band is only meaningful after roll, pitch and camera height have
  # placed the physical floor at base_link z=0.
  LOCAL_GROUND_FILTER_EFFECTIVE=false
fi
if [[ "$RTABMAP_DATABASE" = /* ]]; then
  DATABASE_PATH="$RTABMAP_DATABASE"
else
  DATABASE_PATH="$ROOT_DIR/$RTABMAP_DATABASE"
fi
if [[ "${OCTOMAP_SAVE_PATH:-maps_3d/octomap_latest.bt}" = /* ]]; then
  OCTOMAP_OUTPUT_PATH="${OCTOMAP_SAVE_PATH:-maps_3d/octomap_latest.bt}"
else
  OCTOMAP_OUTPUT_PATH="$ROOT_DIR/${OCTOMAP_SAVE_PATH:-maps_3d/octomap_latest.bt}"
fi
DATABASE_SIGNATURE_PATH="${DATABASE_PATH}.config"
DATABASE_COLOR_V4_MARKER="${DATABASE_PATH}.native_rgb_v4"
CALIBRATION_RESTART_MARKER="$ROOT_DIR/visual_laser_slam/.camera_calibration_restart_required"

# Reaching this point means this process sourced the latest calibration env.
# A marker written by a previous calibration run is now acknowledged.
if [ -f "$CALIBRATION_RESTART_MARKER" ]; then
  log "[calibration] Applying updated camera extrinsic from $ENV_FILE"
  rm -f -- "$CALIBRATION_RESTART_MARKER"
fi

usb_attr() {
  local dev="$1" attr="$2" path
  path="$(readlink -f "/sys/class/tty/${dev##*/}/device" 2>/dev/null || true)"
  while [ -n "$path" ] && [ "$path" != "/" ]; do
    if [ -r "$path/$attr" ]; then
      tr -d '\r\n' < "$path/$attr"
      return 0
    fi
    path="${path%/*}"
  done
  return 1
}

auto_detect_ports() {
  local dev product
  for dev in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$dev" ] || continue
    product="$(usb_attr "$dev" idProduct 2>/dev/null || true)"
    case "${product,,}" in
      7523) [ -n "${CHASSIS_PORT:-}" ] || CHASSIS_PORT="$dev" ;;
      55d4) [ -n "${LIDAR_PORT:-}" ] || LIDAR_PORT="$dev" ;;
    esac
  done
  [ -n "${CHASSIS_PORT:-}" ] || CHASSIS_PORT=/dev/ttyUSB0
  if [ -z "${LIDAR_PORT:-}" ]; then
    if [ -e /dev/ttyACM0 ]; then LIDAR_PORT=/dev/ttyACM0; else LIDAR_PORT=/dev/ttyUSB1; fi
  fi
}

wait_topic() {
  local topic="$1" timeout_sec="${2:-45}"
  kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
  # Keep exactly one DDS participant alive for the whole wait. The old loop
  # created a new `ros2 topic list` and `echo` participant every second, which
  # could exhaust CycloneDDS while the 2D/3D stack was still starting.
  if timeout --signal=TERM --kill-after=2s "${timeout_sec}s" \
      ros2 topic echo "$topic" --once \
        --qos-reliability best_effort >/dev/null 2>&1; then
    log "[ready] data $topic"
    return 0
  fi
  return 1
}

wait_boolean_true() {
  local topic="$1" timeout_sec="${2:-120}" output=""
  local deadline=$((SECONDS + timeout_sec))
  # /localization_ready is latched false at startup. Reading only one sample
  # returns that valid false immediately, so keep sampling until verification
  # publishes true or the actual deadline expires.
  while [ "$SECONDS" -lt "$deadline" ]; do
    output="$(timeout --signal=TERM --kill-after=1s 3s \
      ros2 topic echo "$topic" --once \
        --qos-reliability reliable \
        --qos-durability transient_local 2>/dev/null || true)"
    if printf '%s\n' "$output" | grep -Eiq 'data:[[:space:]]*true'; then
      return 0
    fi
    kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
    sleep 1
  done
  return 1
}

wait_transient_topic() {
  local topic="$1" timeout_sec="${2:-45}"
  timeout --signal=TERM --kill-after=2s "${timeout_sec}s" \
    ros2 topic echo "$topic" --once \
      --qos-reliability reliable \
      --qos-durability transient_local >/dev/null 2>&1
}

wait_graph_name() {
  local graph_kind="$1" name="$2" timeout_sec="${3:-45}" output=""
  local deadline=$((SECONDS + timeout_sec))
  while [ "$SECONDS" -lt "$deadline" ]; do
    output="$(timeout --signal=TERM --kill-after=1s 3s \
      ros2 "$graph_kind" list 2>/dev/null || true)"
    if printf '%s\n' "$output" | grep -Fxq "$name"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

require_lifecycle_active() {
  local node="$1" state
  kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
  state="$(timeout --signal=TERM --kill-after=2s 8s \
    ros2 lifecycle get "$node" 2>&1 || true)"
  if printf '%s\n' "$state" | grep -Eiq '(^|[[:space:]])active[[:space:]]*\[3\]'; then
    log "[ready] lifecycle $node = active"
    return 0
  fi
  log "[ERROR] lifecycle $node is not active: ${state:-no response}"
  return 1
}

wait_lifecycle_active() {
  local node="$1" timeout_sec="${2:-45}" state=""
  local deadline=$((SECONDS + timeout_sec))
  while [ "$SECONDS" -lt "$deadline" ]; do
    state="$(timeout --signal=TERM --kill-after=1s 5s \
      ros2 lifecycle get "$node" 2>&1 || true)"
    if printf '%s\n' "$state" | \
        grep -Eiq '(^|[[:space:]])active[[:space:]]*\[3\]'; then
      log "[ready] lifecycle $node = active"
      return 0
    fi
    sleep 1
  done
  log "[ERROR] lifecycle $node did not become active: ${state:-no response}"
  return 1
}

wait_topic_publisher() {
  local topic="$1" timeout_sec="${2:-30}" info=""
  local deadline=$((SECONDS + timeout_sec))
  while [ "$SECONDS" -lt "$deadline" ]; do
    kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
    info="$(timeout --signal=TERM --kill-after=1s 3s \
      ros2 topic info "$topic" 2>/dev/null || true)"
    if printf '%s\n' "$info" | \
        grep -Eq 'Publisher count:[[:space:]]*[1-9][0-9]*'; then
      log "[ready] publisher $topic"
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_lidar_data() {
  local topic="$1" timeout_sec="${2:-20}"
  if wait_topic "$topic" "$timeout_sec"; then
    return 0
  fi
  # In navigation mode the C++ collision gate is a long-lived subscriber to
  # the filtered scan. Its fresh scan_alive report is stronger evidence than
  # a short-lived CLI probe, and avoids killing a healthy stack if ros2cli
  # itself has a transient discovery failure.
  if is_true "${ENABLE_NAVIGATION:-false}" && \
      grep -Eq \
        'COLLISION_GATE .*scan_alive=true|FUSION_STATUS .*scan_alive=True' \
        "$RUNTIME_LOG"; then
    log "[ready] data $topic (confirmed by in-process 2D collision gate)"
    return 0
  fi
  return 1
}

process_is_running() {
  local pid="$1" state
  kill -0 "$pid" 2>/dev/null || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
  # If ps is momentarily unavailable, kill -0 is still stronger evidence that
  # the owned child exists. Only a confirmed zombie is considered finished.
  [ -n "$state" ] || return 0
  [[ "$state" != Z* ]]
}

check_pkg() {
  ros2 pkg prefix "$1" >/dev/null 2>&1 || die "Missing ROS package: $1"
}

ensure_orbbec_camera() {
  local wrapper_root="$SOURCE_WS/src/OrbbecSDK_ROS2"
  if ros2 pkg prefix orbbec_camera >/dev/null 2>&1; then
    return 0
  fi

  [ -f "$wrapper_root/orbbec_camera/package.xml" ] || \
    die "Missing bundled Orbbec wrapper: $wrapper_root/orbbec_camera"
  [ -f "$wrapper_root/orbbec_camera_msgs/package.xml" ] || \
    die "Missing bundled Orbbec messages: $wrapper_root/orbbec_camera_msgs"

  log "[build] orbbec_camera is not installed in the current overlay."
  log "[build] Building the bundled Gemini2 Humble wrapper (one worker)..."
  reset_cached_package orbbec_camera
  reset_cached_package orbbec_camera_msgs
  if ! PYTHONNOUSERSITE=1 colcon --log-base "$LOG_BASE" build \
      --base-paths \
        "$wrapper_root/orbbec_camera_msgs" \
        "$wrapper_root/orbbec_camera" \
      --build-base "$BUILD_BASE" \
      --install-base "$INSTALL_BASE" \
      --symlink-install \
      --parallel-workers 1 \
      --packages-select orbbec_camera_msgs orbbec_camera \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      2>&1 | tee -a "$RUNTIME_LOG"; then
    die "Bundled orbbec_camera build failed. Check Orbbec SDK v2.9.3 ARM64 and rosdep dependencies."
  fi
  [ -f "$INSTALL_BASE/setup.bash" ] || \
    die "Orbbec build did not create $INSTALL_BASE/setup.bash"
  # shellcheck disable=SC1090
  source "$INSTALL_BASE/setup.bash"
  ros2 pkg prefix orbbec_camera >/dev/null 2>&1 || \
    die "orbbec_camera is still unavailable after a successful build"
  log "[ready] Bundled orbbec_camera is available in the isolated overlay."
}

reset_cached_package() {
  local package="$1" path
  [ -n "$CACHE_WS" ] && [ "$CACHE_WS" != "/" ] || \
    die "Refusing to clean an invalid build cache root: ${CACHE_WS:-<empty>}"
  for path in "$BUILD_BASE/$package" "$INSTALL_BASE/$package"; do
    case "$path" in
      "$CACHE_WS"/*) rm -rf -- "$path" ;;
      *) die "Refusing to clean path outside build cache: $path" ;;
    esac
  done
}

prepare_humble_build_cache() {
  local source_root="$1"
  local marker="$CACHE_WS/.humble-build-environment-v3"
  local legacy_marker="$CACHE_WS/.humble-build-environment-v2"
  local ubuntu_version="unknown"
  local python_path="${CAR_SYSTEM_PYTHON:-/usr/bin/python3}"
  local python_version="unknown"
  local source_path="unknown"
  local expected="" previous="" normalized_previous="" path=""

  [ -n "$CACHE_WS" ] && [ "$CACHE_WS" != "/" ] || \
    die "Refusing to use an invalid build cache root: ${CACHE_WS:-<empty>}"
  source_path="$(readlink -f "$source_root" 2>/dev/null || true)"
  [ -n "$source_path" ] || die "Cannot resolve build source: $source_root"
  if [ -r /etc/os-release ]; then
    ubuntu_version="$(. /etc/os-release; printf '%s:%s' "${ID:-unknown}" "${VERSION_ID:-unknown}")"
  fi
  [ -x "$python_path" ] || die "System Python is unavailable: $python_path"
  python_version="$($python_path -c 'import platform; print(platform.python_version())')"
  expected="schema=humble-v3
ros=humble
os=$ubuntu_version
python=$python_path:$python_version
source=$source_path"

  if [ -f "$marker" ]; then
    previous="$(cat "$marker")"
  elif [ -f "$legacy_marker" ]; then
    previous="$(cat "$legacy_marker")"
    normalized_previous="$(printf '%s\n' "$previous" | \
      sed 's/^schema=humble-v2$/schema=humble-v3/' | \
      grep -v '^revision=')"
    if [ "$normalized_previous" = "$expected" ]; then
      previous="$expected"
      printf '%s\n' "$expected" >"$marker"
      log "[build] Migrated the compatible build cache; no full rebuild needed."
    fi
  fi
  if [ "$previous" != "$expected" ]; then
    if [ -n "$previous" ]; then
      log "[build] Humble build environment changed; invalidating stale cache."
    else
      log "[build] Initializing the Humble build-cache contract."
    fi
    for path in "$BUILD_BASE" "$INSTALL_BASE" "$LOG_BASE"; do
      case "$path" in
        "$CACHE_WS"/*) rm -rf -- "$path" ;;
        *) die "Refusing to clean path outside build cache: $path" ;;
      esac
    done
    mkdir -p "$CACHE_WS"
    printf '%s\n' "$expected" >"$marker"
  fi
}

source_tree_fingerprint() {
  local source_dir="$1"
  find "$source_dir" -type f \
      ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 | \
    LC_ALL=C sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'
}

wait_parameter_value() {
  local node="$1" parameter="$2" expected="$3" timeout_sec="${4:-30}"
  local failure_level="${5:-ERROR}"
  local value=""
  local deadline=$((SECONDS + timeout_sec))
  while [ "$SECONDS" -lt "$deadline" ]; do
    kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
    value="$(timeout --signal=TERM --kill-after=1s 3s \
      ros2 --no-daemon param get "$node" "$parameter" 2>/dev/null || true)"
    if printf '%s\n' "$value" | grep -Fq "$expected"; then
      log "[ready] $node $parameter=$expected"
      return 0
    fi
    sleep 1
  done
  log "[$failure_level] $node $parameter expected=$expected actual=${value:-unavailable}"
  return 1
}

verify_navigation_source_contract() {
  local controller_override
  controller_override="$SOURCE_WS/src/lidar_py/config/nav2_dual_3d_dwb_humble_override.yaml"
  python3 - "$NAV_COSTMAP_OVERRIDE" "$controller_override" "$ENABLE_STVL" <<'PY'
import pathlib
import sys

import yaml

costmap_path = pathlib.Path(sys.argv[1])
controller_path = pathlib.Path(sys.argv[2])
stvl_enabled = sys.argv[3].strip().lower() in {"1", "true", "yes", "on"}


def load(path):
    if not path.is_file():
        raise SystemExit(f"missing navigation parameter file: {path}")
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise SystemExit(f"navigation parameter file is empty: {path}")
    return document


costmap = load(costmap_path)
controller = load(controller_path)
for label in ("local_costmap", "global_costmap"):
    try:
        cost_params = costmap[label][label]["ros__parameters"]
        controller_params = controller[label][label]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise SystemExit(
            f"{label} parameter tree is incomplete: {exc}") from exc

    if stvl_enabled:
        lethal = int(cost_params.get("lethal_cost_threshold", -1))
        if lethal != 65:
            raise SystemExit(
                f"{label} lethal_cost_threshold expected 65, got {lethal}")
        if "static_layer" not in cost_params.get("plugins", []):
            raise SystemExit(
                f"{label} does not include the persistent Cartographer static layer")
        static_layer = cost_params.get("static_layer", {})
        if static_layer.get("footprint_clearing_enabled") is not True:
            raise SystemExit(
                f"{label} must clear stale static cells below the robot footprint")
        if label == "global_costmap" and (
                "visual_wall_global_stvl_layer" in cost_params.get("plugins", [])):
            raise SystemExit(
                "global RTAB wall projection duplicates Cartographer walls")

    # The controller override is loaded before the costmap override. If the
    # latter supplies an inflation radius, it is the effective final value.
    controller_inflation = controller_params.get("inflation_layer", {})
    costmap_inflation = cost_params.get("inflation_layer", {})
    radius = costmap_inflation.get(
        "inflation_radius",
        controller_inflation.get("inflation_radius", -1.0),
    )
    if abs(float(radius) - 0.49) > 1e-6:
        raise SystemExit(
            f"{label} inflation_radius expected 0.49, got {radius}")
    scaling = costmap_inflation.get(
        "cost_scaling_factor",
        controller_inflation.get("cost_scaling_factor", -1.0),
    )
    if abs(float(scaling) - 14.0) > 1e-6:
        raise SystemExit(
            f"{label} cost_scaling_factor expected 14.0, got {scaling}")

try:
    follow_path = controller["controller_server"]["ros__parameters"][
        "FollowPath"]
    planner = controller["planner_server"]["ros__parameters"]["GridBased"]
except (KeyError, TypeError) as exc:
    raise SystemExit(f"DWB navigation parameter tree is incomplete: {exc}") from exc
if planner.get("allow_unknown") is not False:
    raise SystemExit(
        "Smac must reject unknown space outside the observed map")
if follow_path.get("plugin") != (
        "nav2_rotation_shim_controller::RotationShimController"):
    raise SystemExit("FollowPath must use the Humble RotationShim controller")
if follow_path.get("primary_controller") != "dwb_core::DWBLocalPlanner":
    raise SystemExit("FollowPath primary controller must be DWB")
controller_params = controller["controller_server"]["ros__parameters"]
no_shim = controller_params.get("FollowPathNoShim", {})
if no_shim.get("plugin") != "dwb_core::DWBLocalPlanner":
    raise SystemExit("FollowPathNoShim DWB fallback is missing")
for key, expected in (
        ("max_vel_x", 0.20),
        ("max_vel_theta", 0.209),
        ("angular_dist_threshold", 2.80)):
    value = follow_path.get(key, -1.0)
    if abs(float(value) - expected) > 1e-6:
        raise SystemExit(
            f"DWB/RotationShim {key} expected {expected}, got {value}")
if float(no_shim.get("min_vel_x", 0.0)) >= 0.0:
    raise SystemExit("FollowPathNoShim must retain bounded reverse sampling")
PY
}

release_motion_interlock() {
  local response=""
  wait_graph_name service /robot/set_system_ready 15 || {
    log "[ERROR] chassis readiness service /robot/set_system_ready is unavailable"
    return 1
  }
  response="$(timeout --signal=TERM --kill-after=1s 8s \
    ros2 service call /robot/set_system_ready std_srvs/srv/SetBool \
      '{data: true}' 2>&1 || true)"
  if ! printf '%s\n' "$response" | \
      grep -Eq 'success[=:][[:space:]]*(True|true)|system_ready=true'; then
    log "[ERROR] chassis rejected readiness handshake: ${response:-no response}"
    return 1
  fi
  log "[ready] Confirmed /robot/set_system_ready=true"
}

start_navigation_lifecycle() {
  local response=""
  wait_graph_name service /lifecycle_manager_navigation/manage_nodes 30 || {
    log "[ERROR] Nav2 lifecycle manager service is unavailable"
    return 1
  }
  log "[nav2] Core mapping/perception is live; activating Nav2..."
  response="$(timeout --signal=TERM --kill-after=2s 75s \
    ros2 service call /lifecycle_manager_navigation/manage_nodes \
      nav2_msgs/srv/ManageLifecycleNodes '{command: 0}' 2>&1 || true)"
  if ! printf '%s\n' "$response" | \
      grep -Eiq 'success[=:][[:space:]]*(True|true)'; then
    log "[ERROR] Nav2 lifecycle startup failed: ${response:-no response}"
    return 1
  fi
  log "[ready] Nav2 lifecycle manager completed startup"
}

rtabmap_database_healthy() {
  local database="$1"
  python3 - "$database" <<'PY'
import sqlite3
import sys

path = sys.argv[1]
try:
    con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    quick = con.execute("PRAGMA quick_check").fetchone()
    if not quick or str(quick[0]).lower() != "ok":
        print(f"SQLite quick_check failed: {quick}", file=sys.stderr)
        raise SystemExit(1)

    tables = {
        row[0].lower(): row[0]
        for row in con.execute("SELECT name FROM sqlite_master WHERE type='table'")
    }
    word_table = tables.get("word")
    ref_table = tables.get("map_node_word")
    if word_table and ref_table:
        words = con.execute(f'SELECT COUNT(*) FROM "{word_table}"').fetchone()[0]
        refs = con.execute(f'SELECT COUNT(*) FROM "{ref_table}"').fetchone()[0]
        if refs > 0 and words == 0:
            print(
                f"RTAB-Map dictionary is inconsistent: {refs} references but 0 words",
                file=sys.stderr,
            )
            raise SystemExit(1)
    con.close()
except (sqlite3.DatabaseError, OSError) as exc:
    print(f"RTAB-Map database check failed: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
}

auto_detect_ports
mkdir -p "$RUN_DIR" "$(dirname "$DATABASE_PATH")" "$CACHE_WS"
: >"$RUNTIME_LOG"
stop_registered_stack
printf '%s\n' "$$" >"$RUN_DIR/launcher.pid"
mkdir -p "$STACK_STATE_DIR"
printf '%s\n' "$$" >"$STACK_PID_FILE.tmp"
printf '%s\n' "$STACK_MODE" >"$STACK_MODE_FILE.tmp"
printf '%s\n' "$ROOT_DIR" >"$STACK_ROOT_FILE.tmp"
printf '%s\n' "$RUN_DIR" >"$STACK_RUN_DIR_FILE.tmp"
mv -f "$STACK_PID_FILE.tmp" "$STACK_PID_FILE"
mv -f "$STACK_MODE_FILE.tmp" "$STACK_MODE_FILE"
mv -f "$STACK_ROOT_FILE.tmp" "$STACK_ROOT_FILE"
mv -f "$STACK_RUN_DIR_FILE.tmp" "$STACK_RUN_DIR_FILE"

# Check camera ownership before package checks or incremental compilation, so
# a busy Gemini2 fails quickly with the process/device details in runtime.log.
prepare_camera_ownership

[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble was not found at /opt/ros/humble"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
[ "${ROS_DISTRO:-}" = "humble" ] || die "Expected ROS_DISTRO=humble, got ${ROS_DISTRO:-unset}"
if [ -n "${ORBBEC_SETUP:-}" ]; then
  [ -f "$ORBBEC_SETUP" ] || die "ORBBEC_SETUP does not exist: $ORBBEC_SETUP"
  # shellcheck disable=SC1090
  source "$ORBBEC_SETUP"
fi
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_CONFIG="$ROOT_DIR/visual_laser_slam/cyclonedds_dual_3d.xml"
[ -f "$CYCLONEDDS_CONFIG" ] || die "Missing CycloneDDS config: $CYCLONEDDS_CONFIG"
export CYCLONEDDS_URI="${DUAL_3D_CYCLONEDDS_URI:-file://$CYCLONEDDS_CONFIG}"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

command -v ros2 >/dev/null 2>&1 || die "ros2 is unavailable"
command -v colcon >/dev/null 2>&1 || die "colcon is unavailable"
command -v setsid >/dev/null 2>&1 || die "setsid is unavailable"
prepare_humble_build_cache "$SOURCE_WS/src"

# A daemon started by an earlier run does not inherit this launcher's DDS
# profile. Restart it before any graph query so stale participant-index state
# cannot produce a false startup failure.
ros2 daemon stop >/dev/null 2>&1 || true
export ROS2CLI_NO_DAEMON=1

# Source cached install overlay so packages like orbbec_camera are visible
# to check_pkg before the incremental rebuild step.
if [ -f "$INSTALL_BASE/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$INSTALL_BASE/setup.bash"
fi

# A fresh clone, a renamed project directory or a build fingerprint change can
# legitimately clear the isolated install tree. Rebuild the bundled camera
# wrapper before package validation instead of failing with a misleading
# "Missing ROS package: orbbec_camera" message.
ensure_orbbec_camera

for pkg in orbbec_camera cartographer_ros laser_filters rtabmap_slam rtabmap_rviz_plugins octomap_server robot_state_publisher xacro rmw_cyclonedds_cpp depth_image_proc rclcpp_components; do
  check_pkg "$pkg"
done
if is_true "$ENABLE_NAVIGATION"; then
  for pkg in nav2_controller nav2_rotation_shim_controller dwb_core dwb_plugins dwb_critics nav2_costmap_2d nav2_bt_navigator nav2_smac_planner nav2_velocity_smoother; do
    check_pkg "$pkg"
  done
  if is_true "$USE_RVIZ"; then
    check_pkg nav2_rviz_plugins
  fi
  if is_true "${ENABLE_STVL:-true}"; then
    check_pkg spatio_temporal_voxel_layer
  fi
fi
if is_true "$LOCALIZATION_MODE"; then
  for pkg in cartographer_ros_msgs nav2_map_server nav2_lifecycle_manager; do
    check_pkg "$pkg"
  done
  python3 -c 'import scipy.ndimage' >/dev/null 2>&1 || \
    die "Missing python3-scipy. Install it once: sudo apt install python3-scipy"
fi
if is_true "$ENABLE_VISUAL_FUSION"; then
  check_pkg rtabmap_odom
  check_pkg robot_localization
fi
if is_true "$ENABLE_NAVIGATION"; then
  verify_navigation_source_contract || \
    die "Navigation YAML contract is invalid; Nav2 was not launched"
  log "[ready] Navigation source contract: static walls=65, robot start footprint=clear, unknown space=blocked, global RTAB wall duplication=off, inflation=0.49m/14.0"
fi

# RViz is intentionally owned by this outer runner and opened before colcon or
# the sensor stack. It can wait for the map/TF topics while the rest starts.
if is_true "$USE_RVIZ" && ! is_true "$LOCALIZATION_MODE"; then
  rviz_config="$RVIZ_CONFIG_FILE"
  [ -f "$rviz_config" ] || die "RViz profile is missing: $rviz_config"
  log "[rviz] Opening the 2D/3D view first; map and TF will appear when ROS is ready."
  rviz_supervisor &
  RVIZ_PID=$!
fi

if [ ! -e "$CHASSIS_PORT" ] || [ ! -e "$LIDAR_PORT" ]; then
  log "[serial] Configured ports are not both present; waiting up to 15 seconds for USB enumeration."
  configured_chassis="$CHASSIS_PORT"
  configured_lidar="$LIDAR_PORT"
  for _ in $(seq 1 15); do
    if [ -e "$configured_chassis" ] && [ -e "$configured_lidar" ]; then
      CHASSIS_PORT="$configured_chassis"
      LIDAR_PORT="$configured_lidar"
      break
    fi
    CHASSIS_PORT=""
    LIDAR_PORT=""
    auto_detect_ports
    [ -e "$CHASSIS_PORT" ] && [ -e "$LIDAR_PORT" ] && break
    sleep 1
  done
fi

for port in "$CHASSIS_PORT" "$LIDAR_PORT"; do
  [ -e "$port" ] || die "Serial port does not exist: $port"
  if [ ! -r "$port" ] || [ ! -w "$port" ]; then
    log "[serial] Granting temporary access to $port"
    sudo chmod a+rw "$port" || die "Cannot access $port"
  fi
done

# Do not silently steal the camera, serial ports or TF tree from another stack.
if ros2 node list 2>/dev/null | grep -Eq '/(cartographer_node|chassis_node|rtabmap_3d/rtabmap|depth_image_to_local_cloud_v21)$'; then
  die "A mapping stack is already running. Stop it before starting STEP11."
fi

MAP_CONFIG_SIGNATURE="$(printf '%s\n' \
  "camera_xyz=${CAMERA_X:-0.30},${CAMERA_Y:-0.0},${CAMERA_Z:-0.371}" \
  "camera_rpy_deg=${CAMERA_ROLL_DEG:-0.0},${CAMERA_PITCH_DEG:-0.0},${CAMERA_YAW_DEG:-0.0}" \
  "alignment=${CAMERA_DEPTH_REGISTRATION:-false},${CAMERA_ALIGN_MODE:-HW},${CAMERA_ALIGN_TARGET_STREAM:-COLOR}" \
  "color=${COLOR_WIDTH:-640}x${COLOR_HEIGHT:-480}@${COLOR_FPS:-15}" \
  "depth=${DEPTH_WIDTH:-640}x${DEPTH_HEIGHT:-400}@${DEPTH_FPS:-15}" \
  "global_3d_voxel=${GLOBAL_3D_VOXEL:-0.05}")"
MAP_CONFIG_SIGNATURE="${MAP_CONFIG_SIGNATURE}
rtab_pose_authority=cartographer_map_pose_v3
rtab_rgb_pipeline=native_rgb_depth_image_proc_register_v4
rtab_grid_filter=normals_k20_angle15_ground005_cluster010_min10_v1"

RESET_DATABASE_NOW=false
if is_true "${RESET_GLOBAL_3D_MAP:-false}"; then
  RESET_DATABASE_NOW=true
elif [ -f "$DATABASE_PATH" ] && [ ! -f "$DATABASE_COLOR_V4_MARKER" ]; then
  log "[database] Migrating to native-RGB registration V4; old black C2D nodes cannot be mixed."
  RESET_DATABASE_NOW=true
elif [ -f "$DATABASE_PATH" ] && ! rtabmap_database_healthy "$DATABASE_PATH"; then
  log "[database] Existing RTAB-Map database is corrupt or internally inconsistent."
  RESET_DATABASE_NOW=true
elif is_true "${AUTO_RESET_3D_MAP_ON_CONFIG_CHANGE:-true}" && [ -f "$DATABASE_PATH" ]; then
  if [ ! -f "$DATABASE_SIGNATURE_PATH" ] || [ "$(cat "$DATABASE_SIGNATURE_PATH")" != "$MAP_CONFIG_SIGNATURE" ]; then
    log "[database] Camera/alignment configuration changed; the old 3D map cannot be reused safely."
    RESET_DATABASE_NOW=true
  fi
fi

if is_true "$RESET_DATABASE_NOW" && [ -f "$DATABASE_PATH" ]; then
  backup="${DATABASE_PATH%.db}_backup_${RUN_STAMP}.db"
  mv "$DATABASE_PATH" "$backup"
  log "[database] Previous map archived as $backup"
fi
printf '%s' "$MAP_CONFIG_SIGNATURE" > "$DATABASE_SIGNATURE_PATH"
touch "$DATABASE_COLOR_V4_MARKER"

if is_true "$AUTO_BUILD" || [ ! -f "$INSTALL_BASE/setup.bash" ]; then
  log "[build] Building isolated STEP11 workspace..."
  local_cloud_source_fingerprint="$(source_tree_fingerprint \
    "$SOURCE_WS/src/local_depth_cloud_cpp")"
  local_cloud_fingerprint_marker="$CACHE_WS/.local-depth-cloud-source.sha256"
  previous_local_cloud_fingerprint=""
  [ -f "$local_cloud_fingerprint_marker" ] && \
    previous_local_cloud_fingerprint="$(cat "$local_cloud_fingerprint_marker")"
  local_cloud_cached_bin="$INSTALL_BASE/local_depth_cloud_cpp/lib/local_depth_cloud_cpp/depth_image_to_local_cloud_v21_node"
  if [ -z "$previous_local_cloud_fingerprint" ] && \
      [ -x "$local_cloud_cached_bin" ] && \
      grep -aFq "$LOCAL_CLOUD_PIPELINE_VERSION" "$local_cloud_cached_bin"; then
    previous_local_cloud_fingerprint="$local_cloud_source_fingerprint"
    printf '%s\n' "$local_cloud_source_fingerprint" > \
      "$local_cloud_fingerprint_marker"
    log "[build] Adopted the compatible cached local_depth_cloud_cpp binary."
  fi
  if [ "$previous_local_cloud_fingerprint" != "$local_cloud_source_fingerprint" ]; then
    reset_cached_package local_depth_cloud_cpp
    log "[build] local_depth_cloud_cpp source changed; rebuilding that package only."
  fi
  build_paths=(
    "$SOURCE_WS/src/local_depth_cloud_cpp"
    "$SOURCE_WS/src/lidar_py"
  )
  build_packages=(local_depth_cloud_cpp lidar_py)
  if is_true "$ENABLE_NAVIGATION"; then
    build_paths+=(
      "$SOURCE_WS/src/frontier_exploration_ros2"
      "$SOURCE_WS/src/short_goal_bt"
    )
    build_packages+=(frontier_exploration_ros2 short_goal_bt)
  fi
  if is_true "$LOCALIZATION_MODE"; then
    build_paths+=("$SOURCE_WS/src/reloc_rviz_panel")
    build_packages+=(reloc_rviz_panel)
  fi
  PYTHONNOUSERSITE=1 colcon --log-base "$LOG_BASE" build \
    --base-paths "${build_paths[@]}" \
    --build-base "$BUILD_BASE" \
    --install-base "$INSTALL_BASE" \
    --symlink-install \
    --packages-select "${build_packages[@]}"
  printf '%s\n' "$local_cloud_source_fingerprint" > \
    "$local_cloud_fingerprint_marker"
fi
[ -f "$INSTALL_BASE/setup.bash" ] || die "Build did not create $INSTALL_BASE/setup.bash"
# shellcheck disable=SC1090
source "$INSTALL_BASE/setup.bash"
check_pkg lidar_py
check_pkg local_depth_cloud_cpp
if is_true "$LOCALIZATION_MODE"; then
  check_pkg reloc_rviz_panel
fi
if is_true "$USE_RVIZ" && is_true "$LOCALIZATION_MODE"; then
  rviz_config="$RVIZ_CONFIG_FILE"
  [ -f "$rviz_config" ] || die "RViz profile is missing: $rviz_config"
  log "[rviz] Opening localization view after the manual-panel plugin build."
  rviz_supervisor &
  RVIZ_PID=$!
fi
LOCAL_CLOUD_BIN="$(ros2 pkg prefix local_depth_cloud_cpp)/lib/local_depth_cloud_cpp/depth_image_to_local_cloud_v21_node"
[ -x "$LOCAL_CLOUD_BIN" ] || \
  die "C++ point-cloud node is missing after build: $LOCAL_CLOUD_BIN"
grep -aFq "$LOCAL_CLOUD_PIPELINE_VERSION" "$LOCAL_CLOUD_BIN" || \
  die "C++ point-cloud node failed build-version check: expected $LOCAL_CLOUD_PIPELINE_VERSION"
if is_true "$ENABLE_NAVIGATION"; then
  check_pkg short_goal_bt
  COLLISION_GATE_BIN="$(ros2 pkg prefix local_depth_cloud_cpp)/lib/local_depth_cloud_cpp/local_cloud_collision_gate_node"
  [ -x "$COLLISION_GATE_BIN" ] || \
    die "C++ collision gate is missing after build: $COLLISION_GATE_BIN"
  VISUAL_WALL_FILTER_BIN="$(ros2 pkg prefix local_depth_cloud_cpp)/lib/local_depth_cloud_cpp/persistent_visual_wall_filter_node"
  [ -x "$VISUAL_WALL_FILTER_BIN" ] || \
    die "C++ persistent visual wall filter is missing after build: $VISUAL_WALL_FILTER_BIN"
  check_pkg frontier_exploration_ros2
fi

log "============================================================"
log "  STEP11 dual-resolution 2D + 3D SLAM"
log "  Platform        : Ubuntu 22.04 + ROS 2 Humble"
log "  ROS domain      : $ROS_DOMAIN_ID"
log "  DDS profile     : $CYCLONEDDS_URI"
log "  2D authority    : Cartographer V13 + $CARTOGRAPHER_ODOM_TOPIC"
log "  2D SLAM config  : $CARTOGRAPHER_CONFIG"
  log "  Global color 3D : RTAB-Map enabled=$ENABLE_RTABMAP, ${RTABMAP_RATE:-2.0} Hz"
  log "  RTAB auto-pause : ${RTABMAP_ON_DEMAND_PAUSE:-false} (false keeps loop closure alive)"
log "  Navigation      : $ENABLE_NAVIGATION (SmacPlanner2D + safe DWB dual controller)"
log "  Nav activation  : $(if is_true "$ENABLE_NAVIGATION"; then printf 'staged after sensor readiness'; else printf 'disabled'; fi)"
log "  Dynamic 3D layer: STVL=${ENABLE_STVL:-true} (bounded recent + filtered RTAB walls)"
log "  2D scan input   : /scan_timed_v2_filtered (filter=$ENABLE_FIXED_SCAN_FILTER)"
log "  Visual EKF      : $ENABLE_VISUAL_FUSION (RGB-D vx/vy + STM32 yaw/vx)"
log "  odom TF owner   : $(if is_true "$ENABLE_VISUAL_FUSION"; then printf 'robot_localization'; else printf 'chassis_node'; fi)"
log "  Occupancy 3D    : RTAB optimized OctoMap ${RTABMAP_RATE:-2.0} Hz, voxel ${GLOBAL_3D_VOXEL:-0.05} m"
log "  OctoMap save    : ${SAVE_OCTOMAP_ON_EXIT:-true} -> $OCTOMAP_OUTPUT_PATH"
log "  3D database     : $DATABASE_PATH"
log "  Local 3D        : ${LOCAL_CLOUD_TOPIC:-/local_highres_cloud_v21} @ ${LOCAL_RATE:-15.0} Hz"
if is_true "$ENABLE_NAVIGATION" && is_true "$ENABLE_STVL"; then
  log "  Obstacle memory : hard stop=1 frame; Nav2 mark=3 frames + geometry guard"
  log "                    RGB-D obstacles decay local/global=6s/8s"
  log "                    /map walls >=65% persist; filtered RTAB walls hold 15s"
  log "  Costmap clearance: measured 66.5cm body + 1cm padding, inflation 0.49m/14.0"
fi
log "  Local FOV       : x ${LOCAL_X_MIN:-0.15}..${LOCAL_X_MAX:-4.0}, y ${LOCAL_Y_MIN:--2.5}..${LOCAL_Y_MAX:-2.5} m"
log "  LiDAR position  : (${LIDAR_X:-0.20}, ${LIDAR_Y:-0.0}, ${LIDAR_Z:-0.4235}) m"
log "  Camera position : (${CAMERA_X:-0.30}, ${CAMERA_Y:-0.0}, ${CAMERA_Z:-0.371}) m"
log "  Camera R/P/Y    : (${CAMERA_ROLL_DEG:-0.0}, ${CAMERA_PITCH_DEG:-0.0}, ${CAMERA_YAW_DEG:-0.0}) deg"
log "  Ground calibrated: ${CAMERA_GROUND_CALIBRATED:-false}"
log "  Local ground filter: $LOCAL_GROUND_FILTER_EFFECTIVE (${LOCAL_GROUND_Z_MIN:--0.10}..${LOCAL_GROUND_Z_MAX:-0.02} m)"
log "  Full extrinsic   : ${CAMERA_EXTRINSIC_CALIBRATED:-false}"
  log "  RGB-D alignment : native RGB + native depth; RTAB gets registered depth"
log "  RGB-D timing    : ${CAMERA_TIME_DOMAIN:-global} clock, pair <= ${RGBD_SYNC_MAX_INTERVAL:-0.045}s"
log "  STM32 timing    : adaptive=${NAVI_ADAPTIVE_CLOCK_SYNC:-true}, window=${NAVI_CLOCK_WINDOW_SAMPLES:-250} samples"
log "  NAVI watchdog   : ${NAVI_MOTION_WATCHDOG_ENABLED:-true}, ${NAVI_MOTION_WATCHDOG_WINDOW_SEC:-0.75}s / ${NAVI_MOTION_WATCHDOG_TRANSLATION_M:-0.08}m / ${NAVI_MOTION_WATCHDOG_YAW_DEG:-3.0}deg"
log "  LiDAR           : $LIDAR_PORT @ $LIDAR_BAUD"
log "  STM32           : $CHASSIS_PORT @ 115200"
log "  RViz            : $USE_RVIZ"
log "  Localization    : $LOCALIZATION_MODE"
if is_true "$LOCALIZATION_MODE"; then
  log "  Static map      : $LOCALIZATION_MAP_YAML"
  log "  Frozen state    : $LOCALIZATION_PBSTREAM"
fi
log "  Platform        : $([ "$JETSON_PLATFORM" = true ] && printf 'Jetson' || printf 'generic')"
log "  RTAB workers    : $RTABMAP_THREADS"
log "  Runtime log     : $RUNTIME_LOG"
log "============================================================"

if ! is_true "${CAMERA_EXTRINSIC_CALIBRATED:-false}"; then
  log "[WARNING] CAMERA_EXTRINSIC_CALIBRATED=false"
  log "[WARNING] Point-cloud direction/height is provisional. Calibrate CAMERA_*_DEG first."
fi
if is_true "${LOCAL_GROUND_FILTER:-true}" && \
    ! is_true "${CAMERA_GROUND_CALIBRATED:-false}"; then
  log "[WARNING] Local ground removal is disabled until ground calibration passes."
  log "[WARNING] Run ./CALIBRATE_CAMERA_EXTRINSIC.sh, restart, then calibrate YAW."
fi

cmd=(ros2 launch lidar_py dual_resolution_3d_slam.launch.py
  "use_rviz:=false"
  "enable_navigation:=$ENABLE_NAVIGATION"
  "nav_autostart:=$NAV_AUTOSTART"
  "localization_mode:=$LOCALIZATION_MODE"
  "localization_map_yaml:=$LOCALIZATION_MAP_YAML"
  "cartographer_load_state_filename:=$LOCALIZATION_PBSTREAM"
  "rviz_config_file:=$RVIZ_CONFIG_FILE"
  "cartographer_config:=$CARTOGRAPHER_CONFIG"
  "enable_visual_fusion:=$ENABLE_VISUAL_FUSION"
  "chassis_publish_tf:=$CHASSIS_PUBLISH_TF"
  "cartographer_odom_topic:=$CARTOGRAPHER_ODOM_TOPIC"
  "rtabmap_odom_topic:=$RTABMAP_ODOM_TOPIC"
  "nav_costmap_override_file:=$NAV_COSTMAP_OVERRIDE"
  "enable_fixed_scan_filter:=$ENABLE_FIXED_SCAN_FILTER"
  "filtered_scan_topic:=/scan_timed_v2_filtered"
  "enable_rtabmap:=$ENABLE_RTABMAP"
  "rtabmap_on_demand_pause:=${RTABMAP_ON_DEMAND_PAUSE:-false}"
  "use_rtabmap_viz:=$USE_RTABMAP_VIZ"
  "lidar_serial_port:=$LIDAR_PORT"
  "lidar_baudrate:=$LIDAR_BAUD"
  "chassis_serial_port:=$CHASSIS_PORT"
  "laser_yaw_deg:=${LASER_YAW_DEG:-0.0}"
  "laser_x:=${LIDAR_X:-0.20}"
  "laser_y:=${LIDAR_Y:-0.0}"
  "laser_z:=${LIDAR_Z:-0.4235}"
  "scan_angle_sign:=${SCAN_ANGLE_SIGN:--1.0}"
  "navi_yaw_sign:=${NAVI_YAW_SIGN:-1.0}"
  "navi_vx_sign:=${NAVI_VX_SIGN:-1.0}"
  "navi_vz_sign:=${NAVI_VZ_SIGN:-1.0}"
  "navi_yaw_offset_deg:=${NAVI_YAW_OFFSET_DEG:-0.0}"
  "navi_odom_yaw_source:=${NAVI_ODOM_YAW_SOURCE:-absolute}"
  "navi_max_yaw_rate_deg_s:=${NAVI_MAX_YAW_RATE_DEG_S:-120.0}"
  "navi_adaptive_clock_sync:=${NAVI_ADAPTIVE_CLOCK_SYNC:-true}"
  "navi_clock_window_samples:=${NAVI_CLOCK_WINDOW_SAMPLES:-250}"
  "navi_clock_max_adjustment_ns:=${NAVI_CLOCK_MAX_ADJUSTMENT_NS:-20000}"
  "navi_motion_watchdog_enabled:=${NAVI_MOTION_WATCHDOG_ENABLED:-true}"
  "navi_motion_watchdog_pose_enabled:=${NAVI_MOTION_WATCHDOG_POSE_ENABLED:-false}"
  "navi_motion_watchdog_warmup_sec:=${NAVI_MOTION_WATCHDOG_WARMUP_SEC:-6.0}"
  "navi_motion_watchdog_window_sec:=${NAVI_MOTION_WATCHDOG_WINDOW_SEC:-0.75}"
  "navi_motion_watchdog_translation_m:=${NAVI_MOTION_WATCHDOG_TRANSLATION_M:-0.08}"
  "navi_motion_watchdog_yaw_deg:=${NAVI_MOTION_WATCHDOG_YAW_DEG:-3.0}"
  "nav_zero_command_cancel_sec:=${NAV_ZERO_COMMAND_CANCEL_SEC:-25.0}"
  "require_system_ready_for_motion:=$ENABLE_NAVIGATION"
  "require_depth_baseline_for_ps2:=${REQUIRE_DEPTH_BASELINE_FOR_PS2:-true}"
  "color_width:=${COLOR_WIDTH:-640}"
  "color_height:=${COLOR_HEIGHT:-480}"
  "color_fps:=${COLOR_FPS:-15}"
  "depth_width:=${DEPTH_WIDTH:-640}"
  "depth_height:=${DEPTH_HEIGHT:-400}"
  "depth_fps:=${DEPTH_FPS:-15}"
  "depth_registration:=${CAMERA_DEPTH_REGISTRATION:-false}"
  "align_mode:=${CAMERA_ALIGN_MODE:-HW}"
  "align_target_stream:=${CAMERA_ALIGN_TARGET_STREAM:-COLOR}"
  "camera_time_domain:=${CAMERA_TIME_DOMAIN:-global}"
  "camera_time_sync_period:=${CAMERA_TIME_SYNC_PERIOD:-10.0}"
  "camera_enable_frame_drop_log:=${CAMERA_ENABLE_FRAME_DROP_LOG:-true}"
  "camera_frame_timestamp_csv_file:=$LOG_DIR/orbbec_frame_timestamp.csv"
  "rgbd_sync_max_interval:=${RGBD_SYNC_MAX_INTERVAL:-0.045}"
  "rgbd_sync_max_interval_ms:=${RGBD_SYNC_MAX_INTERVAL_MS:-45.0}"
  "rgbd_sync_warn_p95_ms:=${RGBD_SYNC_WARN_P95_MS:-35.0}"
  "camera_x:=${CAMERA_X:-0.30}"
  "camera_y:=${CAMERA_Y:-0.0}"
  "camera_z:=${CAMERA_Z:-0.40}"
  "camera_roll:=$CAMERA_ROLL"
  "camera_pitch:=$CAMERA_PITCH"
  "camera_yaw:=$CAMERA_YAW"
  "database_path:=$DATABASE_PATH"
  "rtabmap_rate:=${RTABMAP_RATE:-2.0}"
  "rtabmap_threads:=$RTABMAP_THREADS"
  "global_3d_voxel:=${GLOBAL_3D_VOXEL:-0.08}"
  "global_3d_range_max:=${GLOBAL_3D_RANGE_MAX:-4.0}"
  "use_octomap:=${USE_OCTOMAP:-true}"
  "local_cloud_topic:=${LOCAL_CLOUD_TOPIC:-/local_highres_cloud_v21}"
  "local_sensor_cloud_topic:=${LOCAL_SENSOR_CLOUD_TOPIC:-/local_highres_cloud_v21/sensor}"
  "local_persistent_sensor_cloud_topic:=${LOCAL_PERSISTENT_SENSOR_CLOUD_TOPIC:-/local_highres_cloud_v21/persistent_sensor}"
  "local_stats_topic:=${LOCAL_STATS_TOPIC:-/local_highres_cloud_v21/stats}"
  "local_marker_topic:=${LOCAL_MARKER_TOPIC:-/local_highres_cloud_v21/crop_markers}"
  "local_rate:=${LOCAL_RATE:-15.0}"
  "local_stride:=${LOCAL_STRIDE:-2}"
  "local_voxel:=${LOCAL_VOXEL:-0.03}"
  "local_min_range:=${LOCAL_MIN_RANGE:-0.20}"
  "local_max_range:=${LOCAL_MAX_RANGE:-4.0}"
  "local_x_min:=${LOCAL_X_MIN:-0.15}"
  "local_x_max:=${LOCAL_X_MAX:-4.0}"
  "local_y_min:=${LOCAL_Y_MIN:--2.5}"
  "local_y_max:=${LOCAL_Y_MAX:-2.5}"
  "local_z_min:=${LOCAL_Z_MIN:--0.5}"
  "local_z_max:=${LOCAL_Z_MAX:-2.0}"
  "local_ground_filter:=$LOCAL_GROUND_FILTER_EFFECTIVE"
  "local_ground_z_min:=${LOCAL_GROUND_Z_MIN:--0.10}"
  "local_ground_z_max:=${LOCAL_GROUND_Z_MAX:-0.02}"
  "local_spatial_filter:=${LOCAL_SPATIAL_FILTER:-true}"
  "local_spatial_threshold_m:=${LOCAL_SPATIAL_THRESHOLD_M:-0.08}"
  "local_spatial_threshold_ratio:=${LOCAL_SPATIAL_THRESHOLD_RATIO:-0.025}"
  "local_spatial_min_neighbors:=${LOCAL_SPATIAL_MIN_NEIGHBORS:-2}"
  "local_temporal_filter:=${LOCAL_TEMPORAL_FILTER:-true}"
  "local_temporal_alpha:=${LOCAL_TEMPORAL_ALPHA:-0.65}"
  "local_temporal_max_delta_m:=${LOCAL_TEMPORAL_MAX_DELTA_M:-0.06}"
  "local_voxel_outlier_filter:=${LOCAL_VOXEL_OUTLIER_FILTER:-true}"
  "local_voxel_min_neighbors:=${LOCAL_VOXEL_MIN_NEIGHBORS:-1}")

setsid stdbuf -oL -eL "${cmd[@]}" >>"$RUNTIME_LOG" 2>&1 &
LAUNCH_PID=$!
printf '%s\n' "$LAUNCH_PID" >"$RUN_DIR/ros_launch.pid"
tail -n +1 -F "$RUNTIME_LOG" &
TAIL_PID=$!

sleep 2
if is_true "$USE_RVIZ" && ! kill -0 "$RVIZ_PID" 2>/dev/null; then
  die "RViz exited before the mapping stack became ready"
fi
if ! wait_parameter_value \
    /depth_image_to_local_cloud_v21 pipeline_version \
    "$LOCAL_CLOUD_PIPELINE_VERSION" 30 WARNING; then
  # The executable was already verified by an embedded version marker before
  # launch. ros2cli service discovery can lag under Jetson startup load, so
  # real RGB/depth/cloud messages below remain the authoritative readiness gate.
  log "[WARNING] Point-cloud parameter query is delayed; waiting for real sensor data."
fi
wait_lidar_data /scan_timed_v2 20 || \
  die "LiDAR did not publish a valid /scan_timed_v2 revolution"
if is_true "$ENABLE_FIXED_SCAN_FILTER"; then
  wait_lidar_data /scan_timed_v2_filtered 20 || \
    die "2D LiDAR filter did not publish /scan_timed_v2_filtered"
fi

log "[startup] Verifying the motion-feedback and Cartographer safety chain..."
wait_topic /odom 20 || die "STM32 odometry /odom did not start"
wait_topic /imu_cartographer 20 || \
  die "STM32 planar IMU /imu_cartographer did not start"
if is_true "$LOCALIZATION_MODE"; then
  wait_transient_topic /map 30 || \
    die "Static localization map server did not publish /map"
  # In localization mode Cartographer deliberately starts with no active
  # trajectory.  /cartographer_pose_odom is created only after the scan-to-map
  # relocalizer matches the frozen state, so waiting for it here deadlocks the
  # very node that must create it.
  while ! wait_boolean_true /localization_ready 150; do
    kill -0 "$LAUNCH_PID" 2>/dev/null || \
      die "ROS launch exited while waiting for startup relocalization"
    log "[localization] Automatic match is still unverified; keeping RViz and the manual Relocalize control alive. Motion remains locked."
  done
else
  wait_topic /cartographer_pose_odom 30 || \
    die "Cartographer corrected pose did not start"
  wait_topic /map 30 || die "Cartographer occupancy map did not start"
fi

log "[startup] Verifying Gemini2 and the live collision cloud..."
if ! wait_topic /camera/color/image_raw 60; then
  camera_ownership_report
  die "Gemini2 RGB stream did not start"
fi
if ! wait_topic /camera/depth/image_raw 30; then
  camera_ownership_report
  die "Gemini2 depth stream did not start"
fi
wait_topic "${LOCAL_CLOUD_TOPIC:-/local_highres_cloud_v21}" 45 || \
  die "STEP10V2.1 local cloud did not start"
if is_true "$ENABLE_VISUAL_FUSION"; then
  wait_topic /visual_odom 60 || die "RGB-D visual odometry did not start"
  wait_topic /odometry/filtered 30 || die "Visual/wheel EKF did not start"
fi

if is_true "$ENABLE_NAVIGATION"; then
  log "[startup] Verifying the live collision input before activating Nav2..."
  wait_topic "${LOCAL_SENSOR_CLOUD_TOPIC:-/local_highres_cloud_v21/sensor}" 30 || \
    die "Recent geometry-filtered navigation cloud did not start"
  wait_topic /local_cloud_collision_stop 30 || \
    die "C++ local collision gate did not start"
  if is_true "$LOCALIZATION_MODE"; then
    log "[localization] Waiting for a verified scan-to-map match..."
    wait_boolean_true /localization_ready 150 || \
      die "Relocalization was not verified; Nav2 and host motion remain locked"
    wait_lifecycle_active /controller_server 60 || \
      die "Relocalization succeeded but Nav2 did not become active"
  else
    start_navigation_lifecycle || \
      die "Nav2 lifecycle activation failed; movement remains locked"
  fi
  # Nested costmap parameter services do not exist until the
  # controller/planner lifecycle nodes have been configured. Source YAML was
  # checked before launch, so release immediately after lifecycle startup and
  # never leave an active action server behind a host-motion gate.
  release_motion_interlock || \
    die "Could not release the startup motion interlock"
  log "[ready] Startup motion interlock released; PS2/Nav2 motion is enabled."

  # This is a runtime audit, not a startup-kill switch. ros2cli discovery can
  # be delayed even when the lifecycle nodes and their costmaps are healthy.
  # The source files were already validated before launch; an unavailable CLI
  # probe must not tear down Cartographer, the camera and every Nav2 process.
  log "[startup] Auditing active costmap parameters (non-fatal CLI check)..."
  runtime_costmap_contract_ok=true
  wait_parameter_value \
    /local_costmap/local_costmap lethal_cost_threshold 65 4 WARNING || \
    runtime_costmap_contract_ok=false
  wait_parameter_value \
    /global_costmap/global_costmap lethal_cost_threshold 65 4 WARNING || \
    runtime_costmap_contract_ok=false
  wait_parameter_value \
    /local_costmap/local_costmap inflation_layer.inflation_radius 0.49 4 WARNING || \
    runtime_costmap_contract_ok=false
  wait_parameter_value \
    /global_costmap/global_costmap inflation_layer.inflation_radius 0.49 4 WARNING || \
    runtime_costmap_contract_ok=false
  if [ "$runtime_costmap_contract_ok" != true ]; then
    log "[WARNING] Runtime costmap query was incomplete; source contract passed and navigation remains running."
  fi
fi

log "[startup] Checking non-blocking 3D-memory and exploration diagnostics..."
if is_true "$ENABLE_NAVIGATION"; then
  require_lifecycle_active /controller_server || \
    log "[WARNING] Nav2 controller lifecycle query is delayed."
  require_lifecycle_active /bt_navigator || \
    log "[WARNING] Nav2 BT lifecycle query is delayed."
  wait_graph_name action /navigate_to_pose 15 || \
    log "[WARNING] Nav2 action discovery is delayed; wait before setting a goal."
  wait_topic /local_costmap/costmap 15 || \
    log "[WARNING] Nav2 local costmap display has not published yet."
  wait_topic /global_costmap/costmap 15 || \
    log "[WARNING] Nav2 global costmap display has not published yet."
  wait_topic /local_costmap/voxel_grid 15 || \
    log "[WARNING] Bounded local RGB-D voxel display has not published yet."
  wait_topic /local_highres_cloud_v21/clear_sensor 15 || \
    log "[WARNING] Depth-valid clearing stream is not ready; bounded STVL decay remains active."
fi
wait_topic /dual_3d/rgbd_timestamp_stats 15 || \
  log "[WARNING] RGB-D timestamp diagnostics did not publish yet."
if is_true "$ENABLE_RTABMAP"; then
  wait_topic /rtabmap_3d/info 30 || \
    log "[WARNING] RTAB-Map persistent graph did not publish diagnostics yet."
  if is_true "$ENABLE_NAVIGATION"; then
    wait_topic /rtabmap_3d/navigation_walls 30 || \
      log "[WARNING] Persistent visual-wall stream did not publish yet."
  fi
fi
if is_true "${USE_OCTOMAP:-true}"; then
  wait_topic /rtabmap_3d/octomap_occupied_space 30 || \
    log "[WARNING] Optimized OctoMap display did not publish yet."
fi
if is_true "$ENABLE_NAVIGATION"; then
  wait_graph_name service /control_exploration 15 || \
    log "[WARNING] Frontier exploration service is unavailable; manual Nav2 remains enabled."
fi

log ""
log "[ready] All three layers are running. Move slowly for the first 10 seconds."
log "[check] ros2 run tf2_ros tf2_echo map base_link"
log "[check] ros2 topic echo ${LOCAL_STATS_TOPIC:-/local_highres_cloud_v21/stats} --once"
log "[check] ros2 topic echo /dual_3d/rgbd_timestamp_stats --once"
log "[check] ros2 topic hz /cartographer_pose_odom"
log "[check] ros2 topic hz /rtabmap_3d/mapData"
log "[check] ros2 topic hz /rtabmap_3d/octomap_occupied_space"
if is_true "$ENABLE_VISUAL_FUSION"; then
  log "[visual-fusion] Cartographer input=/odometry/filtered; odom TF owner=robot_localization."
  log "[visual-fusion] RTAB-Map input=/cartographer_pose_odom (map-corrected, no TF authority)."
  log "[visual-fusion] Check: ros2 topic hz /visual_odom /odometry/filtered"
else
  log "[stable] /odometry/filtered and visual odometry are intentionally absent."
fi
if is_true "$ENABLE_NAVIGATION"; then
  log "[navigation] RViz Navigation 2 panel sends /navigate_to_pose goals directly."
  log "[navigation] Use the panel's Cancel button for an immediate mid-route stop."
  log "[navigation] PS2 is available during startup/idle; only an active Nav2 goal takes MOVE."
  log "[navigation] Nav2 success, cancel or failure returns control to PS2 automatically."
  log "[navigation] Blue path=/plan; orange arc=/lookahead_arc; commands are open-loop rate smoothed."
  log "[navigation] Live 3 cm RGB-D obstacles + filtered persistent visual walls feed Nav2."
  log "[navigation] Legacy depth baseline is disabled; 2D/3D collision watchdogs remain enabled."
fi

monitor_ticks=0
while process_is_running "$LAUNCH_PID"; do
  # A direct wait can return early when an unrelated GUI/logging child changes
  # state. Poll only the ROS launch PID and tolerate interrupted sleeps.
  sleep 1 || true
  monitor_ticks=$((monitor_ticks + 1))
  if [ $((monitor_ticks % 10)) -eq 0 ]; then
    printf '[launcher] [monitor] heartbeat launch_pid=%s rviz_pid=%s\n' \
      "$LAUNCH_PID" "${RVIZ_PID:-none}" >>"$RUNTIME_LOG"
  fi
done

set +e
wait "$LAUNCH_PID" 2>/dev/null
launch_status=$?
set -e
log "[monitor] ROS launch process exited with status $launch_status."
STOP_REASON="ros_launch_exited_status_$launch_status"
SHUTDOWN_REQUESTED=true
exit "$launch_status"
