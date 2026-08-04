#!/usr/bin/env bash
set -Eeo pipefail

# Unified one-key launcher for tested Cartographer V13 + native Jazzy Nav2.
#
# open_all.sh:
#   - periodic map/NAVI logging is available and controlled by the web page
#   - logging defaults to OFF with a 3 second interval
#   - no final PGM/YAML or PBStream is saved on exit
#
# open_all_log.sh reuses this file with SAVE_FINAL_ARTIFACTS=true.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_WS="${LIDAR_WS:-$ROOT_DIR/lidar/chapt1_ws}"
ROAD_DIR="${ROAD_DIR:-$ROOT_DIR/road_v5_5_640_modular_v7_unified_io}"
WEB_DIR="${WEB_DIR:-$ROOT_DIR/web}"
WEB_STOP_SCRIPT="$ROOT_DIR/web_ctrl/stop_web.py"

# Jazzy rosidl generation can fail below a non-ASCII physical build path.
# Keep source in the project, but build the C++ service packages through an
# ASCII-only symlink and install prefix.
ASCII_WS_BASE="${CAR_JAZZY_BUILD_ROOT:-$HOME/.cache/huichuan_agv_jazzy_ws}"
ASCII_SRC_LINK="$ASCII_WS_BASE/src"
AUTO_BUILD_BASE="$ASCII_WS_BASE/build"
AUTO_INSTALL_BASE="$ASCII_WS_BASE/install"
AUTO_LOG_BASE="$ASCII_WS_BASE/log"
SYSTEM_PYTHON="${CAR_SYSTEM_PYTHON:-/usr/bin/python3}"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${ROS_DOMAIN:-88}}"
NAV_PROFILE="${NAV_PROFILE:-jazzy_native}"
AUTO_START="${AUTO_START:-false}"
ENABLE_VISION="${ENABLE_VISION:-true}"
START_WEB="${START_WEB:-true}"
USE_RVIZ="${USE_RVIZ:-true}"
SHOW_NAVI_GUI="${SHOW_NAVI_GUI:-false}"
SKIP_BUILD="${SKIP_BUILD:-false}"
SAVE_FINAL_ARTIFACTS="${SAVE_FINAL_ARTIFACTS:-false}"
RUN_PROFILE_NAME="${RUN_PROFILE_NAME:-open_all}"
LOG_DEFAULT_INTERVAL_SEC="${LOG_DEFAULT_INTERVAL_SEC:-3.0}"

# Test-9 mapping baseline.  Navigation/web work must not silently change these
# three files because even a small scan or matcher change invalidates the
# vehicle test results.
EXPECTED_CARTOGRAPHER_HASH="00dfd1c721f0fe8c61ac6f2b417001920694e4fc77e895fb4a1f194330c910d9"
EXPECTED_SCAN_LAUNCH_HASH="ee90d9f1a7ced49fe91b87ac816686179f13782eb1ecb2175d983644818e5894"
EXPECTED_SCAN_FILTER_HASH="8583a2ca7e99a29b13f2fc339df468e621562d61f0adfa1e7e1828254705b306"

RUN_STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_DIR="$ROOT_DIR/SLAM_Log/${RUN_PROFILE_NAME}_${RUN_STAMP}"
LOGGER_BASE="$RUN_DIR/Logger"
FINAL_DIR="$RUN_DIR/Final"
RUNTIME_STACK_LOG="$RUN_DIR/runtime_stack.log"

declare -a PROCESS_PIDS=()
declare -a PROCESS_NAMES=()
declare -a KNOWN_STACK_PATTERNS=(
    "rosbridge_websocket"
    "depth_obstacle_node.py"
    "slam_logger.py"
    "cartographer_node"
    "cartographer_occupancy_grid_node"
    "chassis_node"
    "lidar_node"
    "scan_to_scan_filter_chain"
    "controller_server"
    "velocity_smoother"
    "planner_server"
    "behavior_server"
    "bt_navigator"
    "waypoint_follower"
    "lifecycle_manager_navigation"
    "frontier_explorer"
    "frontier_web_bridge"
    "auto_map_saver"
    "web_goal_nav_node"
    "web_path_preview_node"
    "safety_fusion_node"
    "rviz2"
    "node_modules/vite/bin/vite.js"
    "ros2 launch lidar_py"
    "ros2 run rosbridge_server"
    "cartographer_auto_mapping_jazzy_launch.py"
    "ros2 bag record"
    "_ros2_daemon"
)
CLEANUP_STARTED=false
CLEANUP_STAGE_PID=""
ROS_ENV_READY=false
STACK_STARTED=false
STACK_PGID=""
LAST_STARTED_PID=""

log() {
    printf '%s\n' "$*"
}

die() {
    log "[error] $*"
    if [ -s "$RUNTIME_STACK_LOG" ]; then
        log "[error] Matching failure lines from $RUNTIME_STACK_LOG:"
        grep -Eai '\[error\]|\[fatal\]|exception|failed|pluginlib|class_loader|process has died|terminate called' \
            "$RUNTIME_STACK_LOG" | tail -n 80 || true
        log "[error] Last 100 lines from $RUNTIME_STACK_LOG:"
        tail -n 100 "$RUNTIME_STACK_LOG" || true
    fi
    exit 1
}

is_true() {
    case "${1,,}" in
        1|true|yes|on) return 0 ;;
        *) return 1 ;;
    esac
}

hard_timeout() {
    local duration="$1"
    shift
    timeout --signal=TERM --kill-after=1s "$duration" "$@"
}

kill_process_tree() {
    local signal="$1"
    local parent_pid="$2"
    local child_pid

    while read -r child_pid; do
        [ -n "$child_pid" ] || continue
        kill_process_tree "$signal" "$child_pid"
    done < <(pgrep -P "$parent_pid" 2>/dev/null || true)
    kill -s "$signal" "$parent_pid" 2>/dev/null || true
}

usb_device_attribute() {
    local dev="$1"
    local attribute="$2"
    local tty_name="${dev##*/}"
    local sys_path

    sys_path="$(readlink -f "/sys/class/tty/$tty_name/device" 2>/dev/null || true)"
    while [ -n "$sys_path" ] && [ "$sys_path" != "/" ]; do
        if [ -r "$sys_path/$attribute" ]; then
            tr -d '\r\n' <"$sys_path/$attribute"
            return 0
        fi
        case "$sys_path" in
            /sys/*) sys_path="${sys_path%/*}" ;;
            *) break ;;
        esac
    done
    return 1
}

serial_by_id_alias() {
    local dev="$1"
    local resolved_dev
    local alias

    resolved_dev="$(readlink -f "$dev" 2>/dev/null || true)"
    for alias in /dev/serial/by-id/*; do
        [ -L "$alias" ] || continue
        if [ "$(readlink -f "$alias" 2>/dev/null || true)" = "$resolved_dev" ]; then
            printf '%s' "$alias"
            return 0
        fi
    done
    return 1
}

print_serial_candidates() {
    local dev vendor product manufacturer model alias
    local found=false

    log "[serial] Detected serial devices:"
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$dev" ] || continue
        found=true
        vendor="$(usb_device_attribute "$dev" idVendor 2>/dev/null || true)"
        product="$(usb_device_attribute "$dev" idProduct 2>/dev/null || true)"
        manufacturer="$(usb_device_attribute "$dev" manufacturer 2>/dev/null || true)"
        model="$(usb_device_attribute "$dev" product 2>/dev/null || true)"
        alias="$(serial_by_id_alias "$dev" 2>/dev/null || true)"
        log "  - $dev VID:PID=${vendor:-????}:${product:-????} model=${model:-unknown} manufacturer=${manufacturer:-unknown}"
        [ -n "$alias" ] && log "    by-id: $alias"
    done
    [ "$found" = true ] || log "  - none"
}

auto_detect_ports() {
    local dev id_product
    local detected_lidar=""
    local detected_chassis=""
    local explicit_lidar="${LIDAR_PORT:-${LIDAR_PORT_ENV:-${LIDAR_OVERRIDE:-}}}"
    local explicit_chassis="${CHASSIS_PORT:-${CHASSIS_PORT_ENV:-${CHASSIS_OVERRIDE:-}}}"
    local lidar_source=""
    local chassis_source=""
    local -a acm_candidates=()
    local -a usb_candidates=()

    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$dev" ] || continue
        case "$dev" in
            /dev/ttyACM*) acm_candidates+=("$dev") ;;
            /dev/ttyUSB*) usb_candidates+=("$dev") ;;
        esac

        id_product="$(usb_device_attribute "$dev" idProduct 2>/dev/null || true)"
        case "${id_product,,}" in
            55d4)
                detected_lidar="$dev"
                lidar_source="USB PID 55d4"
                ;;
            7523)
                detected_chassis="$dev"
                chassis_source="USB PID 7523"
                ;;
        esac
    done

    if [ -n "$explicit_lidar" ]; then
        LIDAR_PORT="$explicit_lidar"
        lidar_source="environment override"
    elif [ -n "$detected_lidar" ]; then
        LIDAR_PORT="$detected_lidar"
    elif [ -e /dev/ttyACM0 ]; then
        LIDAR_PORT=/dev/ttyACM0
        lidar_source="ACM fallback"
    elif [ -e /dev/ttyUSB1 ]; then
        LIDAR_PORT=/dev/ttyUSB1
        lidar_source="tested USB1 fallback"
    elif [ "${#usb_candidates[@]}" -ge 2 ]; then
        LIDAR_PORT="${usb_candidates[1]}"
        lidar_source="second USB fallback"
    else
        LIDAR_PORT="${acm_candidates[0]:-/dev/ttyACM0}"
        lidar_source="last-resort fallback"
    fi

    if [ -n "$explicit_chassis" ]; then
        CHASSIS_PORT="$explicit_chassis"
        chassis_source="environment override"
    elif [ -n "$detected_chassis" ]; then
        CHASSIS_PORT="$detected_chassis"
    elif [ -e /dev/ttyUSB0 ]; then
        CHASSIS_PORT=/dev/ttyUSB0
        chassis_source="tested USB0 fallback"
    elif [ "${#usb_candidates[@]}" -ge 1 ]; then
        CHASSIS_PORT="${usb_candidates[0]}"
        chassis_source="first USB fallback"
    else
        CHASSIS_PORT=/dev/ttyUSB0
        chassis_source="last-resort fallback"
    fi

    LIDAR_BAUD="${LIDAR_BAUD:-115200}"

    print_serial_candidates
    log "[serial] LiDAR -> $LIDAR_PORT ($lidar_source)"
    log "[serial] STM32 -> $CHASSIS_PORT ($chassis_source)"

    [ "$LIDAR_PORT" != "$CHASSIS_PORT" ] ||
        die "LiDAR and STM32 resolved to the same device: $LIDAR_PORT"
}

port_is_listening() {
    local port="$1"
    if command -v ss >/dev/null 2>&1; then
        ss -ltn 2>/dev/null | awk '{print $4}' |
            grep -qE "(^|:)$port$"
        return
    fi
    (echo >/dev/tcp/127.0.0.1/"$port") >/dev/null 2>&1
}

wait_for_port() {
    local port="$1"
    local timeout_sec="$2"
    local label="$3"
    local elapsed=0

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        if port_is_listening "$port"; then
            log "[ready] $label is listening on :$port"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log "[error] $label did not open port $port within ${timeout_sec}s"
    return 1
}

wait_for_ros_node() {
    local node_name="$1"
    local timeout_sec="$2"
    local elapsed=0
    local nodes=""

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        nodes="$(timeout 3s ros2 node list 2>/dev/null || true)"
        if printf '%s\n' "$nodes" | grep -Fxq "$node_name"; then
            log "[ready] ROS node $node_name"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log "[error] ROS node $node_name was not ready within ${timeout_sec}s"
    return 1
}

wait_for_ros_lifecycle_active() {
    local node_name="$1"
    local timeout_sec="$2"
    local elapsed=0
    local state=""

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        state="$(timeout 3s ros2 lifecycle get "$node_name" 2>/dev/null || true)"
        if printf '%s\n' "$state" |
            grep -Eq '(^|[[:space:]])active([[:space:]]|\[|$)'; then
            log "[ready] ROS lifecycle $node_name: active"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log "[error] ROS lifecycle $node_name did not become active within ${timeout_sec}s"
    [ -n "$state" ] && log "[error] Last lifecycle response: $state"
    return 1
}

wait_for_ros_action() {
    local action_name="$1"
    local timeout_sec="$2"
    local elapsed=0
    local actions=""

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        actions="$(timeout 3s ros2 action list 2>/dev/null || true)"
        if printf '%s\n' "$actions" | grep -Fxq "$action_name"; then
            log "[ready] ROS action $action_name"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log "[error] ROS action $action_name was not ready within ${timeout_sec}s"
    return 1
}

wait_for_ros_service() {
    local service_name="$1"
    local timeout_sec="$2"
    local elapsed=0
    local services=""

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        services="$(timeout 3s ros2 service list 2>/dev/null || true)"
        if printf '%s\n' "$services" | grep -Fxq "$service_name"; then
            log "[ready] ROS service $service_name"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log "[error] ROS service $service_name was not ready within ${timeout_sec}s"
    return 1
}

wait_for_topic_sample() {
    local topic_name="$1"
    local topic_type="$2"
    local timeout_sec="$3"
    local label="$4"
    local deadline=$((SECONDS + timeout_sec))
    local next_report=$((SECONDS + 10))
    local info=""

    log "[wait] Waiting for first $label on $topic_name..."
    while [ "$SECONDS" -lt "$deadline" ]; do
        if hard_timeout 4s ros2 topic echo \
            "$topic_name" "$topic_type" --once \
            --qos-reliability best_effort \
            >/dev/null 2>&1; then
            log "[ready] First $label received on $topic_name"
            return 0
        fi
        if [ -n "$STACK_PGID" ] && ! process_group_alive "$STACK_PGID"; then
            log "[error] Mapping launch exited while waiting for $topic_name"
            return 1
        fi
        if [ "$SECONDS" -ge "$next_report" ]; then
            log "[wait] Still waiting for $label on $topic_name..."
            next_report=$((SECONDS + 10))
        fi
        sleep 1
    done

    log "[error] No $label arrived on $topic_name within ${timeout_sec}s"
    info="$(hard_timeout 5s ros2 topic info --verbose "$topic_name" 2>/dev/null || true)"
    [ -n "$info" ] && log "$info"
    return 1
}

start_nav2_lifecycle() {
    local response_file="$RUN_DIR/nav2_lifecycle_startup.log"
    local call_pid status=0 response=""

    wait_for_ros_service /lifecycle_manager_navigation/manage_nodes 30 ||
        return 1
    log "[nav2] Cartographer map is live; starting all Nav2 lifecycle nodes..."
    : >"$response_file"
    hard_timeout 75s ros2 service call \
        /lifecycle_manager_navigation/manage_nodes \
        nav2_msgs/srv/ManageLifecycleNodes \
        "{command: 0}" >"$response_file" 2>&1 &
    call_pid=$!
    while kill -0 "$call_pid" 2>/dev/null; do
        sleep 5
        if kill -0 "$call_pid" 2>/dev/null; then
            log "[wait] Nav2 lifecycle startup is still running..."
        fi
    done
    if wait "$call_pid"; then
        status=0
    else
        status=$?
    fi
    response="$(cat "$response_file" 2>/dev/null || true)"
    if [ "$status" -ne 0 ]; then
        log "[error] Nav2 lifecycle startup call failed or timed out (status=$status)."
        [ -n "$response" ] && log "$response"
        return 1
    fi
    if ! printf '%s\n' "$response" |
        grep -Eiq 'success[=:][[:space:]]*true'; then
        log "[error] Nav2 lifecycle manager rejected startup."
        [ -n "$response" ] && log "$response"
        return 1
    fi
    log "[ready] Nav2 lifecycle manager completed startup"
}

assert_single_ros_node() {
    local node_name="$1"
    local count
    count="$(timeout 5s ros2 node list 2>/dev/null |
        grep -Fxc "$node_name" || true)"
    if [ "$count" != "1" ]; then
        log "[error] Expected exactly one $node_name, found $count."
        log "[error] A stale/local/remote ROS stack on domain $ROS_DOMAIN_ID can corrupt TF and mapping."
        return 1
    fi
    log "[ready] Unique ROS node $node_name"
}

assert_single_topic_publisher() {
    local topic_name="$1"
    local info count
    info="$(timeout 5s ros2 topic info "$topic_name" 2>/dev/null || true)"
    count="$(printf '%s\n' "$info" | awk -F: '
        /Publisher count/ {gsub(/[[:space:]]/, "", $2); print $2; exit}')"
    if [ "$count" != "1" ]; then
        log "[error] Expected one publisher on $topic_name, found ${count:-unknown}."
        log "[error] Refusing to run Cartographer with an ambiguous sensor/odometry source."
        [ -n "$info" ] && log "$info"
        return 1
    fi
    log "[ready] Unique publisher $topic_name"
}

verify_mapping_baseline() {
    local file expected actual
    local -a checks=(
        "$LIDAR_SHARE/config/cartographer_2d_v9_tightened.lua|$EXPECTED_CARTOGRAPHER_HASH"
        "$LIDAR_SHARE/launch/cartographer_scan_v2_launch.py|$EXPECTED_SCAN_LAUNCH_HASH"
        "$LIDAR_SHARE/config/laser_filter.yaml|$EXPECTED_SCAN_FILTER_HASH"
    )

    command -v sha256sum >/dev/null 2>&1 ||
        die "sha256sum is required to verify the tested mapping baseline."
    for file in "${checks[@]}"; do
        expected="${file#*|}"
        file="${file%%|*}"
        [ -r "$file" ] || die "Tested mapping file is missing: $file"
        actual="$(sha256sum "$file" | awk '{print tolower($1)}')"
        if [ "$actual" != "$expected" ]; then
            die "Tested mapping baseline changed: $file (expected $expected, got $actual)"
        fi
        log "[baseline] $(basename "$file") sha256=$actual"
    done
}

start_process_group() {
    local name="$1"
    shift

    setsid "$@" &
    local pid=$!
    PROCESS_PIDS+=("$pid")
    PROCESS_NAMES+=("$name")
    LAST_STARTED_PID="$pid"
    log "[start] $name (pid=$pid, pgid=$pid)"
}

start_process_group_logged() {
    local name="$1"
    local output_file="$2"
    shift 2

    : >"$output_file"
    setsid "$@" >>"$output_file" 2>&1 &
    local pid=$!
    PROCESS_PIDS+=("$pid")
    PROCESS_NAMES+=("$name")
    LAST_STARTED_PID="$pid"
    log "[start] $name (pid=$pid, pgid=$pid, log=$output_file)"
}

process_group_alive() {
    local pgid="$1"
    kill -0 -- "-$pgid" 2>/dev/null
}

signal_process_group() {
    local pgid="$1"
    local signal_name="$2"
    kill -s "$signal_name" -- "-$pgid" 2>/dev/null || true
}

wait_for_process_groups() {
    local timeout_sec="$1"
    local elapsed=0
    local pid
    local any_alive

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        any_alive=false
        for pid in "${PROCESS_PIDS[@]}"; do
            if process_group_alive "$pid"; then
                any_alive=true
                break
            fi
        done
        if [ "$any_alive" = false ]; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

list_known_stack_processes() {
    local pattern line pid state
    local -A seen=()

    for pattern in "${KNOWN_STACK_PATTERNS[@]}"; do
        while IFS= read -r line; do
            [ -n "$line" ] || continue
            pid="${line%% *}"
            [ -n "$pid" ] || continue
            [ -z "${seen[$pid]+x}" ] || continue
            state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
            case "$state" in
                Z*) continue ;;
            esac
            seen["$pid"]=1
            printf '%s\n' "$line"
        done < <(pgrep -af "$pattern" 2>/dev/null || true)
    done
}

show_resource_holders() {
    local leftovers serial_output port_output

    leftovers="$(list_known_stack_processes)"
    if [ -n "$leftovers" ]; then
        log "[cleanup] Old stack processes found:"
        while IFS= read -r line; do
            [ -n "$line" ] && log "  $line"
        done <<<"$leftovers"
    else
        log "[cleanup] No known old stack process found."
    fi

    serial_output="$(fuser -v "$LIDAR_PORT" "$CHASSIS_PORT" 2>&1 || true)"
    if [ -n "$serial_output" ]; then
        log "[cleanup] Serial device holders:"
        while IFS= read -r line; do
            [ -n "$line" ] && log "  $line"
        done <<<"$serial_output"
    fi

    port_output="$(fuser -v 9090/tcp 5173/tcp 8080/tcp 2>&1 || true)"
    if [ -n "$port_output" ]; then
        log "[cleanup] Network port holders:"
        while IFS= read -r line; do
            [ -n "$line" ] && log "  $line"
        done <<<"$port_output"
    fi
}

kill_known_stack_processes() {
    local signal_name="$1"
    local pattern

    for pattern in "${KNOWN_STACK_PATTERNS[@]}"; do
        pkill "-$signal_name" -f "$pattern" 2>/dev/null || true
    done
}

cleanup_before_start() {
    local leftovers
    local serial_holders=""

    log "[0/7] Checking and cleaning leftovers from an earlier run..."
    show_resource_holders

    if [ -f "$WEB_STOP_SCRIPT" ]; then
        python3 "$WEB_STOP_SCRIPT" >/dev/null 2>&1 || true
    fi

    kill_known_stack_processes TERM
    fuser -k "$LIDAR_PORT" "$CHASSIS_PORT" 2>/dev/null || true
    fuser -k 9090/tcp 5173/tcp 8080/tcp 2>/dev/null || true
    sleep 2
    kill_known_stack_processes KILL
    fuser -k 9090/tcp 5173/tcp 8080/tcp 2>/dev/null || true

    leftovers="$(list_known_stack_processes)"
    serial_holders="$(fuser "$LIDAR_PORT" "$CHASSIS_PORT" 2>/dev/null || true)"
    if [ -n "$leftovers" ] || [ -n "$serial_holders" ] ||
        port_is_listening 9090 || port_is_listening 5173 ||
        port_is_listening 8080; then
        log "[error] Some old processes or resources are still active:"
        show_resource_holders
        die "Refusing to start a second incomplete robot stack."
    fi

    log "[cleanup] Old stack, serial holders and web ports are clear."
}

grant_device_permissions() {
    local dev
    local -a required_devices=("$LIDAR_PORT" "$CHASSIS_PORT")
    local -a optional_devices=()

    if is_true "$ENABLE_VISION"; then
        for dev in /dev/video0 /dev/video1; do
            [ -e "$dev" ] && optional_devices+=("$dev")
        done
    fi

    log "[serial] Checking device permissions before ROS startup..."
    for dev in "${required_devices[@]}" "${optional_devices[@]}"; do
        if [ -r "$dev" ] && [ -w "$dev" ]; then
            log "  $(ls -l "$dev")"
            continue
        fi

        log "[serial] $dev is not writable; requesting permission once..."
        if ! chmod 666 "$dev" 2>/dev/null; then
            sudo chmod 666 "$dev" ||
                die "Unable to grant permission for $dev. Run: sudo chmod 666 $dev"
        fi
        [ -r "$dev" ] && [ -w "$dev" ] ||
            die "Current user still cannot read/write $dev"
        log "  $(ls -l "$dev")"
    done
}

check_jazzy_build_dependencies() {
    is_true "$SKIP_BUILD" && return 0

    local package
    local -a missing=()
    local -a required_apt_packages=(
        python3-empy
        python3-lark
        python3-yaml
        ros-jazzy-navigation2
        ros-jazzy-nav2-bringup
        ros-jazzy-cartographer-ros
        ros-jazzy-laser-filters
        ros-jazzy-rosbridge-server
        ros-jazzy-rmw-cyclonedds-cpp
    )

    for package in "${required_apt_packages[@]}"; do
        dpkg -s "$package" >/dev/null 2>&1 || missing+=("$package")
    done

    if [ "${#missing[@]}" -gt 0 ]; then
        log "[error] Missing ROS 2 Jazzy runtime/build dependencies:"
        printf '  - %s\n' "${missing[@]}"
        log "Install them with:"
        log "  sudo apt install -y ${missing[*]}"
        die "Build dependencies are incomplete."
    fi

    [ -x "$SYSTEM_PYTHON" ] ||
        die "System Python is missing or not executable: $SYSTEM_PYTHON"
    if ! "$SYSTEM_PYTHON" -I -c 'import em, lark, yaml' >/dev/null 2>&1; then
        log "[error] Jazzy build modules are unavailable to $SYSTEM_PYTHON."
        log "Install them with:"
        log "  sudo apt install -y python3-empy python3-lark python3-yaml"
        die "System Python build modules are incomplete."
    fi
}

publish_web_control() {
    local json_payload="$1"
    hard_timeout 2s ros2 topic pub --once \
        /robot/web_control std_msgs/msg/String \
        "{data: '$json_payload'}" \
        >/dev/null 2>&1 || true
}

prepare_safe_shutdown() {
    [ "$ROS_ENV_READY" = true ] || return 0

    log "[stop] Sending stop/cancel commands before final save..."

    # Stop both goal producers first. Then send a zero MOVE frame and disable
    # further serial motion output so Nav2 cannot restart the car while the
    # full-save profile is writing its final files.
    publish_web_control '{"command":"auto_mapping_stop"}'
    publish_web_control '{"command":"cancel_nav_goal"}'
    publish_web_control '{"command":"serial_command","action":"zero_move"}'
    sleep 0.2
    publish_web_control '{"command":"runtime_options","motion_serial_enabled":false}'
    publish_web_control '{"command":"slam_log_disable"}'

    hard_timeout 2s ros2 topic pub --once \
        /cmd_vel_safe geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        >/dev/null 2>&1 || true
    sleep 1
    log "[stop] Stop/cancel command stage completed."
}

save_final_artifacts() {
    if ! is_true "$SAVE_FINAL_ARTIFACTS"; then
        log "[save] Final save is disabled for profile $RUN_PROFILE_NAME."
        return 0
    fi
    if [ "$STACK_STARTED" != true ]; then
        log "[warn] Mapping stack never reached the started state; no final files can be saved."
        return 0
    fi
    if [ "$ROS_ENV_READY" != true ]; then
        log "[warn] ROS environment is unavailable; no final files can be saved."
        return 0
    fi

    if [ -z "$STACK_PGID" ] || ! process_group_alive "$STACK_PGID"; then
        log "[warn] Mapping launch group is not reported alive; checking ROS services anyway."
    fi

    mkdir -p "$FINAL_DIR"
    local map_base="$FINAL_DIR/final_map"
    local pbstream_path="$FINAL_DIR/result.pbstream"
    local request
    request="{filename: '$pbstream_path'}"

    log "[save] Final-save sequence started. Press Ctrl+C again only to abort without saving."
    log "[save] Saving Cartographer PBStream first..."
    if hard_timeout 5s ros2 service list 2>/dev/null |
        grep -Fxq "/write_state"; then
        if hard_timeout 45s ros2 service call \
            /write_state cartographer_ros_msgs/srv/WriteState "$request"; then
            if [ -s "$pbstream_path" ]; then
                log "[save] PBStream: $pbstream_path"
            else
                log "[warn] /write_state returned but PBStream is missing or empty."
            fi
        else
            log "[warn] PBStream save failed or timed out."
        fi
    else
        log "[warn] /write_state is unavailable; PBStream was not saved."
    fi

    log "[save] Saving final PGM/YAML..."
    if hard_timeout 35s ros2 run nav2_map_server map_saver_cli \
        -f "$map_base" \
        --ros-args \
        -p map_subscribe_transient_local:=true \
        -p save_map_timeout:=20.0; then
        if [ -s "$map_base.pgm" ] && [ -s "$map_base.yaml" ]; then
            log "[save] Final map: $map_base.pgm + $map_base.yaml"
        else
            log "[warn] map_saver_cli returned but PGM/YAML is missing or empty."
        fi
    else
        log "[warn] Final PGM/YAML save failed or timed out."
    fi

    if [ -s "$pbstream_path" ] &&
        [ -s "$map_base.pgm" ] && [ -s "$map_base.yaml" ]; then
        log "[save] Final-save sequence completed successfully."
    else
        log "[warn] Final-save sequence completed with missing files. See messages above."
    fi
}

shutdown_nav2_lifecycle() {
    local node

    [ "$ROS_ENV_READY" = true ] || return 0
    [ "$STACK_STARTED" = true ] || return 0
    log "[stop] Requesting an orderly Nav2 lifecycle shutdown..."

    if hard_timeout 4s ros2 service list 2>/dev/null |
        grep -Fxq "/lifecycle_manager_navigation/manage_nodes"; then
        if hard_timeout 20s ros2 service call \
            /lifecycle_manager_navigation/manage_nodes \
            nav2_msgs/srv/ManageLifecycleNodes \
            "{command: 4}" >/dev/null 2>&1; then
            log "[stop] Nav2 lifecycle manager completed shutdown."
            sleep 2
            return 0
        fi
        log "[warn] Nav2 lifecycle manager shutdown timed out; using node fallback."
    fi

    # Fallback if the lifecycle-manager service cannot complete. Clean plugin state
    # before the launch process receives SIGINT to avoid CycloneDDS teardown
    # races in controller_server/behavior_server.
    for node in /waypoint_follower /bt_navigator /behavior_server /planner_server /velocity_smoother /controller_server; do
        hard_timeout 6s ros2 lifecycle set "$node" shutdown >/dev/null 2>&1 || true
    done
    sleep 2
}

run_cleanup_stage() {
    local stage_function="$1"
    local stage_status=0

    "$stage_function" &
    CLEANUP_STAGE_PID=$!
    if wait "$CLEANUP_STAGE_PID"; then
        stage_status=0
    else
        stage_status=$?
    fi
    CLEANUP_STAGE_PID=""
    return "$stage_status"
}

force_shutdown() {
    local exit_code="${1:-130}"
    local pid

    trap - HUP INT TERM EXIT
    log ""
    log "[stop] Forced shutdown requested; aborting save and killing remaining processes..."
    if [ -n "$CLEANUP_STAGE_PID" ] && kill -0 "$CLEANUP_STAGE_PID" 2>/dev/null; then
        kill_process_tree KILL "$CLEANUP_STAGE_PID"
    fi
    for pid in "${PROCESS_PIDS[@]}"; do
        process_group_alive "$pid" && signal_process_group "$pid" KILL
    done
    kill_known_stack_processes KILL
    fuser -k "$LIDAR_PORT" "$CHASSIS_PORT" 2>/dev/null || true
    fuser -k 9090/tcp 5173/tcp 8080/tcp 2>/dev/null || true
    if [ "$ROS_ENV_READY" = true ]; then
        hard_timeout 1s ros2 daemon stop >/dev/null 2>&1 || true
    fi
    exit "$exit_code"
}

request_shutdown() {
    local exit_code="$1"
    if [ "$CLEANUP_STARTED" = true ]; then
        force_shutdown "$exit_code"
    fi
    exit "$exit_code"
}

cleanup() {
    local exit_code=$?
    local index pid name

    if [ "$CLEANUP_STARTED" = true ]; then
        return
    fi
    CLEANUP_STARTED=true
    trap - EXIT
    trap 'force_shutdown 143' HUP
    trap 'force_shutdown 130' INT
    trap 'force_shutdown 143' TERM

    log ""
    log "[stop] Stopping robot and shutting down the complete stack..."
    log "[stop] Press Ctrl+C again at any time to force-kill without waiting for saves."
    run_cleanup_stage prepare_safe_shutdown ||
        log "[warn] Stop/cancel stage returned an error; continuing cleanup."
    run_cleanup_stage save_final_artifacts ||
        log "[warn] Final-save stage returned an error; continuing cleanup."
    run_cleanup_stage shutdown_nav2_lifecycle ||
        log "[warn] Nav2 lifecycle stage returned an error; continuing cleanup."

    for ((index=${#PROCESS_PIDS[@]} - 1; index >= 0; index--)); do
        pid="${PROCESS_PIDS[$index]}"
        name="${PROCESS_NAMES[$index]}"
        if process_group_alive "$pid"; then
            log "[stop] SIGINT -> $name (pgid=$pid)"
            signal_process_group "$pid" INT
        fi
    done

    if ! wait_for_process_groups 10; then
        log "[stop] Some groups ignored SIGINT; sending SIGTERM..."
        for pid in "${PROCESS_PIDS[@]}"; do
            process_group_alive "$pid" && signal_process_group "$pid" TERM
        done
    fi

    if ! wait_for_process_groups 4; then
        log "[stop] Forcing the remaining groups to exit..."
        for pid in "${PROCESS_PIDS[@]}"; do
            process_group_alive "$pid" && signal_process_group "$pid" KILL
        done
    fi

    for pid in "${PROCESS_PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done

    kill_known_stack_processes TERM
    sleep 1
    kill_known_stack_processes KILL
    fuser -k 9090/tcp 5173/tcp 8080/tcp 2>/dev/null || true

    if [ "$ROS_ENV_READY" = true ]; then
        hard_timeout 3s ros2 daemon stop >/dev/null 2>&1 || true
    fi

    log "[stop] Shutdown complete. Session directory: $RUN_DIR"
    exit "$exit_code"
}

trap 'request_shutdown 130' INT
trap 'request_shutdown 143' TERM
trap 'request_shutdown 143' HUP
trap cleanup EXIT

[ "$NAV_PROFILE" = "jazzy_native" ] ||
    die "NAV_PROFILE is fixed to jazzy_native on Ubuntu 24.04 / ROS 2 Jazzy."

# Normalize display toggles so the launch summary is the exact value passed
# into ROS, including when a parent shell exported an older setting.
if is_true "$USE_RVIZ"; then USE_RVIZ=true; else USE_RVIZ=false; fi
if is_true "$SHOW_NAVI_GUI"; then SHOW_NAVI_GUI=true; else SHOW_NAVI_GUI=false; fi

auto_detect_ports

log "============================================================"
log "  CAR unified launcher"
log "  Profile       : $RUN_PROFILE_NAME"
log "  SLAM backend  : Cartographer V13 only"
log "  Nav profile   : $NAV_PROFILE (SmacPlanner2D + Regulated Pure Pursuit)"
log "  LiDAR         : $LIDAR_PORT @ $LIDAR_BAUD"
log "  STM32         : $CHASSIS_PORT @ 115200"
log "  ROS domain    : $ROS_DOMAIN_ID"
log "  Vision        : $ENABLE_VISION"
log "  Auto frontier : $AUTO_START"
log "  RViz           : $USE_RVIZ"
log "  Nav activation : after first Cartographer /map"
log "  Build Python   : $SYSTEM_PYTHON"
log "  Periodic log  : web controlled, default OFF / ${LOG_DEFAULT_INTERVAL_SEC}s"
log "  Final save    : $SAVE_FINAL_ARTIFACTS"
log "  NAVI GUI      : $SHOW_NAVI_GUI"
log "  Runtime log   : $RUNTIME_STACK_LOG"
log "============================================================"

[ -f /opt/ros/jazzy/setup.bash ] ||
    die "/opt/ros/jazzy/setup.bash was not found. Install ROS 2 Jazzy first."
[ -d "$LIDAR_WS" ] || die "ROS workspace not found: $LIDAR_WS"
[ -f "$ROOT_DIR/slam_logger.py" ] ||
    die "slam_logger.py not found under $ROOT_DIR"
[ -d "$ROAD_DIR" ] || die "Vision project not found: $ROAD_DIR"
[ -f "$WEB_DIR/package.json" ] || die "Web project not found: $WEB_DIR"
[ -f "$WEB_DIR/node_modules/vite/bin/vite.js" ] ||
    die "Vite is missing. Run npm install in: $WEB_DIR"
[ -e "$LIDAR_PORT" ] ||
    die "LiDAR serial device is missing: $LIDAR_PORT"
[ -e "$CHASSIS_PORT" ] ||
    die "STM32 serial device is missing: $CHASSIS_PORT"
command -v pgrep >/dev/null 2>&1 || die "pgrep is missing. Install package: procps"
command -v pkill >/dev/null 2>&1 || die "pkill is missing. Install package: procps"
command -v ps >/dev/null 2>&1 || die "ps is missing. Install package: procps"
command -v fuser >/dev/null 2>&1 || die "fuser is missing. Install package: psmisc"
command -v readlink >/dev/null 2>&1 || die "readlink is missing. Install package: coreutils"
command -v dpkg >/dev/null 2>&1 || die "dpkg is missing."

cleanup_before_start
grant_device_permissions
mkdir -p "$LOGGER_BASE"
if is_true "$SAVE_FINAL_ARTIFACTS"; then
    mkdir -p "$FINAL_DIR"
fi

unset PYTHONPATH ROS_PACKAGE_PATH ROS_ROOT ROS_ETC_DIR ROS_VERSION
unset CMAKE_PREFIX_PATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID
export VISION_CODE_DIR="$ROAD_DIR"
ROS_ENV_READY=true

command -v colcon >/dev/null 2>&1 || die "colcon is not installed."
command -v setsid >/dev/null 2>&1 || die "setsid is not installed."
command -v timeout >/dev/null 2>&1 || die "timeout is not installed."
command -v node >/dev/null 2>&1 || die "Node.js is not installed."
check_jazzy_build_dependencies

mkdir -p "$ASCII_WS_BASE"
if [ -L "$ASCII_SRC_LINK" ]; then
    rm -f "$ASCII_SRC_LINK"
elif [ -e "$ASCII_SRC_LINK" ]; then
    die "ASCII workspace source path exists and is not a symlink: $ASCII_SRC_LINK"
fi
ln -s "$LIDAR_WS/src" "$ASCII_SRC_LINK"
cd "$ASCII_WS_BASE"
if ! is_true "$SKIP_BUILD"; then
    log "[1/7] Building Jazzy frontier, C++ perception and lidar_py via ASCII workspace..."
    PYTHONNOUSERSITE=1 colcon --log-base "$AUTO_LOG_BASE" build \
        --base-paths "$ASCII_SRC_LINK" \
        --build-base "$AUTO_BUILD_BASE" \
        --install-base "$AUTO_INSTALL_BASE" \
        --symlink-install \
        --packages-up-to lidar_py \
        --cmake-args "-DPython3_EXECUTABLE=$SYSTEM_PYTHON"
else
    log "[1/7] SKIP_BUILD=true; using the existing build."
fi

[ -f "$AUTO_INSTALL_BASE/setup.bash" ] ||
    die "Build output is missing: $AUTO_INSTALL_BASE/setup.bash"
source "$AUTO_INSTALL_BASE/setup.bash"

for package in \
    nav2_smac_planner nav2_regulated_pure_pursuit_controller \
    nav2_velocity_smoother frontier_exploration_ros2 lidar_py; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 ||
        die "Required Jazzy package is not visible: $package"
done

LIDAR_SHARE="$(ros2 pkg prefix lidar_py)/share/lidar_py"
NAV_PARAMS_FILE="$LIDAR_SHARE/config/nav2_auto_mapping_jazzy.yaml"
CONTROLLER_OVERRIDE_FILE="$LIDAR_SHARE/config/nav2_dual_3d_rpp_override.yaml"
FRONTIER_PARAMS_FILE="$LIDAR_SHARE/config/frontier_auto_mapping_jazzy.yaml"
BT_XML_FILE="$LIDAR_SHARE/behavior_trees/navigate_to_pose_jazzy.xml"
THROUGH_BT_XML_FILE="$LIDAR_SHARE/behavior_trees/navigate_through_poses_jazzy.xml"

[ -f "$NAV_PARAMS_FILE" ] || die "Nav2 config missing: $NAV_PARAMS_FILE"
[ -f "$CONTROLLER_OVERRIDE_FILE" ] ||
    die "Smac/RPP override missing: $CONTROLLER_OVERRIDE_FILE"
[ -f "$FRONTIER_PARAMS_FILE" ] || die "Frontier config missing: $FRONTIER_PARAMS_FILE"
[ -f "$BT_XML_FILE" ] || die "Behavior tree missing: $BT_XML_FILE"
[ -f "$THROUGH_BT_XML_FILE" ] || die "Through-poses behavior tree missing: $THROUGH_BT_XML_FILE"
verify_mapping_baseline

timeout 5s ros2 daemon stop >/dev/null 2>&1 || true
timeout 8s ros2 daemon start >/dev/null 2>&1 || true

log "[2/7] Starting ROSBridge..."
start_process_group "rosbridge" \
    ros2 run rosbridge_server rosbridge_websocket \
    --ros-args -p port:=9090 -p address:=0.0.0.0
wait_for_port 9090 20 "ROSBridge"

log "[3/7] Starting tested Cartographer V13 + native Jazzy Nav2..."
log "[3/7] Full ROS stack output is written directly to: $RUNTIME_STACK_LOG"
start_process_group_logged "mapping-nav-stack" "$RUNTIME_STACK_LOG" \
    env PYTHONPATH="/usr/lib/python3/dist-packages:$HOME/.local/lib/python3.12/site-packages:$ROAD_DIR:${PYTHONPATH:-}" \
    ros2 launch lidar_py cartographer_auto_mapping_jazzy_launch.py \
    lidar_serial_port:="$LIDAR_PORT" \
    lidar_baudrate:="$LIDAR_BAUD" \
    chassis_serial_port:="$CHASSIS_PORT" \
    nav_autostart:="false" \
    explorer_autostart:="$AUTO_START" \
    require_depth_baseline:="$ENABLE_VISION" \
    nav_params_file:="$NAV_PARAMS_FILE" \
    controller_override_file:="$CONTROLLER_OVERRIDE_FILE" \
    frontier_params_file:="$FRONTIER_PARAMS_FILE" \
    bt_xml_file:="$BT_XML_FILE" \
    through_bt_xml_file:="$THROUGH_BT_XML_FILE" \
    show_serial_window:="$SHOW_NAVI_GUI" \
    launch_rviz:="$USE_RVIZ"
STACK_PGID="$LAST_STARTED_PID"

wait_for_ros_node /chassis_node 45 || die "Chassis node startup failed."
wait_for_ros_node /lidar_node 45 || die "LiDAR node startup failed."
wait_for_ros_node /cartographer_node 45 || die "Cartographer startup failed."
wait_for_ros_node /cartographer_occupancy_grid_node 45 ||
    die "Cartographer occupancy-grid startup failed."
wait_for_ros_node /controller_server 45 || die "Nav2 controller server startup failed."
wait_for_ros_node /velocity_smoother 45 || die "Nav2 velocity smoother startup failed."
wait_for_ros_node /planner_server 45 || die "Nav2 planner server startup failed."
wait_for_ros_node /behavior_server 45 || die "Nav2 behavior server startup failed."
wait_for_ros_node /bt_navigator 45 || die "Nav2 BT navigator startup failed."
wait_for_ros_node /waypoint_follower 45 || die "Nav2 waypoint follower startup failed."
wait_for_ros_node /frontier_explorer 45 || die "Jazzy frontier explorer startup failed."
wait_for_ros_node /frontier_web_bridge 45 || die "Frontier web bridge startup failed."
wait_for_ros_node /lifecycle_manager_navigation 45 ||
    die "Nav2 lifecycle manager startup failed."
if is_true "$USE_RVIZ"; then
    wait_for_ros_node /rviz2 30 ||
        die "RViz was requested but /rviz2 did not start. Check DISPLAY and Qt errors above."
fi
for node in \
    /chassis_node /lidar_node /cartographer_node \
    /cartographer_occupancy_grid_node \
    /controller_server /velocity_smoother /planner_server /behavior_server \
    /bt_navigator /waypoint_follower /frontier_explorer /frontier_web_bridge; do
    assert_single_ros_node "$node" ||
        die "Duplicate ROS node detected before Nav2 activation."
done
for topic in /odom /imu_cartographer /scan_timed_v2_filtered; do
    assert_single_topic_publisher "$topic" ||
        die "Duplicate or missing Cartographer input detected."
done
wait_for_topic_sample /odom nav_msgs/msg/Odometry 30 "odometry sample" ||
    die "STM32 NAVI odometry is not arriving; check 0x07 frames and $CHASSIS_PORT."
wait_for_topic_sample /imu_cartographer sensor_msgs/msg/Imu 30 "IMU sample" ||
    die "Cartographer IMU is not arriving; check STM32 yaw/wz telemetry."
wait_for_topic_sample /scan_timed_v2_filtered sensor_msgs/msg/LaserScan 45 \
    "filtered LiDAR scan" ||
    die "Filtered LiDAR scans are not arriving; check $LIDAR_PORT and scan filtering."
wait_for_topic_sample /map nav_msgs/msg/OccupancyGrid 60 "Cartographer map" ||
    die "Cartographer did not publish /map. Check sensor timestamps and the runtime log above."
start_nav2_lifecycle ||
    die "Nav2 lifecycle startup failed after the first Cartographer map."
wait_for_ros_lifecycle_active /controller_server 45 ||
    die "Nav2 controller_server exists but is not active. Check the first lifecycle error above."
wait_for_ros_lifecycle_active /velocity_smoother 45 ||
    die "Nav2 velocity_smoother exists but is not active."
wait_for_ros_lifecycle_active /planner_server 45 ||
    die "Nav2 planner_server exists but is not active. Check SmacPlanner2D loading above."
wait_for_ros_lifecycle_active /behavior_server 45 ||
    die "Nav2 behavior_server exists but is not active."
wait_for_ros_lifecycle_active /bt_navigator 45 ||
    die "Nav2 bt_navigator exists but is not active."
wait_for_ros_lifecycle_active /waypoint_follower 45 ||
    die "Nav2 waypoint_follower exists but is not active."
wait_for_ros_action /navigate_to_pose 30 ||
    die "Nav2 action /navigate_to_pose is unavailable; web navigation cannot work."
wait_for_ros_action /compute_path_to_pose 30 ||
    die "Nav2 action /compute_path_to_pose is unavailable; web path preview cannot work."
wait_for_ros_service /control_exploration 30 ||
    die "Native Jazzy frontier control service is unavailable."
wait_for_ros_service /auto_mapping/set_enabled 30 ||
    die "Frontier web bridge service is unavailable."
STACK_STARTED=true

if is_true "$ENABLE_VISION"; then
    log "[4/7] Starting SDK depth obstacle and MJPEG..."
    start_process_group "depth-obstacle" \
        env PYTHONPATH="/usr/lib/python3/dist-packages:$HOME/.local/lib/python3.12/site-packages:$ROAD_DIR:${PYTHONPATH:-}" \
        "$SYSTEM_PYTHON" "$ROAD_DIR/depth_obstacle_node.py"
    wait_for_port 8080 35 "MJPEG video"
else
    log "[4/7] Vision is disabled for this run."
fi

log "[5/7] Starting web-controlled SLAM logger..."
start_process_group "slam-logger" \
    python3 "$ROOT_DIR/slam_logger.py" \
    --ros-args \
    -p log_dir:="$LOGGER_BASE" \
    -p save_interval_sec:="$LOG_DEFAULT_INTERVAL_SEC" \
    -p start_enabled:=false
wait_for_ros_node /slam_logger 20

if is_true "$START_WEB"; then
    log "[6/7] Starting web UI..."
    start_process_group "web-ui" \
        bash -c 'cd "$1" && exec node "$1/node_modules/vite/bin/vite.js" --host 0.0.0.0' \
        _ "$WEB_DIR"
    wait_for_port 5173 20 "Web UI"
else
    log "[6/7] Web UI is disabled for this run."
fi

log "[7/7] Startup checks passed."
HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
HOST_IP="${HOST_IP:-127.0.0.1}"

log ""
log "============================================================"
log "  Local web    : http://127.0.0.1:5173"
log "  LAN web      : http://$HOST_IP:5173"
if is_true "$ENABLE_VISION"; then
    log "  Video        : http://$HOST_IP:8080/video_feed"
fi
log "  Session      : $RUN_DIR"
log "  Runtime log  : $RUNTIME_STACK_LOG"
log "  Periodic log : controlled by the web SLAM Log switch"
if is_true "$SAVE_FINAL_ARTIFACTS"; then
    log "  On exit      : save Final/final_map.pgm + .yaml + result.pbstream"
else
    log "  On exit      : do not save a final map or PBStream"
fi
log "  Stop         : Ctrl+C once, then wait for shutdown complete"
log "============================================================"

while true; do
    sleep 2
    for index in "${!PROCESS_PIDS[@]}"; do
        if ! process_group_alive "${PROCESS_PIDS[$index]}"; then
            die "Process group exited unexpectedly: ${PROCESS_NAMES[$index]}"
        fi
    done
done
