#!/usr/bin/env bash
set -Eeo pipefail

# Optional preflight for Ubuntu 24.04 / ROS 2 Jazzy. open_all.sh performs the
# same critical checks, so this script is useful after installation or a pull,
# but it is not required before every run.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_WS="${LIDAR_WS:-$ROOT_DIR/lidar/chapt1_ws}"
ASCII_WS_BASE="${CAR_JAZZY_BUILD_ROOT:-$HOME/.cache/huichuan_agv_jazzy_ws}"
ASCII_SRC_LINK="$ASCII_WS_BASE/src"
SYSTEM_PYTHON="${CAR_SYSTEM_PYTHON:-/usr/bin/python3}"
BUILD_REQUESTED=false

if [ "${1:-}" = "--build" ]; then
    BUILD_REQUESTED=true
elif [ -n "${1:-}" ] && [ "${1:-}" != "--check-only" ]; then
    echo "Usage: $0 [--check-only|--build]"
    exit 2
fi

fail() {
    echo "[FAIL] $*" >&2
    exit 1
}

pass() {
    echo "[ OK ] $*"
}

[ -f /opt/ros/jazzy/setup.bash ] || fail "Missing /opt/ros/jazzy/setup.bash"
source /opt/ros/jazzy/setup.bash
pass "ROS_DISTRO=${ROS_DISTRO:-unknown}"
[ "${ROS_DISTRO:-}" = "jazzy" ] || fail "ROS 2 Jazzy is not active"

for command in colcon ros2 sha256sum; do
    command -v "$command" >/dev/null 2>&1 || fail "Missing command: $command"
done
for command in setsid timeout; do
    command -v "$command" >/dev/null 2>&1 || fail "Missing command: $command"
done
for script in \
    "$ROOT_DIR/open_all.sh" \
    "$ROOT_DIR/open_all_log.sh" \
    "$ROOT_DIR/validate_auto_mapping_jazzy.sh"; do
    bash -n "$script" || fail "Bash syntax error: $script"
done
pass "Launcher and validation Bash syntax is valid"
[ -x "$SYSTEM_PYTHON" ] || fail "System Python is missing: $SYSTEM_PYTHON"
"$SYSTEM_PYTHON" -I -c 'import em, lark, yaml' >/dev/null 2>&1 ||
    fail "Install system build modules: sudo apt install -y python3-empy python3-lark python3-yaml"
pass "Jazzy build Python=$SYSTEM_PYTHON (user/uv Python is isolated)"

for package in \
    cartographer_ros laser_filters rosbridge_server rmw_cyclonedds_cpp \
    nav2_controller nav2_planner nav2_smac_planner \
    dwb_core dwb_critics dwb_plugins \
    nav2_rotation_shim_controller nav2_smoother nav2_constrained_smoother \
    nav2_behaviors nav2_bt_navigator nav2_lifecycle_manager \
    nav2_waypoint_follower nav2_map_server; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 || fail "Missing ROS package: $package"
done
pass "Jazzy Cartographer/Nav2 runtime packages are installed"

CARTOGRAPHER_EXECUTABLES="$(ros2 pkg executables cartographer_ros | awk '{print $2}')"
for executable in cartographer_node cartographer_occupancy_grid_node; do
    grep -Fxq "$executable" <<<"$CARTOGRAPHER_EXECUTABLES" ||
        fail "Missing Jazzy Cartographer executable: $executable"
done
pass "Jazzy Cartographer executable names are valid"

LIDAR_SRC="$LIDAR_WS/src/lidar_py"
for file in \
    "$LIDAR_SRC/launch/cartographer_scan_v2_launch.py" \
    "$LIDAR_SRC/launch/cartographer_auto_mapping_jazzy_launch.py" \
    "$LIDAR_SRC/config/cartographer_2d_v9_tightened.lua" \
    "$LIDAR_SRC/config/laser_filter.yaml" \
    "$LIDAR_SRC/config/nav2_auto_mapping_jazzy.yaml" \
    "$LIDAR_SRC/config/frontier_auto_mapping_jazzy.yaml" \
    "$LIDAR_SRC/config/lattice_forward_turnaround_5cm.json" \
    "$LIDAR_SRC/behavior_trees/navigate_to_pose_jazzy.xml" \
    "$LIDAR_WS/src/frontier_exploration_ros2/package.xml" \
    "$LIDAR_WS/src/short_goal_bt/package.xml"; do
    [ -r "$file" ] || fail "Missing project file: $file"
done
pass "Jazzy source/config set is complete"

"$SYSTEM_PYTHON" -I - "$LIDAR_SRC" "$ROOT_DIR" <<'PY'
import json
import pathlib
import py_compile
import sys
import xml.etree.ElementTree as ET

import yaml

root = pathlib.Path(sys.argv[1])
repo_root = pathlib.Path(sys.argv[2])
scan_launch = (
    root / "launch" / "cartographer_scan_v2_launch.py"
).read_text(encoding="utf-8")
if 'executable="cartographer_occupancy_grid_node"' not in scan_launch:
    raise SystemExit("Jazzy occupancy-grid executable name is missing")
if 'executable="occupancy_grid_node"' in scan_launch:
    raise SystemExit("obsolete Cartographer occupancy-grid executable is present")

launcher = (repo_root / "open_all.sh").read_text(encoding="utf-8")
required_startup_guards = (
    'nav_autostart:="false"',
    'PYTHONPATH="/usr/lib/python3/dist-packages:$HOME/.local/lib/python3.12/site-packages:$ROAD_DIR:${PYTHONPATH:-}"',
    '"$SYSTEM_PYTHON" "$ROAD_DIR/depth_obstacle_node.py"',
    'wait_for_topic_sample /odom nav_msgs/msg/Odometry',
    'wait_for_topic_sample /imu_cartographer sensor_msgs/msg/Imu',
    'wait_for_topic_sample /scan_timed_v2_filtered sensor_msgs/msg/LaserScan',
    'wait_for_topic_sample /map nav_msgs/msg/OccupancyGrid',
    'start_nav2_lifecycle',
)
for guard in required_startup_guards:
    if guard not in launcher:
        raise SystemExit(f"missing staged startup guard: {guard}")

for name in ("nav2_auto_mapping_jazzy.yaml", "frontier_auto_mapping_jazzy.yaml"):
    with (root / "config" / name).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not data:
        raise SystemExit(f"invalid YAML: {name}")

with (root / "config" / "lattice_forward_turnaround_5cm.json").open(
        "r", encoding="utf-8") as stream:
    lattice = json.load(stream)
if "lattice_metadata" not in lattice or not lattice.get("primitives"):
    raise SystemExit("invalid State Lattice JSON")

nav = yaml.safe_load(
    (root / "config" / "nav2_auto_mapping_jazzy.yaml").read_text(
        encoding="utf-8"))
controller = nav["controller_server"]["ros__parameters"]
if controller.get("progress_checker_plugins") != ["progress_checker"]:
    raise SystemExit("Jazzy requires progress_checker_plugins as a string list")
if "progress_checker_plugin" in controller:
    raise SystemExit("obsolete singular progress_checker_plugin is present")
if controller["FollowPath"].get("plugin") != (
        "nav2_rotation_shim_controller::RotationShimController"):
    raise SystemExit("unexpected Jazzy Rotation Shim plugin type")
if controller["FollowPath"].get("primary_controller") != (
        "dwb_core::DWBLocalPlanner"):
    raise SystemExit("unexpected Jazzy DWB primary controller type")
if nav["planner_server"]["ros__parameters"]["GridBased"].get("plugin") != (
        "nav2_smac_planner::SmacPlannerLattice"):
    raise SystemExit("unexpected Jazzy State Lattice plugin type")

for name in ("navigate_to_pose_jazzy.xml", "navigate_through_poses_jazzy.xml"):
    tree = ET.parse(root / "behavior_trees" / name)
    if tree.getroot().attrib.get("BTCPP_format") != "4":
        raise SystemExit(f"{name} is not BehaviorTree.CPP v4")

for path in (
    root / "launch" / "cartographer_auto_mapping_jazzy_launch.py",
    root / "lidar_py" / "frontier_web_bridge.py",
    root / "lidar_py" / "web_goal_nav_node.py",
    root / "lidar_py" / "web_path_preview_node.py",
):
    py_compile.compile(str(path), doraise=True)
PY
pass "YAML, lattice, BT.CPP v4, staged startup and Python syntax are valid"

declare -A EXPECTED_HASHES=(
    ["$LIDAR_SRC/config/cartographer_2d_v9_tightened.lua"]="00dfd1c721f0fe8c61ac6f2b417001920694e4fc77e895fb4a1f194330c910d9"
    ["$LIDAR_SRC/launch/cartographer_scan_v2_launch.py"]="5650100fbdaf7fd40bdb6cc8dfaa1d642b7fa44aa914028c48817af5be5a9106"
    ["$LIDAR_SRC/config/laser_filter.yaml"]="8583a2ca7e99a29b13f2fc339df468e621562d61f0adfa1e7e1828254705b306"
)
for file in "${!EXPECTED_HASHES[@]}"; do
    actual="$(sha256sum "$file" | awk '{print tolower($1)}')"
    [ "$actual" = "${EXPECTED_HASHES[$file]}" ] ||
        fail "Frozen mapping baseline changed: $file"
done
pass "Frozen Cartographer V13 baseline hashes match"

SMOKE_PIDS=()
SMOKE_NAMES=()
SMOKE_DIR=""

cleanup_smoke() {
    local pid
    for pid in "${SMOKE_PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT -- "-$pid" 2>/dev/null || true
        fi
    done
    sleep 0.5
    for pid in "${SMOKE_PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL -- "-$pid" 2>/dev/null || true
        fi
        wait "$pid" 2>/dev/null || true
    done
    SMOKE_PIDS=()
    SMOKE_NAMES=()
}

show_smoke_failure() {
    local node="$1"
    echo "[FAIL] Jazzy lifecycle configure failed: /$node" >&2
    if [ -r "$SMOKE_DIR/$node.log" ]; then
        grep -E -i \
            "error|fatal|exception|failed|plugin|parameter|segmentation" \
            "$SMOKE_DIR/$node.log" | tail -n 80 >&2 || true
        echo "--- $node.log tail ---" >&2
        tail -n 80 "$SMOKE_DIR/$node.log" >&2 || true
    fi
    cleanup_smoke
    exit 1
}

start_smoke_node() {
    local node="$1"
    shift
    env ROS_DOMAIN_ID="$SMOKE_DOMAIN" \
        RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        RCUTILS_LOGGING_BUFFERED_STREAM=1 \
        setsid "$@" >"$SMOKE_DIR/$node.log" 2>&1 &
    SMOKE_PIDS+=("$!")
    SMOKE_NAMES+=("$node")
}

wait_for_smoke_node() {
    local node="$1"
    local deadline=$((SECONDS + 20))
    while [ "$SECONDS" -lt "$deadline" ]; do
        # A node can appear in the graph before its lifecycle services are
        # ready.  Waiting on `lifecycle get` removes that discovery race.
        if timeout 3s ros2 lifecycle get "/$node" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.25
    done
    show_smoke_failure "$node"
}

configure_smoke_node() {
    local node="$1"
    local output
    if ! output="$(timeout 20s ros2 lifecycle set "/$node" configure 2>&1)"; then
        echo "$output" >&2
        show_smoke_failure "$node"
    fi
    echo "$output" | grep -qi "success" || {
        echo "$output" >&2
        show_smoke_failure "$node"
    }
    pass "Jazzy lifecycle configure: /$node"
}

run_nav2_lifecycle_smoke() {
    local runtime_nav_params="$1"
    local runtime_lattice="$2"
    local runtime_bt="$3"
    local runtime_through_bt="$4"

    SMOKE_DOMAIN="${CAR_VALIDATION_ROS_DOMAIN_ID:-187}"
    SMOKE_DIR="$(mktemp -d "${TMPDIR:-/tmp}/car_nav2_smoke.XXXXXX")"
    trap cleanup_smoke INT TERM EXIT

    # Keep the CLI and every tested server on the same DDS implementation and
    # domain.  A previously started ros2 daemon may belong to another domain.
    export ROS_DOMAIN_ID="$SMOKE_DOMAIN"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 daemon stop >/dev/null 2>&1 || true

    # Configure-only testing loads every plugin and parses every parameter but
    # never activates a velocity publisher, so no hardware or map is required.
    start_smoke_node controller_server \
        ros2 run nav2_controller controller_server --ros-args \
        --params-file "$runtime_nav_params"
    start_smoke_node smoother_server \
        ros2 run nav2_smoother smoother_server --ros-args \
        --params-file "$runtime_nav_params"
    start_smoke_node planner_server \
        ros2 run nav2_planner planner_server --ros-args \
        --params-file "$runtime_nav_params" \
        -p "GridBased.lattice_filepath:=$runtime_lattice"
    start_smoke_node behavior_server \
        ros2 run nav2_behaviors behavior_server --ros-args \
        --params-file "$runtime_nav_params"
    start_smoke_node bt_navigator \
        ros2 run nav2_bt_navigator bt_navigator --ros-args \
        --params-file "$runtime_nav_params" \
        -p "default_nav_to_pose_bt_xml:=$runtime_bt" \
        -p "default_nav_through_poses_bt_xml:=$runtime_through_bt"
    start_smoke_node waypoint_follower \
        ros2 run nav2_waypoint_follower waypoint_follower --ros-args \
        --params-file "$runtime_nav_params"

    for node in controller_server smoother_server planner_server \
        behavior_server bt_navigator waypoint_follower; do
        wait_for_smoke_node "$node"
        configure_smoke_node "$node"
    done

    cleanup_smoke
    trap - INT TERM EXIT
    rm -rf "$SMOKE_DIR"
    SMOKE_DIR=""
    pass "All Jazzy Nav2 plugins passed configure-only runtime smoke testing"
}

if [ "$BUILD_REQUESTED" = true ]; then
    mkdir -p "$ASCII_WS_BASE"
    if [ -L "$ASCII_SRC_LINK" ]; then
        rm -f "$ASCII_SRC_LINK"
    elif [ -e "$ASCII_SRC_LINK" ]; then
        fail "ASCII source path exists and is not a symlink: $ASCII_SRC_LINK"
    fi
    ln -s "$LIDAR_WS/src" "$ASCII_SRC_LINK"
    PYTHONNOUSERSITE=1 colcon --log-base "$ASCII_WS_BASE/log" build \
        --base-paths "$ASCII_SRC_LINK" \
        --build-base "$ASCII_WS_BASE/build" \
        --install-base "$ASCII_WS_BASE/install" \
        --symlink-install \
        --packages-up-to lidar_py \
        --cmake-args "-DPython3_EXECUTABLE=$SYSTEM_PYTHON"
    source "$ASCII_WS_BASE/install/setup.bash"
    for package in frontier_exploration_ros2 short_goal_bt lidar_py; do
        ros2 pkg prefix "$package" >/dev/null 2>&1 || fail "Build cannot find $package"
    done
    pass "Jazzy workspace build completed"

    RUNTIME_SHARE="$ASCII_WS_BASE/install/lidar_py/share/lidar_py"
    run_nav2_lifecycle_smoke \
        "$RUNTIME_SHARE/config/nav2_auto_mapping_jazzy.yaml" \
        "$RUNTIME_SHARE/config/lattice_forward_turnaround_5cm.json" \
        "$RUNTIME_SHARE/behavior_trees/navigate_to_pose_jazzy.xml" \
        "$RUNTIME_SHARE/behavior_trees/navigate_through_poses_jazzy.xml"
fi

echo "Jazzy migration preflight passed."
