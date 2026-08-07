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
    "$ROOT_DIR/START_DUAL_2D_3D_NAVIGATION.sh" \
    "$ROOT_DIR/START_DUAL_2D_3D_NAVIGATION_VISUAL_FUSION.sh" \
    "$ROOT_DIR/visual_laser_slam/run_dual_resolution_3d_slam.sh" \
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
    nav2_regulated_pure_pursuit_controller nav2_velocity_smoother \
    nav2_behaviors nav2_bt_navigator nav2_lifecycle_manager \
    nav2_waypoint_follower nav2_map_server spatio_temporal_voxel_layer \
    robot_localization rtabmap_odom; do
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
    "$LIDAR_SRC/config/cartographer_2d_v9_nav_guarded.lua" \
    "$LIDAR_SRC/config/laser_filter.yaml" \
    "$LIDAR_SRC/config/nav2_auto_mapping_jazzy.yaml" \
    "$LIDAR_SRC/config/nav2_dual_3d_rpp_override.yaml" \
    "$LIDAR_SRC/config/nav2_dual_3d_stvl_override.yaml" \
    "$LIDAR_SRC/config/ekf_dual_3d_visual_fusion.yaml" \
    "$LIDAR_SRC/config/frontier_auto_mapping_jazzy.yaml" \
    "$LIDAR_SRC/behavior_trees/navigate_to_pose_jazzy.xml" \
    "$LIDAR_SRC/behavior_trees/navigate_through_poses_jazzy.xml" \
    "$LIDAR_WS/src/frontier_exploration_ros2/package.xml" \
    "$LIDAR_WS/src/local_depth_cloud_cpp/package.xml" \
    "$LIDAR_WS/src/local_depth_cloud_cpp/src/depth_image_to_local_cloud_v21_node.cpp"; do
    [ -r "$file" ] || fail "Missing project file: $file"
done
pass "Jazzy source/config set is complete"

"$SYSTEM_PYTHON" -I - "$LIDAR_SRC" "$ROOT_DIR" <<'PY'
import math
import pathlib
import py_compile
import sys
import xml.etree.ElementTree as ET

import yaml

root = pathlib.Path(sys.argv[1])
repo_root = pathlib.Path(sys.argv[2])


class UniqueKeyLoader(yaml.SafeLoader):
    """Reject duplicate YAML keys instead of silently keeping the last one."""


def construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"duplicate key: {key!r}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    construct_unique_mapping,
)


def load_yaml(path):
    return yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)


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
    'controller_override_file:="$CONTROLLER_OVERRIDE_FILE"',
    'start_nav2_lifecycle',
)
for guard in required_startup_guards:
    if guard not in launcher:
        raise SystemExit(f"missing staged startup guard: {guard}")

for name in (
    "nav2_auto_mapping_jazzy.yaml",
    "nav2_dual_3d_rpp_override.yaml",
    "nav2_dual_3d_stvl_override.yaml",
    "ekf_dual_3d_visual_fusion.yaml",
    "frontier_auto_mapping_jazzy.yaml",
):
    data = load_yaml(root / "config" / name)
    if not isinstance(data, dict) or not data:
        raise SystemExit(f"invalid YAML: {name}")

nav = load_yaml(root / "config" / "nav2_auto_mapping_jazzy.yaml")
rpp = load_yaml(root / "config" / "nav2_dual_3d_rpp_override.yaml")
stvl = load_yaml(root / "config" / "nav2_dual_3d_stvl_override.yaml")
ekf = load_yaml(root / "config" / "ekf_dual_3d_visual_fusion.yaml")
bt_params = nav["bt_navigator"]["ros__parameters"]
if "plugin_lib_names" in bt_params:
    raise SystemExit(
        "omit bt_navigator.plugin_lib_names on Jazzy; an empty YAML list "
        "becomes an unset parameter and crashes bt_navigator")
active_nav_text = repr((nav, rpp, stvl))
for retired_plugin in (
    "RotationShimController",
    "DWBLocalPlanner",
    "SmacPlannerLattice",
    "MPPIController",
    "ConstrainedSmoother",
):
    if retired_plugin in active_nav_text:
        raise SystemExit(
            f"retired navigation plugin remains active: {retired_plugin}")
controller = rpp["controller_server"]["ros__parameters"]
if float(controller.get("failure_tolerance", 0.0)) < 2.0:
    raise SystemExit(
        "controller collision tolerance is too short for startup costmap settling")
if controller.get("progress_checker_plugins") != ["progress_checker"]:
    raise SystemExit("Jazzy requires progress_checker_plugins as a string list")
if "progress_checker_plugin" in controller:
    raise SystemExit("obsolete singular progress_checker_plugin is present")
if controller["FollowPath"].get("plugin") != (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"):
    raise SystemExit("unexpected Jazzy Regulated Pure Pursuit plugin type")
progress = controller["progress_checker"]
if abs(float(progress.get("movement_time_allowance", 0.0)) - 12.0) > 1e-6:
    raise SystemExit(
        "progress timeout must outlast bounded RGB-D decay and planner retry")
goal_checker = controller["goal_checker"]
if float(goal_checker.get("yaw_goal_tolerance", 0.0)) < math.pi:
    raise SystemExit(
        "point navigation must not spin indefinitely for the RViz goal arrow")
nav_guard = (
    root / "config" / "cartographer_2d_v9_nav_guarded.lua"
).read_text(encoding="utf-8")
if "POSE_GRAPH.optimize_every_n_nodes = 0" not in nav_guard:
    raise SystemExit(
        "navigation Cartographer must defer pose-graph optimization until stop")
if rpp["planner_server"]["ros__parameters"]["GridBased"].get("plugin") != (
        "nav2_smac_planner::SmacPlanner2D"):
    raise SystemExit("unexpected Jazzy SmacPlanner2D plugin type")
planner = rpp["planner_server"]["ros__parameters"]["GridBased"]
if planner.get("allow_unknown") is not False:
    raise SystemExit(
        "Smac must reject unknown space outside the observed map")
if float(planner.get("cost_travel_multiplier", 0.0)) < 2.0:
    raise SystemExit("SmacPlanner2D path is not sufficiently cost-aware")
if int(planner.get("smoother", {}).get("max_iterations", -1)) != 0:
    raise SystemExit(
        "SmacPlanner2D smoothing bypass is missing; use smoother.max_iterations=0")
local_costmap = stvl["local_costmap"]["local_costmap"]["ros__parameters"]
global_costmap = stvl["global_costmap"]["global_costmap"]["ros__parameters"]
local_plugins = local_costmap["plugins"]
global_plugins = global_costmap["plugins"]
for required in ("static_layer", "obstacle_layer", "depth_stvl_layer",
                 "visual_wall_stvl_layer", "inflation_layer"):
    if required not in local_plugins:
        raise SystemExit(f"local STVL plugin missing: {required}")
for required in ("static_layer", "obstacle_layer", "depth_global_stvl_layer",
                 "inflation_layer"):
    if required not in global_plugins:
        raise SystemExit(f"global STVL plugin missing: {required}")
if "visual_wall_global_stvl_layer" in global_plugins:
    raise SystemExit(
        "persistent RTAB walls must not duplicate global Cartographer walls")
for stale in ("depth_persistent_stvl_layer",
              "depth_persistent_global_stvl_layer"):
    if stale in local_plugins or stale in global_plugins:
        raise SystemExit(
            f"unbounded RGB-D persistent STVL must stay disabled: {stale}")
for label, costmap in (
        ("local", local_costmap),
        ("global", global_costmap)):
    if int(costmap.get("lethal_cost_threshold", -1)) != 65:
        raise SystemExit(
            f"{label} static map must retain Cartographer probability walls")
    if costmap.get("trinary_costmap") is not True:
        raise SystemExit(f"{label} static occupancy conversion is not trinary")
    if costmap.get("use_maximum") is not True:
        raise SystemExit(
            f"{label} later sensor layers may overwrite static map walls")
    static_layer = costmap.get("static_layer", {})
    if static_layer.get("map_topic") != "/map":
        raise SystemExit(f"{label} static layer is not bound to /map")
if local_costmap.get("static_layer", {}).get(
        "footprint_clearing_enabled") is not True:
    raise SystemExit(
        "local static layer must clear startup speckles under the robot footprint")
if global_costmap.get("static_layer", {}).get(
        "footprint_clearing_enabled") is not False:
    raise SystemExit(
        "global static walls must not be erased under the robot footprint")
global_stvl = stvl["global_costmap"]["global_costmap"]["ros__parameters"][
    "depth_global_stvl_layer"]
local_stvl = stvl["local_costmap"]["local_costmap"]["ros__parameters"][
    "depth_stvl_layer"]
for label, layer, expected_decay in (
        ("local recent", local_stvl, 4.0),
        ("global recent", global_stvl, 8.0)):
    if int(layer.get("decay_model", -1)) != 0:
        raise SystemExit(
            f"{label} RGB-D STVL must use bounded linear decay_model=0")
    if abs(float(layer.get("voxel_decay", -1.0)) - expected_decay) > 1e-6:
        raise SystemExit(f"{label} RGB-D memory duration changed unexpectedly")
    mark = layer.get("rgbd_mark", {})
    if mark.get("topic") != "/local_highres_cloud_v21/sensor":
        raise SystemExit(
            f"{label} RGB-D marking bypasses temporal/geometry confirmation")
    clear = layer.get("rgbd_clear", {})
    if clear.get("topic") != "/local_highres_cloud_v21/clear_sensor":
        raise SystemExit(
            f"{label} RGB-D clearing bypasses the valid-depth guard")
    if float(clear.get("min_z", 0.0)) < 0.19:
        raise SystemExit(
            f"{label} RGB-D clearing must not erase the camera near blind zone")
    if float(clear.get("max_z", 0.0)) < 3.4:
        raise SystemExit(
            f"{label} RGB-D clearing frustum does not cover the marking range")
if not local_stvl.get("publish_voxel_map"):
    raise SystemExit(
        "bounded local RGB-D voxel map must remain available for RViz audit")
if global_stvl.get("publish_voxel_map"):
    raise SystemExit(
        "global RGB-D layer must not compete for the RViz voxel topic")
wall_layer = local_costmap["visual_wall_stvl_layer"]
if float(wall_layer.get("voxel_decay", 0.0)) < 12.0:
    raise SystemExit(
        "local RTAB wall memory expires between full-cloud updates")

for label in ("local_costmap", "global_costmap"):
    params = rpp[label][label]["ros__parameters"]
    if abs(float(params.get("footprint_padding", -1.0)) - 0.01) > 1e-6:
        raise SystemExit(f"{label} measured-body safety padding changed")
    inflation = params.get("inflation_layer", {})
    if abs(float(inflation.get("inflation_radius", -1.0)) - 0.49) > 1e-6:
        raise SystemExit(f"{label} doorway-safe inflation radius changed")
    if abs(float(inflation.get("cost_scaling_factor", -1.0)) - 14.0) > 1e-6:
        raise SystemExit(f"{label} soft inflation decay changed")

follow_path = rpp["controller_server"]["ros__parameters"]["FollowPath"]
for key, expected in (
        ("lookahead_dist", 0.40),
        ("min_lookahead_dist", 0.30),
        ("max_lookahead_dist", 0.58),
        ("curvature_lookahead_dist", 0.35),
        ("regulated_linear_scaling_min_speed", 0.07),
        ("rotate_to_heading_min_angle", 1.05),
        ("inflation_cost_scaling_factor", 14.0)):
    if abs(float(follow_path.get(key, -1.0)) - expected) > 1e-6:
        raise SystemExit(f"RPP doorway contract changed: {key}")
if not follow_path.get("use_cost_regulated_linear_velocity_scaling"):
    raise SystemExit("RPP doorway cost regulation is disabled")

for name in ("navigate_to_pose_jazzy.xml", "navigate_through_poses_jazzy.xml"):
    tree = ET.parse(root / "behavior_trees" / name)
    if tree.getroot().attrib.get("BTCPP_format") != "4":
        raise SystemExit(f"{name} is not BehaviorTree.CPP v4")
    text = (root / "behavior_trees" / name).read_text(encoding="utf-8")
    if "<SmoothPath " in text:
        raise SystemExit(f"{name} still restarts FollowPath through SmoothPath")
    if '<RateController hz="1.0">' not in text:
        raise SystemExit(f"{name} must use the stable official 1 Hz pipeline")
    if '<PathExpiringTimer seconds="3.0" path="{path}"/>' not in text:
        raise SystemExit(f"{name} is missing bounded three-second path hysteresis")
    if '<IsPathValid path="{path}"/>' not in text:
        raise SystemExit(f"{name} is missing immediate obstacle invalidation")
    if "RetainValidPathAfterTransientPlannerFailure" not in text:
        raise SystemExit(
            f"{name} cannot retain a valid path after a transient planner error")
    if '<RecoveryNode number_of_retries="2" name="FollowPath">' not in text:
        raise SystemExit(
            f"{name} does not bound controller recovery to two attempts")
    if "<Wait " in text:
        raise SystemExit(
            f"{name} must not count planner idling as a navigation recovery")
    if 'name="NavigateRecovery"' in text:
        raise SystemExit(
            f"{name} contains the retired outer recovery loop")
    controller_gate = (
        '<WouldAControllerRecoveryHelp '
        'error_code="{follow_path_error_code}"/>')
    if text.count(controller_gate) != 1:
        raise SystemExit(
            f"{name} has an ambiguous controller recovery gate")
    first_backup = text.find("<BackUp ")
    if first_backup < 0 or text.find(controller_gate) > first_backup:
        raise SystemExit(
            f"{name} backup is not gated by controller failure")
    if text.count("<BackUp ") > 2:
        raise SystemExit(
            f"{name} contains excessive reverse recovery actions")
    if "<ClearEntireCostmap " in text:
        raise SystemExit(
            f"{name} must not erase persistent unseen obstacles during recovery")
    if text.find("<BackUp ") < 0 or text.find("<BackUp ") > text.find("<Spin "):
        raise SystemExit(f"{name} must attempt guarded reverse before spinning")
    if '<Sequence name="BackOutThenSmallTurn">' not in text:
        raise SystemExit(
            f"{name} can spin at a doorway without first backing out")
    if text.count("<Spin ") != 1:
        raise SystemExit(
            f"{name} contains an ambiguous or excessive spin recovery")

ekf_params = ekf["ekf_filter_node"]["ros__parameters"]
if ekf_params.get("world_frame") != "odom" or not ekf_params.get("publish_tf"):
    raise SystemExit("visual EKF must be the odom->base_link authority")
if ekf_params.get("odom0") != "/odom" or ekf_params.get("odom1") != "/visual_odom":
    raise SystemExit("visual EKF input topics changed unexpectedly")
if not ekf_params.get("predict_to_current_time"):
    raise SystemExit("visual EKF current-time prediction is disabled")
if not ekf_params.get("smooth_lagged_data"):
    raise SystemExit("visual EKF lagged RGB-D handling is disabled")
odom0_config = ekf_params.get("odom0_config")
odom1_config = ekf_params.get("odom1_config")
if not isinstance(odom0_config, list) or len(odom0_config) != 15:
    raise SystemExit("visual EKF odom0_config must contain 15 booleans")
if not isinstance(odom1_config, list) or len(odom1_config) != 15:
    raise SystemExit("visual EKF odom1_config must contain 15 booleans")
if odom0_config[5] is not True or odom0_config[6] is not True:
    raise SystemExit("STM32 absolute yaw and forward velocity must feed EKF")
if any(odom0_config[index] for index in (0, 1, 7, 11)):
    raise SystemExit("correlated STM32 pose/rate channels were re-enabled")
if odom1_config[6] is not True or odom1_config[7] is not True:
    raise SystemExit("RGB-D planar velocity must feed EKF")
if any(odom1_config[index] for index in (0, 1, 5, 11)):
    raise SystemExit("unvalidated visual pose/yaw channels were re-enabled")

dual_launch = (
    root / "launch" / "dual_resolution_3d_slam.launch.py"
).read_text(encoding="utf-8")
if '"publish_null_when_lost": False' not in dual_launch:
    raise SystemExit("visual tracking loss must not inject zero velocity")
if 'prefix="nice -n 8"' not in dual_launch:
    raise SystemExit("RTAB-Map must run below the collision-cloud priority")
for thread_limit in (
        '"OMP_NUM_THREADS": "2"',
        '"OPENBLAS_NUM_THREADS": "1"',
        '"MKL_NUM_THREADS": "1"'):
    if thread_limit not in dual_launch:
        raise SystemExit(
            f"RTAB-Map worker isolation missing: {thread_limit}")
if '"clear_sensor_output_topic": "/local_highres_cloud_v21/clear_sensor"' not in dual_launch:
    raise SystemExit("depth-valid STVL clearing topic is not launched")
for contract in (
    '"persistent_mark_confirmation_enabled": True',
    '"persistent_mark_confirmation_frames": 3',
    '"persistent_mark_max_gap_frames": 1',
    '"persistent_mark_neighbor_radius": 1',
    '"persistent_geometry_guard_enabled": True',
    '"recent_mark_ground_guard_height_m": 0.050',
    '"recent_mark_min_vertical_span_m": 0.030',
    '"persistent_mark_ground_guard_height_m": 0.080',
    '"persistent_mark_min_vertical_span_m": 0.060',
    '"mark_geometry_neighbor_radius": 1',
):
    if contract not in dual_launch:
        raise SystemExit(
            f"persistent obstacle temporal confirmation is not launched: {contract}")
depth_cloud_cpp = (
    repo_root / "lidar" / "chapt1_ws" / "src" / "local_depth_cloud_cpp" /
    "src" / "depth_image_to_local_cloud_v21_node.cpp"
).read_text(encoding="utf-8")
for contract in (
    "min_clear_valid_depth_ratio_",
    "build_persistent_mark_cloud(const GroundPlane & ground_plane)",
    "geometry_qualified_mark(",
    "sensor_cloud_pub_->publish(confirmed_sensor_cloud)",
    "persistent_sensor_cloud_pub_->publish(persistent_sensor_cloud)",
    "clear_sensor_cloud_pub_->publish(immediate_sensor_cloud)",
    "Suppressing STVL clearing",
):
    if contract not in depth_cloud_cpp:
        raise SystemExit(
            f"depth-valid persistent clearing guard missing: {contract}")
rviz_profile = (
    root / "rviz" / "dual_resolution_3d_slam.rviz"
).read_text(encoding="utf-8")
for contract in (
    "Class: nav2_rviz_plugins/Navigation 2",
    "Class: nav2_rviz_plugins/GoalTool",
    "Name: 3-Frame Recent Geometry Marks (4s Local)",
    "Name: Strict Geometry Candidates (Debug Only)",
    "Name: Nav2 Bounded Obstacle Memory (3 cm / 4s)",
    "Name: Nav2 Local Costmap (8x8 m near robot)",
    "Name: Nav2 Global Planner Costmap (Path Audit)",
):
    if contract not in rviz_profile:
        raise SystemExit(f"RViz navigation/audit control missing: {contract}")
if "Class: rviz_default_plugins/SetGoal" in rviz_profile:
    raise SystemExit("RViz still bypasses the Nav2 action-aware goal tool")
runner = (
    repo_root / "visual_laser_slam" / "run_dual_resolution_3d_slam.sh"
).read_text(encoding="utf-8")
for contract in (
    "CHASSIS_PUBLISH_TF=false",
    "CARTOGRAPHER_ODOM_TOPIC=/odometry/filtered",
    "RTABMAP_ODOM_TOPIC=/cartographer_pose_odom",
    "check_pkg nav2_rviz_plugins",
    'LOCAL_CLOUD_PIPELINE_VERSION="v6.34"',
    "reset_cached_package local_depth_cloud_cpp",
    "require_system_ready_for_motion:=$ENABLE_NAVIGATION",
    "navi_motion_watchdog_pose_enabled:",
    "nav_zero_command_cancel_sec:",
    '"nav_autostart:=$NAV_AUTOSTART"',
    "start_navigation_lifecycle",
    "nav2_msgs/srv/ManageLifecycleNodes",
    "verify_navigation_source_contract",
    "Navigation source contract: static walls=65, unknown space=blocked, global RTAB wall duplication=off, inflation=0.49m/14.0",
    "/local_costmap/local_costmap lethal_cost_threshold 65",
    "/global_costmap/global_costmap lethal_cost_threshold 65",
    "/local_costmap/local_costmap inflation_layer.inflation_radius 0.49",
    "/global_costmap/global_costmap inflation_layer.inflation_radius 0.49",
    "Auditing active costmap parameters (non-fatal CLI check)",
    "Runtime costmap query was incomplete; source contract passed",
    "release_motion_interlock",
    "ros2 service call /robot/set_system_ready std_srvs/srv/SetBool",
    'timeout --signal=TERM --kill-after=1s 3s',
    "Recent geometry-filtered navigation cloud did not start",
):
    if contract not in runner:
        raise SystemExit(f"visual fusion TF/odometry contract missing: {contract}")
nav_start = runner.find("  start_navigation_lifecycle ||")
motion_release = runner.find("  release_motion_interlock ||")
costmap_diagnostic = runner.find("  wait_topic /local_costmap/costmap 15")
source_contract = runner.find("  verify_navigation_source_contract ||")
runtime_contract = runner.find(
    "    /local_costmap/local_costmap lethal_cost_threshold 65 4 WARNING",
    nav_start)
if min(
        source_contract, nav_start, motion_release, runtime_contract,
        costmap_diagnostic) < 0 or not (
        source_contract < nav_start < motion_release
        < runtime_contract < costmap_diagnostic):
    raise SystemExit(
        "Source costmap validation must precede Nav2 activation; motion release "
        "must happen before non-fatal runtime parameter and topic diagnostics")
if 'kill -USR1 "$RVIZ_PID"' in runner or "refresh_requested" in runner:
    raise SystemExit(
        "RViz must not be intentionally killed/restarted after Nav2 activation")
dual_launch_text = (
    root / "launch" / "dual_resolution_3d_slam.launch.py"
).read_text(encoding="utf-8")
for contract in (
    'DeclareLaunchArgument("nav_autostart", default_value="true")',
    '"nav_autostart": LaunchConfiguration("nav_autostart")',
    '"navi_motion_watchdog_pose_enabled", default_value="false"',
    '"navi_motion_watchdog_pose_enabled": LaunchConfiguration(',
    '"nav_zero_command_cancel_sec", default_value="25.0"',
    '"scan_self_filter_half_length": 0.33',
    '"scan_self_filter_half_width": 0.33',
):
    if contract not in dual_launch_text:
        raise SystemExit(f"staged Nav2 launch contract missing: {contract}")
chassis_source = (
    root / "lidar_py" / "chassis_node.py"
).read_text(encoding="utf-8")
for contract in (
    "'/robot/set_system_ready'",
    "release_ps2_on_shutdown",
    "SYSTEM_READY motion interlock released by",
    "self.motion_serial_enabled = not self.auto_nav_ps2_handoff",
    "STARTUP_PS2_RELEASE",
    "NAV_STALL_CANCEL",
    "nav_zero_command_cancel_sec",
    "nav_sensor_health_timeout_sec",
    "nav_sensor_fault_cancel_sec",
    "required 2D/3D navigation sensor input has been stale",
    "navi_motion_watchdog_pose_enabled",
    "Shutdown: zero speed sent",
):
    if contract not in chassis_source:
        raise SystemExit(f"chassis readiness/release contract missing: {contract}")
if "SYSTEM_STARTUP_NOT_READY_ZERO_HOLD" in chassis_source:
    raise SystemExit(
        "startup Twist gate must not steal idle control back from PS2")
safety_source = (
    root / "lidar_py" / "safety_fusion_node.py"
).read_text(encoding="utf-8")
for contract in (
    '"/robot/navigation_sensor_healthy"',
    "self.collision_self_filtered_point_count",
    "self._navigation_sensor_healthy()",
):
    if contract not in safety_source:
        raise SystemExit(f"navigation sensor health contract missing: {contract}")
collision_gate_source = (
    repo_root / "lidar" / "chapt1_ws" / "src" /
    "local_depth_cloud_cpp" / "src" /
    "local_cloud_collision_gate_node.cpp"
).read_text(encoding="utf-8")
for contract in (
    '"scan_self_filter_half_length"',
    '"scan_self_filter_half_width"',
    "scan_self_filtered_count_",
    "A real obstacle cannot occupy the chassis' physical footprint",
):
    if contract not in collision_gate_source:
        raise SystemExit(f"2D scan self-filter contract missing: {contract}")
fusion_launcher = (
    repo_root / "START_DUAL_2D_3D_NAVIGATION_VISUAL_FUSION.sh"
).read_text(encoding="utf-8")
for contract in (
    "DUAL_3D_ENABLE_VISUAL_FUSION=true",
    "DUAL_3D_CARTOGRAPHER_CONFIG=cartographer_2d_v9_nav_guarded.lua",
):
    if contract not in fusion_launcher:
        raise SystemExit(f"visual fusion launcher contract missing: {contract}")

for path in (
    root / "launch" / "cartographer_auto_mapping_jazzy_launch.py",
    root / "launch" / "dual_resolution_3d_slam.launch.py",
    root / "lidar_py" / "chassis_node.py",
    root / "lidar_py" / "safety_fusion_node.py",
    root / "lidar_py" / "frontier_web_bridge.py",
    root / "lidar_py" / "web_goal_nav_node.py",
    root / "lidar_py" / "web_path_preview_node.py",
):
    py_compile.compile(str(path), doraise=True)
PY
pass "RPP/Smac/STVL YAML, BT.CPP v4, staged startup and Python syntax are valid"

declare -A EXPECTED_HASHES=(
    ["$LIDAR_SRC/config/cartographer_2d_v9_tightened.lua"]="00dfd1c721f0fe8c61ac6f2b417001920694e4fc77e895fb4a1f194330c910d9"
    ["$LIDAR_SRC/launch/cartographer_scan_v2_launch.py"]="20506a9609532576244d2fe7a8e0be9bd5a66396dad4f1605348667949ba6f77"
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
    local runtime_rpp_params="$2"
    local runtime_stvl_params="$3"
    local runtime_bt="$4"
    local runtime_through_bt="$5"

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
        --params-file "$runtime_nav_params" \
        --params-file "$runtime_rpp_params" \
        --params-file "$runtime_stvl_params"
    start_smoke_node velocity_smoother \
        ros2 run nav2_velocity_smoother velocity_smoother --ros-args \
        --params-file "$runtime_nav_params" \
        --params-file "$runtime_rpp_params"
    start_smoke_node planner_server \
        ros2 run nav2_planner planner_server --ros-args \
        --params-file "$runtime_nav_params" \
        --params-file "$runtime_rpp_params" \
        --params-file "$runtime_stvl_params"
    start_smoke_node behavior_server \
        ros2 run nav2_behaviors behavior_server --ros-args \
        --params-file "$runtime_nav_params" \
        --params-file "$runtime_rpp_params"
    start_smoke_node bt_navigator \
        ros2 run nav2_bt_navigator bt_navigator --ros-args \
        --params-file "$runtime_nav_params" \
        -p "default_nav_to_pose_bt_xml:=$runtime_bt" \
        -p "default_nav_through_poses_bt_xml:=$runtime_through_bt"
    start_smoke_node waypoint_follower \
        ros2 run nav2_waypoint_follower waypoint_follower --ros-args \
        --params-file "$runtime_nav_params"

    for node in controller_server velocity_smoother planner_server \
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
    for package in frontier_exploration_ros2 local_depth_cloud_cpp lidar_py; do
        ros2 pkg prefix "$package" >/dev/null 2>&1 || fail "Build cannot find $package"
    done
    pass "Jazzy workspace build completed"

    RUNTIME_SHARE="$ASCII_WS_BASE/install/lidar_py/share/lidar_py"
    run_nav2_lifecycle_smoke \
        "$RUNTIME_SHARE/config/nav2_auto_mapping_jazzy.yaml" \
        "$RUNTIME_SHARE/config/nav2_dual_3d_rpp_override.yaml" \
        "$RUNTIME_SHARE/config/nav2_dual_3d_stvl_override.yaml" \
        "$RUNTIME_SHARE/behavior_trees/navigate_to_pose_jazzy.xml" \
        "$RUNTIME_SHARE/behavior_trees/navigate_through_poses_jazzy.xml"
fi

echo "Jazzy migration preflight passed."
