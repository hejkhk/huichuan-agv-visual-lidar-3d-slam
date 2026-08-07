#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_SRC="$ROOT_DIR/lidar/chapt1_ws/src/lidar_py"
LOCAL_DEPTH_SRC="$ROOT_DIR/lidar/chapt1_ws/src/local_depth_cloud_cpp"
ORBBEC_SRC="$ROOT_DIR/lidar/chapt1_ws/src/OrbbecSDK_ROS2"
SYSTEM_PYTHON="${CAR_SYSTEM_PYTHON:-/usr/bin/python3}"
BUILD_ROOT="${CAR_HUMBLE_BUILD_ROOT:-$HOME/.cache/huichuan_agv_humble_ws}"

fail() { echo "[FAIL] $*" >&2; exit 1; }
pass() { echo "[PASS] $*"; }

if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  [ "${ID:-}" = ubuntu ] || fail "Expected Ubuntu, got ${ID:-unknown}"
  [ "${VERSION_ID:-}" = "22.04" ] ||
    fail "Expected Ubuntu 22.04, got ${VERSION_ID:-unknown}"
  pass "Ubuntu 22.04"
fi

[ -f /opt/ros/humble/setup.bash ] || fail "Missing /opt/ros/humble/setup.bash"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
[ "${ROS_DISTRO:-}" = humble ] || fail "ROS 2 Humble is not active"
pass "ROS_DISTRO=humble"

for command in ros2 colcon "$SYSTEM_PYTHON"; do
  command -v "$command" >/dev/null 2>&1 || fail "Missing command: $command"
done
"$SYSTEM_PYTHON" -I -c 'import em, lark, yaml' >/dev/null 2>&1 ||
  fail "Install python3-empy python3-lark python3-yaml"

required_packages=(
  laser_filters rtabmap_slam rtabmap_odom octomap_server \
  robot_localization nav2_controller nav2_planner nav2_bt_navigator \
  nav2_behaviors nav2_velocity_smoother nav2_smac_planner \
  nav2_rotation_shim_controller nav2_regulated_pure_pursuit_controller \
  dwb_core dwb_plugins dwb_critics behaviortree_cpp_v3 \
  spatio_temporal_voxel_layer map_msgs \
  rmw_cyclonedds_cpp depth_image_proc robot_state_publisher xacro
)
if [ "${CAR_VALIDATION_SKIP_EXTERNAL:-0}" != "1" ]; then
  required_packages+=(cartographer_ros)
else
  echo "[WARN] CI-only mode: external Cartographer/Orbbec runtime checks skipped."
fi
for package in "${required_packages[@]}"; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || fail "Missing ROS package: $package"
done
pass "Humble Cartographer/Nav2/RTAB-Map/STVL dependencies"

# Hosted CI cannot link or exercise Jetson camera binaries, but it still
# verifies that the complete ARM64 runtime payload is present in the branch.
for file in \
  "$ORBBEC_SRC/orbbec_camera/package.xml" \
  "$ORBBEC_SRC/orbbec_camera_msgs/package.xml" \
  "$ORBBEC_SRC/orbbec_description/package.xml" \
  "$ORBBEC_SRC/orbbec_camera/SDK/lib/arm64/libOrbbecSDK.so.2.9.3" \
  "$ORBBEC_SRC/orbbec_camera/SDK/lib/arm64/extensions/depthengine/libdepthengine.so.2.0" \
  "$ORBBEC_SRC/orbbec_camera/SDK/lib/arm64/extensions/frameprocessor/libob_frame_processor.so" \
  "$ORBBEC_SRC/orbbec_camera/SDK/lib/arm64/extensions/filters/libFilterProcessor.so" \
  "$ORBBEC_SRC/orbbec_camera/SDK/lib/arm64/extensions/filters/libob_priv_filter.so"; do
  [ -r "$file" ] || fail "Missing Jetson Orbbec runtime file: $file"
done
pass "Jetson Orbbec wrapper and ARM64 SDK payload"

[ ! -f "$ROOT_DIR/lidar/chapt1_ws/src/short_goal_bt/COLCON_IGNORE" ] ||
  fail "Humble short_goal_bt is unexpectedly excluded from workspace builds"
if colcon list --base-paths "$ROOT_DIR/lidar/chapt1_ws/src" 2>/dev/null | \
    awk '{print $1}' | grep -Fxq short_goal_bt; then
  pass "Humble BT.CPP v3 short_goal_bt is discoverable"
else
  fail "Humble short_goal_bt is not discoverable by colcon"
fi

declare -a BASELINE_HASHES=(
  "00dfd1c721f0fe8c61ac6f2b417001920694e4fc77e895fb4a1f194330c910d9:$LIDAR_SRC/config/cartographer_2d_v9_tightened.lua"
  "20506a9609532576244d2fe7a8e0be9bd5a66396dad4f1605348667949ba6f77:$LIDAR_SRC/launch/cartographer_scan_v2_launch.py"
  "8583a2ca7e99a29b13f2fc339df468e621562d61f0adfa1e7e1828254705b306:$LIDAR_SRC/config/laser_filter.yaml"
)
for baseline in "${BASELINE_HASHES[@]}"; do
  expected="${baseline%%:*}"
  file="${baseline#*:}"
  actual="$(sha256sum "$file" | awk '{print tolower($1)}')"
  [ "$actual" = "$expected" ] ||
    fail "Stable mapping baseline changed: $file (expected $expected, got $actual)"
done
pass "Cartographer V13 mapping baseline hashes"

if ! ros2 pkg prefix orbbec_camera >/dev/null 2>&1; then
  echo "[WARN] orbbec_camera is not visible; source its Humble workspace before starting."
fi

for file in \
  "$LIDAR_SRC/launch/cartographer_auto_mapping_humble_launch.py" \
  "$LIDAR_SRC/launch/dual_resolution_3d_slam.launch.py" \
  "$LIDAR_SRC/launch/cartographer_scan_v2_localization_launch.py" \
  "$LIDAR_SRC/config/cartographer_2d_localization.lua" \
  "$LIDAR_SRC/lidar_py/cartographer_reloc.py" \
  "$LIDAR_SRC/lidar_py/localization_bringup.py" \
  "$LIDAR_SRC/lidar_py/system_resource_monitor.py" \
  "$LIDAR_SRC/lidar_py/slam_correction_logic.py" \
  "$LIDAR_SRC/lidar_py/slam_correction_guard.py" \
  "$LOCAL_DEPTH_SRC/src/mutable_navigation_map_node.cpp" \
  "$LIDAR_SRC/rviz/dual_resolution_3d_localization.rviz" \
  "$ROOT_DIR/lidar/chapt1_ws/src/reloc_rviz_panel/reloc_panel_plugin.xml" \
  "$LIDAR_SRC/config/nav2_auto_mapping_humble.yaml" \
  "$LIDAR_SRC/config/nav2_dual_3d_dwb_humble_override.yaml" \
  "$LIDAR_SRC/config/nav2_dual_3d_stvl_override.yaml" \
  "$LIDAR_SRC/config/frontier_auto_mapping_humble.yaml" \
  "$LIDAR_SRC/behavior_trees/navigate_to_pose_humble.xml" \
  "$LIDAR_SRC/behavior_trees/navigate_through_poses_humble.xml"; do
  [ -r "$file" ] || fail "Missing Humble project file: $file"
done

"$SYSTEM_PYTHON" -I - "$LIDAR_SRC" <<'PY'
import pathlib
import sys
import xml.etree.ElementTree as ET
import yaml

root = pathlib.Path(sys.argv[1])

def load_yaml(name):
    with (root / "config" / name).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise SystemExit(f"empty YAML: {name}")
    return data

nav = load_yaml("nav2_auto_mapping_humble.yaml")
dwb = load_yaml("nav2_dual_3d_dwb_humble_override.yaml")
stvl = load_yaml("nav2_dual_3d_stvl_override.yaml")
load_yaml("frontier_auto_mapping_humble.yaml")

bt = nav["bt_navigator"]["ros__parameters"]
if not bt.get("plugin_lib_names"):
    raise SystemExit("Humble bt_navigator.plugin_lib_names is required")
if "short_goal_behind_bt_node" not in bt["plugin_lib_names"]:
    raise SystemExit("Humble custom short-goal BT plugin is not loaded")
if bt.get("bt_loop_duration") != 50:
    raise SystemExit("Jetson Humble BT loop must remain bounded at 20 Hz")
if bt.get("default_server_timeout") != 500:
    raise SystemExit("Jetson Humble BT service timeout must tolerate CPU load")

expected_costmap_plugins = {
    "obstacle_layer": "nav2_costmap_2d::ObstacleLayer",
    "inflation_layer": "nav2_costmap_2d::InflationLayer",
}
for costmap_name in ("local_costmap", "global_costmap"):
    params = nav[costmap_name][costmap_name]["ros__parameters"]
    for layer, plugin in expected_costmap_plugins.items():
        if layer in params and params[layer].get("plugin") != plugin:
            raise SystemExit(f"invalid Humble costmap plugin: {costmap_name}.{layer}")
if nav["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"].get(
        "plugin") != "nav2_costmap_2d::StaticLayer":
    raise SystemExit("invalid Humble StaticLayer plugin name")

behaviors = nav["behavior_server"]["ros__parameters"]
expected_behaviors = {
    "spin": "nav2_behaviors/Spin",
    "backup": "nav2_behaviors/BackUp",
    "drive_on_heading": "nav2_behaviors/DriveOnHeading",
    "wait": "nav2_behaviors/Wait",
}
for behavior, plugin in expected_behaviors.items():
    if behaviors[behavior].get("plugin") != plugin:
        raise SystemExit(f"invalid Humble behavior plugin: {behavior}")
if behaviors.get("costmap_topic") != "local_costmap/costmap_raw":
    raise SystemExit("Humble behavior_server costmap_topic is missing")
if behaviors.get("footprint_topic") != "local_costmap/published_footprint":
    raise SystemExit("Humble behavior_server footprint_topic is missing")
for leaked_name in (
        "local_costmap_topic", "global_costmap_topic",
        "local_footprint_topic", "global_footprint_topic"):
    if leaked_name in behaviors:
        raise SystemExit(f"post-Humble behavior_server parameter present: {leaked_name}")

waypoint = nav["waypoint_follower"]["ros__parameters"]["wait_at_waypoint"]
if waypoint.get("plugin") != "nav2_waypoint_follower::WaitAtWaypoint":
    raise SystemExit("invalid Humble waypoint executor plugin name")

controller = dwb["controller_server"]["ros__parameters"]
if controller.get("controller_frequency") != 10.0:
    raise SystemExit("Jetson Humble hybrid controller loop must remain at 10 Hz")
if controller.get("progress_checker_plugin") != "progress_checker":
    raise SystemExit("Humble requires singular progress_checker_plugin")
if "progress_checker_plugins" in controller:
    raise SystemExit("Jazzy progress_checker_plugins leaked into Humble")
if controller["progress_checker"].get("plugin") != "nav2_controller::SimpleProgressChecker":
    raise SystemExit("Humble SimpleProgressChecker is not selected")
if controller["progress_checker"].get("movement_time_allowance") != 15.0:
    raise SystemExit("hybrid fallback progress allowance changed")
follow = controller["FollowPath"]
if follow.get("plugin") != "nav2_rotation_shim_controller::RotationShimController":
    raise SystemExit("invalid Humble RotationShim plugin name")
if follow.get("primary_controller") != (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"):
    raise SystemExit("FollowPath does not wrap RPP")
no_shim = controller.get("FollowPathNoShim", {})
if no_shim.get("plugin") != "dwb_core::DWBLocalPlanner":
    raise SystemExit("NoShim DWB controller is missing")
if follow.get("desired_linear_vel") != 0.20:
    raise SystemExit("navigation gear-2 velocity changed")
if follow.get("lookahead_dist") != 0.40:
    raise SystemExit("hybrid RPP lookahead changed")
if follow.get("use_collision_detection") is not True:
    raise SystemExit("RPP footprint collision detection is disabled")
if follow.get("allow_reversing") is not False:
    raise SystemExit("RPP must delegate reverse maneuvers to DWB")
if no_shim.get("min_vel_x", 0.0) >= 0.0:
    raise SystemExit("NoShim controller lost bounded reverse sampling")
planner = dwb["planner_server"]["ros__parameters"]["GridBased"]
if planner.get("plugin") != "nav2_smac_planner/SmacPlanner2D":
    raise SystemExit("Humble SmacPlanner2D plugin name is invalid")
if planner.get("allow_unknown") is not False:
    raise SystemExit("global planner must reject unknown map space")

for costmap_name in ("local_costmap", "global_costmap"):
    params = stvl[costmap_name][costmap_name]["ros__parameters"]
    if "static_layer" not in params["plugins"]:
        raise SystemExit(f"{costmap_name} lost Cartographer static walls")
    if params["static_layer"].get("plugin") != "nav2_costmap_2d::StaticLayer":
        raise SystemExit(f"invalid Humble StaticLayer override: {costmap_name}")
    if params["static_layer"].get("footprint_clearing_enabled") is not True:
        raise SystemExit(
            f"{costmap_name} must clear stale static cells below the robot footprint")
    if params["static_layer"].get("map_topic") != "/map":
        raise SystemExit(f"{costmap_name} StaticLayer must consume mutable /map")
    if params["static_layer"].get("subscribe_to_updates") is not True:
        raise SystemExit(f"{costmap_name} StaticLayer must consume /map_updates")
    if params.get("use_maximum") is not True:
        raise SystemExit(f"{costmap_name} lost maximum safety combination")
    inflation = dwb[costmap_name][costmap_name]["ros__parameters"]["inflation_layer"]
    if inflation.get("inflation_radius") != 0.10:
        raise SystemExit(f"{costmap_name} inflation radius changed")
    if inflation.get("cost_scaling_factor") != 8.0:
        raise SystemExit(f"{costmap_name} inflation scaling changed")

for name in ("navigate_to_pose_humble.xml", "navigate_through_poses_humble.xml"):
    xml_root = ET.parse(root / "behavior_trees" / name).getroot()
    if "BTCPP_format" in xml_root.attrib:
        raise SystemExit(f"BT.CPP 4 marker is invalid on Humble: {name}")
    text = ET.tostring(xml_root, encoding="unicode")
    for forbidden in ("error_code_id=", "WouldAPlannerRecoveryHelp", "WouldAControllerRecoveryHelp"):
        if forbidden in text:
            raise SystemExit(f"Jazzy BT port/node leaked into {name}: {forbidden}")
    if 'PathExpiringTimer seconds="1.0"' not in text:
        raise SystemExit(
            f"{name} can retain a pre-loop-correction path for too long")
    if name == "navigate_to_pose_humble.xml":
        for required in (
                "InitialPathPreRotate", "SpinSafetyCheck", "SelectController",
                "ControllerSelected", "ReverseEscapeMonitor",
                "SmoothRPPThenManeuverableDWB", "FollowPathNoShim"):
            if required not in text:
                raise SystemExit(f"custom Humble navigation behavior missing: {required}")

dual = (root / "launch" / "dual_resolution_3d_slam.launch.py").read_text(encoding="utf-8")
for required in (
    "cartographer_auto_mapping_humble_launch.py",
    "navigate_to_pose_humble.xml",
    "navigate_through_poses_humble.xml",
    "nav2_dual_3d_dwb_humble_override.yaml",
    '"camera_time_domain", default_value="global"',
    '"rgbd_sync_max_interval", default_value="0.045"',
    '"rgbd_sync_warn_p95_ms", default_value="45.0"',
    '"max_input_age_ms": 250.0',
    '"age_warn_ms": 220.0',
    '"stall_warn_gap_ms": 150.0',
    '"fixed_scan_min_raw_points", default_value="180"',
    "cartographer_scan_v2_localization_launch.py",
    "localization_map_server",
    "mutable_navigation_map_node",
    '"topic_name": "/localization_reference_map"',
    '"reference_map_topic": "/localization_reference_map"',
    '"output_map_topic": "/map"',
    '"update_topic": "/map_updates"',
    "cartographer_reloc",
    "localization_bringup",
    '"strong_match_score": 0.75',
    '"strong_match_min_margin": 0.012',
    '"depth_qos": "DEFAULT"',
    '"depth_camera_info_qos": "DEFAULT"',
    "system_resource_monitor",
    "resource_usage_csv_file",
    "slam_correction_guard",
    '"hold_topic": "/slam_correction_hold"',
):
    if required not in dual:
        raise SystemExit(f"dual-resolution launch does not select {required}")
if dual.count('"output_map_topic": "/map"') != 1:
    raise SystemExit("localization mode must have exactly one explicit /map owner")
if '"topic_name": "/map"' in dual:
    raise SystemExit("reference map_server must never publish the mutable /map")

mutable_map = (
    root.parent / "local_depth_cloud_cpp" / "src" /
    "mutable_navigation_map_node.cpp"
).read_text(encoding="utf-8")
for required in (
        "mark_confirmations", "clear_confirmations", "collect_ray_cells",
        "NAV_MAP_POSE_JUMP", "restore_reference_on_pose_jump",
        "NAV_MAP_LOOP_CORRECTION", "slam_correction_hold_topic",
        "OccupancyGridUpdate", "localization_ready"):
    if required not in mutable_map:
        raise SystemExit(f"mutable navigation map safety contract missing: {required}")
resource_monitor = (
    root / "lidar_py" / "system_resource_monitor.py"
).read_text(encoding="utf-8")
for required in (
        "RESOURCE_SYSTEM", "RESOURCE_GROUP", "RESOURCE_PRESSURE",
        "CSV_FIELDS", '"camera"', '"cartographer"', '"nav2"'):
    if required not in resource_monitor:
        raise SystemExit(f"resource monitor contract missing: {required}")
setup_text = (root / "setup.py").read_text(encoding="utf-8")
if "system_resource_monitor = lidar_py.system_resource_monitor:main" not in setup_text:
    raise SystemExit("resource monitor console entry point is missing")
if "slam_correction_guard = lidar_py.slam_correction_guard:main" not in setup_text:
    raise SystemExit("SLAM correction guard console entry point is missing")

guard = (root / "lidar_py" / "slam_correction_guard.py").read_text(
    encoding="utf-8")
for required in (
        'lookup_transform(', 'self.map_frame,', 'self.odom_frame,',
        '"/slam_correction_hold"', "SLAM_CORRECTION_HOLD",
        "SlamCorrectionDetector", "except Exception:", "if rclpy.ok():"):
    if required not in guard:
        raise SystemExit(f"SLAM correction guard contract missing: {required}")

safety = (root / "lidar_py" / "safety_fusion_node.py").read_text(
    encoding="utf-8")
for required in (
        "slam_correction_hold_topic", "self.slam_correction_hold",
        "DurabilityPolicy.TRANSIENT_LOCAL",
        'self.active_command_source = "slam_correction_hold"'):
    if required not in safety:
        raise SystemExit(f"velocity hold contract missing: {required}")

project_root = root.parents[3]
online_loop_entry_points = (
    project_root / "START_DUAL_2D_3D_NAVIGATION.sh",
    project_root / "START_DUAL_2D_3D_NAVIGATION_VISUAL_FUSION.sh",
    project_root / "visual_laser_slam" / "run_dual_resolution_3d_slam.sh",
)
for entry_point in online_loop_entry_points:
    entry_text = entry_point.read_text(encoding="utf-8")
    if "cartographer_2d_v9_mapping_balanced.lua" not in entry_text:
        raise SystemExit(
            f"{entry_point.name} must use the proven online loop profile")
    if "cartographer_2d_v9_nav_guarded.lua" in entry_text:
        raise SystemExit(
            f"{entry_point.name} still disables online loop optimization")

localization_config = (
    root / "config" / "cartographer_2d_localization.lua"
).read_text(encoding="utf-8")
for required in (
        "POSE_GRAPH.optimize_every_n_nodes = 40",
        "POSE_GRAPH.constraint_builder.sampling_ratio = 0.8",
        "fast_correlative_scan_matcher.linear_search_window = 1.5",
        "fast_correlative_scan_matcher.angular_search_window = math.rad(3.0)",
        "pure_localization_trimmer"):
    if required not in localization_config:
        raise SystemExit(
            f"localization loop-closure contract missing: {required}")

localization_launch = (
    root / "launch" / "cartographer_scan_v2_localization_launch.py"
).read_text(encoding="utf-8")
if "-start_trajectory_with_default_topics=false" not in localization_launch:
    raise SystemExit("localization must defer the active trajectory until matching")
reloc = (root / "lidar_py" / "cartographer_reloc.py").read_text(encoding="utf-8")
if "DeleteTrajectory" in reloc or "/delete_trajectory" in reloc:
    raise SystemExit("Humble relocalizer still references unavailable DeleteTrajectory")
if "qos_profile_sensor_data" not in reloc:
    raise SystemExit("Humble relocalizer must subscribe to LiDAR with sensor-data QoS")
for service in ('"/get_trajectory_states"', '"/finish_trajectory"', '"/start_trajectory"'):
    if service not in reloc:
        raise SystemExit(f"Humble relocalizer uses the wrong Cartographer service path: {service}")
bringup = (root / "lidar_py" / "localization_bringup.py").read_text(encoding="utf-8")
for required in (
        '"/robot/navigation_sensor_healthy"',
        '"/robot/system_ready"',
        "self.ready and self.sensor_healthy",
        "self.started and not self.paused"):
    if required not in bringup:
        raise SystemExit(
            f"localization bringup lost the in-process motion gate: {required}")
runner = (root.parents[3] / "visual_laser_slam" / "run_dual_resolution_3d_slam.sh").read_text(encoding="utf-8")
for required in (
        'fixed_scan_min_raw_points:=${FIXED_SCAN_MIN_RAW_POINTS:-180}',
        'fixed_scan_min_valid_points:=${FIXED_SCAN_MIN_VALID_POINTS:-0}',
        'RGBD_SYNC_WARN_P95_MS:-45.0'):
    if required not in runner:
        raise SystemExit(
            f"Jetson launcher lost its measured LiDAR/RGB-D threshold: {required}")
if "while ! wait_boolean_true /localization_ready 150" not in runner:
    raise SystemExit("localization launcher must wait for the verified localization gate")
if "Automatic match is still unverified" not in runner:
    raise SystemExit("localization failure must preserve RViz manual recovery")
if 'while [ "$SECONDS" -lt "$deadline" ]' not in runner:
    raise SystemExit("boolean readiness gate must keep sampling after its initial false value")
for evidence in (
        "confirmed by startup relocalizer",
        "Gemini2 RGB and depth streams",
        "STEP10V2.1 local cloud data",
        "C++ collision gate with fresh cloud and scan"):
    if evidence not in runner:
        raise SystemExit(
            f"Jetson startup is missing fast in-process readiness evidence: {evidence}")
if "wait_topic /camera/color/image_raw 60" in runner:
    raise SystemExit(
        "launcher must not deserialize full RGB frames with ros2cli before releasing motion")
if "wait_topic /camera/depth/image_raw 30" in runner:
    raise SystemExit(
        "launcher must not deserialize full depth frames with ros2cli before releasing motion")
if "strong_match_score" not in reloc or "strong_match_min_margin" not in reloc:
    raise SystemExit("startup relocalizer is missing the guarded strong-match gate")
if "retry_wait" not in reloc or "max_auto_attempts" not in reloc:
    raise SystemExit("startup relocalizer is missing bounded automatic retries")
if "wait_topic /cartographer_pose_odom 30" not in runner:
    raise SystemExit("mapping launcher must still verify Cartographer corrected pose")
for required in (
        "queue_incremental_package", ".lidar-py-source.sha256",
        ".short-goal-bt-source.sha256", "skipping colcon",
        "wait_transient_topic /localization_reference_map 30",
        "wait_topic_publisher /map_updates 15",
        "wait_topic /slam_correction_hold 10",
        "resource_usage.csv"):
    if required not in runner:
        raise SystemExit(
            f"per-package incremental build contract is missing: {required}")
PY
pass "Humble YAML, plugin and BT.CPP 3 contracts"

"$SYSTEM_PYTHON" -m compileall -q \
  "$LIDAR_SRC/launch" "$LIDAR_SRC/lidar_py" || fail "Python syntax check failed"
pass "Python launch/node syntax"

for script in \
  "$ROOT_DIR/open_all.sh" "$ROOT_DIR/open_all_log.sh" \
  "$ROOT_DIR/START_DUAL_2D_3D_MAPPING.sh" \
  "$ROOT_DIR/START_DUAL_2D_3D_NAVIGATION.sh" \
  "$ROOT_DIR/START_DUAL_2D_3D_LOCALIZATION.sh" \
  "$ROOT_DIR/visual_laser_slam/run_dual_resolution_3d_slam.sh" \
  "$ROOT_DIR/CALIBRATE_CAMERA_EXTRINSIC.sh" \
  "$ROOT_DIR/CALIBRATE_CAMERA_YAW.sh"; do
  bash -n "$script" || fail "Bash syntax check failed: $script"
done
pass "One-click and calibration script syntax"

if [ "${1:-}" = "--build" ]; then
  # reloc_rviz_panel is a UI plugin rather than a dependency of lidar_py, so
  # name it explicitly or hosted CI could pass without ever compiling it.
  declare -a BUILD_SELECTION=(--packages-up-to lidar_py reloc_rviz_panel)
  if [ "${CAR_VALIDATION_SKIP_EXTERNAL:-0}" = "1" ]; then
    # orbbec_camera is an exec dependency of lidar_py. --packages-skip would
    # leave it in colcon's dependency graph and make ament_python wait for its
    # environment hooks, so CI must ignore these hardware packages entirely.
    # The real driver is built and exercised on Jetson.
    BUILD_SELECTION+=(
      --packages-ignore orbbec_camera orbbec_camera_msgs orbbec_description
    )
  elif [ "${CAR_VALIDATION_BUILD_VENDOR:-0}" = "1" ]; then
    # Keep the camera wrapper explicit. This prevents a package.xml change
    # from silently turning the full ARM64 build into a project-only build.
    BUILD_SELECTION=(
      --packages-up-to lidar_py reloc_rviz_panel orbbec_camera
    )
  fi
  mkdir -p "$BUILD_ROOT"
  if [ -L "$BUILD_ROOT/src" ]; then
    rm -f "$BUILD_ROOT/src"
  elif [ -e "$BUILD_ROOT/src" ]; then
    fail "Build source path exists and is not a symlink: $BUILD_ROOT/src"
  fi
  ln -s "$ROOT_DIR/lidar/chapt1_ws/src" "$BUILD_ROOT/src"
  PYTHONNOUSERSITE=1 colcon --log-base "$BUILD_ROOT/log" build \
    --base-paths "$BUILD_ROOT/src" \
    --build-base "$BUILD_ROOT/build" \
    --install-base "$BUILD_ROOT/install" \
    --symlink-install "${BUILD_SELECTION[@]}" \
    --cmake-args "-DPython3_EXECUTABLE=$SYSTEM_PYTHON" || fail "Humble build failed"
  pass "Humble workspace build"

  # Load exactly the overlay built above, then configure the real Humble
  # pluginlib consumers. YAML parsing alone cannot detect an invalid class ID.
  # shellcheck disable=SC1091
  source "$BUILD_ROOT/install/setup.bash"
  if [ "${CAR_VALIDATION_BUILD_VENDOR:-0}" = "1" ]; then
    ros2 pkg prefix orbbec_camera >/dev/null 2>&1 ||
      fail "Full ARM64 build did not install orbbec_camera"
    pass "Bundled Orbbec ARM64 wrapper build"
  fi
  MERGED_PARAMS="$BUILD_ROOT/humble_nav2_smoke.yaml"
  "$SYSTEM_PYTHON" -I - \
    "$LIDAR_SRC/config/nav2_auto_mapping_humble.yaml" \
    "$LIDAR_SRC/config/nav2_dual_3d_dwb_humble_override.yaml" \
    "$LIDAR_SRC/config/nav2_dual_3d_stvl_override.yaml" \
    "$MERGED_PARAMS" <<'PY'
import copy
import pathlib
import sys
import yaml


def merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


merged = {}
for source in sys.argv[1:-1]:
    with pathlib.Path(source).open(encoding="utf-8") as stream:
        merged = merge(merged, yaml.safe_load(stream) or {})
with pathlib.Path(sys.argv[-1]).open("w", encoding="utf-8") as stream:
    bt_dir = pathlib.Path(sys.argv[1]).parent.parent / "behavior_trees"
    merged.setdefault("bt_navigator", {}).setdefault("ros__parameters", {}).update({
        "default_nav_to_pose_bt_xml": str(bt_dir / "navigate_to_pose_humble.xml"),
        "default_nav_through_poses_bt_xml": str(
            bt_dir / "navigate_through_poses_humble.xml"),
    })
    yaml.safe_dump(merged, stream, sort_keys=False)
PY

  smoke_lifecycle_configure() {
    local package="$1"
    local executable="$2"
    local node="$3"
    local log_file="$BUILD_ROOT/${node}_configure.log"
    local pid=""
    local visible=0
    local transition=""

    ros2 run "$package" "$executable" --ros-args \
      --params-file "$MERGED_PARAMS" \
      >"$log_file" 2>&1 &
    pid=$!
    for _ in $(seq 1 60); do
      if ros2 node list 2>/dev/null | grep -Fxq "/$node"; then
        visible=1
        break
      fi
      if ! kill -0 "$pid" 2>/dev/null; then
        break
      fi
      sleep 0.25
    done
    if [ "$visible" -ne 1 ]; then
      cat "$log_file" >&2 || true
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
      fail "$node did not enter the ROS graph"
    fi

    transition="$(timeout 20 ros2 lifecycle set "/$node" configure 2>&1 || true)"
    if ! grep -Fqi "Transitioning successful" <<<"$transition"; then
      echo "$transition" >&2
      cat "$log_file" >&2 || true
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
      fail "$node rejected Humble parameters or plugin class IDs"
    fi
    kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    pass "$node lifecycle configure"
  }

  smoke_lifecycle_configure nav2_controller controller_server controller_server
  smoke_lifecycle_configure nav2_planner planner_server planner_server
  smoke_lifecycle_configure nav2_behaviors behavior_server behavior_server
  smoke_lifecycle_configure nav2_bt_navigator bt_navigator bt_navigator
  pass "Humble Nav2 pluginlib lifecycle smoke tests"
fi

echo "Humble migration preflight passed."
