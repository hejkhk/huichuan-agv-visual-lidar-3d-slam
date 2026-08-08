"""Static contracts for the integrated localization and UI launchers."""

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[5]


def test_integrated_ui_launcher_forces_project_domain_88():
    launcher = (PROJECT_ROOT / "START_UI_LOCALIZATION_NAVIGATION.sh").read_text(
        encoding="utf-8")
    ui_launcher = (PROJECT_ROOT / "CAR UI V5.2" / "run.sh").read_text(
        encoding="utf-8")

    assert 'export HUICHUAN_ROS_DOMAIN_ID="${HUICHUAN_ROS_DOMAIN_ID:-88}"' in launcher
    assert 'export ROS_DOMAIN_ID="$HUICHUAN_ROS_DOMAIN_ID"' in launcher
    assert 'export ROS_DOMAIN_ID="${HUICHUAN_ROS_DOMAIN_ID:-88}"' in ui_launcher


def test_camera_visual_is_parallel_without_changing_calibration_arguments():
    urdf = (
        PROJECT_ROOT
        / "lidar/chapt1_ws/src/lidar_py/urdf/agv_box.urdf.xacro"
    ).read_text(encoding="utf-8")
    camera_joint = urdf.split('joint name="base_to_camera_model"', 1)[1]
    camera_joint = camera_joint.split("</joint>", 1)[0]

    assert 'rpy="0 0 0"' in camera_joint
    assert "$(arg camera_pitch)" not in camera_joint
    assert '<xacro:arg name="camera_pitch"' in urdf


def test_loaded_pbstream_starts_bootstrap_trajectory():
    launch = (
        PROJECT_ROOT
        / "lidar/chapt1_ws/src/lidar_py/launch/"
          "cartographer_scan_v2_localization_launch.py"
    ).read_text(encoding="utf-8")
    runner = (
        PROJECT_ROOT / "visual_laser_slam/run_dual_resolution_3d_slam.sh"
    ).read_text(encoding="utf-8")

    assert '"-start_trajectory_with_default_topics=true"' in launch
    assert 'CARTOGRAPHER_CONFIG="cartographer_2d_bootstrap_localization.lua"' in runner


def test_navigation_cloud_uses_fixed_calibrated_ground_band():
    launch = (
        PROJECT_ROOT
        / "lidar/chapt1_ws/src/lidar_py/launch/dual_resolution_3d_slam.launch.py"
    ).read_text(encoding="utf-8")

    assert '"adaptive_ground_plane": False' in launch
    assert '"ground_z_min": typed("local_ground_z_min", float)' in launch
    assert '"ground_z_max": typed("local_ground_z_max", float)' in launch


def test_local_cloud_binary_and_launcher_use_same_pipeline_version():
    runner = (
        PROJECT_ROOT / "visual_laser_slam/run_dual_resolution_3d_slam.sh"
    ).read_text(encoding="utf-8")
    source = (
        PROJECT_ROOT
        / "lidar/chapt1_ws/src/local_depth_cloud_cpp/src/"
          "depth_image_to_local_cloud_v21_node.cpp"
    ).read_text(encoding="utf-8")

    assert 'LOCAL_CLOUD_PIPELINE_VERSION="v6.36"' in runner
    assert 'constexpr char kPipelineVersion[] = "v6.36"' in source


def test_navigation_cloud_separates_mark_clear_and_collision_inputs():
    launch = (
        PROJECT_ROOT
        / "lidar/chapt1_ws/src/lidar_py/launch/dual_resolution_3d_slam.launch.py"
    ).read_text(encoding="utf-8")
    source = (
        PROJECT_ROOT
        / "lidar/chapt1_ws/src/local_depth_cloud_cpp/src/"
          "depth_image_to_local_cloud_v21_node.cpp"
    ).read_text(encoding="utf-8")

    assert '"input_topic": LaunchConfiguration("local_immediate_obstacle_topic")' in launch
    assert '"recent_mark_ground_guard_height_m": 0.120' in launch
    assert '"persistent_mark_ground_guard_height_m": 0.150' in launch
    assert "immediate_obstacle_cloud_pub_->publish(immediate_obstacle_cloud)" in source
    assert "raw_sensor_points_buffer_" in source
    assert "clear_sensor_cloud_pub_->publish(raw_clear_sensor_cloud)" in source


def test_launcher_prefers_persistent_serial_aliases():
    runner = (
        PROJECT_ROOT / "visual_laser_slam/run_dual_resolution_3d_slam.sh"
    ).read_text(encoding="utf-8")

    assert "persistent_serial_alias()" in runner
    assert 'LIDAR_PORT="$(persistent_serial_alias "$LIDAR_PORT")"' in runner
