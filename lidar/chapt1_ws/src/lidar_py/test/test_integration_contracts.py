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
