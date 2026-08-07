"""Staged RGB-D, wheel/IMU, 2D LiDAR and 3D OctoMap test profiles.

Profiles:
  camera         Gemini2 RGB-D stream only
  visual_odom    Gemini2 + RTAB-Map RGB-D odometry
  wheel_imu      STM32 wheel odometry + IMU z-rate + robot_localization EKF
  fusion         RGB-D odometry + wheel odometry + IMU EKF
  slam           fusion profile + validated 2D LiDAR + Cartographer map building
  pointcloud      raw Gemini2 depth PointCloud2 test
  filtered_cloud  raw cloud + robot-centric crop/rate/voxel filtering
  octomap_odom    fusion odometry + filtered cloud + OctoMap in odom frame
  dual_map        STEP5 2D Cartographer + filtered cloud + OctoMap in map frame
  local_highres   STEP10 Python filter from full PointCloud2
  local_highres_v2 STEP10V2 C++ direct depth-image projection (no source PointCloud2)
  local_highres_v21 STEP10V2.1 stable latest-frame worker and spike diagnostics
  rgbd_sync_test independent Color/Depth timestamp synchronization diagnostics
  visual_odom_sync synchronized 15/15Hz RTAB-Map visual odometry test

The existing open_all.sh and validated navigation launch are intentionally not
modified. This launch is a separate test chain so each layer can be verified
before it is merged into the full robot stack.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


VALID_PROFILES = {
    "camera",
    "visual_odom",
    "wheel_imu",
    "fusion",
    "slam",
    "slam_clean",
    "pointcloud",
    "filtered_cloud",
    "octomap_odom",
    "dual_map",
    "local_highres",
    "local_highres_v2",
    "local_highres_v21",
    "rgbd_sync_test",
    "visual_odom_sync",
    "visual_odom_sync_lite",
    "visual_odom_baseline",
}


def _as_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def _string_parameter(name):
    """Pass RTAB-Map core parameters using the string type it registers."""
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def _setup(context):
    pkg_dir = get_package_share_directory("lidar_py")
    profile = LaunchConfiguration("profile").perform(context).strip().lower()
    if profile not in VALID_PROFILES:
        raise RuntimeError(
            "Unknown visual SLAM profile %r. Choose one of: %s"
            % (profile, ", ".join(sorted(VALID_PROFILES)))
        )

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    launch_rviz = _as_bool(LaunchConfiguration("launch_rviz").perform(context))
    actions = [LogInfo(msg=f"[visual_laser_slam] profile={profile}")]

    need_pointcloud = profile in {
        "pointcloud",
        "filtered_cloud",
        "octomap_odom",
        "dual_map",
        "local_highres",
    }
    need_cloud_filter = profile in {"filtered_cloud", "octomap_odom", "dual_map"}
    need_octomap = profile in {"octomap_odom", "dual_map"}
    need_local_highres = profile == "local_highres"
    need_local_highres_v2 = profile == "local_highres_v2"
    need_local_highres_v21 = profile == "local_highres_v21"
    need_rgbd_sync_test = profile == "rgbd_sync_test"
    clean_visual_slam = profile == "slam_clean"
    need_visual_odom_sync = profile in {
        "visual_odom_sync", "visual_odom_sync_lite", "slam_clean"
    }

    need_camera = profile in {
        "camera",
        "visual_odom",
        "visual_odom_sync",
        "visual_odom_sync_lite",
        "visual_odom_baseline",
        "fusion",
        "slam",
        "slam_clean",
        "pointcloud",
        "filtered_cloud",
        "octomap_odom",
        "dual_map",
        "local_highres",
        "local_highres_v2",
        "local_highres_v21",
        "rgbd_sync_test",
    }
    need_visual_odom = profile in {
        "visual_odom",
        "visual_odom_sync",
        "visual_odom_sync_lite",
        "visual_odom_baseline",
        "fusion",
        "slam",
        "slam_clean",
        "octomap_odom",
        "dual_map",
    }
    need_chassis = profile in {
        "wheel_imu",
        "fusion",
        "slam",
        "slam_clean",
        "octomap_odom",
        "dual_map",
    }
    need_ekf = need_chassis
    need_camera_extrinsic = profile in {
        "fusion",
        "slam",
        "slam_clean",
        "filtered_cloud",
        "octomap_odom",
        "dual_map",
        "local_highres",
        "local_highres_v2",
        "local_highres_v21",
        "rgbd_sync_test",
        "visual_odom_sync",
        "visual_odom_sync_lite",
        "visual_odom_baseline",
    }
    need_lidar_slam = profile in {"slam", "slam_clean", "dual_map"}

    if need_camera:
        camera_launch = os.path.join(pkg_dir, "launch", "gemini2_rgbd_640.launch.py")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "camera_name": LaunchConfiguration("camera_name"),
                    "enable_color": (
                        LaunchConfiguration("v2_enable_color")
                        if (need_local_highres_v2 or need_local_highres_v21) else "true"
                    ),
                    "color_width": LaunchConfiguration("color_width"),
                    "color_height": LaunchConfiguration("color_height"),
                    "color_fps": LaunchConfiguration("color_fps"),
                    "depth_width": LaunchConfiguration("depth_width"),
                    "depth_height": LaunchConfiguration("depth_height"),
                    "depth_fps": LaunchConfiguration("depth_fps"),
                    "depth_registration": (
                        LaunchConfiguration("v2_depth_registration")
                        if (need_local_highres_v2 or need_local_highres_v21) else "true"
                    ),
                    "align_mode": "HW",
                    "align_target_stream": "COLOR",
                    "enable_frame_sync": (
                        LaunchConfiguration("v2_enable_frame_sync")
                        if (need_local_highres_v2 or need_local_highres_v21) else "true"
                    ),
                    "enable_point_cloud": "true" if need_pointcloud else "false",
                    "enable_colored_point_cloud": "false",
                    "ordered_pc": "false",
                    "depth_qos": "SENSOR_DATA" if (need_local_highres_v2 or need_local_highres_v21 or need_rgbd_sync_test or need_visual_odom_sync) else "default",
                    "depth_camera_info_qos": (
                        "SENSOR_DATA" if (need_local_highres_v2 or need_local_highres_v21 or need_rgbd_sync_test or need_visual_odom_sync) else "default"
                    ),
                    "color_qos": "SENSOR_DATA" if (need_local_highres_v2 or need_local_highres_v21 or need_rgbd_sync_test or need_visual_odom_sync) else "default",
                    "color_camera_info_qos": (
                        "SENSOR_DATA" if (need_local_highres_v2 or need_local_highres_v21 or need_rgbd_sync_test or need_visual_odom_sync) else "default"
                    ),
                    "enable_noise_removal_filter": (
                        LaunchConfiguration("v2_enable_noise_removal_filter")
                        if (need_local_highres_v2 or need_local_highres_v21) else "true"
                    ),
                    "enable_depth_auto_exposure_priority": (
                        LaunchConfiguration("v21_enable_depth_auto_exposure_priority")
                        if need_local_highres_v21 else "false"
                    ),
                    "enable_sync_host_time": (
                        LaunchConfiguration("camera_enable_sync_host_time")
                    ),
                    "time_domain": LaunchConfiguration("camera_time_domain"),
                    "time_sync_period": LaunchConfiguration("camera_time_sync_period"),
                    "enable_frame_timestamp_csv": (
                        LaunchConfiguration("camera_enable_frame_timestamp_csv")
                    ),
                    "frame_timestamp_csv_file": (
                        LaunchConfiguration("camera_frame_timestamp_csv_file")
                    ),
                    "enable_accel": "false",
                    "enable_gyro": "false",
                    "publish_tf": "true",
                }.items(),
            )
        )

    if need_camera_extrinsic:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera_static_tf",
                output="screen",
                arguments=[
                    "--x", LaunchConfiguration("camera_x"),
                    "--y", LaunchConfiguration("camera_y"),
                    "--z", LaunchConfiguration("camera_z"),
                    "--roll", LaunchConfiguration("camera_roll"),
                    "--pitch", LaunchConfiguration("camera_pitch"),
                    "--yaw", LaunchConfiguration("camera_yaw"),
                    "--frame-id", "base_link",
                    "--child-frame-id", "camera_link",
                ],
            )
        )

    if need_visual_odom:
        sync_tuned = profile in {"visual_odom_sync", "visual_odom_sync_lite"}
        visual_topic = "/visual_odom_clean" if clean_visual_slam else "/visual_odom"

        if clean_visual_slam:
            # STEP11 reuses the configuration that passed the isolated 640 test.
            # RTAB-Map does NOT publish odom->base_link here; EKF is the sole TF authority.
            visual_parameters = {
                "use_sim_time": use_sim_time,
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": False,
                "approx_sync": True,
                "approx_sync_max_interval": _typed(
                    "visual_clean_approx_sync_max_interval", float
                ),
                "topic_queue_size": _typed("visual_clean_topic_queue_size", int),
                "sync_queue_size": _typed("visual_clean_sync_queue_size", int),
                "qos": 2,
                "qos_camera_info": 2,
                "wait_for_transform": _typed(
                    "visual_clean_wait_for_transform", float
                ),
                "wait_imu_to_init": False,
                "publish_null_when_lost": True,
                "Odom/ImageBufferSize": "1",
                "Odom/Strategy": "0",
                "Odom/ResetCountdown": "5",
                "Odom/GuessMotion": _string_parameter(
                    "visual_clean_odom_guess_motion"
                ),
                "Odom/ImageDecimation": _string_parameter(
                    "visual_clean_odom_image_decimation"
                ),
                "Vis/MinInliers": "15",
            }
        else:
            visual_parameters = {
                "use_sim_time": use_sim_time,
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": profile in {
                    "visual_odom", "visual_odom_sync",
                    "visual_odom_sync_lite", "visual_odom_baseline"
                },
                "approx_sync": True,
                "approx_sync_max_interval": (
                    _typed("visual_sync_max_interval", float)
                    if sync_tuned else 0.0
                ),
                "queue_size": (5 if sync_tuned else 20),
                "topic_queue_size": (5 if sync_tuned else 20),
                "sync_queue_size": (10 if sync_tuned else 20),
                "qos": (2 if sync_tuned else 1),
                "qos_camera_info": (2 if sync_tuned else 1),
                "wait_for_transform": (0.10 if sync_tuned else 0.30),
                "wait_imu_to_init": False,
                "publish_null_when_lost": True,
                "Odom/Strategy": "0",
                "Odom/ResetCountdown": "5",
                "Odom/GuessMotion": "true",
                "Vis/MinInliers": "15",
            }

        actions.append(
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name=(
                    "rgbd_odometry_step11_clean"
                    if clean_visual_slam else "rgbd_odometry"
                ),
                namespace="rtabmap",
                output="screen",
                parameters=[visual_parameters],
                remappings=[
                    ("rgb/image", "/camera/color/image_raw"),
                    ("depth/image", "/camera/depth/image_raw"),
                    ("rgb/camera_info", "/camera/color/camera_info"),
                    ("odom", visual_topic),
                ],
            )
        )

    if need_chassis:
        actions.append(
            Node(
                package="lidar_py",
                executable="chassis_node",
                name="chassis_node_visual_slam",
                output="screen",
                parameters=[{
                    "serial_port": LaunchConfiguration("chassis_serial_port"),
                    "baudrate": 115200,
                    "pulse_per_rev": 8388608.0,
                    "gear_ratio": 25.0,
                    "wheel_radius": 0.0755,
                    "wheel_base_h": 0.2145,
                    "wheel_track_w": 0.2825,
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "odom_topic": "/wheel/odom",
                    "publish_tf": False,
                    "use_imu_rp": False,
                    "publish_imu": False,
                    "publish_cartographer_planar_imu": True,
                    "publish_rate": 50.0,
                    "odom_publish_mode": "navi",
                    "cmd_vel_topic": "/cmd_vel_visual_slam_test",
                    "navi_yaw_sign": 1.0,
                    "navi_vx_sign": 1.0,
                    "navi_vz_sign": 1.0,
                    "navi_yaw_offset_deg": 0.0,
                    "navi_odom_yaw_source": "gyro",
                    "navi_vx_scale": 1.0,
                    "navi_vx_deadband_mps": 0.003,
                    "navi_turn_vx_scale": 1.0,
                    "navi_turn_wz_threshold_rad_s": 0.25,
                    "navi_vz_deadband_deg_s": 0.15,
                    "show_serial_window": False,
                    "serial_defaults_on_start": True,
                    "mapping_mode_on_start": True,
                }],
            )
        )

    if need_ekf:
        if profile == "wheel_imu":
            ekf_name = "ekf_wheel_imu.yaml"
        elif profile == "slam_clean":
            ekf_name = "ekf_step11_visual_wheel_imu.yaml"
        else:
            ekf_name = "ekf_visual_wheel_imu.yaml"
        actions.append(
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    os.path.join(pkg_dir, "config", ekf_name),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[("odometry/filtered", "/odometry/filtered")],
            )
        )

    if need_cloud_filter:
        actions.append(
            Node(
                package="lidar_py",
                executable="point_cloud_filter_node",
                name="depth_point_cloud_filter",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "input_topic": LaunchConfiguration("point_cloud_topic"),
                    "output_topic": LaunchConfiguration("filtered_cloud_topic"),
                    "base_frame": "base_link",
                    "max_rate_hz": _typed("cloud_max_rate_hz", float),
                    "sample_stride": _typed("cloud_sample_stride", int),
                    "voxel_size": _typed("cloud_voxel_size", float),
                    "min_range": _typed("cloud_min_range", float),
                    "max_range": _typed("cloud_max_range", float),
                    "transform_timeout": _typed("cloud_transform_timeout", float),
                    "base_x_min": _typed("cloud_base_x_min", float),
                    "base_x_max": _typed("cloud_base_x_max", float),
                    "base_y_min": _typed("cloud_base_y_min", float),
                    "base_y_max": _typed("cloud_base_y_max", float),
                    "base_z_min": _typed("cloud_base_z_min", float),
                    "base_z_max": _typed("cloud_base_z_max", float),
                    "remove_self": _typed("cloud_remove_self", bool),
                    "self_x_min": _typed("cloud_self_x_min", float),
                    "self_x_max": _typed("cloud_self_x_max", float),
                    "self_y_min": _typed("cloud_self_y_min", float),
                    "self_y_max": _typed("cloud_self_y_max", float),
                    "self_z_min": _typed("cloud_self_z_min", float),
                    "self_z_max": _typed("cloud_self_z_max", float),
                    "log_every_n": _typed("cloud_log_every_n", int),
                }],
            )
        )

    if need_local_highres:
        actions.append(
            Node(
                package="lidar_py",
                executable="local_highres_cloud_node",
                name="local_highres_cloud_filter",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "input_topic": LaunchConfiguration("point_cloud_topic"),
                    "output_topic": LaunchConfiguration("local_cloud_topic"),
                    "stats_topic": LaunchConfiguration("local_cloud_stats_topic"),
                    "marker_topic": LaunchConfiguration("local_cloud_marker_topic"),
                    "output_frame": "base_link",
                    "max_rate_hz": _typed("local_max_rate_hz", float),
                    "sample_stride": _typed("local_sample_stride", int),
                    "voxel_size": _typed("local_voxel_size", float),
                    "min_range": _typed("local_min_range", float),
                    "max_range": _typed("local_max_range", float),
                    "transform_timeout": _typed("local_transform_timeout", float),
                    "x_min": _typed("local_x_min", float),
                    "x_max": _typed("local_x_max", float),
                    "y_min": _typed("local_y_min", float),
                    "y_max": _typed("local_y_max", float),
                    "z_min": _typed("local_z_min", float),
                    "z_max": _typed("local_z_max", float),
                    "remove_self": _typed("local_remove_self", bool),
                    "self_x_min": _typed("local_self_x_min", float),
                    "self_x_max": _typed("local_self_x_max", float),
                    "self_y_min": _typed("local_self_y_min", float),
                    "self_y_max": _typed("local_self_y_max", float),
                    "self_z_min": _typed("local_self_z_min", float),
                    "self_z_max": _typed("local_self_z_max", float),
                    "ground_filter_enabled": _typed(
                        "local_ground_filter_enabled", bool
                    ),
                    "ground_z_min": _typed("local_ground_z_min", float),
                    "ground_z_max": _typed("local_ground_z_max", float),
                    "stats_period_sec": _typed("local_stats_period_sec", float),
                    "publish_markers": _typed("local_publish_markers", bool),
                }],
            )
        )

    if need_local_highres_v2:
        actions.append(
            Node(
                package="local_depth_cloud_cpp",
                executable="depth_image_to_local_cloud_node",
                name="depth_image_to_local_cloud",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "depth_topic": LaunchConfiguration("v2_depth_topic"),
                    "camera_info_topic": LaunchConfiguration("v2_camera_info_topic"),
                    "output_topic": LaunchConfiguration("v2_cloud_topic"),
                    "stats_topic": LaunchConfiguration("v2_stats_topic"),
                    "marker_topic": LaunchConfiguration("v2_marker_topic"),
                    "output_frame": "base_link",
                    "max_rate_hz": _typed("v2_max_rate_hz", float),
                    "pixel_stride": _typed("v2_pixel_stride", int),
                    "depth_unit_scale": _typed("v2_depth_unit_scale", float),
                    "voxel_size": _typed("v2_voxel_size", float),
                    "min_range": _typed("v2_min_range", float),
                    "max_range": _typed("v2_max_range", float),
                    "transform_timeout": _typed("v2_transform_timeout", float),
                    "roi_u_min": _typed("v2_roi_u_min", int),
                    "roi_u_max": _typed("v2_roi_u_max", int),
                    "roi_v_min": _typed("v2_roi_v_min", int),
                    "roi_v_max": _typed("v2_roi_v_max", int),
                    "x_min": _typed("v2_x_min", float),
                    "x_max": _typed("v2_x_max", float),
                    "y_min": _typed("v2_y_min", float),
                    "y_max": _typed("v2_y_max", float),
                    "z_min": _typed("v2_z_min", float),
                    "z_max": _typed("v2_z_max", float),
                    "remove_self": _typed("v2_remove_self", bool),
                    "self_x_min": _typed("v2_self_x_min", float),
                    "self_x_max": _typed("v2_self_x_max", float),
                    "self_y_min": _typed("v2_self_y_min", float),
                    "self_y_max": _typed("v2_self_y_max", float),
                    "self_z_min": _typed("v2_self_z_min", float),
                    "self_z_max": _typed("v2_self_z_max", float),
                    "ground_filter_enabled": _typed(
                        "v2_ground_filter_enabled", bool
                    ),
                    "ground_z_min": _typed("v2_ground_z_min", float),
                    "ground_z_max": _typed("v2_ground_z_max", float),
                    "stats_period_sec": _typed("v2_stats_period_sec", float),
                    "publish_markers": _typed("v2_publish_markers", bool),
                }],
            )
        )

    if need_local_highres_v21:
        actions.append(
            Node(
                package="local_depth_cloud_cpp",
                executable="depth_image_to_local_cloud_v21_node",
                name="depth_image_to_local_cloud_v21",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "depth_topic": LaunchConfiguration("v21_depth_topic"),
                    "camera_info_topic": LaunchConfiguration("v21_camera_info_topic"),
                    "output_topic": LaunchConfiguration("v21_cloud_topic"),
                    "stats_topic": LaunchConfiguration("v21_stats_topic"),
                    "marker_topic": LaunchConfiguration("v21_marker_topic"),
                    "output_frame": "base_link",
                    "max_rate_hz": _typed("v21_max_rate_hz", float),
                    "pixel_stride": _typed("v21_pixel_stride", int),
                    "depth_unit_scale": _typed("v21_depth_unit_scale", float),
                    "voxel_size": _typed("v21_voxel_size", float),
                    "min_range": _typed("v21_min_range", float),
                    "max_range": _typed("v21_max_range", float),
                    "transform_timeout": _typed("v21_transform_timeout", float),
                    "max_input_age_ms": _typed("v21_max_input_age_ms", float),
                    "roi_u_min": _typed("v21_roi_u_min", int),
                    "roi_u_max": _typed("v21_roi_u_max", int),
                    "roi_v_min": _typed("v21_roi_v_min", int),
                    "roi_v_max": _typed("v21_roi_v_max", int),
                    "x_min": _typed("v21_x_min", float),
                    "x_max": _typed("v21_x_max", float),
                    "y_min": _typed("v21_y_min", float),
                    "y_max": _typed("v21_y_max", float),
                    "z_min": _typed("v21_z_min", float),
                    "z_max": _typed("v21_z_max", float),
                    "remove_self": _typed("v21_remove_self", bool),
                    "self_x_min": _typed("v21_self_x_min", float),
                    "self_x_max": _typed("v21_self_x_max", float),
                    "self_y_min": _typed("v21_self_y_min", float),
                    "self_y_max": _typed("v21_self_y_max", float),
                    "self_z_min": _typed("v21_self_z_min", float),
                    "self_z_max": _typed("v21_self_z_max", float),
                    "ground_filter_enabled": _typed(
                        "v21_ground_filter_enabled", bool
                    ),
                    "ground_z_min": _typed("v21_ground_z_min", float),
                    "ground_z_max": _typed("v21_ground_z_max", float),
                    "stats_period_sec": _typed("v21_stats_period_sec", float),
                    "stats_window_size": _typed("v21_stats_window_size", int),
                    "process_warn_ms": _typed("v21_process_warn_ms", float),
                    "age_warn_ms": _typed("v21_age_warn_ms", float),
                    "stall_warn_gap_ms": _typed("v21_stall_warn_gap_ms", float),
                    "publish_markers": _typed("v21_publish_markers", bool),
                }],
            )
        )

    if need_rgbd_sync_test or profile == "visual_odom_sync":
        actions.append(
            Node(
                package="local_depth_cloud_cpp",
                executable="rgbd_timestamp_monitor_node",
                name="rgbd_timestamp_monitor",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "color_topic": LaunchConfiguration("sync_color_topic"),
                    "depth_topic": LaunchConfiguration("sync_depth_topic"),
                    "stats_topic": LaunchConfiguration("sync_stats_topic"),
                    "max_pair_interval_ms": _typed("sync_max_pair_interval_ms", float),
                    "warn_p95_ms": _typed("sync_warn_p95_ms", float),
                    "window_size": _typed("sync_window_size", int),
                }],
            )
        )

    if need_lidar_slam:
        cartographer_scan_topic = "/scan_timed_v2"
        filtered_scan_topic = "/scan_timed_v2_filtered"
        actions.extend([
            Node(
                package="lidar_py",
                executable="lidar_node",
                name="lidar_node_visual_fusion",
                output="screen",
                parameters=[{
                    "serial_port": LaunchConfiguration("lidar_serial_port"),
                    "baudrate": _typed("lidar_baudrate", int),
                    "frame_id": "laser_frame",
                    "scan_interval": 0.1,
                    "laser_yaw_deg": _typed("laser_yaw_deg", float),
                    "scan_angle_sign": _typed("scan_angle_sign", float),
                    "publish_timed_scan": True,
                    "timed_scan_topic": "/scan_timed",
                    "publish_fixed_timed_scan": True,
                    "fixed_timed_scan_topic": cartographer_scan_topic,
                    "fixed_scan_bins": 360,
                    "fixed_scan_min_raw_points": 180,
                    "fixed_scan_max_raw_points": 720,
                    "fixed_scan_min_valid_points": 180,
                    "fixed_scan_min_time_sec": 0.10,
                    "fixed_scan_max_time_sec": 0.35,
                    "clock_max_adjustment_ns": 100000,
                }],
            ),
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                name="scan_to_scan_filter_chain_visual_fusion",
                output="screen",
                parameters=[os.path.join(pkg_dir, "config", "laser_filter.yaml")],
                remappings=[
                    ("scan", cartographer_scan_topic),
                    ("scan_filtered", filtered_scan_topic),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node_visual_fusion",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory", os.path.join(pkg_dir, "config"),
                    "-configuration_basename", "cartographer_2d_visual_fusion.lua",
                ],
                remappings=[
                    ("scan", filtered_scan_topic),
                    ("odom", "/odometry/filtered"),
                    ("imu", "/imu_cartographer"),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node_visual_fusion",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
            ),
            Node(
                package="lidar_py",
                executable="robot_pose_publisher",
                name="robot_pose_publisher_visual_fusion",
                output="screen",
                parameters=[{
                    "map_frame": "map",
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "orientation_source": "map",
                    "publish_rate": 10.0,
                    "topic": "/robot_pose",
                }],
            ),
        ])

    if need_octomap:
        octomap_world_frame = "map" if profile == "dual_map" else "odom"
        actions.append(
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server_3d",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "frame_id": octomap_world_frame,
                    "base_frame_id": "base_link",
                    "resolution": _typed("octomap_resolution", float),
                    "sensor_model.max_range": _typed("octomap_max_range", float),
                    "sensor_model.hit": 0.70,
                    "sensor_model.miss": 0.40,
                    "sensor_model.min": 0.12,
                    "sensor_model.max": 0.97,
                    "point_cloud_min_z": _typed("octomap_point_min_z", float),
                    "point_cloud_max_z": _typed("octomap_point_max_z", float),
                    "occupancy_min_z": _typed("octomap_occupancy_min_z", float),
                    "occupancy_max_z": _typed("octomap_occupancy_max_z", float),
                    "filter_ground_plane": _typed("octomap_filter_ground", bool),
                    "filter_speckles": True,
                    "compress_map": True,
                    "incremental_2D_projection": True,
                    "use_height_map": True,
                    "colored_map": False,
                    "publish_free_space": False,
                    "latch": _typed("octomap_latch", bool),
                }],
                remappings=[
                    ("cloud_in", LaunchConfiguration("filtered_cloud_topic")),
                ],
            )
        )

    if launch_rviz:
        rviz_by_profile = {
            "camera": "rgbd_camera_test.rviz",
            "pointcloud": "depth_pointcloud_test.rviz",
            "filtered_cloud": "filtered_pointcloud_test.rviz",
            "octomap_odom": "octomap_odom_debug.rviz",
            "dual_map": "dual_2d_3d_mapping.rviz",
            "local_highres": "local_highres_cloud_test.rviz",
            "local_highres_v2": "local_highres_cloud_v2_test.rviz",
            "local_highres_v21": "local_highres_cloud_v21_test.rviz",
            "rgbd_sync_test": "rgbd_camera_test.rviz",
            "visual_odom_sync": "visual_odom_sync_test.rviz",
            "visual_odom_sync_lite": "visual_odom_sync_lite.rviz",
            "visual_odom_baseline": "visual_odom_sync_lite.rviz",
            "slam": "visual_laser_slam_map.rviz",
            "slam_clean": "visual_laser_slam_map.rviz",
        }
        rviz_config = rviz_by_profile.get(profile, "visual_laser_slam_debug.rviz")
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name=f"rviz2_{profile}",
                output="screen",
                arguments=["-d", os.path.join(pkg_dir, "rviz", rviz_config)],
            )
        )

    return actions


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("profile", default_value="camera"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="15"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="400"),
        DeclareLaunchArgument("depth_fps", default_value="10"),
        DeclareLaunchArgument("v2_enable_color", default_value="false"),
        DeclareLaunchArgument("v2_depth_registration", default_value="false"),
        DeclareLaunchArgument("v2_enable_frame_sync", default_value="false"),
        DeclareLaunchArgument(
            "v2_enable_noise_removal_filter", default_value="false"
        ),
        DeclareLaunchArgument("v21_enable_depth_auto_exposure_priority", default_value="false"),
        DeclareLaunchArgument("camera_enable_sync_host_time", default_value="true"),
        DeclareLaunchArgument("camera_time_domain", default_value="device"),
        DeclareLaunchArgument("camera_time_sync_period", default_value="60"),
        DeclareLaunchArgument("camera_enable_frame_timestamp_csv", default_value="false"),
        DeclareLaunchArgument(
            "camera_frame_timestamp_csv_file",
            default_value="/tmp/orbbec_frame_timestamp.csv",
        ),
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument("chassis_serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_serial_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
        DeclareLaunchArgument("laser_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("scan_angle_sign", default_value="-1.0"),
        DeclareLaunchArgument("point_cloud_topic", default_value="/camera/depth/points"),
        DeclareLaunchArgument(
            "filtered_cloud_topic", default_value="/camera/depth/points_filtered"
        ),
        DeclareLaunchArgument("cloud_max_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("cloud_sample_stride", default_value="4"),
        DeclareLaunchArgument("cloud_voxel_size", default_value="0.06"),
        DeclareLaunchArgument("cloud_min_range", default_value="0.30"),
        DeclareLaunchArgument("cloud_max_range", default_value="4.0"),
        DeclareLaunchArgument("cloud_transform_timeout", default_value="0.20"),
        DeclareLaunchArgument("cloud_base_x_min", default_value="-0.50"),
        DeclareLaunchArgument("cloud_base_x_max", default_value="4.50"),
        DeclareLaunchArgument("cloud_base_y_min", default_value="-3.00"),
        DeclareLaunchArgument("cloud_base_y_max", default_value="3.00"),
        DeclareLaunchArgument("cloud_base_z_min", default_value="-1.00"),
        DeclareLaunchArgument("cloud_base_z_max", default_value="2.50"),
        DeclareLaunchArgument("cloud_remove_self", default_value="false"),
        DeclareLaunchArgument("cloud_self_x_min", default_value="-0.40"),
        DeclareLaunchArgument("cloud_self_x_max", default_value="0.40"),
        DeclareLaunchArgument("cloud_self_y_min", default_value="-0.40"),
        DeclareLaunchArgument("cloud_self_y_max", default_value="0.40"),
        DeclareLaunchArgument("cloud_self_z_min", default_value="-0.20"),
        DeclareLaunchArgument("cloud_self_z_max", default_value="1.20"),
        DeclareLaunchArgument("cloud_log_every_n", default_value="30"),
        DeclareLaunchArgument("local_cloud_topic", default_value="/local_highres_cloud"),
        DeclareLaunchArgument(
            "local_cloud_stats_topic", default_value="/local_highres_cloud/stats"
        ),
        DeclareLaunchArgument(
            "local_cloud_marker_topic",
            default_value="/local_highres_cloud/crop_markers",
        ),
        DeclareLaunchArgument("local_max_rate_hz", default_value="12.0"),
        DeclareLaunchArgument("local_sample_stride", default_value="1"),
        DeclareLaunchArgument("local_voxel_size", default_value="0.025"),
        DeclareLaunchArgument("local_min_range", default_value="0.20"),
        DeclareLaunchArgument("local_max_range", default_value="4.0"),
        DeclareLaunchArgument("local_transform_timeout", default_value="0.03"),
        DeclareLaunchArgument("local_x_min", default_value="0.20"),
        DeclareLaunchArgument("local_x_max", default_value="4.00"),
        DeclareLaunchArgument("local_y_min", default_value="-2.50"),
        DeclareLaunchArgument("local_y_max", default_value="2.50"),
        DeclareLaunchArgument("local_z_min", default_value="-0.50"),
        DeclareLaunchArgument("local_z_max", default_value="2.00"),
        DeclareLaunchArgument("local_remove_self", default_value="true"),
        DeclareLaunchArgument("local_self_x_min", default_value="-0.36"),
        DeclareLaunchArgument("local_self_x_max", default_value="0.36"),
        DeclareLaunchArgument("local_self_y_min", default_value="-0.36"),
        DeclareLaunchArgument("local_self_y_max", default_value="0.36"),
        DeclareLaunchArgument("local_self_z_min", default_value="-0.10"),
        DeclareLaunchArgument("local_self_z_max", default_value="0.90"),
        DeclareLaunchArgument(
            "local_ground_filter_enabled", default_value="false"
        ),
        DeclareLaunchArgument("local_ground_z_min", default_value="-0.06"),
        DeclareLaunchArgument("local_ground_z_max", default_value="0.08"),
        DeclareLaunchArgument("local_stats_period_sec", default_value="1.0"),
        DeclareLaunchArgument("local_publish_markers", default_value="true"),
        DeclareLaunchArgument("v21_depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("v21_camera_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("v21_cloud_topic", default_value="/local_highres_cloud_v21"),
        DeclareLaunchArgument("v21_stats_topic", default_value="/local_highres_cloud_v21/stats"),
        DeclareLaunchArgument("v21_marker_topic", default_value="/local_highres_cloud_v21/crop_markers"),
        DeclareLaunchArgument("v21_max_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("v21_pixel_stride", default_value="2"),
        DeclareLaunchArgument("v21_depth_unit_scale", default_value="0.001"),
        DeclareLaunchArgument("v21_voxel_size", default_value="0.03"),
        DeclareLaunchArgument("v21_min_range", default_value="0.20"),
        DeclareLaunchArgument("v21_max_range", default_value="4.0"),
        DeclareLaunchArgument("v21_transform_timeout", default_value="0.50"),
        DeclareLaunchArgument("v21_max_input_age_ms", default_value="150.0"),
        DeclareLaunchArgument("v21_roi_u_min", default_value="0"),
        DeclareLaunchArgument("v21_roi_u_max", default_value="-1"),
        DeclareLaunchArgument("v21_roi_v_min", default_value="0"),
        DeclareLaunchArgument("v21_roi_v_max", default_value="-1"),
        DeclareLaunchArgument("v21_x_min", default_value="0.15"),
        DeclareLaunchArgument("v21_x_max", default_value="4.00"),
        DeclareLaunchArgument("v21_y_min", default_value="-2.50"),
        DeclareLaunchArgument("v21_y_max", default_value="2.50"),
        DeclareLaunchArgument("v21_z_min", default_value="-0.50"),
        DeclareLaunchArgument("v21_z_max", default_value="2.00"),
        DeclareLaunchArgument("v21_remove_self", default_value="true"),
        DeclareLaunchArgument("v21_self_x_min", default_value="-0.36"),
        DeclareLaunchArgument("v21_self_x_max", default_value="0.36"),
        DeclareLaunchArgument("v21_self_y_min", default_value="-0.36"),
        DeclareLaunchArgument("v21_self_y_max", default_value="0.36"),
        DeclareLaunchArgument("v21_self_z_min", default_value="-0.10"),
        DeclareLaunchArgument("v21_self_z_max", default_value="0.90"),
        DeclareLaunchArgument("v21_ground_filter_enabled", default_value="false"),
        DeclareLaunchArgument("v21_ground_z_min", default_value="-0.06"),
        DeclareLaunchArgument("v21_ground_z_max", default_value="0.08"),
        DeclareLaunchArgument("v21_stats_period_sec", default_value="1.0"),
        DeclareLaunchArgument("v21_stats_window_size", default_value="300"),
        DeclareLaunchArgument("v21_process_warn_ms", default_value="50.0"),
        DeclareLaunchArgument("v21_age_warn_ms", default_value="120.0"),
        DeclareLaunchArgument("v21_stall_warn_gap_ms", default_value="120.0"),
        DeclareLaunchArgument("v21_publish_markers", default_value="true"),
        DeclareLaunchArgument("sync_color_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("sync_depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("sync_stats_topic", default_value="/rgbd_timestamp_sync/stats"),
        DeclareLaunchArgument("sync_max_pair_interval_ms", default_value="40.0"),
        DeclareLaunchArgument("sync_warn_p95_ms", default_value="25.0"),
        DeclareLaunchArgument("sync_window_size", default_value="300"),
        DeclareLaunchArgument("visual_sync_max_interval", default_value="0.025"),
        DeclareLaunchArgument(
            "visual_clean_approx_sync_max_interval", default_value="0.020"
        ),
        DeclareLaunchArgument("visual_clean_topic_queue_size", default_value="1"),
        DeclareLaunchArgument("visual_clean_sync_queue_size", default_value="3"),
        DeclareLaunchArgument(
            "visual_clean_wait_for_transform", default_value="0.10"
        ),
        DeclareLaunchArgument(
            "visual_clean_odom_guess_motion", default_value="false"
        ),
        DeclareLaunchArgument(
            "visual_clean_odom_image_decimation", default_value="1"
        ),

        DeclareLaunchArgument("v2_depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument(
            "v2_camera_info_topic", default_value="/camera/depth/camera_info"
        ),
        DeclareLaunchArgument("v2_cloud_topic", default_value="/local_highres_cloud_v2"),
        DeclareLaunchArgument(
            "v2_stats_topic", default_value="/local_highres_cloud_v2/stats"
        ),
        DeclareLaunchArgument(
            "v2_marker_topic", default_value="/local_highres_cloud_v2/crop_markers"
        ),
        DeclareLaunchArgument("v2_max_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("v2_pixel_stride", default_value="2"),
        DeclareLaunchArgument("v2_depth_unit_scale", default_value="0.001"),
        DeclareLaunchArgument("v2_voxel_size", default_value="0.03"),
        DeclareLaunchArgument("v2_min_range", default_value="0.20"),
        DeclareLaunchArgument("v2_max_range", default_value="4.0"),
        DeclareLaunchArgument("v2_transform_timeout", default_value="0.015"),
        DeclareLaunchArgument("v2_roi_u_min", default_value="0"),
        DeclareLaunchArgument("v2_roi_u_max", default_value="-1"),
        DeclareLaunchArgument("v2_roi_v_min", default_value="0"),
        DeclareLaunchArgument("v2_roi_v_max", default_value="-1"),
        DeclareLaunchArgument("v2_x_min", default_value="0.15"),
        DeclareLaunchArgument("v2_x_max", default_value="4.00"),
        DeclareLaunchArgument("v2_y_min", default_value="-2.50"),
        DeclareLaunchArgument("v2_y_max", default_value="2.50"),
        DeclareLaunchArgument("v2_z_min", default_value="-0.50"),
        DeclareLaunchArgument("v2_z_max", default_value="2.00"),
        DeclareLaunchArgument("v2_remove_self", default_value="true"),
        DeclareLaunchArgument("v2_self_x_min", default_value="-0.36"),
        DeclareLaunchArgument("v2_self_x_max", default_value="0.36"),
        DeclareLaunchArgument("v2_self_y_min", default_value="-0.36"),
        DeclareLaunchArgument("v2_self_y_max", default_value="0.36"),
        DeclareLaunchArgument("v2_self_z_min", default_value="-0.10"),
        DeclareLaunchArgument("v2_self_z_max", default_value="0.90"),
        DeclareLaunchArgument("v2_ground_filter_enabled", default_value="false"),
        DeclareLaunchArgument("v2_ground_z_min", default_value="-0.06"),
        DeclareLaunchArgument("v2_ground_z_max", default_value="0.08"),
        DeclareLaunchArgument("v2_stats_period_sec", default_value="1.0"),
        DeclareLaunchArgument("v2_publish_markers", default_value="true"),
        DeclareLaunchArgument("octomap_resolution", default_value="0.08"),
        DeclareLaunchArgument("octomap_max_range", default_value="4.0"),
        DeclareLaunchArgument("octomap_point_min_z", default_value="-2.0"),
        DeclareLaunchArgument("octomap_point_max_z", default_value="3.0"),
        DeclareLaunchArgument("octomap_occupancy_min_z", default_value="-1.0"),
        DeclareLaunchArgument("octomap_occupancy_max_z", default_value="2.5"),
        DeclareLaunchArgument("octomap_filter_ground", default_value="false"),
        DeclareLaunchArgument("octomap_latch", default_value="false"),
    ]

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1"),
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        *arguments,
        OpaqueFunction(function=_setup),
    ])
