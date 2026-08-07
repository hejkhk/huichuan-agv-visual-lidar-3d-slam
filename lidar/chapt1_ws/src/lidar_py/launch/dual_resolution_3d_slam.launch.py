"""Stable 2D SLAM plus persistent low-res and live high-res 3D perception.

Cartographer owns map->odom. Chassis owns odom->base_link in the stable profile,
or robot_localization owns it in the guarded visual-fusion profile. RTAB-Map
consumes Cartographer's corrected pose but never publishes TF, so visual loop
closures cannot move the planar navigation tree.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def string_parameter(name):
    """RTAB-Map core settings are declared as ROS string parameters."""
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    pkg_dir = get_package_share_directory("lidar_py")
    stable_2d_launch = os.path.join(
        pkg_dir, "launch", "cartographer_scan_v2_launch.py")
    localization_2d_launch = os.path.join(
        pkg_dir, "launch", "cartographer_scan_v2_localization_launch.py")
    navigation_launch = os.path.join(
        pkg_dir, "launch", "cartographer_auto_mapping_humble_launch.py")
    project_bt_dir = os.path.join(pkg_dir, "behavior_trees")
    controller_override = os.path.join(
        pkg_dir, "config", "nav2_all_beifen_humble_override.yaml")
    stvl_override = os.path.join(
        pkg_dir, "config", "nav2_dual_3d_stvl_override.yaml")
    camera_launch = os.path.join(
        pkg_dir, "launch", "gemini2_experimental.launch.py")
    rviz_config = os.path.join(
        pkg_dir, "rviz", "dual_resolution_3d_slam.rviz")
    robot_model = os.path.join(pkg_dir, "urdf", "agv_box.urdf.xacro")
    visual_ekf_config = os.path.join(
        pkg_dir, "config", "ekf_dual_3d_visual_fusion.yaml")

    arguments = [
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("enable_navigation", default_value="false"),
        DeclareLaunchArgument("nav_autostart", default_value="true"),
        DeclareLaunchArgument("localization_mode", default_value="false"),
        DeclareLaunchArgument("localization_map_yaml", default_value=""),
        DeclareLaunchArgument(
            "mutable_map_mark_confirmations", default_value="3"),
        DeclareLaunchArgument(
            "mutable_map_clear_confirmations", default_value="20"),
        DeclareLaunchArgument(
            "mutable_map_evidence_rate", default_value="5.0"),
        DeclareLaunchArgument(
            "cartographer_load_state_filename", default_value=""),
        DeclareLaunchArgument(
            "rviz_config_file", default_value=rviz_config),
        DeclareLaunchArgument(
            "cartographer_config",
            default_value="cartographer_2d_v9_tightened.lua"),
        DeclareLaunchArgument("enable_visual_fusion", default_value="false"),
        DeclareLaunchArgument("chassis_publish_tf", default_value="true"),
        DeclareLaunchArgument("cartographer_odom_topic", default_value="/odom"),
        DeclareLaunchArgument(
            "rtabmap_odom_topic", default_value="/cartographer_pose_odom"),
        DeclareLaunchArgument(
            "nav_costmap_override_file", default_value=stvl_override),
        DeclareLaunchArgument("enable_rtabmap", default_value="true"),
        DeclareLaunchArgument("rtabmap_on_demand_pause", default_value="false"),
        DeclareLaunchArgument("use_rtabmap_viz", default_value="false"),
        DeclareLaunchArgument("lidar_serial_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
        DeclareLaunchArgument("chassis_serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("laser_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("laser_x", default_value="0.20"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.4235"),
        DeclareLaunchArgument("scan_angle_sign", default_value="-1.0"),
        DeclareLaunchArgument("navi_yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("navi_vx_sign", default_value="1.0"),
        DeclareLaunchArgument("navi_vz_sign", default_value="1.0"),
        DeclareLaunchArgument("navi_yaw_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("navi_odom_yaw_source", default_value="absolute"),
        DeclareLaunchArgument("navi_max_yaw_rate_deg_s", default_value="120.0"),
        DeclareLaunchArgument("odom_publish_mode", default_value="navi"),
        DeclareLaunchArgument("navi_adaptive_clock_sync", default_value="true"),
        DeclareLaunchArgument("navi_clock_window_samples", default_value="250"),
        DeclareLaunchArgument("navi_clock_max_adjustment_ns", default_value="20000"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_enabled", default_value="true"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_pose_enabled", default_value="false"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_warmup_sec", default_value="6.0"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_window_sec", default_value="0.75"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_translation_m", default_value="0.08"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_yaw_deg", default_value="3.0"),
        DeclareLaunchArgument(
            "nav_zero_command_cancel_sec", default_value="25.0"),
        DeclareLaunchArgument(
            "require_system_ready_for_motion", default_value="false"),
        DeclareLaunchArgument("show_serial_window", default_value="false"),
        DeclareLaunchArgument(
            "require_depth_baseline_for_ps2", default_value="true"),
        DeclareLaunchArgument("enable_fixed_scan_filter", default_value="true"),
        DeclareLaunchArgument("fixed_scan_min_raw_points", default_value="180"),
        DeclareLaunchArgument("fixed_scan_min_valid_points", default_value="0"),
        DeclareLaunchArgument("filtered_scan_topic", default_value="/scan_timed_v2_filtered"),
        DeclareLaunchArgument("rtabmap_scan_topic", default_value="/scan_timed_v2"),

        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="15"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="400"),
        DeclareLaunchArgument("depth_fps", default_value="15"),
        DeclareLaunchArgument("depth_registration", default_value="false"),
        DeclareLaunchArgument("align_mode", default_value="SW"),
        DeclareLaunchArgument("align_target_stream", default_value="DEPTH"),
        DeclareLaunchArgument("camera_time_domain", default_value="global"),
        DeclareLaunchArgument("camera_time_sync_period", default_value="10.0"),
        DeclareLaunchArgument("camera_enable_frame_drop_log", default_value="false"),
        DeclareLaunchArgument("camera_frame_timestamp_csv_file", default_value=""),
        DeclareLaunchArgument("rgbd_sync_max_interval", default_value="0.045"),
        DeclareLaunchArgument("rgbd_sync_max_interval_ms", default_value="45.0"),
        DeclareLaunchArgument("rgbd_sync_warn_p95_ms", default_value="45.0"),
        DeclareLaunchArgument("camera_x", default_value="0.30"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.40"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),

        DeclareLaunchArgument("database_path", default_value="/tmp/rtabmap_3d.db"),
        DeclareLaunchArgument("rtabmap_rate", default_value="2.0"),
        DeclareLaunchArgument("rtabmap_threads", default_value="2"),
        DeclareLaunchArgument("global_3d_voxel", default_value="0.08"),
        DeclareLaunchArgument("global_3d_range_max", default_value="4.0"),
        DeclareLaunchArgument("use_octomap", default_value="true"),
        DeclareLaunchArgument("enable_resource_monitor", default_value="true"),
        DeclareLaunchArgument(
            "resource_monitor_report_interval", default_value="20.0"),
        DeclareLaunchArgument(
            "resource_usage_csv_file", default_value=""),
        DeclareLaunchArgument(
            "resource_monitor_project_root", default_value=""),
        DeclareLaunchArgument(
            "resource_monitor_run_directory", default_value=""),

        DeclareLaunchArgument("local_cloud_topic", default_value="/local_highres_cloud_v21"),
        DeclareLaunchArgument(
            "local_sensor_cloud_topic",
            default_value="/local_highres_cloud_v21/sensor"),
        DeclareLaunchArgument(
            "local_persistent_sensor_cloud_topic",
            default_value="/local_highres_cloud_v21/persistent_sensor"),
        DeclareLaunchArgument("local_stats_topic", default_value="/local_highres_cloud_v21/stats"),
        DeclareLaunchArgument("local_marker_topic", default_value="/local_highres_cloud_v21/crop_markers"),
        DeclareLaunchArgument("local_rate", default_value="15.0"),
        DeclareLaunchArgument("local_stride", default_value="2"),
        DeclareLaunchArgument("local_voxel", default_value="0.03"),
        DeclareLaunchArgument("local_min_range", default_value="0.20"),
        DeclareLaunchArgument("local_max_range", default_value="4.0"),
        DeclareLaunchArgument("local_x_min", default_value="0.15"),
        DeclareLaunchArgument("local_x_max", default_value="4.0"),
        DeclareLaunchArgument("local_y_min", default_value="-2.5"),
        DeclareLaunchArgument("local_y_max", default_value="2.5"),
        DeclareLaunchArgument("local_z_min", default_value="-0.5"),
        DeclareLaunchArgument("local_z_max", default_value="2.0"),
        DeclareLaunchArgument("local_ground_filter", default_value="false"),
        DeclareLaunchArgument("local_ground_z_min", default_value="-0.10"),
        DeclareLaunchArgument("local_ground_z_max", default_value="0.02"),
        DeclareLaunchArgument("local_spatial_filter", default_value="true"),
        DeclareLaunchArgument("local_spatial_threshold_m", default_value="0.08"),
        DeclareLaunchArgument("local_spatial_threshold_ratio", default_value="0.025"),
        DeclareLaunchArgument("local_spatial_min_neighbors", default_value="2"),
        DeclareLaunchArgument("local_temporal_filter", default_value="true"),
        DeclareLaunchArgument("local_temporal_alpha", default_value="0.65"),
        DeclareLaunchArgument("local_temporal_max_delta_m", default_value="0.06"),
        DeclareLaunchArgument("local_voxel_outlier_filter", default_value="true"),
        DeclareLaunchArgument("local_voxel_min_neighbors", default_value="1"),
    ]

    stable_2d_arguments = {
            "use_rviz": "false",
            "lidar_serial_port": LaunchConfiguration("lidar_serial_port"),
            "lidar_baudrate": LaunchConfiguration("lidar_baudrate"),
            "chassis_serial_port": LaunchConfiguration("chassis_serial_port"),
            "laser_yaw_deg": LaunchConfiguration("laser_yaw_deg"),
            "laser_x": LaunchConfiguration("laser_x"),
            "laser_y": LaunchConfiguration("laser_y"),
            "laser_z": LaunchConfiguration("laser_z"),
            "scan_angle_sign": LaunchConfiguration("scan_angle_sign"),
            "navi_yaw_sign": LaunchConfiguration("navi_yaw_sign"),
            "navi_vx_sign": LaunchConfiguration("navi_vx_sign"),
            "navi_vz_sign": LaunchConfiguration("navi_vz_sign"),
            "navi_yaw_offset_deg": LaunchConfiguration("navi_yaw_offset_deg"),
            "navi_odom_yaw_source": LaunchConfiguration("navi_odom_yaw_source"),
            "navi_max_yaw_rate_deg_s": LaunchConfiguration(
                "navi_max_yaw_rate_deg_s"),
            "odom_publish_mode": LaunchConfiguration("odom_publish_mode"),
            "navi_adaptive_clock_sync": LaunchConfiguration("navi_adaptive_clock_sync"),
            "navi_clock_window_samples": LaunchConfiguration("navi_clock_window_samples"),
            "navi_clock_max_adjustment_ns": LaunchConfiguration("navi_clock_max_adjustment_ns"),
            "navi_motion_watchdog_enabled": LaunchConfiguration(
                "navi_motion_watchdog_enabled"),
            "navi_motion_watchdog_pose_enabled": LaunchConfiguration(
                "navi_motion_watchdog_pose_enabled"),
            "navi_motion_watchdog_warmup_sec": LaunchConfiguration(
                "navi_motion_watchdog_warmup_sec"),
            "navi_motion_watchdog_window_sec": LaunchConfiguration(
                "navi_motion_watchdog_window_sec"),
            "navi_motion_watchdog_translation_m": LaunchConfiguration(
                "navi_motion_watchdog_translation_m"),
            "navi_motion_watchdog_yaw_deg": LaunchConfiguration(
                "navi_motion_watchdog_yaw_deg"),
            "nav_zero_command_cancel_sec": LaunchConfiguration(
                "nav_zero_command_cancel_sec"),
            "require_system_ready_for_motion": LaunchConfiguration(
                "require_system_ready_for_motion"),
            "show_serial_window": LaunchConfiguration("show_serial_window"),
            "require_depth_baseline_for_ps2": LaunchConfiguration(
                "require_depth_baseline_for_ps2"),
            "auto_nav_ps2_handoff": LaunchConfiguration(
                "enable_navigation"),
            "chassis_publish_tf": LaunchConfiguration(
                "chassis_publish_tf"),
            "cartographer_odom_topic": LaunchConfiguration(
                "cartographer_odom_topic"),
            "enable_fixed_scan_filter": LaunchConfiguration("enable_fixed_scan_filter"),
            "fixed_scan_min_raw_points": LaunchConfiguration(
                "fixed_scan_min_raw_points"),
            "fixed_scan_min_valid_points": LaunchConfiguration(
                "fixed_scan_min_valid_points"),
            "filtered_scan_topic": LaunchConfiguration("filtered_scan_topic"),
            "cartographer_config": LaunchConfiguration(
                "cartographer_config"),
            "cartographer_load_state_filename": LaunchConfiguration(
                "cartographer_load_state_filename"),
            "occupancy_grid_topic": PythonExpression([
                "'/cartographer_localization_map' if '",
                LaunchConfiguration("localization_mode"),
                "' == 'true' else '/map'"
            ]),
        }

    stable_2d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stable_2d_launch),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("localization_mode"), "' != 'true'"
        ])),
        launch_arguments=stable_2d_arguments.items(),
    )

    localization_2d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_2d_launch),
        condition=IfCondition(LaunchConfiguration("localization_mode")),
        launch_arguments=stable_2d_arguments.items(),
    )

    localization_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="localization_map_server",
        output="screen",
        condition=IfCondition(LaunchConfiguration("localization_mode")),
        parameters=[{
            "use_sim_time": False,
            "yaml_filename": LaunchConfiguration("localization_map_yaml"),
            "topic_name": "/localization_reference_map",
            "frame_id": "map",
        }],
    )

    mutable_navigation_map = Node(
        package="local_depth_cloud_cpp",
        executable="mutable_navigation_map_node",
        name="mutable_navigation_map",
        output="screen",
        condition=IfCondition(LaunchConfiguration("localization_mode")),
        parameters=[{
            "reference_map_topic": "/localization_reference_map",
            "output_map_topic": "/map",
            "update_topic": "/map_updates",
            "scan_topic": LaunchConfiguration("filtered_scan_topic"),
            "localization_ready_topic": "/localization_ready",
            "slam_correction_hold_topic": "/slam_correction_hold",
            "map_frame": "map",
            "occupied_threshold": 65,
            "mark_confirmations": typed(
                "mutable_map_mark_confirmations", int),
            "clear_confirmations": typed(
                "mutable_map_clear_confirmations", int),
            "max_evidence_rate_hz": typed(
                "mutable_map_evidence_rate", float),
            "update_publish_rate_hz": 2.0,
            "full_publish_period_sec": 30.0,
            "max_ray_range": 12.0,
            "endpoint_clearance_m": 0.12,
            # Let Cartographer publish the transform matching this scan under
            # normal Jetson scheduling load instead of dropping the update.
            "tf_timeout_sec": 0.50,
            "pose_jump_translation_m": 0.35,
            "pose_jump_yaw_deg": 20.0,
            "freeze_after_pose_jump_sec": 2.0,
            "restore_reference_on_pose_jump": True,
        }],
    )

    localization_map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization_map",
        output="screen",
        condition=IfCondition(LaunchConfiguration("localization_mode")),
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "bond_timeout": 0.0,
            "node_names": ["localization_map_server"],
        }],
    )

    cartographer_reloc = Node(
        package="lidar_py",
        executable="cartographer_reloc",
        name="cartographer_reloc",
        output="screen",
        condition=IfCondition(LaunchConfiguration("localization_mode")),
        parameters=[{
            "map_topic": "/localization_reference_map",
            "scan_topic": LaunchConfiguration("filtered_scan_topic"),
            "odom_topic": "/odom",
            "base_frame": "base_link",
            "laser_frame": "laser_frame",
            "configuration_directory": os.path.join(pkg_dir, "config"),
            "configuration_basename": "cartographer_2d_localization.lua",
            # A single scan cannot safely distinguish two similar corridor
            # locations. Reject an ambiguous match instead of starting a
            # trajectory at the wrong copy of the corridor.
            "min_match_score": 0.40,
            "min_score_margin": 0.035,
            "strong_match_score": 0.90,
            "strong_match_min_margin": 0.035,
            "max_scan_points": 180,
            "trajectory_restart_delay_sec": 1.0,
            "max_verify_tf_age_sec": 0.75,
            "min_verify_tf_advance_sec": 0.50,
            "verify_timeout_sec": 8.0,
            "auto_retry_interval_sec": 5.0,
            "max_auto_attempts": 5,
        }],
    )

    localization_bringup = Node(
        package="lidar_py",
        executable="localization_bringup",
        name="localization_bringup",
        output="screen",
        condition=IfCondition(LaunchConfiguration("localization_mode")),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "enable_color": "true",
            "color_width": LaunchConfiguration("color_width"),
            "color_height": LaunchConfiguration("color_height"),
            "color_fps": LaunchConfiguration("color_fps"),
            # Image streams are lossy sensor data. Reliable QoS can back-pressure
            # the Gemini2 driver when RTAB-Map or RViz is slower than the camera.
            "color_qos": "SENSOR_DATA",
            "color_camera_info_qos": "DEFAULT",
            "enable_depth": "true",
            "depth_width": LaunchConfiguration("depth_width"),
            "depth_height": LaunchConfiguration("depth_height"),
            "depth_fps": LaunchConfiguration("depth_fps"),
            # Humble depth_image_proc::RegisterNode creates reliable input
            # subscriptions. Matching reliable camera publishers prevents the
            # silent RTAB-Map registered-depth starvation seen on Jetson.
            "depth_qos": "DEFAULT",
            "depth_camera_info_qos": "DEFAULT",
            "depth_registration": LaunchConfiguration("depth_registration"),
            "align_mode": LaunchConfiguration("align_mode"),
            "align_target_stream": LaunchConfiguration("align_target_stream"),
            "enable_frame_sync": "true",
            "enable_sync_host_time": "true",
            # Global timestamps continuously convert the device clock into the
            # host clock domain. Device-domain timerSyncWithHost() can jump
            # timestamps while streaming and made live clouds pause for 8-12s.
            "time_domain": LaunchConfiguration("camera_time_domain"),
            "time_sync_period": LaunchConfiguration("camera_time_sync_period"),
            "enable_frame_drop_log": LaunchConfiguration(
                "camera_enable_frame_drop_log"),
            "frame_timestamp_csv_file": LaunchConfiguration(
                "camera_frame_timestamp_csv_file"),
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
            "enable_noise_removal_filter": "true",
            "enable_depth_auto_exposure_priority": "false",
            "publish_tf": "true",
        }.items(),
    )

    base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_static_tf_dual_3d",
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

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", robot_model,
            " camera_x:=", LaunchConfiguration("camera_x"),
            " camera_y:=", LaunchConfiguration("camera_y"),
            " camera_z:=", LaunchConfiguration("camera_z"),
            " camera_roll:=", LaunchConfiguration("camera_roll"),
            " camera_pitch:=", LaunchConfiguration("camera_pitch"),
            " camera_yaw:=", LaunchConfiguration("camera_yaw"),
            " lidar_x:=", LaunchConfiguration("laser_x"),
            " lidar_y:=", LaunchConfiguration("laser_y"),
            " lidar_z:=", LaunchConfiguration("laser_z"),
        ]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="agv_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    local_cloud = Node(
        package="local_depth_cloud_cpp",
        executable="depth_image_to_local_cloud_v21_node",
        name="depth_image_to_local_cloud_v21",
        output="screen",
        parameters=[{
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/depth/camera_info",
            "output_topic": LaunchConfiguration("local_cloud_topic"),
            "sensor_output_topic": LaunchConfiguration("local_sensor_cloud_topic"),
            "persistent_sensor_output_topic": LaunchConfiguration(
                "local_persistent_sensor_cloud_topic"),
            "clear_sensor_output_topic": "/local_highres_cloud_v21/clear_sensor",
            "stats_topic": LaunchConfiguration("local_stats_topic"),
            "marker_topic": LaunchConfiguration("local_marker_topic"),
            "output_frame": "base_link",
            "max_rate_hz": typed("local_rate", float),
            "pixel_stride": typed("local_stride", int),
            "depth_unit_scale": 0.001,
            "min_range": typed("local_min_range", float),
            "max_range": typed("local_max_range", float),
            "voxel_size": typed("local_voxel", float),
            "spatial_filter_enabled": typed("local_spatial_filter", bool),
            "spatial_depth_threshold_m": typed("local_spatial_threshold_m", float),
            "spatial_depth_threshold_ratio": typed(
                "local_spatial_threshold_ratio", float),
            "spatial_min_neighbors": typed("local_spatial_min_neighbors", int),
            "temporal_filter_enabled": typed("local_temporal_filter", bool),
            "temporal_alpha": typed("local_temporal_alpha", float),
            "temporal_max_delta_m": typed("local_temporal_max_delta_m", float),
            "voxel_outlier_filter_enabled": typed(
                "local_voxel_outlier_filter", bool),
            "voxel_min_neighbors": typed("local_voxel_min_neighbors", int),
            # Single-frame output drives the hard collision gate. Costmap marks
            # require temporal confirmation plus a floor/vertical geometry
            # guard, then expire after a bounded STVL interval.
            "persistent_mark_confirmation_enabled": True,
            "persistent_mark_confirmation_frames": 3,
            "persistent_mark_max_gap_frames": 1,
            "persistent_mark_neighbor_radius": 1,
            "persistent_geometry_guard_enabled": True,
            "recent_mark_ground_guard_height_m": 0.050,
            "recent_mark_min_vertical_span_m": 0.030,
            "persistent_mark_ground_guard_height_m": 0.080,
            "persistent_mark_min_vertical_span_m": 0.060,
            "mark_geometry_neighbor_radius": 1,
            "transform_timeout": 0.50,
            # Jetson end-to-end capture age is normally 145-185 ms. Stream
            # freshness is independently guarded by a steady-clock watchdog,
            # so this threshold should reject queued frames, not normal ones.
            "max_input_age_ms": 250.0,
            # An invalid/black depth frame is not allowed to clear STVL memory.
            "min_clear_valid_depth_ratio": 0.05,
            "roi_u_min": 0, "roi_u_max": -1,
            "roi_v_min": 0, "roi_v_max": -1,
            "x_min": typed("local_x_min", float),
            "x_max": typed("local_x_max", float),
            "y_min": typed("local_y_min", float),
            "y_max": typed("local_y_max", float),
            "z_min": typed("local_z_min", float),
            "z_max": typed("local_z_max", float),
            "remove_self": True,
            "self_x_min": -0.36, "self_x_max": 0.36,
            "self_y_min": -0.36, "self_y_max": 0.36,
            "self_z_min": -0.10, "self_z_max": 0.90,
            "ground_filter_enabled": typed("local_ground_filter", bool),
            "ground_z_min": typed("local_ground_z_min", float),
            "ground_z_max": typed("local_ground_z_max", float),
            "adaptive_ground_plane": True,
            "ground_plane_candidate_min_z": -0.12,
            "ground_plane_candidate_max_z": 0.10,
            "ground_plane_fit_tolerance": 0.025,
            "ground_plane_seed_tolerance": 0.06,
            "ground_plane_temporal_alpha": 0.18,
            "ground_plane_max_slope_step": 0.02,
            "ground_plane_max_offset_step": 0.015,
            "ground_plane_remove_below": 0.035,
            "ground_plane_remove_above": 0.020,
            "ground_plane_max_slope": 0.06,
            "ground_plane_min_inliers": 120,
            "ground_plane_min_inlier_ratio": 0.30,
            "ground_speckle_max_height": 0.040,
            "ground_speckle_min_neighbors": 4,
            "publish_markers": True,
            "stats_period_sec": 1.0,
            "stats_window_size": 300,
            "process_warn_ms": 50.0,
            "age_warn_ms": 220.0,
            "stall_warn_gap_ms": 150.0,
        }],
    )

    collision_gate = Node(
        package="local_depth_cloud_cpp",
        executable="local_cloud_collision_gate_node",
        name="local_cloud_collision_gate",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_navigation")),
        parameters=[{
            "input_topic": LaunchConfiguration("local_cloud_topic"),
            "scan_topic": LaunchConfiguration("filtered_scan_topic"),
            "stop_topic": "/local_cloud_collision_stop",
            "status_topic": "/local_cloud_collision_status",
            "x_min": 0.20,
            "x_max": 0.62,
            # 0.333 m physical half-width + 0.027 m hard-stop margin.
            # The previous 0.39 m box caught doorway jambs that remained
            # outside the Nav2 padded footprint.
            "half_width": 0.36,
            "z_min": 0.02,
            "z_max": 1.40,
            "min_points": 6,
            "approach_x_min": 0.20,
            "approach_x_max": 1.20,
            # Slow only for points near the actual swept corridor. Doorway
            # walls at +/-0.4...0.5 m must not force turtle speed.
            "approach_half_width": 0.39,
            "approach_min_points": 3,
            "rear_x_min": -0.62,
            "rear_x_max": -0.20,
            "rear_half_width": 0.39,
            "rotation_radius": 0.52,
            # The measured chassis is 0.665 x 0.665 m. Ignore only returns
            # inside its physical body; points outside this box but inside the
            # 0.52 m swept circle still protect the rotating corners.
            "scan_self_filter_half_length": 0.33,
            "scan_self_filter_half_width": 0.33,
            "scan_min_points": 2,
            "laser_x": 0.20,
            "laser_y": 0.0,
            "laser_yaw": 0.0,
            "scan_timeout_sec": 0.35,
            "hold_sec": 0.25,
            "sample_stride": 1,
        }],
    )

    persistent_visual_walls = Node(
        package="local_depth_cloud_cpp",
        executable="persistent_visual_wall_filter_node",
        name="persistent_visual_wall_filter",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("enable_navigation"), "' == 'true' and '",
            LaunchConfiguration("enable_rtabmap"), "' == 'true'",
        ])),
        parameters=[{
            "input_topic": "/rtabmap_3d/octomap_occupied_space",
            "output_topic": "/rtabmap_3d/navigation_walls",
            "column_size": 0.05,
            "neighborhood_cells": 1,
            "min_z": 0.08,
            "max_z": 1.40,
            "min_vertical_span": 0.25,
            "min_column_points": 4,
            "publish_all_column_points": True,
        }],
    )

    rgbd_timestamp_monitor = Node(
        package="local_depth_cloud_cpp",
        executable="rgbd_timestamp_monitor_node",
        name="rgbd_timestamp_monitor_step11",
        output="screen",
        parameters=[{
            "color_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "stats_topic": "/dual_3d/rgbd_timestamp_stats",
            "max_pair_interval_ms": ParameterValue(
                LaunchConfiguration("rgbd_sync_max_interval_ms"), value_type=float),
            "warn_p95_ms": typed("rgbd_sync_warn_p95_ms", float),
            "window_size": 300,
        }],
    )

    depth_registration = ComposableNodeContainer(
        name="rtabmap_depth_registration_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        # Registered depth is an RTAB-Map input, not a display-only resource.
        # It must stay alive while RTAB-Map is enabled even when RViz MapCloud
        # is hidden and on-demand pausing is disabled.
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("enable_rtabmap"), "' == 'true' or '",
            LaunchConfiguration("enable_visual_fusion"), "' == 'true'",
        ])),
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="register_depth_to_true_rgb",
                parameters=[{
                    "queue_size": 10,
                    "fill_upsampling_holes": True,
                    "use_rgb_timestamp": True,
                }],
                remappings=[
                    ("depth/image_rect", "/camera/depth/image_raw"),
                    ("depth/camera_info", "/camera/depth/camera_info"),
                    ("rgb/camera_info", "/camera/color/camera_info"),
                    ("depth_registered/image_rect",
                     "/camera/rtabmap/depth_registered/image_raw"),
                    ("depth_registered/camera_info",
                     "/camera/rtabmap/depth_registered/camera_info"),
                ],
            ),
        ],
    )

    visual_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        namespace="visual_fusion",
        name="rgbd_odometry",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_visual_fusion")),
        parameters=[{
            "frame_id": "base_link",
            # Keep the message pose in the EKF world frame. No visual odom TF
            # is published, so robot_localization remains the only TF owner.
            "odom_frame_id": "odom",
            "publish_tf": False,
            "approx_sync": True,
            "approx_sync_max_interval": typed("rgbd_sync_max_interval", float),
            "topic_queue_size": 10,
            "sync_queue_size": 10,
            "qos": 2,
            "qos_camera_info": 2,
            "wait_for_transform": 0.20,
            "wait_imu_to_init": False,
            # A visual tracking loss must remove this optional measurement,
            # not inject a synthetic zero velocity into the EKF.
            "publish_null_when_lost": False,
            "Odom/Strategy": "0",
            "Odom/ResetCountdown": "0",
            "Odom/GuessMotion": "false",
            "Odom/ImageDecimation": "1",
            "OdomF2M/MaxSize": "1000",
            "Vis/MinInliers": "20",
            "Kp/MaxFeatures": "1000",
        }],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("depth/image", "/camera/rtabmap/depth_registered/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("odom", "/visual_odom"),
        ],
    )

    visual_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="visual_wheel_ekf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_visual_fusion")),
        parameters=[visual_ekf_config, {"use_sim_time": False}],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        namespace="rtabmap_3d",
        name="rtabmap",
        output="screen",
        # A large visual-loop graph optimization can briefly consume every
        # CPU worker. Keep it below the real-time local collision-cloud path.
        prefix="nice -n 8",
        additional_env={
            "OMP_NUM_THREADS": LaunchConfiguration("rtabmap_threads"),
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
        },
        condition=IfCondition(LaunchConfiguration("enable_rtabmap")),
        parameters=[{
            "frame_id": "base_link",
            # Use Cartographer's display frame directly. publish_tf remains
            # false, so RTAB-Map cannot overwrite map->odom.
            "map_frame_id": "map",
            # Empty means use the remapped Odometry topic. That topic contains
            # Cartographer's corrected map->base_link pose, not raw chassis yaw.
            "odom_frame_id": "",
            "publish_tf": False,
            "subscribe_odom": True,
            "odom_sensor_sync": False,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            # Cartographer owns 2D laser registration. Feeding the same scan to
            # a second pose graph produced conflicting 20+ degree constraints.
            "subscribe_scan": False,
            "subscribe_scan_cloud": False,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "approx_sync_max_interval": typed("rgbd_sync_max_interval", float),
            "topic_queue_size": 10,
            "sync_queue_size": 10,
            "wait_for_transform": 0.20,
            "qos": 2,
            "qos_camera_info": 2,
            "qos_scan": 2,
            "qos_odom": 1,
            "latch": True,
            "database_path": string_parameter("database_path"),
            "Rtabmap/DetectionRate": string_parameter("rtabmap_rate"),
            # Keep OctoMap generation active, but do not serialize the much
            # larger full OctoMap topic that RViz and this project never use.
            "publish_octomap_full": False,
            "RGBD/CreateOccupancyGrid": "true",
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "true",
            # The persistent map is intentionally low resolution. Keep the
            # independent 15 Hz local cloud untouched for obstacle avoidance.
            # Keep every RGB frame pixel for feature extraction. The generated
            # persistent cloud remains bounded by Grid/CellSize.
            "Mem/ImagePreDecimation": "1",
            "RGBD/LinearUpdate": "0.05",
            "RGBD/AngularUpdate": "0.05",
            "RGBD/OptimizeFromGraphEnd": "false",
            "RGBD/OptimizeMaxError": "3.0",
            "RGBD/NeighborLinkRefining": "false",
            "RGBD/ProximityBySpace": "true",
            "RGBD/ProximityPathMaxNeighbors": "10",
            # RTAB-Map may optimize its own coloured 3D graph, but publish_tf
            # remains false so Cartographer is still the sole 2D pose authority.
            "Rtabmap/LoopThr": "0.11",
            "Rtabmap/PublishStats": "true",
            "RGBD/LoopClosureReextractFeatures": "true",
            "Mem/RehearsalSimilarity": "0.30",
            "Vis/MinInliers": "20",
            "Kp/MaxFeatures": "1000",
            "Reg/Strategy": "0",
            "Reg/Force3DoF": "true",
            "Icp/PointToPlane": "false",
            "Icp/MaxCorrespondenceDistance": "0.15",
            "Icp/CorrespondenceRatio": "0.20",
            "Grid/FromDepth": "true",
            "Grid/3D": "true",
            "Grid/CellSize": string_parameter("global_3d_voxel"),
            "Grid/RangeMax": string_parameter("global_3d_range_max"),
            "Grid/DepthDecimation": "1",
            # Explicit normal-based floor segmentation keeps calibrated floor
            # ripple out of RTAB-Map's persistent OctoMap. The independent
            # local C++ cloud retains low obstacles for real-time avoidance.
            "Grid/NormalsSegmentation": "true",
            "Grid/NormalK": "20",
            "Grid/MaxGroundAngle": "15",
            "Grid/MaxGroundHeight": "0.05",
            "Grid/ClusterRadius": "0.10",
            "Grid/MinClusterSize": "10",
            "Grid/FlatObstacleDetected": "true",
            "Grid/MaxObstacleHeight": "2.00",
            "Grid/RayTracing": "true",
            # Ignore millimetre-scale pose-graph corrections when deciding
            # whether to rebuild the complete global grid. Real loop closures
            # and visible corrections above 5 cm still rebuild and realign it.
            "GridGlobal/UpdateError": "0.05",
        }],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("depth/image", "/camera/rtabmap/depth_registered/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("odom", LaunchConfiguration("rtabmap_odom_topic")),
        ],
    )

    rtabmap_loop_monitor = Node(
        package="lidar_py",
        executable="rtabmap_loop_monitor",
        name="rtabmap_loop_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rtabmap")),
        parameters=[{
            "info_topic": "/rtabmap_3d/info",
            "event_topic": "/rtabmap_3d/visual_loop_event",
            "detected_topic": "/rtabmap_3d/visual_loop_detected",
            "status_period_sec": 10.0,
        }],
    )

    resource_monitor = Node(
        package="lidar_py",
        executable="system_resource_monitor",
        name="system_resource_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_resource_monitor")),
        parameters=[{
            "sample_interval_sec": 2.0,
            "report_interval_sec": typed(
                "resource_monitor_report_interval", float),
            "csv_file": LaunchConfiguration("resource_usage_csv_file"),
            "project_root": LaunchConfiguration(
                "resource_monitor_project_root"),
            "run_directory": LaunchConfiguration(
                "resource_monitor_run_directory"),
        }],
    )

    slam_correction_guard = Node(
        package="lidar_py",
        executable="slam_correction_guard",
        name="slam_correction_guard",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_navigation")),
        parameters=[{
            "map_frame": "map",
            "odom_frame": "odom",
            "hold_topic": "/slam_correction_hold",
            "status_topic": "/slam_correction/status",
            "sample_rate_hz": 30.0,
            # map->odom normally refines by roughly 0.5-1.2 degrees per scan.
            # The old 0.5 degree threshold held the vehicle for most of every
            # turn. Only a real discontinuity should stop motion here.
            "translation_threshold_m": 0.20,
            "yaw_threshold_deg": 5.0,
            "window_sec": 0.50,
            "window_translation_threshold_m": 0.30,
            "window_yaw_threshold_deg": 6.0,
            "max_sample_gap_sec": 1.0,
            # /map and both costmaps publish at up to 1 Hz. Keep motion held
            # until a fresh global path has had time to replace the old one.
            "hold_sec": 1.00,
            "startup_grace_sec": 2.0,
        }],
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        namespace="rtabmap_3d",
        name="rtabmap_viz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rtabmap_viz")),
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "",
            "subscribe_odom": True,
            "odom_sensor_sync": False,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_scan": False,
            "approx_sync": True,
        }],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("depth/image", "/camera/rtabmap/depth_registered/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("odom", LaunchConfiguration("rtabmap_odom_topic")),
        ],
    )

    rtabmap_demand_manager = Node(
        package="lidar_py",
        executable="rtabmap_demand_manager",
        name="rtabmap_demand_manager",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("rtabmap_on_demand_pause")),
        parameters=[{
            "map_data_topic": "/rtabmap_3d/mapData",
            "octomap_topic": "/rtabmap_3d/octomap_occupied_space",
            "pause_service": "/rtabmap_3d/pause",
            "resume_service": "/rtabmap_3d/resume",
            "idle_delay_sec": 3.0,
            "startup_grace_sec": 8.0,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_dual_resolution_3d",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        condition=IfCondition(LaunchConfiguration("enable_navigation")),
        launch_arguments={
            "use_sim_time": "false",
            "start_cartographer": "false",
            "launch_rviz": "false",
            "nav_autostart": LaunchConfiguration("nav_autostart"),
            "explorer_autostart": "false",
            # This branch uses the filtered 3D STVL directly and does not
            # depend on the legacy image-baseline gate.
            "require_depth_baseline": "false",
            "local_cloud_topic": LaunchConfiguration(
                "local_sensor_cloud_topic"),
            "local_cloud_timeout_sec": "0.50",
            "require_local_cloud_alive": "true",
            "show_serial_window": "false",
            "controller_override_file": controller_override,
            "costmap_override_file": LaunchConfiguration(
                "nav_costmap_override_file"),
            "bt_xml_file": os.path.join(
                project_bt_dir, "navigate_to_pose_all_beifen_humble.xml"),
            "through_bt_xml_file": os.path.join(
                project_bt_dir,
                "navigate_through_poses_all_beifen_humble.xml"),
        }.items(),
    )

    return LaunchDescription(arguments + [
        LogInfo(msg=[
            "[dual3d] Cartographer config: ",
            LaunchConfiguration("cartographer_config"),
        ]),
        stable_2d,
        localization_2d,
        localization_map_server,
        localization_map_lifecycle,
        mutable_navigation_map,
        cartographer_reloc,
        localization_bringup,
        camera,
        base_to_camera,
        robot_state_publisher,
        local_cloud,
        collision_gate,
        persistent_visual_walls,
        rgbd_timestamp_monitor,
        depth_registration,
        visual_odometry,
        visual_ekf,
        rtabmap,
        rtabmap_loop_monitor,
        resource_monitor,
        slam_correction_guard,
        rtabmap_viz,
        rtabmap_demand_manager,
        navigation,
        rviz,
    ])
