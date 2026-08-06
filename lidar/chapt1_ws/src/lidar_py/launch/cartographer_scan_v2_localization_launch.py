"""Tested Cartographer V13 sensor and mapping launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("lidar_py")
    config_dir = os.path.join(pkg_dir, "config")
    rviz_config_file = os.path.join(pkg_dir, "rviz", "nav2_display.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    chassis_serial_port = LaunchConfiguration("chassis_serial_port")
    laser_yaw_deg = LaunchConfiguration("laser_yaw_deg")
    scan_angle_sign = LaunchConfiguration("scan_angle_sign")
    navi_yaw_sign = LaunchConfiguration("navi_yaw_sign")
    navi_vx_sign = LaunchConfiguration("navi_vx_sign")
    navi_vz_sign = LaunchConfiguration("navi_vz_sign")
    navi_yaw_offset_deg = LaunchConfiguration("navi_yaw_offset_deg")
    navi_odom_yaw_source = LaunchConfiguration("navi_odom_yaw_source")
    navi_max_yaw_rate_deg_s = LaunchConfiguration(
        "navi_max_yaw_rate_deg_s")
    cartographer_config = LaunchConfiguration("cartographer_config")
    cartographer_load_state_filename = LaunchConfiguration(
        "cartographer_load_state_filename")
    occupancy_grid_topic = LaunchConfiguration("occupancy_grid_topic")
    cartographer_scan_topic = LaunchConfiguration("cartographer_scan_topic")
    fixed_scan_bins = LaunchConfiguration("fixed_scan_bins")
    fixed_scan_min_valid_points = LaunchConfiguration(
        "fixed_scan_min_valid_points")
    lidar_clock_max_adjustment_ns = LaunchConfiguration(
        "lidar_clock_max_adjustment_ns")
    odom_publish_mode = LaunchConfiguration("odom_publish_mode")
    navi_vx_scale = LaunchConfiguration("navi_vx_scale")
    navi_turn_vx_scale = LaunchConfiguration("navi_turn_vx_scale")
    navi_turn_wz_threshold_rad_s = LaunchConfiguration(
        "navi_turn_wz_threshold_rad_s")
    navi_adaptive_clock_sync = LaunchConfiguration(
        "navi_adaptive_clock_sync")
    navi_clock_window_samples = LaunchConfiguration(
        "navi_clock_window_samples")
    navi_clock_max_adjustment_ns = LaunchConfiguration(
        "navi_clock_max_adjustment_ns")
    navi_motion_watchdog_enabled = LaunchConfiguration(
        "navi_motion_watchdog_enabled")
    navi_motion_watchdog_pose_enabled = LaunchConfiguration(
        "navi_motion_watchdog_pose_enabled")
    navi_motion_watchdog_warmup_sec = LaunchConfiguration(
        "navi_motion_watchdog_warmup_sec")
    navi_motion_watchdog_window_sec = LaunchConfiguration(
        "navi_motion_watchdog_window_sec")
    navi_motion_watchdog_translation_m = LaunchConfiguration(
        "navi_motion_watchdog_translation_m")
    navi_motion_watchdog_yaw_deg = LaunchConfiguration(
        "navi_motion_watchdog_yaw_deg")
    nav_zero_command_cancel_sec = LaunchConfiguration(
        "nav_zero_command_cancel_sec")
    require_system_ready_for_motion = LaunchConfiguration(
        "require_system_ready_for_motion")
    show_serial_window = LaunchConfiguration("show_serial_window")
    require_depth_baseline_for_ps2 = LaunchConfiguration(
        "require_depth_baseline_for_ps2")
    auto_nav_ps2_handoff = LaunchConfiguration("auto_nav_ps2_handoff")
    chassis_publish_tf = LaunchConfiguration("chassis_publish_tf")
    cartographer_odom_topic = LaunchConfiguration("cartographer_odom_topic")
    enable_fixed_scan_filter = LaunchConfiguration(
        "enable_fixed_scan_filter")
    filtered_scan_topic = LaunchConfiguration("filtered_scan_topic")
    cartographer_input_topic = PythonExpression([
        "'", filtered_scan_topic, "' if '", enable_fixed_scan_filter,
        "' == 'true' else '", cartographer_scan_topic, "'"
    ])

    lidar_node = Node(
        package="lidar_py",
        executable="lidar_node",
        name="lidar_node",
        output="screen",
        parameters=[{
            "serial_port": lidar_serial_port,
            "baudrate": ParameterValue(lidar_baudrate, value_type=int),
            "frame_id": "laser_frame",
            "laser_x": ParameterValue(laser_x, value_type=float),
            "laser_y": ParameterValue(laser_y, value_type=float),
            "laser_z": ParameterValue(laser_z, value_type=float),
            "scan_interval": 0.1,
            "laser_yaw_deg": ParameterValue(laser_yaw_deg, value_type=float),
            "scan_angle_sign": ParameterValue(scan_angle_sign, value_type=float),
            "publish_timed_scan": True,
            "timed_scan_topic": "/scan_timed",
            "publish_fixed_timed_scan": True,
            "fixed_timed_scan_topic": cartographer_scan_topic,
            "fixed_scan_bins": ParameterValue(fixed_scan_bins, value_type=int),
            "fixed_scan_min_raw_points": 300,
            "fixed_scan_max_raw_points": 720,
            "fixed_scan_min_valid_points": ParameterValue(
                fixed_scan_min_valid_points, value_type=int),
            "fixed_scan_min_time_sec": 0.10,
            "fixed_scan_max_time_sec": 0.35,
            "clock_max_adjustment_ns": ParameterValue(
                lidar_clock_max_adjustment_ns, value_type=int),
            "serial_stall_timeout_sec": 1.0,
        }],
    )

    chassis_node = Node(
        package="lidar_py",
        executable="chassis_node",
        name="chassis_node",
        output="screen",
        parameters=[{
            "serial_port": chassis_serial_port,
            "baudrate": 115200,
            "pulse_per_rev": 8388608.0,
            "gear_ratio": 25.0,
            "wheel_radius": 0.0755,
            "wheel_base_h": 0.2145,
            "wheel_track_w": 0.2825,
            "odom_frame": "odom",
            "base_frame": "base_link",
            "odom_topic": "/odom",
            "publish_tf": ParameterValue(chassis_publish_tf, value_type=bool),
            "use_imu_rp": False,
            "publish_cartographer_planar_imu": True,
            "publish_rate": 50.0,
            "odom_publish_mode": odom_publish_mode,
            "cmd_vel_topic": "/cmd_vel_safe",
            "navi_yaw_sign": ParameterValue(navi_yaw_sign, value_type=float),
            "navi_vx_sign": ParameterValue(navi_vx_sign, value_type=float),
            "navi_vz_sign": ParameterValue(navi_vz_sign, value_type=float),
            "navi_yaw_offset_deg": ParameterValue(
                navi_yaw_offset_deg, value_type=float),
            "navi_odom_yaw_source": navi_odom_yaw_source,
            "navi_max_yaw_rate_deg_s": ParameterValue(
                navi_max_yaw_rate_deg_s, value_type=float),
            "navi_vx_scale": ParameterValue(
                navi_vx_scale, value_type=float),
            "navi_vx_deadband_mps": 0.003,
            "navi_turn_vx_scale": ParameterValue(
                navi_turn_vx_scale, value_type=float),
            "navi_turn_wz_threshold_rad_s": ParameterValue(
                navi_turn_wz_threshold_rad_s, value_type=float),
            "navi_adaptive_clock_sync": ParameterValue(
                navi_adaptive_clock_sync, value_type=bool),
            "navi_clock_window_samples": ParameterValue(
                navi_clock_window_samples, value_type=int),
            "navi_clock_max_adjustment_ns": ParameterValue(
                navi_clock_max_adjustment_ns, value_type=int),
            "navi_motion_watchdog_enabled": ParameterValue(
                navi_motion_watchdog_enabled, value_type=bool),
            "navi_motion_watchdog_pose_enabled": ParameterValue(
                navi_motion_watchdog_pose_enabled, value_type=bool),
            "navi_motion_watchdog_warmup_sec": ParameterValue(
                navi_motion_watchdog_warmup_sec, value_type=float),
            "navi_motion_watchdog_window_sec": ParameterValue(
                navi_motion_watchdog_window_sec, value_type=float),
            "navi_motion_watchdog_translation_m": ParameterValue(
                navi_motion_watchdog_translation_m, value_type=float),
            "navi_motion_watchdog_yaw_deg": ParameterValue(
                navi_motion_watchdog_yaw_deg, value_type=float),
            "nav_zero_command_cancel_sec": ParameterValue(
                nav_zero_command_cancel_sec, value_type=float),
            "require_system_ready_for_motion": ParameterValue(
                require_system_ready_for_motion, value_type=bool),
            "navi_vz_deadband_deg_s": 0.15,
            "show_serial_window": ParameterValue(
                show_serial_window, value_type=bool),
            "require_depth_baseline_for_ps2": ParameterValue(
                require_depth_baseline_for_ps2, value_type=bool),
            "auto_nav_ps2_handoff": ParameterValue(
                auto_nav_ps2_handoff, value_type=bool),
            "nav_ps2_release_delay_sec": 0.75,
        }],
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        condition=IfCondition(enable_fixed_scan_filter),
        parameters=[os.path.join(config_dir, "laser_filter.yaml")],
        remappings=[
            ("scan", cartographer_scan_topic),
            ("scan_filtered", filtered_scan_topic),
        ],
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", config_dir,
            "-configuration_basename", cartographer_config,
        ],
        condition=IfCondition(PythonExpression([
            "'", cartographer_load_state_filename, "' == ''"
        ])),
        remappings=[
            ("scan", cartographer_input_topic),
            ("odom", cartographer_odom_topic),
            ("imu", "/imu_cartographer"),
        ],
    )

    cartographer_localization_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", config_dir,
            "-configuration_basename", cartographer_config,
            "-load_state_filename", cartographer_load_state_filename,
        ],
        condition=IfCondition(PythonExpression([
            "'", cartographer_load_state_filename, "' != ''"
        ])),
        remappings=[
            ("scan", cartographer_input_topic),
            ("odom", cartographer_odom_topic),
            ("imu", "/imu_cartographer"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution", "0.05",
            "-publish_period_sec", "1.0",
        ],
        remappings=[("map", occupancy_grid_topic)],
    )

    robot_pose_publisher = Node(
        package="lidar_py",
        executable="robot_pose_publisher",
        name="robot_pose_publisher",
        output="screen",
        parameters=[{
            "map_frame": "map",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "orientation_source": "map",
            # Also feeds /cartographer_pose_odom to the persistent 3D mapper.
            "publish_rate": 30.0,
            "topic": "/robot_pose",
        }],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "lidar_serial_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
        DeclareLaunchArgument("laser_x", default_value="0.20"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.0"),
        DeclareLaunchArgument(
            "chassis_serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("laser_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("scan_angle_sign", default_value="-1.0"),
        DeclareLaunchArgument("navi_yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("navi_vx_sign", default_value="1.0"),
        DeclareLaunchArgument("navi_vz_sign", default_value="1.0"),
        DeclareLaunchArgument("navi_yaw_offset_deg", default_value="0.0"),
        DeclareLaunchArgument(
            "navi_odom_yaw_source", default_value="absolute"),
        DeclareLaunchArgument(
            "navi_max_yaw_rate_deg_s", default_value="120.0"),
        DeclareLaunchArgument(
            "cartographer_config",
            default_value="cartographer_2d_v9_tightened.lua",
            description="Tested Cartographer V13 configuration",
        ),
        DeclareLaunchArgument(
            "cartographer_load_state_filename",
            default_value="",
            description="Optional frozen pbstream used for localization",
        ),
        DeclareLaunchArgument(
            "occupancy_grid_topic", default_value="/map"),
        DeclareLaunchArgument(
            "cartographer_scan_topic", default_value="/scan_timed_v2"),
        DeclareLaunchArgument(
            "fixed_scan_bins",
            default_value="360",
            description="Constant angular bins for the measured-angle scan",
        ),
        DeclareLaunchArgument(
            "fixed_scan_min_valid_points", default_value="0",
            description="Drop sparse fixed scans; zero keeps legacy behavior",
        ),
        DeclareLaunchArgument(
            "lidar_clock_max_adjustment_ns", default_value="100000",
            description="Maximum LiDAR clock offset correction per packet",
        ),
        DeclareLaunchArgument("navi_vx_scale", default_value="1.0"),
        DeclareLaunchArgument("navi_turn_vx_scale", default_value="0.75"),
        DeclareLaunchArgument(
            "navi_turn_wz_threshold_rad_s", default_value="0.25"),
        DeclareLaunchArgument("navi_adaptive_clock_sync", default_value="false"),
        DeclareLaunchArgument("navi_clock_window_samples", default_value="250"),
        DeclareLaunchArgument("navi_clock_max_adjustment_ns", default_value="20000"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_enabled", default_value="true"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_pose_enabled",
            default_value="false",
            description=(
                "Diagnostic map-pose cross-check. Keep false for navigation: "
                "Cartographer loop corrections are not physical chassis motion."
            ),
        ),
        DeclareLaunchArgument(
            "navi_motion_watchdog_warmup_sec", default_value="6.0"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_window_sec", default_value="0.75"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_translation_m", default_value="0.08"),
        DeclareLaunchArgument(
            "navi_motion_watchdog_yaw_deg", default_value="3.0"),
        DeclareLaunchArgument(
            "nav_zero_command_cancel_sec",
            default_value="25.0",
            description=(
                "Cancel a Nav2 goal that has produced no effective safe "
                "velocity for this long, then restore PS2."
            ),
        ),
        DeclareLaunchArgument(
            "require_system_ready_for_motion",
            default_value="false",
            description="Hold MOVE zero until /robot/system_ready is true",
        ),
        DeclareLaunchArgument(
            "odom_publish_mode", default_value="navi",
            description="timer keeps legacy behavior; navi publishes each 0x07 sample",
        ),
        DeclareLaunchArgument("show_serial_window", default_value="true"),
        DeclareLaunchArgument(
            "require_depth_baseline_for_ps2",
            default_value="true",
            description="Require legacy depth baseline before PS2 release",
        ),
        DeclareLaunchArgument(
            "auto_nav_ps2_handoff",
            default_value="false",
            description="Automatically switch STM32 between Nav2 MOVE and idle PS2",
        ),
        DeclareLaunchArgument(
            "chassis_publish_tf", default_value="true",
            description="Disable when robot_localization owns odom->base_link",
        ),
        DeclareLaunchArgument(
            "cartographer_odom_topic", default_value="/odom",
            description="Raw or EKF-filtered odometry input for Cartographer",
        ),
        DeclareLaunchArgument(
            "enable_fixed_scan_filter", default_value="false"),
        DeclareLaunchArgument(
            "filtered_scan_topic", default_value="/scan_timed_v2_filtered"),
        lidar_node,
        chassis_node,
        laser_filter_node,
        cartographer_node,
        cartographer_localization_node,
        occupancy_grid_node,
        robot_pose_publisher,
        rviz2,
    ])
