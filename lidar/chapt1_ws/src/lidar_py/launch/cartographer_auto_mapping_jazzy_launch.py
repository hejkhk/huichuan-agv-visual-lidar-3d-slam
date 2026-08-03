"""Cartographer V13 with distro-aware ROS 2 Nav2 and frontier exploration.

The validated Cartographer launch is included without modifying its mapping
parameters. This file owns navigation, depth safety fusion and web bridges.
The public Humble launcher delegates here while selecting Humble-native files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_dir = get_package_share_directory("lidar_py")
    frontier_dir = get_package_share_directory("frontier_exploration_ros2")
    ros_distro = os.environ.get("ROS_DISTRO", "humble").lower()
    nav_suffix = "humble" if ros_distro == "humble" else "jazzy"

    stable_launch = os.path.join(
        package_dir, "launch", "cartographer_scan_v2_launch.py")
    default_nav_params = os.path.join(
        package_dir, "config", f"nav2_auto_mapping_{nav_suffix}.yaml")
    default_controller_override = os.path.join(
        package_dir, "config",
        "nav2_dual_3d_rpp_humble_override.yaml"
        if nav_suffix == "humble" else "nav2_dual_3d_rpp_override.yaml")
    default_frontier_params = os.path.join(
        package_dir, "config", f"frontier_auto_mapping_{nav_suffix}.yaml")
    default_bt_xml = os.path.join(
        package_dir, "behavior_trees", f"navigate_to_pose_{nav_suffix}.xml")
    default_through_bt_xml = os.path.join(
        package_dir, "behavior_trees",
        f"navigate_through_poses_{nav_suffix}.xml")
    frontier_launch = os.path.join(
        frontier_dir, "launch", "frontier_explorer.launch.py")
    rviz_config = os.path.join(package_dir, "rviz", "nav2_display.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_bool = ParameterValue(use_sim_time, value_type=bool)
    nav_autostart = LaunchConfiguration("nav_autostart")
    explorer_autostart = LaunchConfiguration("explorer_autostart")
    launch_rviz = LaunchConfiguration("launch_rviz")
    nav_params = LaunchConfiguration("nav_params_file")
    controller_override = LaunchConfiguration("controller_override_file")
    costmap_override = LaunchConfiguration("costmap_override_file")
    frontier_params = LaunchConfiguration("frontier_params_file")
    bt_xml = LaunchConfiguration("bt_xml_file")
    through_bt_xml = LaunchConfiguration("through_bt_xml_file")
    require_depth_baseline = LaunchConfiguration("require_depth_baseline")

    common_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    raw_velocity_remappings = common_remappings + [
        ("/cmd_vel", "/cmd_vel_nav_raw")]
    lifecycle_nodes = [
        "controller_server",
        "velocity_smoother",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    stable_cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stable_launch),
        condition=IfCondition(LaunchConfiguration("start_cartographer")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": "false",
            "lidar_serial_port": LaunchConfiguration("lidar_serial_port"),
            "lidar_baudrate": LaunchConfiguration("lidar_baudrate"),
            "chassis_serial_port": LaunchConfiguration("chassis_serial_port"),
            "laser_yaw_deg": LaunchConfiguration("laser_yaw_deg"),
            "scan_angle_sign": LaunchConfiguration("scan_angle_sign"),
            "navi_yaw_sign": "1.0",
            "navi_vx_sign": "1.0",
            "navi_vz_sign": "1.0",
            "navi_yaw_offset_deg": "0.0",
            "navi_odom_yaw_source": "absolute",
            "cartographer_config": LaunchConfiguration(
                "cartographer_config"),
            "cartographer_scan_topic": "/scan_timed_v2",
            "fixed_scan_bins": "360",
            "fixed_scan_min_valid_points": "180",
            "odom_publish_mode": "navi",
            "navi_vx_scale": "1.0",
            "navi_turn_vx_scale": "0.75",
            "navi_turn_wz_threshold_rad_s": "0.25",
            "show_serial_window": LaunchConfiguration("show_serial_window"),
            "enable_fixed_scan_filter": "true",
            "filtered_scan_topic": "/scan_timed_v2_filtered",
        }.items(),
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            nav_params,
            controller_override,
            costmap_override,
            {"use_sim_time": use_sim_time_bool},
        ],
        remappings=raw_velocity_remappings,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[
            nav_params,
            controller_override,
            {"use_sim_time": use_sim_time_bool},
        ],
        remappings=common_remappings + [
            ("/cmd_vel", "/cmd_vel_nav_raw"),
            ("/cmd_vel_smoothed", "/cmd_vel_nav"),
        ],
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            nav_params,
            controller_override,
            costmap_override,
            {"use_sim_time": use_sim_time_bool},
        ],
        remappings=common_remappings,
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            nav_params,
            controller_override,
            {"use_sim_time": use_sim_time_bool},
        ],
        remappings=raw_velocity_remappings,
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            nav_params,
            {
                "use_sim_time": use_sim_time_bool,
                "default_nav_to_pose_bt_xml": ParameterValue(bt_xml, value_type=str),
                "default_nav_through_poses_bt_xml": ParameterValue(
                    through_bt_xml, value_type=str),
            },
        ],
        remappings=common_remappings,
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav_params, {"use_sim_time": use_sim_time_bool}],
        remappings=common_remappings,
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_bool,
            "autostart": ParameterValue(nav_autostart, value_type=bool),
            "node_names": lifecycle_nodes,
        }],
    )

    frontier_explorer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(frontier_launch),
        launch_arguments={
            "params_file": frontier_params,
            "use_sim_time": use_sim_time,
            "log_level": "warn",
            "autostart": explorer_autostart,
            "control_service_enabled": "true",
        }.items(),
    )

    safety_fusion = Node(
        package="lidar_py",
        executable="safety_fusion_node",
        name="safety_fusion_node",
        output="screen",
        parameters=[{
            "nav_cmd_topic": "/cmd_vel_nav",
            "nav_raw_cmd_topic": "/cmd_vel_nav_raw",
            "web_cmd_topic": "/cmd_vel_web",
            "depth_topic": "/depth_obstacle",
            "safe_cmd_topic": "/cmd_vel_safe",
            "wheel_topic": "/wheel_speed_cmd",
            "virtual_scan_topic": "/depth_obstacle_scan",
            "virtual_scan_frame": "base_link",
            "local_cloud_topic": LaunchConfiguration("local_cloud_topic"),
            "collision_stop_topic": "/local_cloud_collision_stop",
            "collision_status_topic": "/local_cloud_collision_status",
            "navigation_sensor_health_topic":
                "/robot/navigation_sensor_healthy",
            "local_cloud_timeout_sec": ParameterValue(
                LaunchConfiguration("local_cloud_timeout_sec"),
                value_type=float),
            "require_local_cloud_alive": ParameterValue(
                LaunchConfiguration("require_local_cloud_alive"),
                value_type=bool),
            "max_v": 0.23,
            "max_w": 0.80,
            "nav_arc_outer_wheel_mps": 0.18,
            "nav_arc_yaw_gain": 1.45,
            "nav_arc_max_w": 0.32,
            "nav_arc_gain_min_v": 0.10,
            "nav_arc_gain_min_radius": 0.70,
            "nav_pure_turn_min_w": 0.08,
            # Keep the measured 1.45 fixed skid-steer correction. Learning is
            # disabled so a short slip event cannot change future curvature.
            "nav_arc_adaptive_enabled": False,
            "nav_arc_gain_min": 1.10,
            "nav_arc_gain_max": 1.80,
            "nav_arc_gain_learning_rate": 0.08,
            "nav_arc_gain_settle_sec": 0.60,
            "nav_arc_gain_update_period_sec": 0.25,
            "odom_topic": "/odom",
            "approach_slow_distance_m": 1.20,
            "approach_stop_distance_m": 0.62,
            "approach_min_scale": 0.30,
            "collision_status_timeout_sec": 0.50,
            "level_release_hold_sec": 0.40,
            "direction_switch_margin": 80,
            "direction_switch_frames": 6,
            "forward_threshold": 0.015,
            "warning_w_min": 0.05,
            "warning_w_max": 0.20,
            "danger_w_min": 0.10,
            "danger_w_max": 0.32,
            "return_w_slew_rate": 1.4,
            "avoid_w_slew_rate": 2.2,
            "distance_config_dir": os.environ.get("VISION_CODE_DIR", ""),
            "danger_distance_mm": 450,
            "warning_distance_mm": 950,
            "critical_distance_mm": 300,
            "virtual_scan_min_range_m": 0.18,
            "virtual_scan_max_range_m": 1.30,
            "virtual_scan_origin_x_m": 0.30,
            "pulse_per_rev": 8388608.0,
            "gear_ratio": 25.0,
            "wheel_radius": 0.0755,
            "wheel_track_w": 0.2825,
            "max_wheel_cnt": 100000000,
            "require_depth_baseline": ParameterValue(
                require_depth_baseline, value_type=bool),
        }],
    )

    web_goal_nav = Node(
        package="lidar_py",
        executable="web_goal_nav_node",
        name="web_goal_nav_node",
        output="screen",
        parameters=[{
            "goal_topic": "/web/nav_goal",
            # RViz's 2D Goal Pose already calls NavigateToPose in Jazzy.
            # Bridging /goal_pose again would submit the same goal twice.
            "bridge_rviz_goal": False,
            "robot_pose_topic": "/robot_pose",
            "nav_action": "/navigate_to_pose",
            "force_yaw_to_goal": True,
            "disable_auto_mapping_on_goal": True,
            "auto_mapping_service": "/auto_mapping/set_enabled",
        }],
    )
    frontier_web_bridge = Node(
        package="lidar_py",
        executable="frontier_web_bridge",
        name="frontier_web_bridge",
        output="screen",
        parameters=[{
            "frontier_control_service": "/control_exploration",
            "web_control_topic": "/robot/web_control",
            "status_topic": "/auto_mapping/status",
            "set_enabled_service": "/auto_mapping/set_enabled",
            "initial_enabled": ParameterValue(explorer_autostart, value_type=bool),
        }],
    )
    path_preview = Node(
        package="lidar_py",
        executable="web_path_preview_node",
        name="web_path_preview_node",
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav_autostart", default_value="true"),
        DeclareLaunchArgument("explorer_autostart", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("require_depth_baseline", default_value="true"),
        DeclareLaunchArgument(
            "local_cloud_topic",
            default_value="/local_highres_cloud_v21/sensor"),
        DeclareLaunchArgument("local_cloud_timeout_sec", default_value="0.50"),
        DeclareLaunchArgument("require_local_cloud_alive", default_value="false"),
        DeclareLaunchArgument("start_cartographer", default_value="true"),
        DeclareLaunchArgument(
            "cartographer_config",
            default_value="cartographer_2d_v9_nav_guarded.lua"),
        DeclareLaunchArgument("nav_params_file", default_value=default_nav_params),
        DeclareLaunchArgument(
            "controller_override_file", default_value=default_controller_override),
        DeclareLaunchArgument(
            "costmap_override_file", default_value=default_nav_params),
        DeclareLaunchArgument("frontier_params_file", default_value=default_frontier_params),
        DeclareLaunchArgument("bt_xml_file", default_value=default_bt_xml),
        DeclareLaunchArgument("through_bt_xml_file", default_value=default_through_bt_xml),
        DeclareLaunchArgument("lidar_serial_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
        DeclareLaunchArgument("chassis_serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("laser_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("scan_angle_sign", default_value="-1.0"),
        DeclareLaunchArgument("show_serial_window", default_value="false"),
        stable_cartographer,
        controller_server,
        velocity_smoother,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        frontier_explorer,
        safety_fusion,
        web_goal_nav,
        frontier_web_bridge,
        path_preview,
        rviz,
    ])
