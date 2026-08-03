"""Isolated STEP10V2.1 launch.

This file does not share profile logic with the validated STEP1-STEP9 launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    pkg_dir = get_package_share_directory("lidar_py")
    camera_launch = os.path.join(pkg_dir, "launch", "gemini2_experimental.launch.py")
    rviz_config = os.path.join(pkg_dir, "rviz", "local_highres_cloud_v21_test.rviz")

    args = [
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("depth_width", default_value="1280"),
        DeclareLaunchArgument("depth_height", default_value="800"),
        DeclareLaunchArgument("depth_fps", default_value="30"),
        DeclareLaunchArgument("camera_x", default_value="0.3"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.4"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("output_topic", default_value="/local_highres_cloud_v21"),
        DeclareLaunchArgument("stats_topic", default_value="/local_highres_cloud_v21/stats"),
        DeclareLaunchArgument("marker_topic", default_value="/local_highres_cloud_v21/crop_markers"),
        DeclareLaunchArgument("max_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("pixel_stride", default_value="2"),
        DeclareLaunchArgument("depth_unit_scale", default_value="0.001"),
        DeclareLaunchArgument("min_range", default_value="0.20"),
        DeclareLaunchArgument("max_range", default_value="4.0"),
        DeclareLaunchArgument("voxel_size", default_value="0.03"),
        DeclareLaunchArgument("transform_timeout", default_value="0.50"),
        DeclareLaunchArgument("max_input_age_ms", default_value="150.0"),
        DeclareLaunchArgument("roi_u_min", default_value="0"),
        DeclareLaunchArgument("roi_u_max", default_value="-1"),
        DeclareLaunchArgument("roi_v_min", default_value="0"),
        DeclareLaunchArgument("roi_v_max", default_value="-1"),
        DeclareLaunchArgument("x_min", default_value="0.15"),
        DeclareLaunchArgument("x_max", default_value="4.0"),
        DeclareLaunchArgument("y_min", default_value="-2.5"),
        DeclareLaunchArgument("y_max", default_value="2.5"),
        DeclareLaunchArgument("z_min", default_value="-0.5"),
        DeclareLaunchArgument("z_max", default_value="2.0"),
        DeclareLaunchArgument("remove_self", default_value="true"),
        DeclareLaunchArgument("self_x_min", default_value="-0.36"),
        DeclareLaunchArgument("self_x_max", default_value="0.36"),
        DeclareLaunchArgument("self_y_min", default_value="-0.36"),
        DeclareLaunchArgument("self_y_max", default_value="0.36"),
        DeclareLaunchArgument("self_z_min", default_value="-0.10"),
        DeclareLaunchArgument("self_z_max", default_value="0.90"),
        DeclareLaunchArgument("ground_filter_enabled", default_value="false"),
        DeclareLaunchArgument("ground_z_min", default_value="-0.06"),
        DeclareLaunchArgument("ground_z_max", default_value="0.08"),
        DeclareLaunchArgument("publish_markers", default_value="true"),
        DeclareLaunchArgument("stats_period_sec", default_value="1.0"),
        DeclareLaunchArgument("stats_window_size", default_value="300"),
        DeclareLaunchArgument("process_warn_ms", default_value="50.0"),
        DeclareLaunchArgument("age_warn_ms", default_value="120.0"),
        DeclareLaunchArgument("stall_warn_gap_ms", default_value="120.0"),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "enable_color": "false",
            "depth_width": LaunchConfiguration("depth_width"),
            "depth_height": LaunchConfiguration("depth_height"),
            "depth_fps": LaunchConfiguration("depth_fps"),
            "depth_registration": "false",
            "enable_frame_sync": "false",
            "enable_sync_host_time": "true",
            "time_domain": "device",
            "time_sync_period": "60.0",
            "frame_timestamp_csv_file": "",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
            "depth_qos": "SENSOR_DATA",
            "depth_camera_info_qos": "SENSOR_DATA",
            "enable_noise_removal_filter": "true",
            "enable_depth_auto_exposure_priority": "false",
            "publish_tf": "true",
        }.items(),
    )

    base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_static_tf_step10v21",
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

    cloud = Node(
        package="local_depth_cloud_cpp",
        executable="depth_image_to_local_cloud_v21_node",
        name="depth_image_to_local_cloud_v21",
        output="screen",
        parameters=[{
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "stats_topic": LaunchConfiguration("stats_topic"),
            "marker_topic": LaunchConfiguration("marker_topic"),
            "output_frame": "base_link",
            "max_rate_hz": typed("max_rate_hz", float),
            "pixel_stride": typed("pixel_stride", int),
            "depth_unit_scale": typed("depth_unit_scale", float),
            "min_range": typed("min_range", float),
            "max_range": typed("max_range", float),
            "voxel_size": typed("voxel_size", float),
            "transform_timeout": typed("transform_timeout", float),
            "max_input_age_ms": typed("max_input_age_ms", float),
            "roi_u_min": typed("roi_u_min", int),
            "roi_u_max": typed("roi_u_max", int),
            "roi_v_min": typed("roi_v_min", int),
            "roi_v_max": typed("roi_v_max", int),
            "x_min": typed("x_min", float),
            "x_max": typed("x_max", float),
            "y_min": typed("y_min", float),
            "y_max": typed("y_max", float),
            "z_min": typed("z_min", float),
            "z_max": typed("z_max", float),
            "remove_self": typed("remove_self", bool),
            "self_x_min": typed("self_x_min", float),
            "self_x_max": typed("self_x_max", float),
            "self_y_min": typed("self_y_min", float),
            "self_y_max": typed("self_y_max", float),
            "self_z_min": typed("self_z_min", float),
            "self_z_max": typed("self_z_max", float),
            "ground_filter_enabled": typed("ground_filter_enabled", bool),
            "ground_z_min": typed("ground_z_min", float),
            "ground_z_max": typed("ground_z_max", float),
            "publish_markers": typed("publish_markers", bool),
            "stats_period_sec": typed("stats_period_sec", float),
            "stats_window_size": typed("stats_window_size", int),
            "process_warn_ms": typed("process_warn_ms", float),
            "age_warn_ms": typed("age_warn_ms", float),
            "stall_warn_gap_ms": typed("stall_warn_gap_ms", float),
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_step10v21",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(args + [camera, base_to_camera, cloud, rviz])
