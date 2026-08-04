"""Isolated Gemini2 launch used only by the new perception experiments.

The validated STEP1-STEP9 launch file is intentionally left untouched.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    args = [
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("serial_number", default_value=""),
        DeclareLaunchArgument("usb_port", default_value=""),
        DeclareLaunchArgument("device_num", default_value="1"),
        DeclareLaunchArgument("uvc_backend", default_value="libuvc"),
        DeclareLaunchArgument("connection_delay", default_value="100"),

        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="15"),
        DeclareLaunchArgument("color_format", default_value="ANY"),
        DeclareLaunchArgument("color_qos", default_value="SENSOR_DATA"),
        DeclareLaunchArgument("color_camera_info_qos", default_value="SENSOR_DATA"),
        DeclareLaunchArgument("enable_color_auto_exposure", default_value="true"),
        DeclareLaunchArgument("enable_color_auto_white_balance", default_value="true"),

        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="400"),
        DeclareLaunchArgument("depth_fps", default_value="15"),
        DeclareLaunchArgument("depth_format", default_value="ANY"),
        DeclareLaunchArgument("depth_qos", default_value="SENSOR_DATA"),
        DeclareLaunchArgument("depth_camera_info_qos", default_value="SENSOR_DATA"),
        DeclareLaunchArgument("depth_registration", default_value="true"),
        DeclareLaunchArgument("align_mode", default_value="HW"),
        DeclareLaunchArgument("align_target_stream", default_value="COLOR"),
        DeclareLaunchArgument("enable_depth_scale", default_value="true"),
        DeclareLaunchArgument("enable_depth_auto_exposure_priority", default_value="false"),

        DeclareLaunchArgument("enable_point_cloud", default_value="false"),
        DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
        DeclareLaunchArgument("point_cloud_qos", default_value="SENSOR_DATA"),
        DeclareLaunchArgument("ordered_pc", default_value="false"),
        DeclareLaunchArgument("enable_ir", default_value="false"),
        DeclareLaunchArgument("enable_accel", default_value="false"),
        DeclareLaunchArgument("enable_gyro", default_value="false"),
        DeclareLaunchArgument("enable_sync_output_accel_gyro", default_value="false"),

        DeclareLaunchArgument("enable_frame_sync", default_value="true"),
        DeclareLaunchArgument("enable_sync_host_time", default_value="true"),
        DeclareLaunchArgument("time_domain", default_value="device"),
        DeclareLaunchArgument("time_sync_period", default_value="60.0"),
        DeclareLaunchArgument("frame_timestamp_csv_file", default_value=""),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("tf_publish_rate", default_value="0.0"),
        DeclareLaunchArgument("camera__frame_id", default_value="camera_link"),
        DeclareLaunchArgument("camera_color_frame_id", default_value="camera_color_frame"),
        DeclareLaunchArgument("camera_depth_frame_id", default_value="camera_depth_frame"),
        DeclareLaunchArgument("color_optical_frame_id", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("depth_optical_frame_id", default_value="camera_depth_optical_frame"),

        DeclareLaunchArgument("enable_decimation_filter", default_value="false"),
        DeclareLaunchArgument("enable_threshold_filter", default_value="false"),
        DeclareLaunchArgument("enable_noise_removal_filter", default_value="true"),
        DeclareLaunchArgument("enable_spatial_filter", default_value="false"),
        DeclareLaunchArgument("enable_temporal_filter", default_value="false"),
        DeclareLaunchArgument("enable_hole_filling_filter", default_value="false"),
        DeclareLaunchArgument("depth_work_mode", default_value="Unbinned Dense Default"),
        DeclareLaunchArgument("retry_on_usb3_detection_failure", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="none"),
    ]

    params = {
        name: LaunchConfiguration(name)
        for name in [
            "camera_name", "serial_number", "usb_port", "device_num", "uvc_backend",
            "connection_delay", "enable_color", "color_width", "color_height", "color_fps",
            "color_format", "color_qos", "color_camera_info_qos",
            "enable_color_auto_exposure", "enable_color_auto_white_balance",
            "enable_depth", "depth_width", "depth_height", "depth_fps", "depth_format",
            "depth_qos", "depth_camera_info_qos", "depth_registration", "align_mode",
            "align_target_stream", "enable_depth_scale", "enable_depth_auto_exposure_priority",
            "enable_point_cloud", "enable_colored_point_cloud", "point_cloud_qos", "ordered_pc",
            "enable_ir", "enable_accel", "enable_gyro", "enable_sync_output_accel_gyro",
            "enable_frame_sync", "enable_sync_host_time", "time_domain", "time_sync_period",
            "frame_timestamp_csv_file", "publish_tf", "tf_publish_rate", "camera__frame_id",
            "camera_color_frame_id", "camera_depth_frame_id", "color_optical_frame_id",
            "depth_optical_frame_id", "enable_decimation_filter", "enable_threshold_filter",
            "enable_noise_removal_filter", "enable_spatial_filter", "enable_temporal_filter",
            "enable_hole_filling_filter", "depth_work_mode", "retry_on_usb3_detection_failure",
            "log_level",
        ]
    }

    camera_node = ComposableNode(
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        name=LaunchConfiguration("camera_name"),
        namespace="",
        parameters=[params],
    )
    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[camera_node],
        output="screen",
    )
    return LaunchDescription(
        args + [GroupAction([PushRosNamespace(LaunchConfiguration("camera_name")), container])]
    )
