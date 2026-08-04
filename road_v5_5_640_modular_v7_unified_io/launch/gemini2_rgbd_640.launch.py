from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Gemini2 低带宽 RGB-D 启动文件。

    目标：
        1. 只开 RGB + Depth，不开点云、不主动开 IR/IMU，先保证树莓派稳定拿图。
        2. RGB 默认 640x480@30。
        3. Depth 默认 640x400@10，开启 depth_registration，让 depth 对齐到 color 坐标。
        4. 后续我们的 Python / ROS2 节点只订阅：
               /camera/color/image_raw
               /camera/depth/image_raw
               /camera/color/camera_info
               /camera/depth/camera_info

    如果 depth_width/depth_height/depth_fps 这个组合不被设备支持，
    启动时可以命令行覆盖，例如 depth_fps:=30 或 depth_width:=320 depth_height:=240。
    """

    args = [
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("serial_number", default_value=""),
        DeclareLaunchArgument("usb_port", default_value=""),
        DeclareLaunchArgument("device_num", default_value="1"),
        DeclareLaunchArgument("uvc_backend", default_value="libuvc"),
        DeclareLaunchArgument("connection_delay", default_value="100"),

        # -------- RGB 彩色流 --------
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument("color_format", default_value="ANY"),
        DeclareLaunchArgument("color_qos", default_value="default"),
        DeclareLaunchArgument("color_camera_info_qos", default_value="default"),
        DeclareLaunchArgument("enable_color_auto_exposure", default_value="true"),
        DeclareLaunchArgument("enable_color_auto_white_balance", default_value="true"),

        # -------- Depth 深度流 --------
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="400"),
        DeclareLaunchArgument("depth_fps", default_value="10"),
        DeclareLaunchArgument("depth_format", default_value="ANY"),
        DeclareLaunchArgument("depth_qos", default_value="default"),
        DeclareLaunchArgument("depth_camera_info_qos", default_value="default"),
        DeclareLaunchArgument("depth_registration", default_value="true"),
        DeclareLaunchArgument("align_mode", default_value="HW"),
        DeclareLaunchArgument("align_target_stream", default_value="COLOR"),
        DeclareLaunchArgument("enable_depth_scale", default_value="true"),

        # -------- 先关掉重负载内容 --------
        DeclareLaunchArgument("enable_point_cloud", default_value="false"),
        DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
        DeclareLaunchArgument("point_cloud_qos", default_value="default"),
        DeclareLaunchArgument("ordered_pc", default_value="false"),
        DeclareLaunchArgument("enable_ir", default_value="false"),
        DeclareLaunchArgument("enable_accel", default_value="false"),
        DeclareLaunchArgument("enable_gyro", default_value="false"),
        DeclareLaunchArgument("enable_sync_output_accel_gyro", default_value="false"),

        # -------- 同步与 TF --------
        DeclareLaunchArgument("enable_frame_sync", default_value="true"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("tf_publish_rate", default_value="0.0"),
        DeclareLaunchArgument("camera__frame_id", default_value="camera_link"),
        DeclareLaunchArgument("camera_color_frame_id", default_value="camera_color_frame"),
        DeclareLaunchArgument("camera_depth_frame_id", default_value="camera_depth_frame"),
        DeclareLaunchArgument("color_optical_frame_id", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("depth_optical_frame_id", default_value="camera_depth_optical_frame"),

        # -------- 滤波：先保持轻量，后续需要再开 --------
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
        "camera_name": LaunchConfiguration("camera_name"),
        "serial_number": LaunchConfiguration("serial_number"),
        "usb_port": LaunchConfiguration("usb_port"),
        "device_num": LaunchConfiguration("device_num"),
        "uvc_backend": LaunchConfiguration("uvc_backend"),
        "connection_delay": LaunchConfiguration("connection_delay"),

        "enable_color": LaunchConfiguration("enable_color"),
        "color_width": LaunchConfiguration("color_width"),
        "color_height": LaunchConfiguration("color_height"),
        "color_fps": LaunchConfiguration("color_fps"),
        "color_format": LaunchConfiguration("color_format"),
        "color_qos": LaunchConfiguration("color_qos"),
        "color_camera_info_qos": LaunchConfiguration("color_camera_info_qos"),
        "enable_color_auto_exposure": LaunchConfiguration("enable_color_auto_exposure"),
        "enable_color_auto_white_balance": LaunchConfiguration("enable_color_auto_white_balance"),

        "enable_depth": LaunchConfiguration("enable_depth"),
        "depth_width": LaunchConfiguration("depth_width"),
        "depth_height": LaunchConfiguration("depth_height"),
        "depth_fps": LaunchConfiguration("depth_fps"),
        "depth_format": LaunchConfiguration("depth_format"),
        "depth_qos": LaunchConfiguration("depth_qos"),
        "depth_camera_info_qos": LaunchConfiguration("depth_camera_info_qos"),
        "depth_registration": LaunchConfiguration("depth_registration"),
        "align_mode": LaunchConfiguration("align_mode"),
        "align_target_stream": LaunchConfiguration("align_target_stream"),
        "enable_depth_scale": LaunchConfiguration("enable_depth_scale"),

        "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
        "enable_colored_point_cloud": LaunchConfiguration("enable_colored_point_cloud"),
        "point_cloud_qos": LaunchConfiguration("point_cloud_qos"),
        "ordered_pc": LaunchConfiguration("ordered_pc"),
        "enable_ir": LaunchConfiguration("enable_ir"),
        "enable_accel": LaunchConfiguration("enable_accel"),
        "enable_gyro": LaunchConfiguration("enable_gyro"),
        "enable_sync_output_accel_gyro": LaunchConfiguration("enable_sync_output_accel_gyro"),

        "enable_frame_sync": LaunchConfiguration("enable_frame_sync"),
        "publish_tf": LaunchConfiguration("publish_tf"),
        "tf_publish_rate": LaunchConfiguration("tf_publish_rate"),
        "camera__frame_id": LaunchConfiguration("camera__frame_id"),
        "camera_color_frame_id": LaunchConfiguration("camera_color_frame_id"),
        "camera_depth_frame_id": LaunchConfiguration("camera_depth_frame_id"),
        "color_optical_frame_id": LaunchConfiguration("color_optical_frame_id"),
        "depth_optical_frame_id": LaunchConfiguration("depth_optical_frame_id"),

        "enable_decimation_filter": LaunchConfiguration("enable_decimation_filter"),
        "enable_threshold_filter": LaunchConfiguration("enable_threshold_filter"),
        "enable_noise_removal_filter": LaunchConfiguration("enable_noise_removal_filter"),
        "enable_spatial_filter": LaunchConfiguration("enable_spatial_filter"),
        "enable_temporal_filter": LaunchConfiguration("enable_temporal_filter"),
        "enable_hole_filling_filter": LaunchConfiguration("enable_hole_filling_filter"),
        "depth_work_mode": LaunchConfiguration("depth_work_mode"),
        "retry_on_usb3_detection_failure": LaunchConfiguration("retry_on_usb3_detection_failure"),
        "log_level": LaunchConfiguration("log_level"),
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
