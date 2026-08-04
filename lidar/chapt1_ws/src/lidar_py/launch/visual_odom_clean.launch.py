"""Clean, isolated RGB-D odometry test.

It uses equal RGB/Depth rates, sensor-data QoS and a single-parent TF tree:
    odom -> base_link -> camera_link
The validated STEP1-STEP9 launch remains unchanged.
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


def string_parameter(name):
    """Keep RTAB-Map core parameters as ROS string parameters.

    Parameters such as Odom/GuessMotion are registered by rtabmap_ros as strings
    even when their textual value is true/false. Passing a ROS bool makes Jazzy
    throw InvalidParameterTypeException before odometry starts.
    """
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    pkg_dir = get_package_share_directory("lidar_py")
    camera_launch = os.path.join(pkg_dir, "launch", "gemini2_experimental.launch.py")
    rviz_config = os.path.join(pkg_dir, "rviz", "visual_odom_clean_test.rviz")

    args = [
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="15"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="400"),
        DeclareLaunchArgument("depth_fps", default_value="15"),
        DeclareLaunchArgument("camera_x", default_value="0.3"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.4"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument("approx_sync_max_interval", default_value="0.020"),
        DeclareLaunchArgument("topic_queue_size", default_value="1"),
        DeclareLaunchArgument("sync_queue_size", default_value="3"),
        DeclareLaunchArgument("wait_for_transform", default_value="0.10"),
        DeclareLaunchArgument("odom_guess_motion", default_value="false"),
        DeclareLaunchArgument("odom_image_decimation", default_value="1"),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "enable_color": "true",
            "color_width": LaunchConfiguration("color_width"),
            "color_height": LaunchConfiguration("color_height"),
            "color_fps": LaunchConfiguration("color_fps"),
            "depth_width": LaunchConfiguration("depth_width"),
            "depth_height": LaunchConfiguration("depth_height"),
            "depth_fps": LaunchConfiguration("depth_fps"),
            "depth_registration": "true",
            "align_mode": "HW",
            "align_target_stream": "COLOR",
            "enable_frame_sync": "true",
            "enable_sync_host_time": "true",
            "time_domain": "device",
            "time_sync_period": "60.0",
            "frame_timestamp_csv_file": "",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
            "color_qos": "SENSOR_DATA",
            "color_camera_info_qos": "SENSOR_DATA",
            "depth_qos": "SENSOR_DATA",
            "depth_camera_info_qos": "SENSOR_DATA",
            "enable_depth_auto_exposure_priority": "false",
            "publish_tf": "true",
        }.items(),
    )

    base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_static_tf_visual_odom_clean",
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

    odom = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry_clean",
        namespace="rtabmap",
        output="screen",
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "publish_tf": True,
            "approx_sync": True,
            "approx_sync_max_interval": typed("approx_sync_max_interval", float),
            "topic_queue_size": typed("topic_queue_size", int),
            "sync_queue_size": typed("sync_queue_size", int),
            "qos": 2,
            "qos_camera_info": 2,
            "wait_for_transform": typed("wait_for_transform", float),
            "Odom/ImageBufferSize": "1",
            "wait_imu_to_init": False,
            "publish_null_when_lost": True,
            "Odom/Strategy": "0",
            "Odom/ResetCountdown": "5",
            "Odom/GuessMotion": string_parameter("odom_guess_motion"),
            "Odom/ImageDecimation": string_parameter("odom_image_decimation"),
            "Vis/MinInliers": "15",
        }],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("depth/image", "/camera/depth/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("odom", "/visual_odom_clean"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_visual_odom_clean",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(args + [camera, base_to_camera, odom, rviz])
