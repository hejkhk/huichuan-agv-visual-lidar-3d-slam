import os
from setuptools import setup

package_name = 'lidar_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         [os.path.join('launch', 'cartographer_scan_v2_launch.py'),
          os.path.join('launch', 'cartographer_auto_mapping_jazzy_launch.py'),
          os.path.join('launch', 'gemini2_rgbd_640.launch.py'),
          os.path.join('launch', 'visual_laser_slam.launch.py')]),
        (os.path.join('share', package_name, 'rviz'),
         [os.path.join('rviz', 'nav2_display.rviz'),
          os.path.join('rviz', 'rgbd_camera_test.rviz'),
          os.path.join('rviz', 'visual_laser_slam_debug.rviz'),
          os.path.join('rviz', 'visual_laser_slam_map.rviz'),
          os.path.join('rviz', 'depth_pointcloud_test.rviz'),
          os.path.join('rviz', 'filtered_pointcloud_test.rviz'),
          os.path.join('rviz', 'octomap_odom_debug.rviz'),
          os.path.join('rviz', 'dual_2d_3d_mapping.rviz')]),

        (os.path.join('share', package_name, 'config'),
         [os.path.join('config', 'laser_filter.yaml'),
          os.path.join('config', 'cartographer_2d_v9_tightened.lua'),
          os.path.join('config', 'frontier_auto_mapping_jazzy.yaml'),
          os.path.join('config', 'nav2_auto_mapping_jazzy.yaml'),
          os.path.join('config', 'lattice_forward_turnaround_5cm.json'),
          os.path.join('config', 'ekf_wheel_imu.yaml'),
          os.path.join('config', 'ekf_visual_wheel_imu.yaml'),
          os.path.join('config', 'cartographer_2d_visual_fusion.lua')]),

        (os.path.join('share', package_name, 'behavior_trees'),
         [os.path.join('behavior_trees', 'navigate_to_pose_jazzy.xml'),
          os.path.join('behavior_trees', 'navigate_through_poses_jazzy.xml')]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lzh',
    maintainer_email='lzh@todo.todo',
    description='Integrated ROS2 Jazzy LiDAR, Cartographer, Nav2 and web bridge nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = lidar_py.lidar_node:main',
            'chassis_node = lidar_py.chassis_node:main',
            'safety_fusion_node = lidar_py.safety_fusion_node:main',
            'robot_pose_publisher = lidar_py.robot_pose_publisher:main',
            'web_path_preview_node = lidar_py.web_path_preview_node:main',
            'web_goal_nav_node = lidar_py.web_goal_nav_node:main',
            'frontier_web_bridge = lidar_py.frontier_web_bridge:main',
            'auto_map_saver = lidar_py.auto_map_saver:main',
            'point_cloud_filter_node = lidar_py.point_cloud_filter_node:main',
        ],


    },
)
