cmake_minimum_required(VERSION 3.8)
project(local_depth_cloud_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(depth_image_to_local_cloud_node
  src/depth_image_to_local_cloud_node.cpp
)
add_executable(depth_image_to_local_cloud_v21_node
  src/depth_image_to_local_cloud_v21_node.cpp
)
add_executable(rgbd_timestamp_monitor_node
  src/rgbd_timestamp_monitor_node.cpp
)
add_executable(local_cloud_collision_gate_node
  src/local_cloud_collision_gate_node.cpp
)
add_executable(persistent_visual_wall_filter_node
  src/persistent_visual_wall_filter_node.cpp
)
add_executable(mutable_navigation_map_node
  src/mutable_navigation_map_node.cpp
)

ament_target_dependencies(depth_image_to_local_cloud_node
  rclcpp
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(depth_image_to_local_cloud_v21_node
  rclcpp
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(rgbd_timestamp_monitor_node
  rclcpp
  builtin_interfaces
  sensor_msgs
  std_msgs
)

ament_target_dependencies(local_cloud_collision_gate_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(persistent_visual_wall_filter_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(mutable_navigation_map_node
  rclcpp
  geometry_msgs
  map_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

target_compile_definitions(depth_image_to_local_cloud_node PRIVATE
  $<$<CONFIG:Debug>:LOCAL_DEPTH_CLOUD_DEBUG>
)

install(TARGETS depth_image_to_local_cloud_node depth_image_to_local_cloud_v21_node rgbd_timestamp_monitor_node local_cloud_collision_gate_node persistent_visual_wall_filter_node mutable_navigation_map_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
