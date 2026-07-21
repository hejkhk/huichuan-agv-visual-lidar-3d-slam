cmake_minimum_required(VERSION 3.8)
project(short_goal_bt)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(short_goal_behind_bt_node SHARED
  src/short_goal_behind_condition.cpp
  src/initial_path_pre_rotate_condition.cpp
  src/reverse_escape_monitor.cpp
  src/spin_safety_condition.cpp)
target_compile_definitions(short_goal_behind_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_compile_features(short_goal_behind_bt_node PUBLIC cxx_std_17)
target_include_directories(short_goal_behind_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(short_goal_behind_bt_node
  behaviortree_cpp
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros)

install(TARGETS short_goal_behind_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_package()
