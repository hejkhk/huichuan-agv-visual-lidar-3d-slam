include "map_builder.lua"
include "trajectory_builder.lua"

-- Visual fusion profile: /odometry/filtered is produced by robot_localization
-- from wheel odometry + IMU z-rate + RTAB-Map RGB-D odometry.

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.06
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(2.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 12.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 135.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 40

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.08
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120

-- V9: TIGHTENED loop closure (from V8-D analysis)
-- V8-D had 9 false matches with scores 56-73%, only true match was 80.5%
POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.constraint_builder.sampling_ratio = 0.8     -- V9: 0.05→0.1→0.2→0.4→0.6 (diminishing), now 0.8 per Codex
POSE_GRAPH.constraint_builder.max_constraint_distance = 3.0    -- V8-D was 15.0 (too far)
POSE_GRAPH.constraint_builder.min_score = 0.75                  -- V8-D was 0.55 (too loose), V6_5 was 0.80 (too strict)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.80  -- V8-D was 0.60
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.optimization_problem.huber_scale = 5.0

return options
