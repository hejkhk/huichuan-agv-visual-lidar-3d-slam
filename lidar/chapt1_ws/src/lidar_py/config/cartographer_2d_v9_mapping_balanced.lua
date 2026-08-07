include "cartographer_2d_v9_tightened.lua"

-- Mapping-only refinement of the proven V9 profile. A slightly wider local
-- search absorbs small skid/IMU prediction errors during turns. The existing
-- translation and rotation penalties remain unchanged, so scan evidence must
-- still justify any correction.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.07
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(2.5)

-- V9 already limits which submaps are considered to 3 m, but Cartographer's
-- inherited inter-submap matcher can still move a selected constraint by up
-- to 7 m / 30 degrees. Repetitive corridor walls produced accepted 7.13 m /
-- 5.39 degree corrections and a 5.81 degree live pose-graph jump on 2026-08-07.
-- The chassis supplies guarded absolute yaw, so valid loop refinement should
-- remain close to that initial estimate.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(3.0)

-- Keep the established 5 cm global-map resolution explicit and allow the
-- shutdown optimization a few more iterations. Runtime loop acceptance,
-- constraint distance, sampling ratio and submap density stay exactly V9.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
POSE_GRAPH.max_num_final_iterations = 300

return options
