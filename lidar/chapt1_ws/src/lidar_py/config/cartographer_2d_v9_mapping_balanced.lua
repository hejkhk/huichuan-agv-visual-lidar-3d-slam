include "cartographer_2d_v9_tightened.lua"

-- Mapping-only refinement of the proven V9 profile. A slightly wider local
-- search absorbs small skid/IMU prediction errors during turns. The existing
-- translation and rotation penalties remain unchanged, so scan evidence must
-- still justify any correction.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.07
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(2.5)

-- Keep the established 5 cm global-map resolution explicit and allow the
-- shutdown optimization a few more iterations. Runtime loop acceptance,
-- constraint distance, sampling ratio and submap density stay exactly V9.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
POSE_GRAPH.max_num_final_iterations = 300

return options
