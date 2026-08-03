include "cartographer_2d_v9_tightened.lua"

-- Navigation repeatedly observes visually similar wall segments while
-- correcting or recovering. Online pose-graph optimization moved
-- map->base_link by 5.49 degrees in one 40 ms sample on 2026-07-26 even though
-- odom was static. Keep every V9 local-SLAM value unchanged, but defer graph
-- optimization until shutdown so a speculative historical match cannot rotate
-- a live costmap. Mapping-only launchers still use the frozen V9 profile and
-- retain their normal online loop closure.
POSE_GRAPH.optimize_every_n_nodes = 0

-- Constraints are retained for final shutdown optimization, but only close,
-- high-confidence historical matches are admitted.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.max_constraint_distance = 1.5
POSE_GRAPH.constraint_builder.min_score = 0.82
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.87
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.optimization_problem.huber_scale = 1.0

return options
