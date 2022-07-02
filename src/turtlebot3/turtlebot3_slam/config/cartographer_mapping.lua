include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.1,
  submap_publish_period_sec = 0.3,
  publish_tracked_pose = true,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- *** Input ***

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.min_z = -0.8 -- default: -0.8
TRAJECTORY_BUILDER_2D.max_z = 2.

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02  -- default: 0.025

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5 -- default: 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200 -- default: 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30. -- default: 50.


-- *** Local SLAM ***

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.  -- default 1.
-- wheel odometry is fine
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20. -- default 10
-- IMU is ok
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20. -- default 40
-- ceres scan matcher 收斂速度配置
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false -- default false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20 -- default 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1 -- default 1

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2  -- default 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.) -- default 1.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05 -- default 0.05 

-- *** Global SLAM ***

POSE_GRAPH.optimize_every_n_nodes = 30 -- default 90

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 -- default 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5 -- backpack_2d 1e5 freight 1e3
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e4 -- backpack_2d 1e5 freight 1e2
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3 -- backpack_2d 1e5 freight 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2 -- backpack_2d 1e5 freight 1e2


return options
