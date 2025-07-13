-- Copyright 2016 The Cartographer Authors
-- Licensed under the Apache License, Version 2.0 (the "License")

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- Frame Configuration
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  
  -- Sensor Configuration
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 0,
  
  -- Timing Parameters (Optimized for TurtleBot3)
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,        -- Reduced from default 0.3
  pose_publish_period_sec = 10e-3,        -- Reduced from default 5e-3
  trajectory_publish_period_sec = 60e-3,   -- Reduced from default 30e-3
  
  -- Sampling Ratios
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  
  -- New Performance Parameters
  num_accumulated_range_data = 1,
  max_laser_distance = 3.5,
  voxel_filter_size = 0.025,
}

-- Map Builder Configuration
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2     -- Reduced from default 4

-- 2D Trajectory Builder
TRAJECTORY_BUILDER_2D = {
  min_range = 0.12,
  max_range = 3.5,
  missing_data_ray_length = 3.0,
  
  -- Sensor Configurations
  use_imu_data = false,
  use_online_correlative_scan_matching = false,  -- Disabled for performance
  
  -- Adaptive Voxel Filter
  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    max_range = 3.5,
  },
  
  -- Loop Closure Config
  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 3.5,
  },
  
  -- Motion Filter
  motion_filter = {
    max_time_seconds = 5.0,
    max_distance_meters = 0.05,
    max_angle_radians = math.rad(0.2),
  },
  
  -- Ceres Scan Matcher
  ceres_scan_matcher = {
    occupied_space_weight = 1.0,
    translation_weight = 10.0,
    rotation_weight = 40.0,
    ceres_solver_options = {
      use_nonmonotonic_steps = true,
      max_num_iterations = 10,            -- Reduced from default 20
      num_threads = 1,
    },
  },
  
  -- Occupancy Grid
  submaps = {
    num_range_data = 60,                  -- Reduced from default 90
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,
    },
    range_data_inserter = {
      hit_probability = 0.55,
      miss_probability = 0.49,
      insert_free_space = true,
    },
  },
}

-- Pose Graph Configuration
POSE_GRAPH = {
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1e3,
    rotation_weight = 3e5,
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 1e5,
    odometry_rotation_weight = 1e5,
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = false,
    fix_z_in_3d = false,
  },
  
  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 5.0,
    min_score = 0.65,
    global_localization_min_score = 0.7,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = false,
    fast_correlative_scan_matcher = {
      linear_search_window = 3.0,
      angular_search_window = math.rad(15.),
      branch_and_bound_depth = 5,
    },
  },
  
  optimization_problem = {
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 1,
    },
  },
  
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.1,
  log_residual_histograms = false,
  global_constraint_search_after_n_seconds = 10.0,
}

return options
