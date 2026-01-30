/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include "gflags/gflags.h"

DECLARE_int32(history_max_record_num);
DECLARE_int32(max_frame_history_num);

DECLARE_bool(enable_scenario_side_pass_multiple_parked_obstacles);
DECLARE_bool(enable_force_pull_over_open_space_parking_test);

DECLARE_bool(publish_estop);

DECLARE_string(traffic_rule_config_filename);
DECLARE_int32(planning_loop_rate);
DECLARE_string(rtk_trajectory_filename);
DECLARE_uint64(rtk_trajectory_forward);
DECLARE_double(rtk_trajectory_resolution);
DECLARE_string(planner_config_path);

// parameter for reference line
DECLARE_bool(prioritize_change_lane);
DECLARE_bool(enable_reference_line_stitching);
DECLARE_double(look_forward_extend_distance);
DECLARE_double(reference_line_stitch_overlap_distance);
DECLARE_string(smoother_config_filename);
DECLARE_bool(enable_smooth_reference_line);
DECLARE_bool(enable_reference_line_provider_thread);
DECLARE_double(default_reference_line_width);
DECLARE_double(smoothed_reference_line_max_diff);
DECLARE_double(reference_line_endpoint_extend_length);

// parameters for trajectory planning
DECLARE_bool(enable_trajectory_stitcher);
DECLARE_double(change_lane_min_length);
DECLARE_double(planning_upper_speed_limit);
DECLARE_double(trajectory_time_length);
DECLARE_double(trajectory_time_min_interval);
DECLARE_double(trajectory_time_max_interval);
DECLARE_double(trajectory_time_high_density_period);

// parameters for trajectory sanity check
DECLARE_bool(enable_trajectory_check);
DECLARE_double(speed_lower_bound);
DECLARE_double(speed_upper_bound);
DECLARE_double(longitudinal_acceleration_lower_bound);
DECLARE_double(longitudinal_acceleration_upper_bound);
DECLARE_double(longitudinal_jerk_lower_bound);
DECLARE_double(longitudinal_jerk_upper_bound);
DECLARE_double(lateral_jerk_bound);
DECLARE_double(kappa_bound);

// STBoundary
DECLARE_double(st_max_s);
DECLARE_double(st_max_t);

// Decision Part
DECLARE_double(static_obstacle_nudge_l_buffer);
DECLARE_double(nonstatic_obstacle_nudge_l_buffer);
DECLARE_double(lateral_ignore_buffer);
DECLARE_double(min_stop_distance_obstacle);
DECLARE_double(max_stop_distance_obstacle);
DECLARE_double(follow_min_distance);
DECLARE_double(yield_distance);
DECLARE_double(follow_time_buffer);
DECLARE_double(signal_expire_time_sec);

// Path Deciders
DECLARE_bool(enable_skip_path_tasks);

DECLARE_double(static_obstacle_speed_threshold);
DECLARE_string(destination_obstacle_id);
DECLARE_double(destination_check_distance);
DECLARE_double(passed_destination_threshold);
DECLARE_double(passed_referenceline_end_threshold);

DECLARE_double(virtual_stop_wall_length);
DECLARE_double(virtual_stop_wall_height);

DECLARE_double(prediction_total_time);
DECLARE_bool(align_prediction_time);
DECLARE_int32(trajectory_point_num_for_debug);

DECLARE_double(turn_signal_distance);

// QpSt optimizer
DECLARE_double(slowdown_profile_deceleration);
DECLARE_double(speed_fallback_distance);
/// thread pool
DECLARE_bool(use_multi_thread_to_add_obstacles);

DECLARE_double(numerical_epsilon);
DECLARE_double(default_cruise_speed);

DECLARE_double(trajectory_time_resolution);
DECLARE_double(trajectory_space_resolution);
DECLARE_double(lateral_acceleration_bound);
DECLARE_double(speed_lon_decision_horizon);
DECLARE_double(lon_collision_buffer);
DECLARE_double(lat_collision_buffer);

/// Lattic parameters
DECLARE_uint64(num_velocity_sample);
DECLARE_bool(enable_backup_trajectory);
DECLARE_double(backup_trajectory_cost);
DECLARE_double(min_velocity_sample_gap);
DECLARE_uint64(num_sample_follow_per_timestamp);
DECLARE_bool(lateral_optimization);
DECLARE_double(weight_lateral_offset);
DECLARE_double(weight_lateral_derivative);
DECLARE_double(weight_lateral_second_order_derivative);
DECLARE_double(weight_lateral_third_order_derivative);
DECLARE_double(weight_lateral_obstacle_distance);
DECLARE_double(lateral_third_order_derivative_max);
// Lattice Evaluate Parameters
DECLARE_double(weight_lon_objective);
DECLARE_double(weight_lon_jerk);
DECLARE_double(weight_lon_collision);
DECLARE_double(weight_lat_offset);
DECLARE_double(weight_lat_comfort);
DECLARE_double(weight_centripetal_acceleration);
DECLARE_double(cost_non_priority_reference_line);
DECLARE_double(weight_same_side_offset);
DECLARE_double(weight_opposite_side_offset);
DECLARE_double(weight_dist_travelled);
DECLARE_double(weight_target_speed);
DECLARE_double(lat_offset_bound);
DECLARE_double(lon_collision_yield_buffer);
DECLARE_double(lon_collision_overtake_buffer);
DECLARE_double(lon_collision_cost_std);
DECLARE_double(default_lon_buffer);
DECLARE_double(time_min_density);
DECLARE_double(comfort_acceleration_factor);
DECLARE_double(polynomial_minimal_param);
DECLARE_double(lattice_stop_buffer);
DECLARE_double(max_s_lateral_optimization);
DECLARE_double(default_delta_s_lateral_optimization);
DECLARE_double(bound_buffer);
DECLARE_double(nudge_buffer);

DECLARE_double(fallback_total_time);
DECLARE_double(fallback_time_unit);

DECLARE_double(speed_bump_speed_limit);
DECLARE_double(default_city_road_speed_limit);
DECLARE_double(default_highway_speed_limit);

// navigation mode
DECLARE_bool(enable_planning_pad_msg);

// open space planner
DECLARE_string(planner_open_space_config_filename);
DECLARE_bool(use_dual_variable_warm_start);
DECLARE_bool(enable_smoother_failsafe);
DECLARE_bool(use_s_curve_speed_smooth);
DECLARE_bool(use_iterative_anchoring_smoother);
DECLARE_bool(enable_parallel_trajectory_smoothing);
DECLARE_bool(enable_parallel_hybrid_a);

DECLARE_double(path_speed_osqp_setting_time_limit);
DECLARE_bool(enable_osqp_debug);
DECLARE_bool(export_chart);
DECLARE_bool(enable_record_debug);
DECLARE_bool(enable_print_curve);

DECLARE_double(default_front_clear_distance);

DECLARE_bool(enable_rss_info);

DECLARE_bool(enable_planning_smoother);
DECLARE_double(smoother_stop_distance);

DECLARE_double(side_pass_driving_width_l_buffer);

DECLARE_double(message_latency_threshold);

DECLARE_uint64(trajectory_stitching_preserved_length);

DECLARE_bool(use_st_drivable_boundary);

DECLARE_bool(use_front_axe_center_in_path_planning);

// learning related
DECLARE_bool(planning_offline_learning);
DECLARE_string(planning_data_dir);
DECLARE_string(planning_offline_bags);
DECLARE_int32(learning_data_obstacle_history_time_sec);
DECLARE_int32(learning_data_frame_num_per_file);
DECLARE_string(planning_birdview_img_feature_renderer_config_file);
DECLARE_int32(min_past_history_points_len);

// Parameter for scenario or task process.
DECLARE_double(path_bounds_decider_resolution);
DECLARE_double(path_bounds_horizon);
DECLARE_double(num_extra_tail_bound_point);
DECLARE_bool(enable_pull_over_at_destination);

DECLARE_double(obstacle_lat_buffer);
DECLARE_double(obstacle_lon_start_buffer);
DECLARE_double(obstacle_lon_end_buffer);
DECLARE_double(obstacle_lon_ignore_buffer);
// parameters for trajectory stitching and reinit planning starting point.
DECLARE_double(replan_lateral_distance_threshold);
DECLARE_double(replan_longitudinal_distance_threshold);

DECLARE_double(replan_time_threshold);

DECLARE_double(trajectory_check_collision_time_step);

DECLARE_double(obstacle_pass_check_distance);

DECLARE_bool(speed_optimize_fail_relax_velocity_constraint);

DECLARE_bool(check_collision_freespace_obstacle_vertices);

DECLARE_double(near_stop_speed);

DECLARE_double(near_stop_deceleration);

DECLARE_double(reference_line_max_forward_heading_diff);
DECLARE_double(reference_line_max_backward_heading_diff);

// Nudge decisider
DECLARE_bool(enable_nudge_decider);
DECLARE_double(path_trim_destination_threshold);

// park generic
DECLARE_double(sqp_obstacle_weight);
DECLARE_bool(enable_obstacle_potential_field);

DECLARE_double(open_space_delta_t);

DECLARE_double(open_space_acc_weight);

DECLARE_double(open_space_jerk_weight);

DECLARE_double(open_space_kappa_weight);

DECLARE_double(open_space_reference_s_weight);

DECLARE_double(open_space_reference_v_weight);

DECLARE_double(open_space_max_forward_v);

DECLARE_double(open_space_max_reverse_v);

DECLARE_double(open_space_max_forward_acc);

DECLARE_double(open_space_max_reverse_acc);

DECLARE_double(open_space_max_jerk);

// Nudge decisider
DECLARE_bool(enable_nudge_decider);
DECLARE_double(max_nudge_check_distance_in_lk);
DECLARE_double(max_nudge_check_distance_in_lc);
DECLARE_double(path_trim_destination_threshold);

// Edge follow buffer
DECLARE_double(edge_follow_buffer);
DECLARE_bool(disable_perception_edge_info);
DECLARE_bool(enable_edge_follow_curvature_buffer);
DECLARE_bool(enable_smooth_edge_follow_buffer);
DECLARE_bool(enable_print_edge_follow_log);

DECLARE_double(normal_look_forward_short_distance);
DECLARE_double(normal_look_forward_long_distance);
DECLARE_double(normal_look_backward_distance);
DECLARE_double(edge_follow_look_forward_short_distance);
DECLARE_double(edge_follow_look_forward_long_distance);
DECLARE_double(edge_follow_look_backward_distance);

// lane escape
DECLARE_bool(enable_lane_escape);

// zone cover
DECLARE_bool(use_zigzag_type_path_lane);
DECLARE_bool(change_end_pose);
DECLARE_bool(calculate_next_trajectory);
DECLARE_bool(enable_non_drivablle_roi);

// for path easy solution
DECLARE_double(ego_front_slack_buffer);
DECLARE_double(relax_ego_radius_buffer);
DECLARE_double(relax_path_s_threshold);
DECLARE_bool(enable_corner_constraint);
DECLARE_bool(enable_expand_obs_corner);
DECLARE_double(expand_obs_corner_lon_buffer);
DECLARE_bool(enable_adc_vertex_constraint);
DECLARE_double(obstacle_lon_end_buffer_park);

DECLARE_bool(enable_control_interactive_replan);

DECLARE_int32(close_range_obstacle_nudge_range_remain_farmes);
DECLARE_double(close_range_obstacle_nudge_pedestrian_waiting_time);

DECLARE_double(path_obs_ref_shift_distance);

DECLARE_double(driving_state_nudge_check_l);