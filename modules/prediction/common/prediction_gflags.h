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

DECLARE_double(double_precision);

// prediction trajectory and dynamic model
DECLARE_double(prediction_trajectory_time_length);
DECLARE_double(prediction_trajectory_time_resolution);
DECLARE_double(min_prediction_trajectory_spatial_length);
DECLARE_bool(enable_trajectory_validation_check);
DECLARE_bool(enable_tracking_adaptation);

DECLARE_double(vehicle_max_linear_acc);
DECLARE_double(vehicle_min_linear_acc);
DECLARE_double(vehicle_max_speed);

// Tracking Adaptation
DECLARE_double(max_tracking_time);
DECLARE_double(max_tracking_dist);

// Map
DECLARE_double(lane_search_radius);
DECLARE_double(lane_search_radius_in_junction);
DECLARE_double(junction_search_radius);
DECLARE_double(pedestrian_nearby_lane_search_radius);
DECLARE_int32(road_graph_max_search_horizon);

// Semantic Map
DECLARE_double(base_image_half_range);
DECLARE_bool(img_show_semantic_map);

// Scenario
DECLARE_double(junction_distance_threshold);
DECLARE_bool(enable_all_junction);
DECLARE_int32(caution_obs_max_nums);
DECLARE_double(caution_distance_threshold);
DECLARE_double(caution_search_distance_ahead);
DECLARE_double(caution_search_distance_backward);
DECLARE_double(caution_search_distance_backward_for_merge);
DECLARE_double(caution_search_distance_backward_for_overlap);
DECLARE_double(caution_pedestrian_approach_time);

// Obstacle features
DECLARE_int32(ego_vehicle_id);
DECLARE_double(scan_length);
DECLARE_double(scan_width);
DECLARE_double(back_dist_ignore_ped);
DECLARE_uint64(cruise_historical_frame_length);
DECLARE_bool(enable_kf_tracking);
DECLARE_double(max_angle_diff_to_adjust_velocity);
DECLARE_double(q_var);
DECLARE_double(r_var);
DECLARE_double(p_var);
DECLARE_double(go_approach_rate);
DECLARE_double(cutin_approach_rate);

DECLARE_int32(min_still_obstacle_history_length);
DECLARE_int32(max_still_obstacle_history_length);
DECLARE_double(still_obstacle_speed_threshold);
DECLARE_double(still_pedestrian_speed_threshold);
DECLARE_double(still_unknown_speed_threshold);
DECLARE_double(still_obstacle_position_std);
DECLARE_double(still_pedestrian_position_std);
DECLARE_double(still_unknown_position_std);
DECLARE_double(slow_obstacle_speed_threshold);
DECLARE_double(max_history_time);
DECLARE_double(target_lane_gap);
DECLARE_double(dense_lane_gap);
DECLARE_int32(max_num_current_lane);
DECLARE_int32(max_num_nearby_lane);
DECLARE_double(max_lane_angle_diff);
DECLARE_int32(max_num_current_lane_in_junction);
DECLARE_int32(max_num_nearby_lane_in_junction);
DECLARE_double(max_lane_angle_diff_in_junction);
DECLARE_double(coeff_mul_sigma);
DECLARE_double(pedestrian_max_speed);
DECLARE_double(pedestrian_max_acc);
DECLARE_double(still_speed);
DECLARE_string(evaluator_vehicle_mlp_file);
DECLARE_string(torch_vehicle_junction_mlp_file);
DECLARE_string(torch_vehicle_junction_map_file);
DECLARE_string(torch_vehicle_semantic_lstm_file);
DECLARE_string(torch_vehicle_semantic_lstm_cpu_file);
DECLARE_string(torch_vehicle_cruise_go_file);
DECLARE_string(torch_vehicle_cruise_cutin_file);
DECLARE_string(torch_vehicle_lane_scanning_file);
DECLARE_string(torch_pedestrian_interaction_position_embedding_file);
DECLARE_string(torch_pedestrian_interaction_social_embedding_file);
DECLARE_string(torch_pedestrian_interaction_single_lstm_file);
DECLARE_string(torch_pedestrian_interaction_prediction_layer_file);
DECLARE_string(torch_lane_aggregating_obstacle_encoding_file);
DECLARE_string(torch_lane_aggregating_lane_encoding_file);
DECLARE_string(torch_lane_aggregating_prediction_layer_file);
DECLARE_string(evaluator_vehicle_rnn_file);
DECLARE_string(evaluator_vehicle_cruise_mlp_file);
DECLARE_int32(max_num_obstacles);
DECLARE_double(valid_position_diff_threshold);
DECLARE_double(valid_position_diff_rate_threshold);
DECLARE_double(split_rate);
DECLARE_double(rnn_min_lane_relatice_s);
DECLARE_bool(adjust_velocity_by_obstacle_heading);
DECLARE_bool(adjust_velocity_by_position_shift);
DECLARE_bool(adjust_vehicle_heading_by_lane);
DECLARE_double(heading_filter_param);
DECLARE_uint64(max_num_lane_point);
DECLARE_double(distance_threshold_to_junction_exit);
DECLARE_double(angle_threshold_to_junction_exit);
DECLARE_uint32(sample_size_for_average_lane_curvature);

// Validation checker
DECLARE_double(centripetal_acc_coeff);

// Junction Scenario
DECLARE_uint32(junction_historical_frame_length);
DECLARE_double(junction_exit_lane_threshold);
DECLARE_double(distance_beyond_junction);
DECLARE_double(defualt_junction_range);
DECLARE_double(distance_to_slow_down_at_stop_sign);

// Evaluator
DECLARE_double(time_to_center_if_not_reach);
DECLARE_double(default_s_if_no_obstacle_in_lane_sequence);
DECLARE_double(default_l_if_no_obstacle_in_lane_sequence);
DECLARE_bool(enable_semantic_map);

// Obstacle trajectory
DECLARE_bool(enable_cruise_regression);
DECLARE_double(lane_sequence_threshold_cruise);
DECLARE_double(lane_sequence_threshold_junction);
DECLARE_double(lane_change_dist);
DECLARE_bool(enable_lane_sequence_acc);
DECLARE_bool(enable_trim_prediction_trajectory);
DECLARE_double(adc_trajectory_search_length);
DECLARE_double(virtual_lane_radius);
DECLARE_double(default_lateral_approach_speed);
DECLARE_double(centripedal_acc_threshold);

// move sequence prediction
DECLARE_double(time_upper_bound_to_lane_center);
DECLARE_double(time_lower_bound_to_lane_center);
DECLARE_double(sample_time_gap);
DECLARE_double(cost_alpha);
DECLARE_double(default_time_to_lat_end_state);
DECLARE_double(turning_curvature_lower_bound);
DECLARE_double(turning_curvature_upper_bound);
DECLARE_double(speed_at_lower_curvature);
DECLARE_double(speed_at_upper_curvature);
DECLARE_double(cost_function_alpha);
DECLARE_double(cost_function_sigma);
DECLARE_bool(use_bell_curve_for_cost_function);

// interaction predictor
DECLARE_double(collision_cost_time_resolution);
DECLARE_double(longitudinal_acceleration_cost_weight);
DECLARE_double(centripedal_acceleration_cost_weight);
DECLARE_double(collision_cost_weight);
DECLARE_double(collision_cost_exp_coefficient);
DECLARE_double(likelihood_exp_coefficient);

// scenario feature extraction
DECLARE_double(lane_distance_threshold);
DECLARE_double(lane_angle_difference_threshold);

// Trajectory evaluation
DECLARE_double(distance_threshold_on_lane);
