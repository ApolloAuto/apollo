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

#ifndef MODULES_PLANNING_COMMON_PLANNING_GFLAGS_H_
#define MODULES_PLANNING_COMMON_PLANNING_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(planning_config_file);
DECLARE_int32(planning_loop_rate);
DECLARE_string(rtk_trajectory_filename);
DECLARE_uint64(backward_trajectory_point_num);
DECLARE_uint64(rtk_trajectory_forward);
DECLARE_double(replanning_threshold);
DECLARE_double(trajectory_resolution);
DECLARE_double(look_backward_distance);
DECLARE_double(look_forward_distance);

DECLARE_double(cycle_duration_in_sec);
DECLARE_double(maximal_delay_sec);
DECLARE_int32(max_history_frame_num);

// parameters for trajectory stitching and reinit planning starting point.
DECLARE_bool(use_stitch);
DECLARE_double(forward_predict_time);
DECLARE_double(replan_distance_threshold);
DECLARE_double(replan_s_threshold);
DECLARE_double(replan_l_threshold);

// parameter for reference line
DECLARE_double(default_reference_line_width);
DECLARE_double(planning_speed_upper_limit);

// parameters for task manager
DECLARE_string(reference_line_decider);

// parameters for trajectory planning
DECLARE_double(trajectory_time_length);
DECLARE_double(trajectory_time_resolution);
DECLARE_double(output_trajectory_time_resolution);

// parameters for trajectory sanity check
DECLARE_double(speed_lower_bound);
DECLARE_double(speed_upper_bound);

DECLARE_double(longitudinal_acceleration_lower_bound);
DECLARE_double(longitudinal_acceleration_upper_bound);

DECLARE_double(lateral_acceleration_bound);
DECLARE_double(lateral_jerk_bound);

DECLARE_double(longitudinal_jerk_lower_bound);
DECLARE_double(longitudinal_jerk_upper_bound);

DECLARE_double(kappa_bound);

// StGraph
DECLARE_double(stgraph_default_point_cost);
DECLARE_double(stgraph_max_acceleration_divide_factor_level_1);
DECLARE_double(stgraph_max_deceleration_divide_factor_level_1);
DECLARE_double(stgraph_max_deceleration_divide_factor_level_2);

// Decision Part
DECLARE_double(static_decision_ignore_s_range);
DECLARE_double(static_decision_nudge_l_buffer);
DECLARE_double(static_decision_stop_buffer);
DECLARE_double(dynamic_decision_follow_range);
DECLARE_double(stop_distance_pedestrian);
DECLARE_double(stop_distance_obstacle);
DECLARE_double(destination_adjust_distance_buffer);
DECLARE_double(nudge_distance_vehicle);
DECLARE_double(nudge_distance_bicycle);
DECLARE_double(nudge_distance_obstacle);
DECLARE_double(follow_min_distance);
DECLARE_double(st_follow_max_start_t);
DECLARE_double(st_follow_min_end_t);
DECLARE_double(stop_line_max_distance);
DECLARE_double(stop_line_min_distance);

DECLARE_string(destination_obstacle_id);
DECLARE_int32(virtual_obstacle_perception_id);
DECLARE_double(virtual_stop_wall_length);
DECLARE_double(virtual_stop_wall_width);
DECLARE_double(virtual_stop_wall_height);

DECLARE_int32(adc_id);
DECLARE_bool(local_debug);
DECLARE_string(new_pobs_dump_file);
DECLARE_double(pcd_merge_thred);
DECLARE_double(vehicle_min_l);
DECLARE_double(vehicle_min_w);
DECLARE_double(vehicle_min_h);
DECLARE_int32(good_pcd_size);
DECLARE_int32(good_pcd_labeling_size);
DECLARE_double(front_sector_angle);
DECLARE_double(max_reasonable_l);
DECLARE_double(min_reasonable_l);
DECLARE_double(max_reasonable_w);
DECLARE_double(min_reasonable_w);
DECLARE_double(merge_range);
DECLARE_double(pcd_labeling_range);
DECLARE_bool(enable_merge_obstacle);
DECLARE_bool(enable_new_pobs_dump);
DECLARE_bool(enable_pcd_labeling);
DECLARE_string(pcd_image_file_prefix);

// The following gflags are for modeling
DECLARE_double(prediction_total_time);
DECLARE_double(prediction_pedestrian_total_time);
DECLARE_double(prediction_model_time);
DECLARE_double(prediction_freq);

// The following gflags are for internal use
DECLARE_bool(enable_unknown_obstacle);
DECLARE_bool(enable_output_to_screen);
DECLARE_bool(enable_EKF_tracking);
DECLARE_bool(enable_acc);
DECLARE_bool(enable_pedestrian_acc);

// Cyclist UKF
DECLARE_double(road_heading_impact);
DECLARE_double(process_noise_diag);
DECLARE_double(measurement_noise_diag);
DECLARE_double(state_covariance_diag_init);
DECLARE_double(ukf_alpha);
DECLARE_double(ukf_beta);
DECLARE_double(ukf_kappa);

DECLARE_int32(stored_frames);
DECLARE_double(min_acc);
DECLARE_double(max_acc);
DECLARE_double(min_pedestrian_acc);
DECLARE_double(max_pedestrian_acc);
DECLARE_double(default_heading);
DECLARE_int32(max_obstacle_size);
DECLARE_double(max_prediction_length);
DECLARE_double(lane_search_radius);
DECLARE_double(junction_search_radius);
DECLARE_double(min_lane_change_distance);
DECLARE_double(proto_double_precision);
DECLARE_double(nearby_obstacle_range);
DECLARE_int32(nearby_obstacle_num);

// Kalman Filter
DECLARE_double(beta);
DECLARE_double(cut_in_beta);
DECLARE_double(q_var);  // for simplicity, Q11=Q22=q_var, Q12=Q21=0
DECLARE_double(r_var);
DECLARE_double(p_var);
DECLARE_double(kf_endl);

// Trajectory
DECLARE_double(car_length);
DECLARE_int32(trajectory_num_frame);
DECLARE_double(trajectory_stretch);
DECLARE_double(coeff_mul_sigma);
DECLARE_bool(enable_velocity_from_history);
DECLARE_int32(num_trajectory_still_pedestrian);

// Offline model
DECLARE_bool(running_middle_mode);
DECLARE_string(feature_middle_file);
DECLARE_string(hdf5_file);
DECLARE_bool(enable_labelling);
DECLARE_bool(enable_feature_to_file);
DECLARE_bool(enable_evaluation);
DECLARE_int32(num_evaluation_frame);
DECLARE_string(feature_file_path);
DECLARE_string(feature_file_format);
DECLARE_double(close_time);
DECLARE_double(close_dist);

// Online model
DECLARE_double(target_lane_gap);
DECLARE_double(model_prob_cutoff);
DECLARE_double(prediction_feature_timeframe);
DECLARE_double(threshold_label_time_delta);
DECLARE_double(prediction_label_timeframe);
DECLARE_double(prediction_buffer_timeframe);
DECLARE_int32(pedestrian_static_length);
DECLARE_double(still_speed);
DECLARE_string(default_vehicle_model_file_path);
DECLARE_string(user_vehicle_model_name);
DECLARE_string(user_vehicle_model_file_path);
DECLARE_double(kf_max_speed);
DECLARE_double(kf_min_speed);
DECLARE_double(kf_still_position_diff);
DECLARE_double(max_angle_diff);
DECLARE_double(pedestrian_max_speed);
DECLARE_double(pedestrian_min_speed);
DECLARE_int32(obstacle_static_length);
DECLARE_double(static_speed_threshold);

// Setting
DECLARE_bool(enable_p_speed_override);
DECLARE_double(threshold_timestamp_diff);
DECLARE_double(pc_pob_tolerance);

// Traffic decision
DECLARE_bool(enable_traffic_decision);

DECLARE_double(polygon_length_box_length_max_diff);

// Traffic light decision
DECLARE_double(length_of_passing_stop_line_buffer);
DECLARE_double(master_min_speed);
DECLARE_double(max_deacceleration_for_red_light_stop);
DECLARE_double(max_deacceleration_for_yellow_light_stop);

DECLARE_int32(trajectory_point_num_for_debug);
DECLARE_double(backward_routing_distance);
DECLARE_double(decision_valid_stop_range);

DECLARE_bool(enable_record_debug);
DECLARE_bool(enable_prediction);

#endif  // MODULES_PLANNING_COMMON_PLANNING_GFLAGS_H_
