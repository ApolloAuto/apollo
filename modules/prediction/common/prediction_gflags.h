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

#ifndef MODULES_PREDICTION_COMMON_PREDICTION_GFLAGS_H_
#define MODULES_PREDICTION_COMMON_PREDICTION_GFLAGS_H_

#include "gflags/gflags.h"

// System gflags
DECLARE_string(prediction_module_name);
DECLARE_string(prediction_conf_file);
DECLARE_string(prediction_adapter_config_filename);

DECLARE_double(prediction_duration);
DECLARE_double(prediction_freq);
DECLARE_double(double_precision);
DECLARE_double(min_prediction_length);

// Bag replay timestamp gap
DECLARE_double(replay_timestamp_gap);

// Map
DECLARE_double(search_radius);

// Obstacle features
DECLARE_bool(enable_kf_tracking);
DECLARE_double(max_acc);
DECLARE_double(min_acc);
DECLARE_double(max_speed);
DECLARE_double(q_var);
DECLARE_double(r_var);
DECLARE_double(p_var);
DECLARE_double(go_approach_rate);
DECLARE_double(cutin_approach_rate);
DECLARE_int32(still_obstacle_history_length);
DECLARE_double(still_obstacle_speed_threshold);
DECLARE_double(still_obstacle_position_std);
DECLARE_double(max_history_time);
DECLARE_double(target_lane_gap);
DECLARE_double(max_lane_angle_diff);
DECLARE_bool(enable_pedestrian_acc);
DECLARE_double(coeff_mul_sigma);
DECLARE_double(pedestrian_min_speed);
DECLARE_double(pedestrian_max_speed);
DECLARE_double(pedestrian_max_acc);
DECLARE_double(prediction_pedestrian_total_time);
DECLARE_int32(num_trajectory_still_pedestrian);
DECLARE_double(still_speed);
DECLARE_string(vehicle_model_file);
DECLARE_int32(max_num_obstacles);

// Obstacle trajectory
DECLARE_double(lane_sequence_threshold);
DECLARE_double(lane_change_dist);

// move sequence prediction
DECLARE_double(time_upper_bound_to_lane_center);
DECLARE_double(time_lower_bound_to_lane_center);
DECLARE_double(sample_time_gap);
DECLARE_double(motion_weight_a);
DECLARE_double(motion_weight_b);
DECLARE_double(motion_weight_c);
DECLARE_double(cost_alpha);
DECLARE_double(default_time_to_lane_center);

#endif  // MODULES_PREDICTION_COMMON_PREDICTION_GFLAGS_H_
