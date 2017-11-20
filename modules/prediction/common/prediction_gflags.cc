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

#include <cmath>

#include "modules/prediction/common/prediction_gflags.h"

// System gflags
DEFINE_string(prediction_module_name, "prediction",
              "Default prediciton module name");
DEFINE_string(prediction_conf_file,
              "modules/prediction/conf/prediction_conf.pb.txt",
              "Default conf file for prediction");
DEFINE_string(prediction_adapter_config_filename,
              "modules/prediction/conf/adapter.conf",
              "Default conf file for prediction");

DEFINE_double(prediction_duration, 3.0, "Prediction duration (in seconds)");
DEFINE_double(prediction_freq, 0.1, "Prediction frequency (in seconds");
DEFINE_double(double_precision, 1e-6, "precision of double");
DEFINE_double(min_prediction_length, 50.0,
              "Minimal length of prediction trajectory");

// Bag replay timestamp gap
DEFINE_double(replay_timestamp_gap, 10.0,
              "Max timestamp gap for rosbag replay");

// Map
DEFINE_double(search_radius, 3.0, "Search radius for a candidate lane");

// Obstacle features
DEFINE_bool(enable_kf_tracking, false, "Use measurements with KF tracking");
DEFINE_double(max_acc, 4.0, "Upper bound of acceleration");
DEFINE_double(min_acc, -4.0, "Lower bound of deceleration");
DEFINE_double(max_speed, 35.0, "Max speed");
DEFINE_double(q_var, 0.01, "Processing noise covariance");
DEFINE_double(r_var, 0.25, "Measurement noise covariance");
DEFINE_double(p_var, 0.1, "Error covariance");
DEFINE_double(go_approach_rate, 0.995,
              "The rate to approach to the reference line of going straight");
DEFINE_double(cutin_approach_rate, 0.9,
              "The rate to approach to the reference line of lane change");
DEFINE_int32(still_obstacle_history_length, 10,
             "Min # historical frames for still obstacles");
DEFINE_double(still_obstacle_speed_threshold, 1.0,
              "Speed threshold for still obstacles");
DEFINE_double(still_obstacle_position_std, 1.0,
              "Position standard deviation for still obstacles");
DEFINE_double(max_history_time, 7.0, "Obstacles' maximal historical time.");
DEFINE_double(target_lane_gap, 2.0, "gap between two lane points.");
DEFINE_double(max_lane_angle_diff, M_PI / 2.0,
              "Max angle difference for a candiate lane");
DEFINE_bool(enable_pedestrian_acc, false, "Enable calculating speed by acc");
DEFINE_double(coeff_mul_sigma, 2.0, "coefficient multiply standard deviation");
DEFINE_double(pedestrian_min_speed, 0.1, "min speed for still pedestrian");
DEFINE_double(pedestrian_max_speed, 10.0, "speed upper bound for pedestrian");
DEFINE_double(pedestrian_max_acc, 2.0, "maximum pedestrian acceleration");
DEFINE_double(prediction_pedestrian_total_time, 10.0,
              "Total prediction time for pedestrians");
DEFINE_int32(num_trajectory_still_pedestrian, 6,
             "number of trajectories for static pedestrian");
DEFINE_double(still_speed, 0.01, "speed considered to be still");
DEFINE_string(evaluator_vehicle_mlp_file,
              "modules/prediction/data/mlp_vehicle_model.bin",
              "mlp model file for vehicle evaluator");
DEFINE_string(evaluator_vehicle_rnn_file,
              "modules/prediction/data/rnn_vehicle_model.bin",
              "rnn model file for vehicle evaluator");
DEFINE_int32(max_num_obstacles, 100,
             "maximal number of obstacles stored in obstacles container.");

// evaluator
DEFINE_double(rnn_min_lane_relatice_s, 5.0,
              "Minimal relative s for RNN model.");

// Obstacle trajectory
DEFINE_double(lane_sequence_threshold, 0.5,
              "Threshold for trimming lane sequence trajectories");
DEFINE_double(lane_change_dist, 10.0, "Lane change distance with ADC");
DEFINE_bool(enable_lane_sequence_acc, false,
            "If use acceleration in lane sequence.");

// move sequence prediction
DEFINE_double(time_upper_bound_to_lane_center, 6.0,
              "Upper bound of time to get to the lane center");
DEFINE_double(time_lower_bound_to_lane_center, 1.0,
              "Lower bound of time to get to the lane center");
DEFINE_double(sample_time_gap, 0.2,
              "Gap of time to sample time to get to the lane center");
DEFINE_double(motion_weight_a, 1.2, "A parameter of motion weight function");
DEFINE_double(motion_weight_b, 5.0, "A parameter of motion weight function");
DEFINE_double(motion_weight_c, 1.2, "A parameter of motion weight function");
DEFINE_double(cost_alpha, 0.1,
              "The coefficient of time to lane center in cost function");
DEFINE_double(default_time_to_lane_center, 2.0,
              "The default time to lane center");
