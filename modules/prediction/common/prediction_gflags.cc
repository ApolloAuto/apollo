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
DEFINE_string(prediction_data_file_prefix, "data/prediction/feature",
              "Prefix of files to store feature data");
DEFINE_bool(prediction_test_mode, false, "Set prediction to test mode");
DEFINE_double(
    prediction_test_duration, -1.0,
    "The runtime duration in test mode (in seconds). Negative value will not "
    "restrict the runtime duration.");

DEFINE_bool(prediction_offline_mode, false, "Prediction offline mode");

DEFINE_double(prediction_duration, 5.0, "Prediction duration (in seconds)");
DEFINE_double(prediction_period, 0.1, "Prediction period (in seconds");
DEFINE_double(double_precision, 1e-6, "precision of double");
DEFINE_double(min_prediction_length, 20.0,
              "Minimal length of prediction trajectory");

// Bag replay timestamp gap
DEFINE_double(replay_timestamp_gap, 10.0,
              "Max timestamp gap for rosbag replay");

// Map
DEFINE_double(lane_search_radius, 3.0, "Search radius for a candidate lane");
DEFINE_double(lane_search_radius_in_junction, 15.0,
              "Search radius for a candidate lane");
DEFINE_double(junction_search_radius, 1.0, "Search radius for a junction");

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

DEFINE_int32(still_obstacle_history_length, 10,
             "Min # historical frames for still obstacles");
DEFINE_double(still_obstacle_speed_threshold, 2.0,
              "Speed threshold for still obstacles");
DEFINE_double(still_pedestrian_speed_threshold, 0.5,
              "Speed threshold for still pedestrians");
DEFINE_double(still_obstacle_position_std, 1.0,
              "Position standard deviation for still obstacles");
DEFINE_double(still_pedestrian_position_std, 0.5,
              "Position standard deviation for still obstacles");
DEFINE_double(max_history_time, 7.0, "Obstacles' maximal historical time.");
DEFINE_double(target_lane_gap, 2.0, "gap between two lane points.");
DEFINE_int32(max_num_current_lane, 2, "Max number to search current lanes");
DEFINE_int32(max_num_nearby_lane, 2, "Max number to search nearby lanes");
DEFINE_double(max_lane_angle_diff, M_PI / 2.0,
              "Max angle difference for a candiate lane");
DEFINE_int32(max_num_current_lane_in_junction, 1,
             "Max number to search current lanes");
DEFINE_int32(max_num_nearby_lane_in_junction, 0,
             "Max number to search nearby lanes");
DEFINE_double(max_lane_angle_diff_in_junction, M_PI / 6.0,
              "Max angle difference for a candiate lane");
DEFINE_bool(enable_pedestrian_acc, false, "Enable calculating speed by acc");
DEFINE_double(coeff_mul_sigma, 2.0, "coefficient multiply standard deviation");
DEFINE_double(pedestrian_max_speed, 10.0, "speed upper bound for pedestrian");
DEFINE_double(pedestrian_max_acc, 2.0, "maximum pedestrian acceleration");
DEFINE_double(prediction_pedestrian_total_time, 10.0,
              "Total prediction time for pedestrians");
DEFINE_double(still_speed, 0.01, "speed considered to be still");
DEFINE_string(evaluator_vehicle_mlp_file,
              "modules/prediction/data/mlp_vehicle_model.bin",
              "mlp model file for vehicle evaluator");
DEFINE_string(evaluator_vehicle_rnn_file,
              "modules/prediction/data/rnn_vehicle_model.bin",
              "rnn model file for vehicle evaluator");
DEFINE_int32(max_num_obstacles, 100,
             "maximal number of obstacles stored in obstacles container.");
DEFINE_double(valid_position_diff_threshold, 0.5,
              "threshold of valid position difference");
DEFINE_double(valid_position_diff_rate_threshold, 0.075,
              "threshold of valid position difference rate");
DEFINE_double(split_rate, 0.5, "obstacle split rate for adjusting velocity");
DEFINE_double(rnn_min_lane_relatice_s, 5.0,
              "Minimal relative s for RNN model.");
DEFINE_bool(adjust_velocity_by_obstacle_heading, false,
            "Use obstacle heading for velocity.");
DEFINE_bool(adjust_velocity_by_position_shift, false,
            "adjust velocity heading to lane heading");
DEFINE_double(heading_filter_param, 0.99, "heading filter parameter");
DEFINE_uint32(max_num_lane_point, 20,
              "The maximal number of lane points to store");

// Validation checker
DEFINE_double(centripetal_acc_coeff, 0.5,
              "Coefficient of centripetal acceleration probability");

// Obstacle trajectory
DEFINE_double(lane_sequence_threshold, 0.5,
              "Threshold for trimming lane sequence trajectories");
DEFINE_double(lane_change_dist, 10.0, "Lane change distance with ADC");
DEFINE_bool(enable_lane_sequence_acc, false,
            "If use acceleration in lane sequence.");
DEFINE_bool(enable_trim_prediction_trajectory, false,
            "If trim the prediction trajectory to avoid crossing"
            "protected adc planning trajectory.");
DEFINE_bool(enable_trajectory_validation_check, false,
            "If check the validity of prediction trajectory.");
DEFINE_double(distance_beyond_junction, 0.5,
              "If the obstacle is in junction more than this threshold,"
              "consider it in junction.");
DEFINE_double(adc_trajectory_search_length, 10.0,
              "How far to search junction along adc planning trajectory");
DEFINE_double(virtual_lane_radius, 0.5, "Radius to search virtual lanes");
DEFINE_double(default_lateral_approach_speed, 0.5,
              "Default lateral speed approaching to center of lane");
DEFINE_double(centripedal_acc_threshold, 2.0,
              "Threshold of centripedal acceleration.");

// move sequence prediction
DEFINE_double(time_upper_bound_to_lane_center, 5.0,
              "Upper bound of time to get to the lane center");
DEFINE_double(time_lower_bound_to_lane_center, 1.0,
              "Lower bound of time to get to the lane center");
DEFINE_double(sample_time_gap, 0.2,
              "Gap of time to sample time to get to the lane center");
DEFINE_double(cost_alpha, 100.0,
              "The coefficient of lateral acceleration in cost function");
DEFINE_double(default_time_to_lat_end_state, 5.0,
              "The default time to lane center");
DEFINE_double(turning_curvature_lower_bound, 0.02,
              "The curvature lower bound of turning lane");
DEFINE_double(turning_curvature_upper_bound, 0.14,
              "The curvature upper bound of turning lane");
DEFINE_double(speed_at_lower_curvature, 8.5,
              "The speed at turning lane with lower bound curvature");
DEFINE_double(speed_at_upper_curvature, 3.0,
              "The speed at turning lane with upper bound curvature");
