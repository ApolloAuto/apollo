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

#include "modules/prediction/common/prediction_gflags.h"

#include <cmath>

DEFINE_double(double_precision, 1e-6, "precision of double");

// prediction trajectory and dynamic model
DEFINE_double(prediction_trajectory_time_length, 6.0,
              "Time length of predicted trajectory (in seconds)");
DEFINE_double(prediction_trajectory_time_resolution, 0.1,
              "Time resolution of predicted trajectory (in seconds");
DEFINE_double(min_prediction_trajectory_spatial_length, 100.0,
              "Minimal spatial length of predicted trajectory");
DEFINE_bool(enable_trajectory_validation_check, false,
            "If check the validity of prediction trajectory.");
DEFINE_bool(enable_tracking_adaptation, false,
            "If enable prediction tracking adaptation");
DEFINE_bool(free_move_predict_with_accelerate, false,
              "If freemove predict with accelerate");

DEFINE_double(vehicle_max_linear_acc, 4.0,
              "Upper bound of vehicle linear acceleration");
DEFINE_double(vehicle_min_linear_acc, -4.0,
              "Lower bound of vehicle linear deceleration");
DEFINE_double(vehicle_max_speed, 35.0, "Max speed of vehicle");

// Tracking Adaptation
DEFINE_double(max_tracking_time, 0.5,
              "Max tracking time for disappear obstacles");
DEFINE_double(max_tracking_dist, 3.0,
              "Max tracking distance for disappear obstacles");

// Map
DEFINE_double(lane_search_radius, 3.0, "Search radius for a candidate lane");
DEFINE_double(lane_search_radius_in_junction, 15.0,
              "Search radius for a candidate lane");
DEFINE_double(junction_search_radius, 1.0, "Search radius for a junction");
DEFINE_double(pedestrian_nearby_lane_search_radius, 5.0,
              "Radius to determine if pedestrian-like obstacle is near lane.");
DEFINE_int32(road_graph_max_search_horizon, 20,
             "Maximal search depth for building road graph");
DEFINE_double(surrounding_lane_search_radius, 3.0,
              "Search radius for surrounding lanes.");

// Semantic Map
DEFINE_double(base_image_half_range, 100.0, "The half range of base image.");
DEFINE_bool(enable_draw_adc_trajectory, true,
            "If draw adc trajectory in semantic map");
DEFINE_bool(img_show_semantic_map, false, "If show the image of semantic map.");

// Scenario
DEFINE_double(junction_distance_threshold, 10.0,
              "Distance threshold "
              "to junction to consider as junction scenario");
DEFINE_bool(enable_all_junction, false,
            "If consider all junction with junction_mlp_model.");
DEFINE_bool(enable_all_pedestrian_caution_in_front, false,
            "If true, then all pedestrian in front of ADC are marked caution.");
DEFINE_bool(enable_rank_caution_obstacles, true,
            "Rank the caution-level obstacles.");
DEFINE_bool(enable_rank_interactive_obstacles, true,
            "Rank the interactive obstacles.");
DEFINE_int32(caution_obs_max_nums, 6,
             "The max number of caution-level obstacles");
DEFINE_double(caution_distance_threshold, 60.0,
              "Distance threshold for caution obstacles");
DEFINE_double(caution_search_distance_ahead, 50.0,
              "The distance ahead to search caution-level obstacles");
DEFINE_double(caution_search_distance_backward, 50.0,
              "The distance backward to search caution-level obstacles");
DEFINE_double(caution_search_distance_backward_for_merge, 60.0,
              "The distance backward to search caution-lebel obstacles "
              "in the case of merging");
DEFINE_double(caution_search_distance_backward_for_overlap, 30.0,
              "The distance backward to search caution-lebel obstacles "
              "in the case of overlap");
DEFINE_double(caution_pedestrian_approach_time, 3.0,
              "The time for a pedestrian to approach adc trajectory");
DEFINE_int32(interactive_obs_max_nums, 6,
             "The max number of interactive obstacles");
DEFINE_double(interaction_distance_threshold, 60.0,
              "Distance threshold for interactive obstacles");
DEFINE_double(interaction_search_distance_ahead, 50.0,
              "The distance ahead to search interactive obstacles");
DEFINE_double(interaction_search_distance_backward, 50.0,
              "The distance backward to search interactive obstacles");
DEFINE_double(interaction_search_distance_backward_for_merge, 60.0,
              "The distance backward to search interactive obstacles "
              "in the case of merging");
DEFINE_double(interaction_search_distance_backward_for_overlap, 30.0,
              "The distance backward to search interactive obstacles "
              "in the case of overlap");

// Obstacle features
DEFINE_int32(ego_vehicle_id, -1, "The obstacle ID of the ego vehicle.");
DEFINE_double(scan_length, 80.0, "The length of the obstacles scan area");
DEFINE_double(scan_width, 12.0, "The width of the obstacles scan area");
DEFINE_double(back_dist_ignore_ped, -2.0,
              "Backward distance to ignore pedestrians.");
DEFINE_uint64(cruise_historical_frame_length, 5,
              "The number of historical frames of the obstacle"
              "that the cruise model will look at.");
DEFINE_bool(enable_kf_tracking, false, "Use measurements with KF tracking");

DEFINE_double(max_angle_diff_to_adjust_velocity, M_PI / 6.0,
              "The maximal angle diff to adjust velocity heading.");
DEFINE_double(q_var, 0.01, "Processing noise covariance");
DEFINE_double(r_var, 0.25, "Measurement noise covariance");
DEFINE_double(p_var, 0.1, "Error covariance");
DEFINE_double(go_approach_rate, 0.995,
              "The rate to approach to the reference line of going straight");
DEFINE_double(cutin_approach_rate, 0.95,
              "The rate to approach to the reference line of cutin");

DEFINE_int32(min_still_obstacle_history_length, 4,
             "Min # historical frames for still obstacles");
DEFINE_int32(max_still_obstacle_history_length, 10,
             "Min # historical frames for still obstacles");
DEFINE_double(still_obstacle_speed_threshold, 0.99,
              "Speed threshold for still obstacles");
DEFINE_double(still_pedestrian_speed_threshold, 0.2,
              "Speed threshold for still pedestrians");
DEFINE_double(still_unknown_speed_threshold, 0.5,
              "Speed threshold for still unknown obstacles");
DEFINE_double(still_obstacle_position_std, 0.5,
              "Position standard deviation for still obstacles");
DEFINE_double(still_pedestrian_position_std, 0.5,
              "Position standard deviation for still pedestrians");
DEFINE_double(still_unknown_position_std, 0.5,
              "Position standard deviation for still unknown obstacles");
DEFINE_double(slow_obstacle_speed_threshold, 2.0,
              "Speed threshold for slow obstacles");
DEFINE_double(max_history_time, 7.0, "Obstacles' maximal historical time.");
DEFINE_double(target_lane_gap, 2.0, "Gap between two lane points.");
DEFINE_double(dense_lane_gap, 0.2,
              "Gap between two adjacent lane points"
              " for constructing dense lane graph.");
DEFINE_int32(max_num_current_lane, 2, "Max number to search current lanes");
DEFINE_int32(max_num_nearby_lane, 2, "Max number to search nearby lanes");
DEFINE_double(max_lane_angle_diff, M_PI / 3.0,
              "Max angle difference for a candidate lane");
DEFINE_int32(max_num_current_lane_in_junction, 3,
             "Max number to search current lanes");
DEFINE_int32(max_num_nearby_lane_in_junction, 2,
             "Max number to search nearby lanes");
DEFINE_double(max_lane_angle_diff_in_junction, M_PI / 4.0,
              "Max angle difference for a candidate lane");
DEFINE_double(coeff_mul_sigma, 2.0, "coefficient multiply standard deviation");
DEFINE_double(pedestrian_max_speed, 10.0, "speed upper bound for pedestrian");
DEFINE_double(pedestrian_max_acc, 2.0, "maximum pedestrian acceleration");
DEFINE_double(still_speed, 0.01, "speed considered to be still");
DEFINE_string(evaluator_vehicle_mlp_file,
              "/apollo/modules/prediction/data/mlp_vehicle_model.bin",
              "mlp model file for vehicle evaluator");
DEFINE_string(evaluator_vehicle_rnn_file,
              "/apollo/modules/prediction/data/rnn_vehicle_model.bin",
              "rnn model file for vehicle evaluator");
DEFINE_string(
    torch_vehicle_jointly_model_file,
    "/apollo/modules/prediction/data/"
    "jointly_prediction_planning_vehicle_model.pt",
    "Vehicle jointly prediction and planning model file");
DEFINE_string(
    torch_vehicle_jointly_model_cpu_file,
    "/apollo/modules/prediction/data/"
    "jointly_prediction_planning_vehicle_cpu_model.pt",
    "Vehicle jointly prediction and planning cpu model file");
DEFINE_string(torch_vehicle_junction_mlp_file,
              "/apollo/modules/prediction/data/junction_mlp_vehicle_model.pt",
              "Vehicle junction MLP model file");
DEFINE_string(torch_vehicle_junction_map_file,
              "/apollo/modules/prediction/data/junction_map_vehicle_model.pt",
              "Vehicle junction map model file");
DEFINE_string(torch_vehicle_semantic_lstm_file,
              "/apollo/modules/prediction/data/semantic_lstm_vehicle_model.pt",
              "Vehicle semantic lstm model file, default for gpu");
DEFINE_string(
    torch_vehicle_semantic_lstm_cpu_file,
    "/apollo/modules/prediction/data/semantic_lstm_vehicle_cpu_model.pt",
    "Vehicle semantic lstm cpu model file");
DEFINE_string(torch_vehicle_cruise_go_file,
              "/apollo/modules/prediction/data/cruise_go_vehicle_model.pt",
              "Vehicle cruise go model file");
DEFINE_string(torch_vehicle_cruise_cutin_file,
              "/apollo/modules/prediction/data/cruise_cutin_vehicle_model.pt",
              "Vehicle cruise cutin model file");
DEFINE_string(torch_vehicle_lane_scanning_file,
              "/apollo/modules/prediction/data/lane_scanning_vehicle_model.pt",
              "Vehicle lane scanning model file");
DEFINE_string(torch_vehicle_vectornet_file,
              "/apollo/modules/prediction/data/vectornet_vehicle_model.pt",
              "Vehicle vectornet model file");
DEFINE_string(torch_vehicle_vectornet_cpu_file,
              "/apollo/modules/prediction/data/vectornet_vehicle_cpu_model.pt",
              "Vehicle vectornet cpu model file");
DEFINE_string(torch_pedestrian_interaction_position_embedding_file,
              "/apollo/modules/prediction/data/"
              "pedestrian_interaction_position_embedding.pt",
              "pedestrian interaction position embedding");
DEFINE_string(torch_pedestrian_interaction_social_embedding_file,
              "/apollo/modules/prediction/data/"
              "pedestrian_interaction_social_embedding.pt",
              "pedestrian interaction social embedding");
DEFINE_string(torch_pedestrian_interaction_single_lstm_file,
              "/apollo/modules/prediction/data/"
              "pedestrian_interaction_single_lstm.pt",
              "pedestrian interaction single lstm");
DEFINE_string(torch_pedestrian_interaction_prediction_layer_file,
              "/apollo/modules/prediction/data/"
              "pedestrian_interaction_prediction_layer.pt",
              "pedestrian interaction prediction layer");
DEFINE_string(
    torch_pedestrian_semantic_lstm_file,
    "/apollo/modules/prediction/data/semantic_lstm_pedestrian_model.pt",
    "Pedestrian semantic lstm model file, default for gpu");
DEFINE_string(
    torch_pedestrian_semantic_lstm_cpu_file,
    "/apollo/modules/prediction/data/semantic_lstm_pedestrian_cpu_model.pt",
    "Pedestrian semantic lstm cpu model file");
DEFINE_string(torch_lane_aggregating_obstacle_encoding_file,
              "/apollo/modules/prediction/data/"
              "traced_online_obs_enc.pt",
              "lane aggregating obstacle encoding layer");
DEFINE_string(torch_lane_aggregating_lane_encoding_file,
              "/apollo/modules/prediction/data/"
              "traced_online_lane_enc.pt",
              "lane aggregating lane encoding layer");
DEFINE_string(torch_lane_aggregating_prediction_layer_file,
              "/apollo/modules/prediction/data/"
              "traced_online_pred_layer.pt",
              "lane aggregating prediction layer");
DEFINE_int32(max_num_obstacles, 300,
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
DEFINE_bool(adjust_vehicle_heading_by_lane, true,
            "adjust vehicle heading by lane");
DEFINE_double(heading_filter_param, 0.98, "heading filter parameter");
DEFINE_uint64(max_num_lane_point, 20,
              "The maximal number of lane points to store");
DEFINE_double(distance_threshold_to_junction_exit, 1.0,
              "Threshold of distance to junction exit");
DEFINE_double(angle_threshold_to_junction_exit, M_PI * 0.25,
              "Threshold of angle to junction exit");
DEFINE_uint32(sample_size_for_average_lane_curvature, 10,
              "The sample size to compute average lane curvature");

// Validation checker
DEFINE_double(centripetal_acc_coeff, 0.5,
              "Coefficient of centripetal acceleration probability");

// Junction Scenario
DEFINE_uint32(junction_historical_frame_length, 5,
              "The number of historical frames of the obstacle"
              "that the junction model will look at.");
DEFINE_double(junction_exit_lane_threshold, 0.1,
              "If a lane extends out of the junction by this value,"
              "consider it as an exit_lane.");
DEFINE_double(distance_beyond_junction, 0.5,
              "If the obstacle is in junction more than this threshold,"
              "consider it in junction.");
DEFINE_double(defualt_junction_range, 10.0,
              "Default value for the range of a junction.");
DEFINE_double(distance_to_slow_down_at_stop_sign, 80.0,
              "The distance to slow down at stop sign");

// Evaluator
DEFINE_double(time_to_center_if_not_reach, 10.0,
              "Default value of time to lane center of not reach.");
DEFINE_uint32(affine_pool_size, 80,
              "The number of cv mat pool size.");
DEFINE_uint32(warm_up_times, 10,
              "The number of torch model warm up times.");
DEFINE_double(default_s_if_no_obstacle_in_lane_sequence, 1000.0,
              "The default s value if no obstacle in the lane sequence.");
DEFINE_double(default_l_if_no_obstacle_in_lane_sequence, 10.0,
              "The default l value if no obstacle in the lane sequence.");
DEFINE_bool(enable_semantic_map, true, "If enable semantic map on prediction");

// Obstacle trajectory
DEFINE_bool(enable_cruise_regression, false,
            "If enable using regression in cruise model");
DEFINE_double(lane_sequence_threshold_cruise, 0.5,
              "Threshold for trimming lane sequence trajectories in cruise");
DEFINE_double(lane_sequence_threshold_junction, 0.5,
              "Threshold for trimming lane sequence trajectories in junction");
DEFINE_double(lane_change_dist, 10.0, "Lane change distance with ADC");
DEFINE_bool(enable_lane_sequence_acc, false,
            "If use acceleration in lane sequence.");
DEFINE_bool(enable_trim_prediction_trajectory, true,
            "If trim the prediction trajectory to avoid crossing"
            "protected adc planning trajectory.");
DEFINE_double(adc_trajectory_search_length, 10.0,
              "How far to search junction along adc planning trajectory");
DEFINE_double(virtual_lane_radius, 2.0, "Radius to search virtual lanes");
DEFINE_double(default_lateral_approach_speed, 0.5,
              "Default lateral speed approaching to center of lane");
DEFINE_double(centripedal_acc_threshold, 2.0,
              "Threshold of centripedal acceleration.");

// move sequence prediction
DEFINE_double(time_upper_bound_to_lane_center, 6.0,
              "Upper bound of time to get to the lane center");
DEFINE_double(time_lower_bound_to_lane_center, 1.0,
              "Lower bound of time to get to the lane center");
DEFINE_double(sample_time_gap, 0.5,
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
DEFINE_double(cost_function_alpha, 0.25,
              "alpha of the cost function for best trajectory selection,"
              "alpha weighs the influence by max lateral acceleration"
              "and that by the total time. The larger alpha gets, the"
              "more cost function values trajectory with shorter times,"
              "and vice versa.");
DEFINE_double(cost_function_sigma, 5.0,
              "This is the sigma for the bell curve that is used by"
              "the cost function in move-sequence-trajectory-predictor."
              "The bell curve has its average equal to the time to cross"
              "lane predicted by the model.");
DEFINE_bool(use_bell_curve_for_cost_function, false,
            "Whether to use bell curve for the cost function or not.");

// interaction predictor
DEFINE_bool(enable_interactive_tag, true,
            "Whether to set interactive tag for obstacles.");
DEFINE_double(collision_cost_time_resolution, 1.0,
              "The time resolution used to compute the collision cost");
DEFINE_double(longitudinal_acceleration_cost_weight, 0.2,
              "The weight of longitudinal acceleration cost");
DEFINE_double(centripedal_acceleration_cost_weight, 0.1,
              "The weight of the cost related to centripedal acceleration");
DEFINE_double(collision_cost_weight, 1.0,
              "The weight of the cost related to collision");
DEFINE_double(collision_cost_exp_coefficient, 1.0,
              "The coefficient in the collision exponential cost function");
DEFINE_double(likelihood_exp_coefficient, 1.0,
              "The coefficient in the likelihood exponential function");

DEFINE_double(lane_distance_threshold, 3.0,
              "The threshold for distance to ego/neighbor lane "
              "in feature extraction");

DEFINE_double(lane_angle_difference_threshold, M_PI * 0.25,
              "The threshold for distance to ego/neighbor lane "
              "in feature extraction");

// Trajectory evaluation
DEFINE_double(distance_threshold_on_lane, 1.5,
              "The threshold of distance in on-lane situation");
