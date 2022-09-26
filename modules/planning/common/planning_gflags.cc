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

#include "modules/planning/common/planning_gflags.h"

#include <limits>

DEFINE_bool(planning_test_mode, false, "Enable planning test mode.");

DEFINE_int32(planning_loop_rate, 10, "Loop rate for planning node");

DEFINE_int32(history_max_record_num, 5,
             "the number of planning history frame to keep");
DEFINE_int32(max_frame_history_num, 1, "The maximum history frame number");

// scenario related
DEFINE_string(scenario_bare_intersection_unprotected_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/bare_intersection_unprotected_config.pb.txt",
              "The bare_intersection_unprotected scenario configuration file");
DEFINE_string(scenario_lane_follow_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/lane_follow_config.pb.txt",
              "The lane_follow scenario configuration file");
DEFINE_string(scenario_lane_follow_hybrid_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/lane_follow_hybrid_config.pb.txt",
              "The lane_follow scenario configuration file for HYBRID");
DEFINE_string(scenario_learning_model_sample_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/learning_model_sample_config.pb.txt",
              "learning_model_sample scenario config file");
DEFINE_string(scenario_narrow_street_u_turn_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/narrow_street_u_turn_config.pb.txt",
              "narrow_street_u_turn scenario config file");
DEFINE_string(scenario_park_and_go_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/park_and_go_config.pb.txt",
              "park_and_go scenario config file");
DEFINE_string(scenario_pull_over_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/pull_over_config.pb.txt",
              "The pull_over scenario configuration file");
DEFINE_string(scenario_emergency_pull_over_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/emergency_pull_over_config.pb.txt",
              "The emergency_pull_over scenario configuration file");
DEFINE_string(scenario_emergency_stop_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/emergency_stop_config.pb.txt",
              "The emergency_stop scenario configuration file");
DEFINE_string(scenario_stop_sign_unprotected_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/stop_sign_unprotected_config.pb.txt",
              "stop_sign_unprotected scenario configuration file");
DEFINE_string(scenario_traffic_light_protected_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/traffic_light_protected_config.pb.txt",
              "traffic_light_protected scenario config file");
DEFINE_string(scenario_traffic_light_unprotected_left_turn_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/traffic_light_unprotected_left_turn_config.pb.txt",
              "traffic_light_unprotected_left_turn scenario config file");
DEFINE_string(scenario_traffic_light_unprotected_right_turn_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/traffic_light_unprotected_right_turn_config.pb.txt",
              "traffic_light_unprotected_right_turn scenario config file");
DEFINE_string(scenario_valet_parking_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/valet_parking_config.pb.txt",
              "valet_parking scenario config file");
DEFINE_string(scenario_deadend_turnaround_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/deadend_turnaround_config.pb.txt",
              "deadend_turnaround scenario config file");
DEFINE_string(scenario_yield_sign_config_file,
              "/apollo/modules/planning/conf/"
              "scenario/yield_sign_config.pb.txt",
              "yield_sign scenario config file");

DEFINE_bool(enable_scenario_bare_intersection, true,
            "enable bare_intersection scenarios in planning");

DEFINE_bool(enable_scenario_park_and_go, true,
            "enable park-and-go scenario in planning");

DEFINE_bool(enable_scenario_pull_over, false,
            "enable pull-over scenario in planning");

DEFINE_bool(enable_scenario_emergency_pull_over, true,
            "enable emergency-pull-over scenario in planning");

DEFINE_bool(enable_scenario_emergency_stop, true,
            "enable emergency-stop scenario in planning");

DEFINE_bool(enable_scenario_side_pass_multiple_parked_obstacles, true,
            "enable ADC to side-pass multiple parked obstacles without"
            "worrying if the obstacles are blocked by others.");

DEFINE_bool(enable_scenario_stop_sign, true,
            "enable stop_sign scenarios in planning");

DEFINE_bool(enable_scenario_traffic_light, true,
            "enable traffic_light scenarios in planning");

DEFINE_bool(enable_scenario_yield_sign, true,
            "enable yield_sign scenarios in planning");

DEFINE_bool(enable_force_pull_over_open_space_parking_test, false,
            "enable force_pull_over_open_space_parking_test");

DEFINE_string(traffic_rule_config_filename,
              "/apollo/modules/planning/conf/traffic_rule_config.pb.txt",
              "Traffic rule config filename");

DEFINE_string(smoother_config_filename,
              "/apollo/modules/planning/conf/qp_spline_smoother_config.pb.txt",
              "The configuration file for qp_spline smoother");

DEFINE_string(rtk_trajectory_filename, "modules/planning/data/garage.csv",
              "Loop rate for planning node");

DEFINE_uint64(rtk_trajectory_forward, 800,
              "The number of points to be included in RTK trajectory "
              "after the matched point");

DEFINE_double(rtk_trajectory_resolution, 0.01,
              "The time resolution of output trajectory for rtk planner.");

DEFINE_bool(publish_estop, false, "publish estop decision in planning");
DEFINE_bool(enable_trajectory_stitcher, true, "enable stitching trajectory");

DEFINE_bool(enable_reference_line_stitching, true,
            "Enable stitching reference line, which can reducing computing "
            "time and improve stability");
DEFINE_double(look_forward_extend_distance, 50,
              "The step size when extending reference line.");
DEFINE_double(reference_line_stitch_overlap_distance, 20,
              "The overlap distance with the existing reference line when "
              "stitching the existing reference line");

DEFINE_bool(enable_smooth_reference_line, true,
            "enable smooth the map reference line");

DEFINE_bool(prioritize_change_lane, false,
            "change lane strategy has higher priority, always use a valid "
            "change lane path if such path exists");
DEFINE_double(change_lane_min_length, 30.0,
              "meters. If the change lane target has longer length than this "
              "threshold, it can shortcut the default lane.");

DEFINE_bool(enable_reference_line_provider_thread, true,
            "Enable reference line provider thread.");

DEFINE_double(default_reference_line_width, 4.0,
              "Default reference line width");

DEFINE_double(smoothed_reference_line_max_diff, 5.0,
              "Maximum position difference between the smoothed and the raw "
              "reference lines.");

DEFINE_double(planning_upper_speed_limit, 31.3,
              "Maximum speed (m/s) in planning.");

DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");

DEFINE_double(threshold_distance_for_destination, 0.01,
              "threshold distance for destination");

DEFINE_double(buffer_in_routing, 0.0, "buffer for select in lane for boundary");

DEFINE_double(buffer_out_routing, -7.0,
              "buffer for select out lane for boundary");
// planning trajectory output time density control
DEFINE_double(
    trajectory_time_min_interval, 0.02,
    "(seconds) Trajectory time interval when publish. The is the min value.");
DEFINE_double(
    trajectory_time_max_interval, 0.1,
    "(seconds) Trajectory time interval when publish. The is the max value.");
DEFINE_double(
    trajectory_time_high_density_period, 1.0,
    "(seconds) Keep high density in the next this amount of seconds. ");

DEFINE_bool(enable_trajectory_check, false,
            "Enable sanity check for planning trajectory.");

DEFINE_double(speed_lower_bound, -0.1, "The lowest speed allowed.");
DEFINE_double(speed_upper_bound, 40.0, "The highest speed allowed.");

DEFINE_double(longitudinal_acceleration_lower_bound, -6.0,
              "The lowest longitudinal acceleration allowed.");
DEFINE_double(longitudinal_acceleration_upper_bound, 4.0,
              "The highest longitudinal acceleration allowed.");
DEFINE_double(lateral_acceleration_bound, 4.0,
              "Bound of lateral acceleration; symmetric for left and right");

DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(longitudinal_jerk_upper_bound, 2.0,
              "The upper bound of longitudinal jerk.");
DEFINE_double(lateral_jerk_bound, 4.0,
              "Bound of lateral jerk; symmetric for left and right");

DEFINE_double(kappa_bound, 0.1979, "The bound for trajectory curvature");

// ST Boundary
DEFINE_double(st_max_s, 100, "the maximum s of st boundary");
DEFINE_double(st_max_t, 8, "the maximum t of st boundary");

// Decision Part
DEFINE_bool(enable_nudge_slowdown, true,
            "True to slow down when nudge obstacles.");

DEFINE_double(static_obstacle_nudge_l_buffer, 0.3,
              "minimum l-distance to nudge a static obstacle (meters)");
DEFINE_double(nonstatic_obstacle_nudge_l_buffer, 0.4,
              "minimum l-distance to nudge a non-static obstacle (meters)");
DEFINE_double(lane_change_obstacle_nudge_l_buffer, 0.3,
              "minimum l-distance to nudge when changing lane (meters)");
DEFINE_double(lateral_ignore_buffer, 3.0,
              "If an obstacle's lateral distance is further away than this "
              "distance, ignore it");
DEFINE_double(max_stop_distance_obstacle, 10.0,
              "max stop distance from in-lane obstacle (meters)");
DEFINE_double(min_stop_distance_obstacle, 6.0,
              "min stop distance from in-lane obstacle (meters)");
DEFINE_double(follow_min_distance, 3.0,
              "min follow distance for vehicles/bicycles/moving objects");
DEFINE_double(follow_min_obs_lateral_distance, 2.5,
              "obstacle min lateral distance to follow");
DEFINE_double(yield_distance, 5.0,
              "min yield distance for vehicles/moving objects "
              "other than pedestrians/bicycles");
DEFINE_double(follow_time_buffer, 2.5,
              "time buffer in second to calculate the following distance.");
DEFINE_double(follow_min_time_sec, 2.0,
              "min follow time in st region before considering a valid follow,"
              " this is to differentiate a moving obstacle cross adc's"
              " current lane and move to a different direction");
DEFINE_double(signal_expire_time_sec, 5.0,
              "traffic light signal info read expire time in sec");
DEFINE_string(destination_obstacle_id, "DEST",
              "obstacle id for converting destination to an obstacle");
DEFINE_double(destination_check_distance, 5.0,
              "if the distance between destination and ADC is less than this,"
              " it is considered to reach destination");

DEFINE_double(virtual_stop_wall_length, 0.1,
              "virtual stop wall length (meters)");
DEFINE_double(virtual_stop_wall_height, 2.0,
              "virtual stop wall height (meters)");

// Path Deciders
DEFINE_bool(enable_skip_path_tasks, false,
            "skip all path tasks and use trimmed previous path");

DEFINE_double(obstacle_lat_buffer, 0.4,
              "obstacle lateral buffer (meters) for deciding path boundaries");
DEFINE_double(obstacle_lon_start_buffer, 3.0,
              "obstacle longitudinal start buffer (meters) for deciding "
              "path boundaries");
DEFINE_double(obstacle_lon_end_buffer, 2.0,
              "obstacle longitudinal end buffer (meters) for deciding "
              "path boundaries");
DEFINE_double(static_obstacle_speed_threshold, 0.5,
              "The speed threshold to decide whether an obstacle is static "
              "or not.");
DEFINE_double(lane_borrow_max_speed, 5.0,
              "The speed threshold for lane-borrow");
DEFINE_int32(long_term_blocking_obstacle_cycle_threshold, 3,
             "The cycle threshold for long-term blocking obstacle.");

// Prediction Part
DEFINE_double(prediction_total_time, 5.0, "Total prediction time");
DEFINE_bool(align_prediction_time, false,
            "enable align prediction data based planning time");

// Trajectory

// according to DMV's rule, turn signal should be on within 200 ft from
// intersection.
DEFINE_double(
    turn_signal_distance, 100.00,
    "In meters. If there is a turn within this distance, use turn signal");

DEFINE_int32(trajectory_point_num_for_debug, 10,
             "number of output trajectory points for debugging");

DEFINE_double(lane_change_prepare_length, 80.0,
              "The distance of lane-change preparation on current lane.");

DEFINE_double(min_lane_change_prepare_length, 10.0,
              "The minimal distance needed of lane-change on current lane.");

DEFINE_double(allowed_lane_change_failure_time, 2.0,
              "The time allowed for lane-change failure before updating"
              "preparation distance.");

DEFINE_bool(enable_smarter_lane_change, false,
            "enable smarter lane change with longer preparation distance.");

// QpSt optimizer
DEFINE_double(slowdown_profile_deceleration, -4.0,
              "The deceleration to generate slowdown profile. unit: m/s^2.");

// SQP solver
DEFINE_bool(enable_sqp_solver, true, "True to enable SQP solver.");

/// thread pool
DEFINE_bool(use_multi_thread_to_add_obstacles, false,
            "use multiple thread to add obstacles.");
DEFINE_bool(enable_multi_thread_in_dp_st_graph, false,
            "Enable multiple thread to calculation curve cost in dp_st_graph.");

/// Lattice Planner
DEFINE_double(numerical_epsilon, 1e-6, "Epsilon in lattice planner.");
DEFINE_double(default_cruise_speed, 5.0, "default cruise speed");
DEFINE_double(trajectory_time_resolution, 0.1,
              "Trajectory time resolution in planning");
DEFINE_double(trajectory_space_resolution, 1.0,
              "Trajectory space resolution in planning");
DEFINE_double(speed_lon_decision_horizon, 200.0,
              "Longitudinal horizon for speed decision making (meter)");
DEFINE_uint64(num_velocity_sample, 6,
              "The number of velocity samples in end condition sampler.");
DEFINE_bool(enable_backup_trajectory, true,
            "If generate backup trajectory when planning fail");
DEFINE_double(backup_trajectory_cost, 1000.0,
              "Default cost of backup trajectory");
DEFINE_double(min_velocity_sample_gap, 1.0,
              "Minimal sampling gap for velocity");
DEFINE_double(lon_collision_buffer, 2.0,
              "The longitudinal buffer to keep distance to other vehicles");
DEFINE_double(lat_collision_buffer, 0.1,
              "The lateral buffer to keep distance to other vehicles");
DEFINE_uint64(num_sample_follow_per_timestamp, 3,
              "The number of sample points for each timestamp to follow");

// Lattice Evaluate Parameters
DEFINE_double(weight_lon_objective, 10.0, "Weight of longitudinal travel cost");
DEFINE_double(weight_lon_jerk, 1.0, "Weight of longitudinal jerk cost");
DEFINE_double(weight_lon_collision, 5.0,
              "Weight of longitudinal collision cost");
DEFINE_double(weight_lat_offset, 2.0, "Weight of lateral offset cost");
DEFINE_double(weight_lat_comfort, 10.0, "Weight of lateral comfort cost");
DEFINE_double(weight_centripetal_acceleration, 1.5,
              "Weight of centripetal acceleration");
DEFINE_double(cost_non_priority_reference_line, 5.0,
              "The cost of planning on non-priority reference line.");
DEFINE_double(weight_same_side_offset, 1.0,
              "Weight of same side lateral offset cost");
DEFINE_double(weight_opposite_side_offset, 10.0,
              "Weight of opposite side lateral offset cost");
DEFINE_double(weight_dist_travelled, 10.0, "Weight of travelled distance cost");
DEFINE_double(weight_target_speed, 1.0, "Weight of target speed cost");
DEFINE_double(lat_offset_bound, 3.0, "The bound of lateral offset");
DEFINE_double(lon_collision_yield_buffer, 1.0,
              "Longitudinal collision buffer for yield");
DEFINE_double(lon_collision_overtake_buffer, 5.0,
              "Longitudinal collision buffer for overtake");
DEFINE_double(lon_collision_cost_std, 0.5,
              "The standard deviation of longitudinal collision cost function");
DEFINE_double(default_lon_buffer, 5.0,
              "Default longitudinal buffer to sample path-time points.");
DEFINE_double(time_min_density, 1.0,
              "Minimal time density to search sample points.");
DEFINE_double(comfort_acceleration_factor, 0.5,
              "Factor for comfort acceleration.");
DEFINE_double(polynomial_minimal_param, 0.01,
              "Minimal time parameter in polynomials.");
DEFINE_double(lattice_stop_buffer, 0.02,
              "The buffer before the stop s to check trajectories.");

DEFINE_bool(lateral_optimization, true,
            "whether using optimization for lateral trajectory generation");
DEFINE_double(weight_lateral_offset, 1.0,
              "weight for lateral offset "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_derivative, 500.0,
              "weight for lateral derivative "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_second_order_derivative, 1000.0,
              "weight for lateral second order derivative "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_third_order_derivative, 1000.0,
              "weight for lateral third order derivative "
              "in lateral trajectory optimization");
DEFINE_double(
    weight_lateral_obstacle_distance, 0.0,
    "weight for lateral obstacle distance in lateral trajectory optimization");
DEFINE_double(lateral_third_order_derivative_max, 0.1,
              "the maximal allowance for lateral third order derivative");
DEFINE_double(lateral_derivative_bound_default, 2.0,
              "the default value for lateral derivative bound.");
DEFINE_double(max_s_lateral_optimization, 60.0,
              "The maximal s for lateral optimization.");
DEFINE_double(default_delta_s_lateral_optimization, 1.0,
              "The default delta s for lateral optimization.");
DEFINE_double(bound_buffer, 0.1, "buffer to boundary for lateral optimization");
DEFINE_double(nudge_buffer, 0.3, "buffer to nudge for lateral optimization");

DEFINE_double(fallback_total_time, 3.0, "total fallback trajectory time");
DEFINE_double(fallback_time_unit, 0.1,
              "fallback trajectory unit time in seconds");

DEFINE_double(speed_bump_speed_limit, 4.4704,
              "the speed limit when passing a speed bump, m/s. The default "
              "speed limit is 10 mph.");
DEFINE_double(default_city_road_speed_limit, 15.67,
              "default speed limit (m/s) for city road. 35 mph.");
DEFINE_double(default_highway_speed_limit, 29.06,
              "default speed limit (m/s) for highway. 65 mph.");

// navigation mode
DEFINE_bool(enable_planning_pad_msg, false,
            "To control whether to enable planning pad message.");

// TODO(all): open space planner, merge with planning conf
DEFINE_string(planner_open_space_config_filename,
              "/apollo/modules/planning/conf/planner_open_space_config.pb.txt",
              "The open space planner configuration file");

DEFINE_double(open_space_planning_period, 4.0,
              "estimated time for open space planner planning period");

DEFINE_double(open_space_prediction_time_horizon, 2.0,
              "the time in second we use from the trajectory of obstacles "
              "given by prediction");

DEFINE_bool(enable_perception_obstacles, true,
            "enable the open space planner to take perception obstacles into "
            "consideration");

DEFINE_bool(enable_open_space_planner_thread, true,
            "Enable thread in open space planner for trajectory publish.");

DEFINE_bool(use_dual_variable_warm_start, true,
            "whether or not enable dual variable warm start ");

DEFINE_bool(use_gear_shift_trajectory, false,
            "allow some time for the vehicle to shift gear");

DEFINE_uint64(open_space_trajectory_stitching_preserved_length,
              std::numeric_limits<uint32_t>::infinity(),
              "preserved points number in trajectory stitching for open space "
              "trajectory");
DEFINE_bool(
    enable_smoother_failsafe, false,
    "whether to use warm start result as final output when smoother fails");

DEFINE_bool(use_s_curve_speed_smooth, false,
            "Whether use s-curve (piecewise_jerk) for smoothing Hybrid Astar "
            "speed/acceleration.");

DEFINE_bool(
    use_iterative_anchoring_smoother, false,
    "Whether use iterative_anchoring_smoother for open space planning ");

DEFINE_bool(
    enable_parallel_trajectory_smoothing, false,
    "Whether to partition the trajectory first and do smoothing in parallel");

DEFINE_bool(enable_osqp_debug, false,
            "True to turn on OSQP verbose debug output in log.");

DEFINE_bool(export_chart, false, "export chart in planning");
DEFINE_bool(enable_record_debug, true,
            "True to enable record debug info in chart format");

DEFINE_double(
    default_front_clear_distance, 300.0,
    "default front clear distance value in case there is no obstacle around.");

DEFINE_double(max_trajectory_len, 1000.0,
              "(unit: meter) max possible trajectory length.");
DEFINE_bool(enable_rss_fallback, false, "trigger rss fallback");
DEFINE_bool(enable_rss_info, true, "enable rss_info in trajectory_pb");
DEFINE_double(rss_max_front_obstacle_distance, 3000.0,
              "(unit: meter) for max front obstacle distance.");

DEFINE_bool(
    enable_planning_smoother, false,
    "True to enable planning smoother among different planning cycles.");
DEFINE_double(smoother_stop_distance, 10.0,
              "(unit: meter) for ADC stop, if it is close to the stop point "
              "within this threshold, current planning will be smoothed.");

DEFINE_bool(enable_parallel_hybrid_a, false,
            "True to enable hybrid a* parallel implementation.");

DEFINE_double(open_space_standstill_acceleration, 0.0,
              "(unit: meter/sec^2) for open space stand still at destination");

DEFINE_bool(enable_dp_reference_speed, true,
            "True to penalize dp result towards default cruise speed");

DEFINE_double(message_latency_threshold, 0.02, "Threshold for message delay");
DEFINE_bool(enable_lane_change_urgency_checking, false,
            "True to check the urgency of lane changing");
DEFINE_double(short_path_length_threshold, 20.0,
              "Threshold for too short path length");

DEFINE_uint64(trajectory_stitching_preserved_length, 20,
              "preserved points number in trajectory stitching");

DEFINE_double(side_pass_driving_width_l_buffer, 0.1,
              "(unit: meter) for side pass driving width l buffer");

DEFINE_bool(use_st_drivable_boundary, false,
            "True to use st_drivable boundary in speed planning");

DEFINE_bool(enable_reuse_path_in_lane_follow, false,
            "True to enable reuse path in lane follow");
DEFINE_bool(
    use_smoothed_dp_guide_line, false,
    "True to penalize speed optimization result to be close to dp guide line");

DEFINE_bool(use_soft_bound_in_nonlinear_speed_opt, true,
            "False to disallow soft bound in nonlinear speed opt");

DEFINE_bool(use_front_axe_center_in_path_planning, false,
            "If using front axe center in path planning, the path can be "
            "more agile.");

DEFINE_bool(use_road_boundary_from_map, false, "get road boundary from HD map");

DEFINE_bool(planning_offline_learning, false,
            "offline learning. read record files and dump learning_data");
DEFINE_string(planning_data_dir, "/apollo/modules/planning/data/",
              "Prefix of files to store feature data");
DEFINE_string(planning_offline_bags, "",
              "a list of source files or directories for offline mode. "
              "The items need to be separated by colon ':'. ");
DEFINE_int32(learning_data_obstacle_history_time_sec, 3.0,
             "time sec (second) of history trajectory points for a obstacle");
DEFINE_int32(learning_data_frame_num_per_file, 100,
             "number of learning_data_frame to write out in one data file.");
DEFINE_string(
    planning_birdview_img_feature_renderer_config_file,
    "/apollo/modules/planning/conf/planning_semantic_map_config.pb.txt",
    "config file for renderer singleton");

DEFINE_bool(
    skip_path_reference_in_side_pass, false,
    "skipping using learning model output as path reference in side pass");
DEFINE_bool(
    skip_path_reference_in_change_lane, true,
    "skipping using learning model output as path reference in change lane");

DEFINE_int32(min_past_history_points_len, 0,
             "minimun past history points length for trainsition from "
             "rule-based planning to learning-based planning");
