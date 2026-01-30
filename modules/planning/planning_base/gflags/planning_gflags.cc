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

#include "modules/planning/planning_base/gflags/planning_gflags.h"

#include <limits>

DEFINE_int32(planning_loop_rate, 10, "Loop rate for planning node");

DEFINE_int32(history_max_record_num, 5,
             "the number of planning history frame to keep");
DEFINE_int32(max_frame_history_num, 1, "The maximum history frame number");

DEFINE_bool(enable_scenario_side_pass_multiple_parked_obstacles, true,
            "enable ADC to side-pass multiple parked obstacles without"
            "worrying if the obstacles are blocked by others.");

DEFINE_bool(enable_force_pull_over_open_space_parking_test, false,
            "enable force_pull_over_open_space_parking_test");

DEFINE_string(
    traffic_rule_config_filename,
    "modules/planning/planning_component/conf/traffic_rule_config.pb.txt",
    "Traffic rule config filename");

DEFINE_string(smoother_config_filename,
              "/apollo/modules/planning/planning_component/conf/"
              "qp_spline_smoother_config.pb.txt",
              "The configuration file for qp_spline smoother");

DEFINE_string(rtk_trajectory_filename,
              "modules/planning/planning_base/data/garage.csv",
              "Loop rate for planning node");

DEFINE_uint64(rtk_trajectory_forward, 800,
              "The number of points to be included in RTK trajectory "
              "after the matched point");

DEFINE_double(rtk_trajectory_resolution, 0.01,
              "The time resolution of output trajectory for rtk planner.");

DEFINE_string(planner_config_path,
              "modules/planning/planning_component/conf/"
              "public_road_planner_config.pb.txt",
              "The configuration for planner.");

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

DEFINE_double(reference_line_endpoint_extend_length, 10.0,
              "Extended length of reference line endpoint");

DEFINE_double(planning_upper_speed_limit, 31.3,
              "Maximum speed (m/s) in planning.");

DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");

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
DEFINE_double(static_obstacle_nudge_l_buffer, 0.3,
              "minimum l-distance to nudge a static obstacle (meters)");
DEFINE_double(nonstatic_obstacle_nudge_l_buffer, 0.4,
              "minimum l-distance to nudge a non-static obstacle (meters)");
DEFINE_double(lateral_ignore_buffer, 3.0,
              "If an obstacle's lateral distance is further away than this "
              "distance, ignore it");
DEFINE_double(max_stop_distance_obstacle, 10.0,
              "max stop distance from in-lane obstacle (meters)");
DEFINE_double(min_stop_distance_obstacle, 6.0,
              "min stop distance from in-lane obstacle (meters)");
DEFINE_double(follow_min_distance, 3.0,
              "min follow distance for vehicles/bicycles/moving objects");
DEFINE_double(yield_distance, 5.0,
              "min yield distance for vehicles/moving objects "
              "other than pedestrians/bicycles");
DEFINE_double(follow_time_buffer, 2.5,
              "time buffer in second to calculate the following distance.");
DEFINE_double(signal_expire_time_sec, 5.0,
              "traffic light signal info read expire time in sec");
DEFINE_string(destination_obstacle_id, "DEST",
              "obstacle id for converting destination to an obstacle");
DEFINE_double(destination_check_distance, 5.0,
              "if the distance between destination and ADC is less than this,"
              " it is considered to reach destination");
DEFINE_double(passed_destination_threshold, 0.05,
              "check adc whether has passed destination");
DEFINE_double(passed_referenceline_end_threshold, 1.5,
              "check adc whether has passed reference line end");
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
DEFINE_double(obstacle_lon_ignore_buffer, 0.0,
              "obstacle longitudinal ignore buffer (meters) for deciding "
              "backside obstacles");
DEFINE_double(static_obstacle_speed_threshold, 0.5,
              "The speed threshold to decide whether an obstacle is static "
              "or not.");

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

// QpSt optimizer
DEFINE_double(slowdown_profile_deceleration, -4.0,
              "The deceleration to generate slowdown profile. unit: m/s^2.");

DEFINE_double(speed_fallback_distance, 3.0,
              "Distance to speed fallback. unit: m.");
/// thread pool
DEFINE_bool(use_multi_thread_to_add_obstacles, false,
            "use multiple thread to add obstacles.");

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
              "/apollo/modules/planning/planning_component/conf/"
              "planner_open_space_config.pb.txt",
              "The open space planner configuration file");

DEFINE_bool(use_dual_variable_warm_start, true,
            "whether or not enable dual variable warm start ");

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

DEFINE_double(path_speed_osqp_setting_time_limit, 0.09,
              "Run time limit for OSQP in seconds.");
DEFINE_bool(enable_osqp_debug, false,
            "True to turn on OSQP verbose debug output in log.");

DEFINE_bool(export_chart, false, "export chart in planning");
DEFINE_bool(enable_record_debug, true,
            "True to enable record debug info in chart format");
DEFINE_bool(enable_print_curve, false,
            "True to enable print curve info into log");

DEFINE_double(
    default_front_clear_distance, 300.0,
    "default front clear distance value in case there is no obstacle around.");

DEFINE_bool(enable_rss_info, true, "enable rss_info in trajectory_pb");

DEFINE_bool(
    enable_planning_smoother, false,
    "True to enable planning smoother among different planning cycles.");
DEFINE_double(smoother_stop_distance, 10.0,
              "(unit: meter) for ADC stop, if it is close to the stop point "
              "within this threshold, current planning will be smoothed.");

DEFINE_bool(enable_parallel_hybrid_a, false,
            "True to enable hybrid a* parallel implementation.");

DEFINE_double(message_latency_threshold, 0.02, "Threshold for message delay");

DEFINE_uint64(trajectory_stitching_preserved_length, 20,
              "preserved points number in trajectory stitching");

DEFINE_double(side_pass_driving_width_l_buffer, 0.1,
              "(unit: meter) for side pass driving width l buffer");

DEFINE_bool(use_st_drivable_boundary, false,
            "True to use st_drivable boundary in speed planning");

DEFINE_bool(use_front_axe_center_in_path_planning, false,
            "If using front axe center in path planning, the path can be "
            "more agile.");

DEFINE_bool(planning_offline_learning, false,
            "offline learning. read record files and dump learning_data");
DEFINE_string(planning_data_dir, "/apollo/modules/planning/planning_base/data/",
              "Prefix of files to store feature data");
DEFINE_string(planning_offline_bags, "",
              "a list of source files or directories for offline mode. "
              "The items need to be separated by colon ':'. ");
DEFINE_int32(learning_data_obstacle_history_time_sec, 3.0,
             "time sec (second) of history trajectory points for a obstacle");
DEFINE_int32(learning_data_frame_num_per_file, 100,
             "number of learning_data_frame to write out in one data file.");
DEFINE_string(planning_birdview_img_feature_renderer_config_file,
              "/apollo/modules/planning/planning_component/conf/"
              "planning_semantic_map_config.pb.txt",
              "config file for renderer singleton");

DEFINE_int32(min_past_history_points_len, 0,
             "minimun past history points length for trainsition from "
             "rule-based planning to learning-based planning");

// Parameter for scenario or task process.
DEFINE_double(path_bounds_decider_resolution, 0.5,
              "The distance along s direction.");
DEFINE_double(path_bounds_horizon, 100, "path bounds horizon");
DEFINE_double(num_extra_tail_bound_point, 20, "The extra tail point number.");
DEFINE_bool(enable_pull_over_at_destination, false,
            "Whether to pull over at destination");
DEFINE_double(replan_lateral_distance_threshold, 0.5,
              "The lateral distance threshold of replan");
DEFINE_double(replan_longitudinal_distance_threshold, 2.5,
              "The longitudinal distance threshold of replan");
DEFINE_double(replan_time_threshold, 7.0, "The time threshold of replan");
DEFINE_double(trajectory_check_collision_time_step, 1.0,
              "checks collision time step for trajectory");

DEFINE_double(obstacle_pass_check_distance, 3.0,
              "at the distance, the obstacle goes around left or right"
              "consider ego and obstacle position");

DEFINE_bool(speed_optimize_fail_relax_velocity_constraint, true,
            "When the speed optimization fails,"
            "relax the speed upper bound constraint and try to optimize again");

DEFINE_bool(check_collision_freespace_obstacle_vertices, false,
            "Detect vehicle collisions with map projection obstacles");

DEFINE_double(
    near_stop_speed, 0.5,
    "Set deceleration as near_stop_deceleration when vehicle speed is smaller "
    "than this value before stopping so that control can keep up.");

DEFINE_double(
    near_stop_deceleration, -1.0,
    "Set deceleration as near_stop_deceleration when vehicle deceleration is "
    "smaller than this value before stopping so that control can keep up.");

DEFINE_double(
    reference_line_max_forward_heading_diff, 2.5,
    "max angle difference between the forward reference line and ego heading");

DEFINE_double(
    reference_line_max_backward_heading_diff, 3.1415926536,
    "max angle difference between the backward reference line and ego heading");

// park generic
DEFINE_double(sqp_obstacle_weight, 100, "sqp_obstacle_weight");

DEFINE_bool(enable_obstacle_potential_field, true,
            "enable_obstacle_potential_field");

DEFINE_double(open_space_delta_t, 1.0,
              "open space sample time for speed planning");

DEFINE_double(open_space_acc_weight, 1.0,
              "weight of open space acceleration in the cost function");

DEFINE_double(open_space_jerk_weight, 10.0,
              "weight of open space jerk in the cost function");

DEFINE_double(open_space_kappa_weight, 1000.0,
              "weight of open space kappa in the cost function");

DEFINE_double(open_space_reference_s_weight, 10.0,
              "weight of open space reference s in the cost function");

DEFINE_double(open_space_reference_v_weight, 10.0,
              "weight of open space reference v in the cost function");

DEFINE_double(open_space_max_forward_v, 2.0,
              "the max forward velocity for open space planning");

DEFINE_double(open_space_max_reverse_v, 2.0,
              "the max reverse velocity for open space planning");

DEFINE_double(open_space_max_forward_acc, 3.0,
              "the max forward acceleration for open space planning");

DEFINE_double(open_space_max_reverse_acc, 2.0,
              "the max reverse acceleration for open space planning");

DEFINE_double(open_space_max_jerk, 4.0, "the max jerk for open space planning");

// Nudge decisider
DEFINE_bool(enable_nudge_decider, true, "Enable use nudge decider");
DEFINE_double(max_nudge_check_distance_in_lk, 4.0,
              "Max nudge check distance in kane keep path boundary decider");
DEFINE_double(max_nudge_check_distance_in_lc, 3.0,
              "Max nudge check distance in lane change path boundary decider");
DEFINE_double(path_trim_destination_threshold, 20.0,
              "Distance threshold to destination in path trim operation");

// Edge follow buffer
DEFINE_double(edge_follow_buffer, 0.3, "Edge follow buffer");
DEFINE_bool(disable_perception_edge_info, false,
            "Disable perception edge info");
DEFINE_bool(enable_edge_follow_curvature_buffer, false,
            "Enable add curvature buffer in edge follow map");
DEFINE_bool(enable_smooth_edge_follow_buffer, false,
            "Enable smooth edge follow buffer");
DEFINE_bool(enable_print_edge_follow_log, false,
            "Enable print edge follow log");

DEFINE_double(normal_look_forward_short_distance, 180,
              "normal look forward short distance in reference line");
DEFINE_double(normal_look_forward_long_distance, 250,
              "normal look forward long distance in reference line");
DEFINE_double(normal_look_backward_distance, 50,
              "normal look backward distance in reference line");
DEFINE_double(edge_follow_look_forward_short_distance, 20,
              "edge follow look forward short distance in reference line");
DEFINE_double(edge_follow_look_forward_long_distance, 50,
              "edge follow look forward long distance in reference line");
DEFINE_double(edge_follow_look_backward_distance, 50,
              "edge follow look backward distance in reference line");

// lane escape
DEFINE_bool(enable_lane_escape, true, "Enable lane escape");

// zone cover
DEFINE_bool(use_zigzag_type_path_lane, false,
            "use arc type path when in zone cover stage");
DEFINE_bool(change_end_pose, false, "change end pose when in zone cover stage");
DEFINE_bool(calculate_next_trajectory, false,
            "calculate next trajectory when in zone cover stage");
DEFINE_bool(enable_non_drivablle_roi, false,
            "enable non drivable roi when in zone cover stage");

// for path easy solution
DEFINE_double(ego_front_slack_buffer, 0.25,
              "the lateral threshold to relaxing the solution space");
DEFINE_double(relax_ego_radius_buffer, 10.0, "relax_ego_radius buffer");
DEFINE_double(relax_path_s_threshold, 5.0, "relax path boundary s threshold");
DEFINE_bool(enable_corner_constraint, false,
            "enable use obstacle corner constraint");
DEFINE_bool(enable_expand_obs_corner, false, "enable expand_obs_corner");
DEFINE_double(expand_obs_corner_lon_buffer, 0.6,
              "expand obs corner lon buffer");
DEFINE_bool(enable_adc_vertex_constraint, false,
            "enable use adc vertex constraint");
DEFINE_double(obstacle_lon_end_buffer_park, 0.6,
              "obstacle longitudinal end buffer (meters) for deciding "
              "path boundaries");

DEFINE_bool(enable_control_interactive_replan, true,
            "enable replan with control interactive masg");

DEFINE_int32(close_range_obstacle_nudge_range_remain_farmes, 5,
             "remain the nudge range in frames");
DEFINE_double(close_range_obstacle_nudge_pedestrian_waiting_time, 2.0,
              "waiting time for pedestrians");

DEFINE_double(path_obs_ref_shift_distance, 0.8,
              "shift the reference to avoid obstacle for path");

DEFINE_double(driving_state_nudge_check_l, 0.2,
              "uudge lateral check threshold for the ego's driving status");