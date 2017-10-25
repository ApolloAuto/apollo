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
DEFINE_int32(planning_loop_rate, 10, "Loop rate for planning node");

DEFINE_string(planning_adapter_config_filename,
              "modules/planning/conf/adapter.conf",
              "The adapter configuration file");

DEFINE_string(rtk_trajectory_filename, "modules/planning/data/garage.csv",
              "Loop rate for planning node");

DEFINE_uint64(rtk_trajectory_forward, 800,
              "The number of points to be included in RTK trajectory "
              "after the matched point");

DEFINE_double(rtk_trajectory_resolution, 0.01,
              "The time resolution of output trajectory for rtk planner.");

DEFINE_bool(publish_estop, false, "publish estop decision in planning");
DEFINE_bool(enable_trajectory_stitcher, true, "enable stitching trajectory");

DEFINE_double(
    look_backward_distance, 30,
    "look backward this distance when creating reference line from routing");

DEFINE_double(
    look_forward_distance, 250,
    "look forward this distance when creating reference line from routing");

DEFINE_double(look_forward_min_distance, 100,
              "minimal look forward this distance when creating reference line "
              "from routing");
DEFINE_double(look_forward_time_sec, 8,
              "look forward time times adc speed to calculate this distance "
              "when creating reference line from routing");

DEFINE_bool(enable_smooth_reference_line, true,
            "enable smooth the map reference line");

DEFINE_int32(max_history_frame_num, 5, "The maximum history frame number");

DEFINE_double(max_collision_distance, 0.1,
              "considered as collision if distance (meters) is smaller than or "
              "equal to this (meters)");

DEFINE_double(replan_distance_threshold, 5.0,
              "The distance threshold of replan");

DEFINE_bool(enable_reference_line_provider_thread, false,
            "Enable reference line provider thread.");

DEFINE_double(default_reference_line_width, 4.0,
              "Default reference line width");

DEFINE_double(smoothed_reference_line_max_diff, 1.0,
              "Maximum position difference between the smoothed and the raw "
              "reference lines.");

DEFINE_double(planning_upper_speed_limit, 31.3,
              "Maximum speed (m/s) in planning.");

DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");
DEFINE_double(trajectory_time_resolution, 0.1,
              "Trajectory time resolution in planning");
DEFINE_double(output_trajectory_time_resolution, 0.01,
              "Trajectory time resolution when publish for EM planner");

DEFINE_bool(enable_trajectory_check, false,
            "Enable sanity check for planning trajectory.");

DEFINE_double(speed_lower_bound, -0.02, "The lowest speed allowed.");
DEFINE_double(speed_upper_bound, 40.0, "The highest speed allowed.");

DEFINE_double(longitudinal_acceleration_lower_bound, -4.5,
              "The lowest longitudinal acceleration allowed.");
DEFINE_double(longitudinal_acceleration_upper_bound, 4.0,
              "The highest longitudinal acceleration allowed.");

DEFINE_double(lateral_acceleration_bound, 4.0, "lateral acceleration bound");

DEFINE_double(lateral_jerk_bound, 4.0,
              "Bound of lateral jerk; symmetric for left and right");

DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(longitudinal_jerk_upper_bound, 4.0,
              "The upper bound of longitudinal jerk.");

DEFINE_double(kappa_bound, 0.20, "The bound for vehicle curvature");
DEFINE_double(dkappa_bound, 0.02,
              "The bound for vehicle curvature change rate");

// ST Boundary
DEFINE_double(st_max_s, 100, "the maximum s of st boundary");
DEFINE_double(st_max_t, 8, "the maximum t of st boundary");

// Decision Part
DEFINE_double(static_obstacle_speed_threshold, 1.0,
              "obstacles are considered as static obstacle if its speed is "
              "less than this value (m/s)");
DEFINE_bool(enable_nudge_decision, true, "enable nudge decision");
DEFINE_double(static_decision_nudge_l_buffer, 0.5, "l buffer for nudge");
DEFINE_double(lateral_ignore_buffer, 2.0,
              "If an obstacle's lateral distance is further away than this "
              "distance, ignore it");
DEFINE_double(stop_distance_obstacle, 10.0,
              "stop distance from in-lane obstacle (meters)");
DEFINE_double(stop_distance_destination, 3.0,
              "stop distance from destination line");
DEFINE_double(nudge_distance_obstacle, 0.3,
              "minimum distance to nudge a obstacle (meters)");
DEFINE_double(follow_min_distance, 10,
              "min follow distance for vehicles/bicycles/moving objects");
DEFINE_double(
    follow_time_buffer, 4.0,
    "follow time buffer (in second) to calculate the following distance.");

DEFINE_string(destination_obstacle_id, "DEST",
              "obstacle id for converting destination to an obstacle");
DEFINE_double(virtual_stop_wall_length, 0.1,
              "virtual stop wall length (meters)");
DEFINE_double(virtual_stop_wall_width, 3.7, "virtual stop wall width (meters)");
DEFINE_double(virtual_stop_wall_height, 2.0,
              "virtual stop wall height (meters)");

// Prediction Part
DEFINE_double(prediction_total_time, 5.0, "Total prediction time");
DEFINE_bool(align_prediction_time, false,
            "enable align prediction data based planning time");

// Trajectory
DEFINE_bool(enable_rule_layer, true,
            "enable rule for trajectory before model computation");

// Traffic decision
/// common
DEFINE_double(stop_max_distance_buffer, 4.0,
              "distance buffer of passing stop line");
DEFINE_double(stop_min_speed, 0.1, "min speed for computing stop");
DEFINE_double(stop_max_deceleration, 6.0, "max deceleration");
/// Clear Zone
DEFINE_string(clear_zone_virtual_object_id_prefix, "CZ_",
              "prefix for converting clear zone id to virtual object id");
/// traffic light
DEFINE_string(signal_light_virtual_object_id_prefix, "SL_",
              "prefix for converting signal id to virtual object id");
DEFINE_double(max_deacceleration_for_yellow_light_stop, 2.0,
              "treat yellow light as red when deceleration (abstract value"
              " in m/s^2) is less than this threshold; otherwise treated"
              " as green light");
/// crosswalk
DEFINE_bool(enable_crosswalk, false, "enable crosswalk");
DEFINE_string(crosswalk_virtual_object_id_prefix, "CW_",
              "prefix for converting crosswalk id to virtual object id");
DEFINE_double(crosswalk_expand_distance, 2.0,
              "crosswalk expand distance(meter) "
              "for pedestrian/bicycle detection");
DEFINE_double(crosswalk_strick_l_distance, 4.0,
              "strick stop rule within this l_distance");
DEFINE_double(crosswalk_loose_l_distance, 5.0,
              "loose stop rule beyond this l_distance");

// planning config file
DEFINE_string(planning_config_file,
              "modules/planning/conf/planning_config.pb.txt",
              "planning config file");

DEFINE_int32(trajectory_point_num_for_debug, 10,
             "number of output trajectory points for debugging");

DEFINE_double(decision_valid_stop_range, 0.5,
              "The valid stop range in decision.");
DEFINE_bool(enable_record_debug, true,
            "True to enable record debug into debug protobuf.");
DEFINE_bool(enable_prediction, true, "True to enable prediction input.");

// QpSt optimizer
DEFINE_bool(enable_slowdown_profile_generator, true,
            "True to enable slowdown speed profile generator.");
DEFINE_double(slowdown_speed_threshold, 8.0,
              "Only generator slowdown profile when adc speed is lower than "
              "this threshold. unit : m/s.");
DEFINE_double(slowdown_profile_deceleration, -1.0,
              "The deceleration to generate slowdown profile. unit: m/s^2.");
DEFINE_bool(enable_follow_accel_constraint, true,
            "Enable follow acceleration constraint.");

// SQP solver
DEFINE_bool(enable_sqp_solver, true, "True to enable SQP solver.");
