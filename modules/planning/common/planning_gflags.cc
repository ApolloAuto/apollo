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
DEFINE_int32(planning_loop_rate, 5, "Loop rate for planning node");

DEFINE_string(rtk_trajectory_filename, "modules/planning/data/garage.csv",
              "Loop rate for planning node");

DEFINE_uint64(backward_trajectory_point_num, 10,
              "The number of points to be included in planning trajectory "
              "before the matched point");

DEFINE_uint64(rtk_trajectory_forward, 800,
              "The number of points to be included in RTK trajectory "
              "after the matched point");

DEFINE_double(trajectory_resolution, 0.01,
              "The time resolution of "
              "output trajectory.");

DEFINE_double(
    look_backward_distance, 60,
    "look backward this distance when creating reference line from routing");

DEFINE_double(
    look_forward_distance, 70,
    "look forward this distance when creating reference line from routing");

DEFINE_int32(max_history_frame_num, 5, "The maximum history frame number");

DEFINE_double(replan_distance_threshold, 5.0,
              "The distance threshold of replan");

DEFINE_double(default_reference_line_width, 4.0,
              "Default reference line width");

DEFINE_double(planning_speed_upper_limit, 10.0, "Maximum speed in planning.");

DEFINE_double(planning_distance, 100, "Planning distance");

DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");
DEFINE_double(trajectory_time_resolution, 0.1,
              "Trajectory time resolution in planning");
DEFINE_double(output_trajectory_time_resolution, 0.05,
              "Trajectory time resolution when publish");

DEFINE_double(speed_lower_bound, 0.0, "The lowest speed allowed.");
DEFINE_double(speed_upper_bound, 40.0, "The highest speed allowed.");

DEFINE_double(longitudinal_acceleration_lower_bound, -4.5,
              "The lowest longitudinal acceleration allowed.");
DEFINE_double(longitudinal_acceleration_upper_bound, 4.0,
              "The highest longitudinal acceleration allowed.");

DEFINE_double(lateral_acceleration_bound, 4.5,
              "Bound of lateral acceleration; symmetric for left and right");
DEFINE_double(lateral_jerk_bound, 4.0,
              "Bound of lateral jerk; symmetric for left and right");

DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(longitudinal_jerk_upper_bound, 4.0,
              "The upper bound of longitudinal jerk.");

DEFINE_double(kappa_bound, 0.23, "The bound for vehicle curvature");

// ST Boundary
DEFINE_double(st_max_s, 80, "the maximum s of st boundary");
DEFINE_double(st_max_t, 10, "the maximum t of st boundary");

// Decision Part
DEFINE_double(static_obstacle_speed_threshold, 1.0,
              "obstacles are considered as static obstacle if its speed is "
              "less than this value (m/s)");
DEFINE_bool(enable_nudge_decision, false, "enable nudge decision");
DEFINE_double(static_decision_ignore_s_range, 3.0,
              "threshold for judging nudge in dp path computing decision");
DEFINE_double(static_decision_nudge_l_buffer, 0.5, "l buffer for nudge");
DEFINE_double(stop_distance_obstacle, 5.0,
              "stop distance from in-lane obstacle (meters)");
DEFINE_double(destination_adjust_distance_buffer, 1.0,
              "distance buffer when adjusting destination stop line");
DEFINE_double(min_driving_width, 2.5,
              "minimum road width(meters) for adc to drive through");
DEFINE_double(nudge_distance_obstacle, 0.3,
              "minimum distance to nudge a obstacle (meters)");
DEFINE_double(follow_min_distance, 10,
              "min follow distance for vehicles/bicycles/moving objects");
DEFINE_double(stop_line_min_distance, 0.0,
              "min distance (meters) to stop line for a valid stop");

DEFINE_string(destination_obstacle_id, "DEST",
              "obstacle id for converting destination to an obstacle");
DEFINE_int32(virtual_obstacle_perception_id, -1,
             "virtual obstacle perception id(a negative int)");
DEFINE_double(virtual_stop_wall_length, 0.1,
              "virtual stop wall length (meters)");
DEFINE_double(virtual_stop_wall_width, 3.7, "virtual stop wall width (meters)");
DEFINE_double(virtual_stop_wall_height, 2.0,
              "virtual stop wall height (meters)");

// Prediction Part
DEFINE_double(prediction_total_time, 5.0, "Total prediction time");
DEFINE_bool(align_prediction_time, true,
            "enable align prediction data based planning time");

// Trajectory
DEFINE_bool(enable_rule_layer, true,
            "enable rule for trajectory before model computation");

// Traffic decision

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

DEFINE_double(planning_speed_limit, 10.0,
              "planning speed limit (m/s)");
