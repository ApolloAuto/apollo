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
DECLARE_double(trajectory_resolution);
DECLARE_double(look_backward_distance);
DECLARE_double(look_forward_distance);
DECLARE_bool(enable_smooth_reference_line);
DECLARE_double(max_collision_distance);

DECLARE_int32(max_history_frame_num);

// parameters for trajectory stitching and reinit planning starting point.
DECLARE_double(replan_distance_threshold);

// parameter for reference line
DECLARE_double(default_reference_line_width);
DECLARE_double(planning_upper_speed_limit);

// parameters for trajectory planning
DECLARE_double(trajectory_time_length);
DECLARE_double(trajectory_time_resolution);
DECLARE_double(output_trajectory_time_resolution);

// parameters for trajectory sanity check
DECLARE_bool(enable_trajectory_check);
DECLARE_double(speed_lower_bound);
DECLARE_double(speed_upper_bound);

DECLARE_double(longitudinal_acceleration_lower_bound);
DECLARE_double(longitudinal_acceleration_upper_bound);

DECLARE_double(lateral_acceleration_bound);
DECLARE_double(lateral_jerk_bound);

DECLARE_double(longitudinal_jerk_lower_bound);
DECLARE_double(longitudinal_jerk_upper_bound);

DECLARE_double(kappa_bound);

// STBoundary
DECLARE_double(st_max_s);
DECLARE_double(st_max_t);

// Decision Part
DECLARE_double(static_obstacle_speed_threshold);
DECLARE_bool(enable_nudge_decision);
DECLARE_double(static_decision_ignore_s_range);
DECLARE_double(static_decision_nudge_l_buffer);
DECLARE_double(stop_distance_obstacle);
DECLARE_double(stop_distance_destination);
DECLARE_double(min_driving_width);
DECLARE_double(nudge_distance_obstacle);
DECLARE_double(follow_min_distance);
DECLARE_double(stop_line_min_distance);

DECLARE_string(destination_obstacle_id);
DECLARE_int32(virtual_obstacle_perception_id);
DECLARE_double(virtual_stop_wall_length);
DECLARE_double(virtual_stop_wall_width);
DECLARE_double(virtual_stop_wall_height);

DECLARE_double(prediction_total_time);
DECLARE_bool(align_prediction_time);
DECLARE_int32(trajectory_point_num_for_debug);
DECLARE_double(decision_valid_stop_range);

DECLARE_bool(enable_record_debug);
DECLARE_bool(enable_prediction);

// QpSt optimizer
DECLARE_bool(enable_slowdown_profile_generator);
DECLARE_double(slowdown_speed_threshold);
DECLARE_double(slowdown_profile_deceleration);
DECLARE_double(qp_st_low_velocity_threshold);

#endif  // MODULES_PLANNING_COMMON_PLANNING_GFLAGS_H_
