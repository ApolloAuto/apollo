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
DECLARE_uint64(rtk_trajectory_backward);
DECLARE_uint64(rtk_trajectory_forward);
DECLARE_double(replanning_threshold);
DECLARE_double(trajectory_resolution);

DECLARE_double(cycle_duration_in_sec);
DECLARE_double(maximal_delay_sec);
DECLARE_int32(max_history_result);
DECLARE_int32(max_frame_size);
// Finite State Machine
DECLARE_int32(state_fail_threshold);

// Object Table
DECLARE_int32(object_table_obstacle_capacity);
DECLARE_int32(object_table_map_object_capacity);

// parameters for trajectory stitching and reinit planning starting point.
DECLARE_bool(use_stitch);
DECLARE_double(forward_predict_time);
DECLARE_double(replan_distance_threshold);
DECLARE_double(replan_s_threshold);
DECLARE_double(replan_l_threshold);

// parameters for task manager
DECLARE_string(reference_line_decider);

// parameters for path planning
DECLARE_double(planning_distance);

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

// Math : active set solver
DECLARE_double(default_active_set_eps_num);
DECLARE_double(default_active_set_eps_den);
DECLARE_double(default_active_set_eps_iter_ref);
DECLARE_bool(default_enable_active_set_debug_info);

// STGraph
DECLARE_double(stgraph_default_point_cost);
DECLARE_double(stgraph_max_acceleration_divide_factor_level_1);
DECLARE_double(stgraph_max_deceleration_divide_factor_level_1);
DECLARE_double(stgraph_max_deceleration_divide_factor_level_2);


#endif /* MODULES_PLANNING_COMMON_PLANNING_GFLAGS_H_ */
