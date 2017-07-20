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

DEFINE_string(planning_config_file,
              "modules/planning/conf/planning_config.pb.txt",
              "planning config file");

DEFINE_int32(planning_loop_rate, 5, "Loop rate for planning node");

DEFINE_string(rtk_trajectory_filename, "modules/planning/data/garage.csv",
              "Loop rate for planning node");

DEFINE_uint64(rtk_trajectory_backward, 10,
              "The number of points to be included in RTK trajectory "
              "before the matched point");

DEFINE_uint64(rtk_trajectory_forward, 800,
              "The number of points to be included in RTK trajectory "
              "after the matched point");

DEFINE_double(replanning_threshold, 2.0,
              "The threshold of position deviation "
              "that triggers the planner replanning");

DEFINE_double(trajectory_resolution, 0.01,
              "The time resolution of "
              "output trajectory.");

DEFINE_double(cycle_duration_in_sec, 0.002, "# of seconds per planning cycle.");
DEFINE_double(maximal_delay_sec, 0.005, "# of seconds for delay.");

DEFINE_int32(max_history_result, 10,
             "The maximal number of result in history.");

DEFINE_int32(max_frame_size, 30, "max size for prediction window");

DEFINE_int32(state_fail_threshold, 5,
             "This is continuous fail threshold for FSM change to fail state.");

DEFINE_int32(object_table_obstacle_capacity, 200,
             "The number of obstacles we hold in the object table.");
DEFINE_int32(object_table_map_object_capacity, 200,
             "The number of map objects we hold in the object table.");

DEFINE_bool(use_stitch, true, "Use trajectory stitch if possible.");
DEFINE_double(forward_predict_time, 0.2,
              "The forward predict time in each planning cycle.");
DEFINE_double(replan_distance_threshold, 5.0,
              "The distance threshold of replan");
DEFINE_double(replan_s_threshold, 5.0,
              "The s difference to real position threshold of replan");
DEFINE_double(replan_l_threshold, 2.5,
              "The l difference to real position threshold of replan");

DEFINE_double(planning_distance, 100, "Planning distance");

DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");
DEFINE_double(trajectory_time_resolution, 0.1,
              "Trajectory time resolution in planning");
DEFINE_double(output_trajectory_time_resolution, 0.05,
              "Trajectory time resolution when publish");

DEFINE_double(speed_lower_bound, 0.0, "The lowest speed allowed." );
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

DEFINE_double(default_active_set_eps_num, 1e-7,
              "qpOases wrapper error control numerator");
DEFINE_double(default_active_set_eps_den, 1e-7,
              "qpOases wrapper error control numerator");
DEFINE_double(default_active_set_eps_iter_ref, 1e-7,
              "qpOases wrapper error control numerator");
DEFINE_bool(default_enable_active_set_debug_info, false,
            "Enable print information");

DEFINE_double(stgraph_default_point_cost, 1e10,
              "The default stgraph point cost.");
DEFINE_double(stgraph_max_acceleration_divide_factor_level_1, 2.0,
              "The divide factor for max acceleration at level 1.");
DEFINE_double(stgraph_max_deceleration_divide_factor_level_1, 3.0,
              "The divide factor for max deceleration at level 1.");
DEFINE_double(stgraph_max_deceleration_divide_factor_level_2, 2.0,
              "The divide factor for max deceleration at level 2.");
