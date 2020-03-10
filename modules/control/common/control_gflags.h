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

#pragma once

#include "gflags/gflags.h"

// data file
DECLARE_string(control_conf_file);
// control common conf file
DECLARE_string(control_common_conf_file);
// mpc controller conf file
DECLARE_string(mpc_controller_conf_file);
// lateral controller conf file
DECLARE_string(lateral_controller_conf_file);
// longitudinal controller conf file
DECLARE_string(longitudinal_controller_conf_file);
// calibration table
DECLARE_string(calibration_table_file);

DECLARE_double(control_test_duration);

DECLARE_bool(enable_csv_debug);

// temporary gflag for test purpose
DECLARE_bool(enable_speed_station_preview);

DECLARE_string(control_node_name);

DECLARE_string(preprocessor_submodule_name);
DECLARE_string(mpc_controller_submodule_name);
DECLARE_string(postprocessor_submodule_name);
DECLARE_string(lat_lon_controller_submodule_name);

DECLARE_bool(is_control_test_mode);
DECLARE_bool(use_preview_speed_for_table);

DECLARE_double(steer_angle_rate);
DECLARE_bool(enable_gain_scheduler);
DECLARE_bool(set_steer_limit);
DECLARE_bool(enable_slope_offset);

DECLARE_double(lock_steer_speed);

DECLARE_bool(enable_navigation_mode_error_filter);
DECLARE_bool(enable_navigation_mode_position_update);

DECLARE_int32(chassis_pending_queue_size);
DECLARE_int32(planning_pending_queue_size);
DECLARE_int32(localization_pending_queue_size);
DECLARE_int32(pad_msg_pending_queue_size);

DECLARE_bool(reverse_heading_control);

DECLARE_bool(trajectory_transform_to_com_reverse);
DECLARE_bool(trajectory_transform_to_com_drive);

DECLARE_bool(enable_maximum_steer_rate_limit);

DECLARE_bool(query_time_nearest_point_only);
DECLARE_bool(query_forward_time_point_only);

DECLARE_bool(enable_feedback_augment_on_high_speed);

DECLARE_bool(enable_gear_drive_negative_speed_protection);

DECLARE_bool(use_osqp_solver);

DECLARE_bool(use_control_submodules);

DECLARE_bool(use_system_time_in_control);
