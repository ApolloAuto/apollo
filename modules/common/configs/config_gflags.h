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
#include <string>

#include "absl/flags/flag.h"
#include "gflags/gflags.h"

// The directory which contains a group of related maps, such as base_map,
// sim_map, routing_topo_grapth, etc.

ABSL_DECLARE_FLAG(std::string, map_dir);
ABSL_DECLARE_FLAG(int32_t, local_utm_zone_id);
ABSL_DECLARE_FLAG(std::string, test_base_map_filename);
ABSL_DECLARE_FLAG(std::string, base_map_filename);

DECLARE_string(sim_map_filename);
DECLARE_string(routing_map_filename);
DECLARE_string(end_way_point_filename);
DECLARE_string(speed_control_filename);

DECLARE_double(look_forward_time_sec);

DECLARE_string(vehicle_config_path);
DECLARE_string(vehicle_model_config_filename);

DECLARE_bool(use_cyber_time);

DECLARE_string(localization_tf2_frame_id);
DECLARE_string(localization_tf2_child_frame_id);
DECLARE_bool(use_navigation_mode);
DECLARE_string(navigation_mode_end_way_point_file);

DECLARE_double(half_vehicle_width);

DECLARE_bool(use_sim_time);

DECLARE_bool(reverse_heading_vehicle_state);

DECLARE_bool(state_transform_to_com_reverse);
DECLARE_bool(state_transform_to_com_drive);
DECLARE_bool(multithread_run);
