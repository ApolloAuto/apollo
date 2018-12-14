/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_COMMON_NAVI_GENERATOR_GFLAGS_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_COMMON_NAVI_GENERATOR_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(navi_generator_module_name);

DECLARE_bool(navi_generator_profiling_mode);

DECLARE_double(navi_generator_profiling_duration);

DECLARE_string(navi_generator_adapter_config_filename);

DECLARE_string(hmi_config_filename);

DECLARE_string(static_file_dir);

DECLARE_string(server_ports);

DECLARE_bool(routing_from_file);

DECLARE_string(routing_response_file);

DECLARE_string(websocket_timeout_ms);

DECLARE_string(ssl_certificate);

DECLARE_double(sim_map_radius);

DECLARE_int32(navi_generator_worker_num);

DECLARE_bool(enable_update_size_check);

DECLARE_uint32(max_update_size);

DECLARE_bool(navi_generator_with_routing_path);

DECLARE_string(request_timeout_ms);

DECLARE_double(voxel_filter_size);

DECLARE_double(voxel_filter_height);

DECLARE_double(system_status_lifetime_seconds);

DECLARE_string(trajectory_util_config_filename);

DECLARE_uint32(quadtile_zoom_level);

#endif
