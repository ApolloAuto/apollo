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

DECLARE_string(dreamview_module_name);

DECLARE_bool(dreamview_profiling_mode);

DECLARE_int32(dreamview_profiling_duration);

DECLARE_string(static_file_dir);

DECLARE_string(server_ports);

DECLARE_bool(routing_from_file);

DECLARE_string(routing_response_file);

DECLARE_string(websocket_timeout_ms);

DECLARE_string(ssl_certificate);

DECLARE_double(sim_map_radius);

DECLARE_int32(dreamview_worker_num);

DECLARE_bool(enable_update_size_check);

DECLARE_uint32(max_update_size);

DECLARE_bool(sim_world_with_routing_path);

DECLARE_string(request_timeout_ms);

DECLARE_double(voxel_filter_size);

DECLARE_double(voxel_filter_height);

DECLARE_double(system_status_lifetime_seconds);

DECLARE_string(lidar_height_yaml);

DECLARE_int32(monitor_msg_pending_queue_size);

DECLARE_string(default_data_collection_config_path);
