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

#ifndef MODULES_DREAMVIEW_BACKEND_COMMON_DREAMVIEW_GFLAGS_H_
#define MODULES_DREAMVIEW_BACKEND_COMMON_DREAMVIEW_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(dreamview_module_name);

DECLARE_string(dreamview_adapter_config_filename);

DECLARE_string(hmi_config_filename);

DECLARE_string(static_file_dir);

DECLARE_string(server_ports);

DECLARE_bool(enable_sim_control);

DECLARE_bool(routing_from_file);

DECLARE_string(routing_response_file);

DECLARE_string(websocket_timeout_ms);

DECLARE_string(ssl_certificate);

DECLARE_double(sim_map_radius);

DECLARE_bool(ignore_planning_debug_data);

#endif  // MODULES_DREAMVIEW_BACKEND_COMMON_DREAMVIEW_GFLAGS_H_
