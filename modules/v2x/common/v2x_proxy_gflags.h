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

/**
 * @file v2x_proxy_gflags.h
 * @brief The gflags used by v2x proxy module
 */

#pragma once

#include "gflags/gflags.h"

namespace apollo {
namespace v2x {

DECLARE_string(grpc_client_host);
DECLARE_string(grpc_server_host);
DECLARE_string(grpc_client_port);
DECLARE_string(grpc_server_port);
DECLARE_int64(x2v_trafficlight_timer_frequency);
DECLARE_bool(debug_flag);
DECLARE_int64(v2x_carstatus_timer_frequency);
DECLARE_string(hdmap_file_name);
DECLARE_double(traffic_light_distance);
}  // namespace v2x
}  // namespace apollo
