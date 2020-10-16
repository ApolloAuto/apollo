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

// IP & PORT
DECLARE_string(grpc_client_host);
DECLARE_string(grpc_server_host);
DECLARE_string(grpc_client_port);
DECLARE_string(grpc_server_port);
DECLARE_string(grpc_debug_server_port);

// Other Flags
DECLARE_int64(x2v_traffic_light_timer_frequency);
DECLARE_int64(v2x_car_status_timer_frequency);
DECLARE_double(traffic_light_distance);
DECLARE_double(heading_difference);
DECLARE_int64(list_size);
DECLARE_int64(msg_timeout);
DECLARE_int64(sim_sending_num);
DECLARE_bool(use_nearest_flag);
DECLARE_int64(spat_period);
DECLARE_double(check_time);
DECLARE_int64(rsu_whitelist_period);
DECLARE_string(rsu_whitelist_name);

}  // namespace v2x
}  // namespace apollo
