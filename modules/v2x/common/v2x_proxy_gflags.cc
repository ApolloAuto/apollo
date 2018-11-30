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
 * @file v2x_proxy_gflags.cc
 * @brief The gflags used by v2x proxy module
 */

#include "modules/v2x/common/v2x_proxy_gflags.h"

namespace apollo {
namespace v2x {

DEFINE_string(grpc_client_host, "192.168.10.123", "grpc client host ip");
DEFINE_string(grpc_server_host, "192.168.10.6", "grpc server host ip");
DEFINE_string(grpc_client_port, "50100", "grpc client port num");
DEFINE_string(grpc_server_port, "50101", "grpc server port num");
DEFINE_int64(x2v_trafficlight_timer_frequency, 10,
             "x2v trafficlight timer frequency");
DEFINE_bool(debug_flag, false, "debug flag");
DEFINE_int64(v2x_carstatus_timer_frequency, 10,
             "v2x carstatus timer frequency");
DEFINE_string(hdmap_file_name,
              "/apollo/modules/map/data/sunnyvale_big_loop/base_map.bin",
              "hdmap file name");
DEFINE_double(traffic_light_distance, 1000.0, "traffic light distance");
}  // namespace v2x
}  // namespace apollo
