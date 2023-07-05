/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/map/tools/map_datachecker/server/worker_gflags.h"

namespace apollo {
namespace hdmap {

// Server address
DEFINE_string(map_datachecker_host, "127.0.0.1", "the grpc server host");
DEFINE_string(map_datachecker_port, "50100", "the grpc server port");
// Cybertron topics
DEFINE_string(topic_bestgnsspos, "/apollo/sensor/gnss/best_pose", "");
// configure file
DEFINE_string(conf_json,
              "/apollo/modules/map/tools/map_datachecker/server/conf/"
              "map-datachecker.json",
              "configure file");

}  // namespace hdmap
}  // namespace apollo
