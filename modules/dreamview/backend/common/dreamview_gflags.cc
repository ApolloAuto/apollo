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

#include "modules/dreamview/backend/common/dreamview_gflags.h"

DEFINE_string(dreamview_module_name, "dreamview", "dreamview module name");

// TODO(siyangy): Use garage map temporarily
DEFINE_string(dreamview_map, "modules/map/data/base_map.txt",
              "File path for the map file to be rendered in frontend.");

DEFINE_string(static_file_dir, "modules/dreamview/frontend/dist",
              "The path to the dreamview distribution directory. The default "
              "value points to built-in version from the Apollo project.");

DEFINE_int32(server_port, 8888, "The port of backend webserver");

DEFINE_bool(
    enable_sim_control, false,
    "Whether to enable SimControl to publish localiztion and chassis message.");

DEFINE_string(routing_request_file, "modules/map/data/garage_routing.pb.txt",
              "File path of the routing request that SimControl will read the "
              "start point from. If this is absent, SimControl will directly "
              "take the RoutingResult directly from ROS to determine the "
              "start point.");
