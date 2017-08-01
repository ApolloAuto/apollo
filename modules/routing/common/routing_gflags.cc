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

#include "common/routing_gflags.h"

DEFINE_string(node_name, "routing", "the name for this node");
DEFINE_string(node_namespace, "routing", "the namespace for this node");
DEFINE_string(signal_probe_service, "/routing/routing_signal",
              "the service name for signal probe");

DEFINE_bool(enable_old_routing, true, "enable old routing");
DEFINE_string(route_topic_for_broadcast, "/routing/routing",
              "the default routing topic");
DEFINE_bool(use_road_id, true, "enable use road id to cut routing result");

DEFINE_string(graph_dir, "", "the default directory of topology graph data");
DEFINE_string(graph_file_name, "routing_map.bin",
              "the default file name of topology graph data");

DEFINE_string(rosparam_name_routing_init_status, "/pnc/routing_initialized",
              "true if routing init ok, used by ADS test");

DEFINE_bool(enable_debug_mode, true, "enable debug mode");
DEFINE_string(debug_route_path, "",
              "the default path of routing result debug file");
DEFINE_string(debug_passage_region_path, "",
              "the default path of passage region debug file");
