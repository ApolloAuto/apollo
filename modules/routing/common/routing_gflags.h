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

#ifndef BAIDU_ADU_ROUTING_COMMON_ROUTING_GFLAGS_H
#define BAIDU_ADU_ROUTING_COMMON_ROUTING_GFLAGS_H

#include "gflags/gflags.h"

DECLARE_string(node_name);
DECLARE_string(node_namespace);
DECLARE_string(signal_probe_service);

DECLARE_bool(enable_old_routing);
DECLARE_string(route_topic_for_broadcast);
DECLARE_bool(use_road_id);

DECLARE_string(graph_dir);
DECLARE_string(graph_file_name);

DECLARE_string(rosparam_name_routing_init_status);

DECLARE_bool(enable_debug_mode);
DECLARE_string(debug_route_path);
DECLARE_string(debug_passage_region_path);

#endif  // BAIDU_ADU_ROUTING_COMMON_ROUTING_GFLAGS_H
