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

#include "modules/routing/common/routing_gflags.h"

DEFINE_string(node_name, "routing", "the name for this node");
DEFINE_string(node_namespace, "routing", "the namespace for this node");

DEFINE_string(adapter_config_filename, "modules/routing/conf/adapter.conf",
              "The adapter config filename");

DEFINE_bool(use_road_id, true, "enable use road id to cut routing result");
DEFINE_double(min_length_for_lane_change, 10.0,
              "min length for lane change, in creater, in meter");
DEFINE_bool(enable_change_lane_in_result, false,
            "contain change lane operator in result");

DEFINE_bool(enable_debug_mode, true, "enable debug mode");
DEFINE_string(debug_route_path, "",
              "the default path of routing result debug file");
DEFINE_string(debug_passage_region_path, "",
              "the default path of passage region debug file");

DEFINE_double(base_speed, 4.167, "base speed for node creator, in m/s");
DEFINE_double(left_turn_penalty, 50,
              "left turn penalty for node creater, in meter");
DEFINE_double(right_turn_penalty, 50,
              "right turn penalty for node creater, in meter");
DEFINE_double(uturn_penalty, 50,
              "left turn penalty for node creater, in meter");
DEFINE_double(change_penalty, 50, "change penalty for edge creater, in meter");
DEFINE_double(base_changing_length, 50,
              "base change length penalty for edge creater, in meter");
