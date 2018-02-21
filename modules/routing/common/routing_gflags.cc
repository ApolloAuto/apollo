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

DEFINE_string(routing_conf_file, "modules/routing/conf/routing_config.pb.txt",
              "default routing conf data file");

DEFINE_string(routing_node_name, "routing", "the name for this node");

DEFINE_string(routing_adapter_config_filename,
              "modules/routing/conf/adapter.conf",
              "The adapter config filename");

DEFINE_double(min_length_for_lane_change, 30.48,
              "meters, which is 100 feet.  Minimum distance needs to travel on "
              "a lane before making a lane change. Recommended by "
              "https://www.oregonlaws.org/ors/811.375");

DEFINE_bool(enable_change_lane_in_result, true,
            "contain change lane operator in result");
