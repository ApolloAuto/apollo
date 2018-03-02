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

#include "modules/map/relative_map/common/relative_map_gflags.h"

DEFINE_string(relative_map_adapter_config_filename,
              "modules/map/relative_map/conf/adapter.conf",
              "gflags conf file for relative map");
DEFINE_string(relative_map_config_filename,
              "modules/map/relative_map/conf/relative_map_config.pb.txt",
              "Relative map configuration file");

DEFINE_int32(relative_map_loop_rate, 10, "Loop rate for relative_map node");

DEFINE_double(max_len_from_navigation_line, 100.0,
              "max navigation path length from navigation line");

DEFINE_double(min_len_for_navigation_lane, 60.0,
              "min generated navigation lane length");

DEFINE_double(ratio_navigation_lane_len_to_speed, 6.0,
              "navigation lane length to adv speed ratio");

DEFINE_bool(
    enable_navigation_line, true,
    "True to consider navigation line info into generate navigation path");
