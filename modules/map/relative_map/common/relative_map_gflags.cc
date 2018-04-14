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

DEFINE_double(max_len_from_navigation_line, 250.0,
              "max navigation path length from navigation line");

DEFINE_double(min_len_for_navigation_lane, 150.0,
              "min generated navigation lane length");

DEFINE_double(max_len_for_navigation_lane, 250.0,
              "max generated navigation lane length");

DEFINE_double(ratio_navigation_lane_len_to_speed, 8.0,
              "navigation lane length to adv speed ratio");

DEFINE_double(max_distance_to_navigation_line, 6.0,
              "max distance to navigation line in navigation mode");

DEFINE_double(min_view_range_to_use_lane_marker, 0.5,
              "min view range to use lane_marker");

DEFINE_double(min_lane_half_width, 1.5, "min lane half width in meters");
DEFINE_double(max_lane_half_width, 2.0, "max lane half width in meters");
