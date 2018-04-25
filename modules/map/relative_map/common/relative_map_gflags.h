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

#ifndef MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_GFLAGS_H_
#define MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(relative_map_adapter_config_filename);
DECLARE_string(relative_map_config_filename);
DECLARE_int32(relative_map_loop_rate);
DECLARE_double(max_len_from_navigation_line);
DECLARE_double(min_len_for_navigation_lane);
DECLARE_double(max_len_for_navigation_lane);
DECLARE_double(ratio_navigation_lane_len_to_speed);
DECLARE_double(max_distance_to_navigation_line);
DECLARE_double(min_view_range_to_use_lane_marker);
DECLARE_double(min_lane_half_width);
DECLARE_double(max_lane_half_width);

#endif  // MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_GFLAGS_H_
