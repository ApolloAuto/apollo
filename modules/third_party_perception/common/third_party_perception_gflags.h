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

#pragma once

#include "gflags/gflags.h"

DECLARE_string(third_party_perception_node_name);

DECLARE_string(adapter_config_filename);

DECLARE_double(third_party_perception_freq);
DECLARE_bool(enable_mobileye);
DECLARE_bool(enable_radar);

// flags to calibrate mobileye, radar and localization
DECLARE_double(mobileye_pos_adjust);
DECLARE_double(radar_pos_adjust);

// object id offset
DECLARE_int32(mobileye_id_offset);
DECLARE_int32(radar_id_offset);

// flags to create fake bounding box
DECLARE_double(default_car_length);
DECLARE_double(default_truck_length);
DECLARE_double(default_bike_length);
DECLARE_double(default_ped_length);
DECLARE_double(default_unknown_length);
DECLARE_double(default_car_width);
DECLARE_double(default_truck_width);
DECLARE_double(default_bike_width);
DECLARE_double(default_ped_width);
DECLARE_double(default_unknown_width);
DECLARE_double(default_height);

// flags to filter radar obstacles
DECLARE_double(filter_y_distance);
DECLARE_double(movable_speed_threshold);
DECLARE_double(movable_heading_threshold);
DECLARE_int32(movable_frames_count_threshold);
DECLARE_int32(keep_radar_frames);

// TODO(QiL) : remove this temporary gflags
DECLARE_bool(use_conti_radar);
DECLARE_double(max_mobileye_obstacle_length);
DECLARE_double(max_mobileye_obstacle_width);
DECLARE_bool(overwrite_mobileye_theta);
