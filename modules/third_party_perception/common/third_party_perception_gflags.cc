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

#include "modules/third_party_perception/common/third_party_perception_gflags.h"

DEFINE_string(node_namespace, "/apollo/third_party_perception", "Global node namespace");
DEFINE_string(node_name, "third_party_perception", "The chassis module name in proto");
DEFINE_string(hmi_name, "third_party_perception", "Module name in HMI");

DEFINE_string(adapter_config_filename, "", "Path for adapter configuration");

DEFINE_double(third_party_perception_freq, 100, "third party perception timer frequency.");
DEFINE_bool(enable_mobileye, true, "switch to turn on/off mobileye obstacles");
DEFINE_bool(enable_delphi_esr, true, "switch to turn on/off delphi_esr obstacles");

// flags to calibrate mobileye, radar and localization 
DEFINE_double(mobileye_pos_adjust, 3.0,
              "adjust mobileye objects's position due to distance between "
              "mobileye and gps.");
DEFINE_double(
    delphi_esr_pos_adjust, 3.0,
    "adjust radar objects's position due to distance between radar and gps.");

// flags to create fake bounding box
DEFINE_double(default_car_length, 5.0, "default car length for bounding box.");
DEFINE_double(default_truck_length, 10.0,
              "default truck length for bounding box.");
DEFINE_double(default_bike_length, 2.0,
              "default bike length for bounding box.");
DEFINE_double(default_ped_length, 0.5, "default ped length for bounding box.");
DEFINE_double(default_unknown_length, 5.0,
              "default unknown length for bounding box.");
DEFINE_double(default_car_width, 3.0, "default car width for bounding box.");
DEFINE_double(default_truck_width, 5.0,
              "default truck width for bounding box.");
DEFINE_double(default_bike_width, 1.0, "default bike width for bounding box.");
DEFINE_double(default_ped_width, 0.5, "default ped width for bounding box.");
DEFINE_double(default_unknown_width, 3.0,
              "default unknown width for bounding box.");

// flags to filter radar obstacles
DEFINE_double(
    filter_y_distance, 7.5,
    "fiter the radar objects far away from the main vehicle on y-axis.");
DEFINE_int32(keep_delphi_esr_frames, 5, "number of delphi esr frames to keep");
