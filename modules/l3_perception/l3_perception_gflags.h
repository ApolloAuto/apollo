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

#ifndef MODULES_L3_PERCEPTION_L3_PERCEPTION_GFLAGS_H_
#define MODULES_L3_PERCEPTION_L3_PERCEPTION_GFLAGS_H_

#include "gflags/gflags.h"

DECLARE_string(node_namespace);
DECLARE_string(node_name);
DECLARE_string(hmi_name);

DECLARE_string(adapter_config_filename);

DECLARE_double(l3_perception_freq);

DECLARE_double(mobileye_pos_adjust);
DECLARE_double(delphi_esr_pos_adjust);

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

DECLARE_double(filter_y_distance);
DECLARE_double(fusion_x_distance);
DECLARE_double(fusion_y_distance);
#endif
