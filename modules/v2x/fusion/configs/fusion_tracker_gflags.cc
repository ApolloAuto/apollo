/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

/**
 * File: fusion_gflags.cpp
 * Brief: Global flags definition.
 */

#include "modules/v2x/fusion/configs/fusion_tracker_gflags.h"

namespace apollo {
namespace v2x {
namespace ft {

// config manager
DEFINE_string(config_path, "/apollo/modules/v2x/data",
              "ADU shared data path, including maps, routings...");
DEFINE_string(v2x_module_name, "v2x_fusion", "name");
DEFINE_string(v2x_fusion_obstacles_topic, "/apollo/msf/obstacles",
              "perception obstacle topic name");
DEFINE_bool(use_v2x, false, "use v2x");
// fusion
DEFINE_string(fusion_conf_file, "fusion_params.pt", "the roi conf path");
// app
DEFINE_string(app_conf_file, "app_config.pt", "the inputs conf path");

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
