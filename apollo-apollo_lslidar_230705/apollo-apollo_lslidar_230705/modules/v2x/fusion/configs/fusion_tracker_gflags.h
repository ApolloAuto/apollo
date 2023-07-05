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
 * File: fusion_gflags.h
 * Brief: Global flags definition.
 */

#pragma once

#include "gflags/gflags.h"

namespace apollo {
namespace v2x {
namespace ft {

// config manager
DECLARE_string(config_path);
DECLARE_string(v2x_module_name);
DECLARE_string(v2x_fusion_obstacles_topic);
DECLARE_bool(use_v2x);
// fusion
DECLARE_string(fusion_conf_file);
// app
DECLARE_string(app_conf_file);

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
