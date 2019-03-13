/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

// sensor_manager
DEFINE_string(obs_sensor_intrinsic_path,
              "/apollo/modules/perception/data/params",
              "The intrinsics/extrinsics dir.");

DEFINE_string(obs_sensor_meta_path,
              "/apollo/modules/perception/production"
              "/data/perception/common/sensor_meta.pt",
              "The SensorManager config file.");

DEFINE_bool(enable_base_object_pool, false, "Enable base object pool.");

// config_manager
DEFINE_string(config_manager_path, "./conf", "The ModelConfig config paths.");
DEFINE_string(work_root, "", "Project work root direcotry.");

}  // namespace perception
}  // namespace apollo
