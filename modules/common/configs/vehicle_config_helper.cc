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

#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace common {
namespace config {

VehicleConfig VehicleConfigHelper::vehicle_config_;

VehicleConfigHelper::VehicleConfigHelper() {}

void VehicleConfigHelper::Init() { Init(FLAGS_vehicle_config_path); }

void VehicleConfigHelper::Init(const std::string& config_file) {
  VehicleConfig params;
  CHECK(apollo::common::util::GetProtoFromFile(config_file, &params))
      << "Unable to parse adapter config file " << config_file;
  Init(params);
}

void VehicleConfigHelper::Init(const VehicleConfig& vehicle_params) {
  vehicle_config_ = vehicle_params;
}

const VehicleConfig& VehicleConfigHelper::GetConfig() {
  return vehicle_config_;
}

}  // namespace config
}  // namespace common
}  // namespace apollo
