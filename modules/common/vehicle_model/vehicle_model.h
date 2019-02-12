/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_model/proto/vehicle_model_config.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"

namespace apollo {
namespace common {

class VehicleModel {
 public:
  VehicleModel() = delete;

  static VehicleState Predict(const double predicted_time_horizon,
                              const VehicleState& cur_vehicle_state);

 private:
  static void RearCenteredKinematicBicycleModel(
      const VehicleModelConfig& vehicle_model_config,
      const VehicleParam& vehicle_param, const double predicted_time_horizon,
      const VehicleState& cur_vehicle_state,
      VehicleState* predicted_vehicle_state);
};

}  // namespace common
}  // namespace apollo
