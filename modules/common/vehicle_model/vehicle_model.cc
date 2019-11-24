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

#include "modules/common/vehicle_model/vehicle_model.h"

#include "cyber/common/file.h"
#include "modules/common/configs/config_gflags.h"

namespace apollo {
namespace common {

void VehicleModel::RearCenteredKinematicBicycleModel(
    const VehicleModelConfig& vehicle_model_config,
    const double predicted_time_horizon, const VehicleState& cur_vehicle_state,
    VehicleState* predicted_vehicle_state) {
  // Kinematic bicycle model centered at rear axis center by Euler forward
  // discretization
  // Assume constant control command and constant z axis position
  CHECK_GT(predicted_time_horizon, 0.0);
  double dt = vehicle_model_config.rc_kinematic_bicycle_model().dt();
  double cur_x = cur_vehicle_state.x();
  double cur_y = cur_vehicle_state.y();
  double cur_z = cur_vehicle_state.z();
  double cur_phi = cur_vehicle_state.heading();
  double cur_v = cur_vehicle_state.linear_velocity();
  double cur_a = cur_vehicle_state.linear_acceleration();
  double next_x = cur_x;
  double next_y = cur_y;
  double next_phi = cur_phi;
  double next_v = cur_v;
  if (dt >= predicted_time_horizon) {
    dt = predicted_time_horizon;
  }

  double countdown_time = predicted_time_horizon;
  bool finish_flag = false;
  static constexpr double kepsilon = 1e-8;
  while (countdown_time > kepsilon && !finish_flag) {
    countdown_time -= dt;
    if (countdown_time < kepsilon) {
      dt = countdown_time + dt;
      finish_flag = true;
    }
    double intermidiate_phi =
        cur_phi + 0.5 * dt * cur_v * cur_vehicle_state.kappa();
    next_phi =
        cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * cur_vehicle_state.kappa();
    next_x =
        cur_x + dt * (cur_v + 0.5 * dt * cur_a) * std::cos(intermidiate_phi);
    next_y =
        cur_y + dt * (cur_v + 0.5 * dt * cur_a) * std::sin(intermidiate_phi);

    next_v = cur_v + dt * cur_a;
    cur_x = next_x;
    cur_y = next_y;
    cur_phi = next_phi;
    cur_v = next_v;
  }

  predicted_vehicle_state->set_x(next_x);
  predicted_vehicle_state->set_y(next_y);
  predicted_vehicle_state->set_z(cur_z);
  predicted_vehicle_state->set_heading(next_phi);
  predicted_vehicle_state->set_kappa(cur_vehicle_state.kappa());
  predicted_vehicle_state->set_linear_velocity(next_v);
  predicted_vehicle_state->set_linear_acceleration(
      cur_vehicle_state.linear_acceleration());
}

VehicleState VehicleModel::Predict(const double predicted_time_horizon,
                                   const VehicleState& cur_vehicle_state) {
  VehicleModelConfig vehicle_model_config;

  CHECK(cyber::common::GetProtoFromFile(FLAGS_vehicle_model_config_filename,
                                        &vehicle_model_config))
      << "Failed to load vehicle model config file "
      << FLAGS_vehicle_model_config_filename;

  // Some models not supported for now
  CHECK(vehicle_model_config.model_type() !=
        VehicleModelConfig::COM_CENTERED_DYNAMIC_BICYCLE_MODEL);
  CHECK(vehicle_model_config.model_type() != VehicleModelConfig::MLP_MODEL);

  VehicleState predicted_vehicle_state;
  if (vehicle_model_config.model_type() ==
      VehicleModelConfig::REAR_CENTERED_KINEMATIC_BICYCLE_MODEL) {
    RearCenteredKinematicBicycleModel(vehicle_model_config,
                                      predicted_time_horizon, cur_vehicle_state,
                                      &predicted_vehicle_state);
  }

  return predicted_vehicle_state;
}

}  // namespace common
}  // namespace apollo
