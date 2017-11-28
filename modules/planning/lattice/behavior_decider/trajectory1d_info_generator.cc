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
/**
 * @file trajectory1d_info_generator.h
 **/

#include "modules/planning/lattice/behavior_decider/trajectory1d_info_generator.h"

namespace apollo {
namespace planning {

Trajectory1dInfoGenerator::Trajectory1dInfoGenerator(
    PlanningTarget planning_target,
    std::array<double, 3> init_lon_condition,
    std::array<double, 3> init_lat_condition) {
  planning_target_ = std::move(planning_target);
  init_lon_condition_ = std::move(init_lon_condition);
  init_lat_condition_ = std::move(init_lat_condition);
}

std::vector<Trajectory1dInfo>
Trajectory1dInfoGenerator::GenerateLongitudinalTrajectoryInfo() const {

}

std::vector<Trajectory1dInfo>
Trajectory1dInfoGenerator::GenerateLateralTrajectoryInfo() const {
  auto end_states = end_condition_sampler_.SampleLatEndConditions(init_lat_condition_);

  std::vector<Trajectory1dInfo> trajectory1d_info;
  for (const auto& end_state : end_states) {

  }
  return trajectory1d_info;
}

} // namespace planning
} // namespace apollo
