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

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_TRAJECTORY1D_INFO_GENERATOR_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_TRAJECTORY1D_INFO_GENERATOR_H_

#include <array>
#include <vector>

#include "modules/planning/lattice/trajectory1d/trajectory1d_info.h"

#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/lattice/behavior_decider/end_condition_sampler.h"

namespace apollo {
namespace planning {

// Interface class from behavior planning to trajectory planning
class Trajectory1dInfoGenerator {
 public:
  Trajectory1dInfoGenerator(PlanningTarget planning_target,
      std::array<double, 3> init_lon_condition,
      std::array<double, 3> init_lat_condition);

  virtual ~Trajectory1dInfoGenerator() = default;

  std::vector<Trajectory1dInfo> GenerateLongitudinalTrajectoryInfo() const;

  std::vector<Trajectory1dInfo> GenerateLateralTrajectoryInfo() const;

 private:
  EndConditionSampler end_condition_sampler_;

  PlanningTarget planning_target_;

  std::array<double, 3> init_lon_condition_;

  std::array<double, 3> init_lat_condition_;


};

} // namespace planning
} // namespace apollo

#endif /* MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_TRAJECTORY1D_INFO_GENERATOR_H_ */
