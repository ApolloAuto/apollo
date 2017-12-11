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
 * @file end_condition_sampler.h
 **/

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_END_CONDITION_SAMPLER_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_END_CONDITION_SAMPLER_H_

#include "modules/planning/proto/lattice_structure.pb.h"

#include <array>
#include <utility>
#include <vector>

#include "modules/planning/lattice/behavior_decider/feasible_region.h"

namespace apollo {
namespace planning {

// Input: planning objective, vehicle kinematic/dynamic constraints,
// Output: sampled ending 1 dimensional states with corresponding time duration.
class EndConditionSampler {
 public:
  EndConditionSampler(const std::array<double, 3>& init_s,
      const std::array<double, 3>& init_d, const double s_dot_limit);

  virtual ~EndConditionSampler();

  std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndConditions() const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForCruising(const double ref_cruise_speed) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForFollowing(const double ref_target_position,
                                     const double ref_target_speed) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForStopping(const double ref_stop_position) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForPathTimeBounds(
      const PlanningTarget& planning_target) const;

 private:
  std::array<double, 3> init_s_;

  std::array<double, 3> init_d_;

  double s_dot_limit_;

  FeasibleRegion* ptr_feasible_region_;

};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_END_CONDITION_SAMPLER_H_
