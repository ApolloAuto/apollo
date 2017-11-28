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

#include <array>
#include <utility>
#include <vector>

namespace apollo {
namespace planning {

// Input: planning objective, vehicle kinematic/dynamic constraints,
// Output: sampled ending 1 dimensional states with corresponding time duration.
class EndConditionSampler {
 public:
  EndConditionSampler();

  virtual ~EndConditionSampler() = default;

  std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndConditions(
      const std::array<double, 3>& init_d) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForCruising(const std::array<double, 3>& init_s,
                                    const double ref_cruise_speed) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForFollowing(const std::array<double, 3>& init_s,
                                     const double ref_target_position,
                                     const double ref_target_speed) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForStopping(const std::array<double, 3>& init_s,
                                    const double ref_stop_position) const;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_END_CONDITION_SAMPLER_H_
