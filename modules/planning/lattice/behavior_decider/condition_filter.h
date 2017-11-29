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

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_CONDITION_FILTER_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_CONDITION_FILTER_H_

#include <vector>
#include <array>

#include "modules/planning/lattice/behavior_decider/adc_neighborhood.h"
#include "modules/planning/lattice/behavior_decider/feasible_region.h"
#include "modules/planning/proto/lattice_structure.pb.h"

namespace apollo {
namespace planning {

class ConditionFilter {
 public:
  ConditionFilter(
      const std::array<double, 3>& init_s, const double speed_limit,
      const ADCNeighborhood& adc_neighborhood);

  void QuerySampleBounds(const double t,
      std::vector<SampleBound>* sample_bounds);

  void QueryBlockIntervals(const double t,
      std::vector<std::pair<CriticalPoint, CriticalPoint>>* block_intervals);

 private:
  void Init(const ADCNeighborhood& adc_neighborhood);

  // Return true only if t is within the range of time slot,
  // but will output block interval anyway(maybe be extension)
  bool QueryBlockInterval(const double t,
      const CriticalCondition& critical_condition,
      std::pair<CriticalPoint, CriticalPoint>* block_interval);

  bool TimeWithin(const double t,
      const CriticalCondition& critical_condition);

 private:
  FeasibleRegion feasible_region_;
  std::vector<CriticalCondition> critical_conditions_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_CONDITION_FILTER_H_
