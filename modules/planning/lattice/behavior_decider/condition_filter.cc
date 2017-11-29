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

#include "modules/planning/lattice/behavior_decider/condition_filter.h"
#include "modules/common/math/linear_interpolation.h"

namespace apollo {
namespace planning {

using CriticalPointPair = std::pair<CriticalPoint, CriticalPoint>;

ConditionFilter::ConditionFilter(
    const std::array<double, 3>& init_s, const double speed_limit,
    const ADCNeighborhood& adc_neighborhood) :
    feasible_region_(init_s, speed_limit) {
  Init(adc_neighborhood);
}

void ConditionFilter::QueryFeasibleInterval(const double t,
      std::vector<CriticalBound>* critical_bounds) {
  // TODO(kechxu) Implement
}

bool ConditionFilter::QueryBlockInterval(const double t,
    const CriticalCondition& critical_condition,
    CriticalPointPair* block_interval) {
  double s_upper = apollo::common::math::lerp(
      critical_condition.upper_left().s(),
      critical_condition.upper_left().t(),
      critical_condition.upper_right().s(),
      critical_condition.upper_right().t(), t);
  double v_upper = apollo::common::math::lerp(
      critical_condition.upper_left().v(),
      critical_condition.upper_left().t(),
      critical_condition.upper_right().v(),
      critical_condition.upper_right().t(), t);
  double s_lower = apollo::common::math::lerp(
      critical_condition.bottom_left().s(),
      critical_condition.bottom_left().t(),
      critical_condition.bottom_right().s(),
      critical_condition.bottom_right().t(), t);
  double v_lower = apollo::common::math::lerp(
      critical_condition.bottom_left().v(),
      critical_condition.bottom_left().t(),
      critical_condition.bottom_right().v(),
      critical_condition.bottom_right().t(), t);
  block_interval->first.set_t(t);
  block_interval->first.set_s(s_lower);
  block_interval->first.set_v(v_lower);
  block_interval->second.set_t(t);
  block_interval->second.set_s(s_upper);
  block_interval->second.set_v(v_upper);
  
  return TimeWithin(t, critical_condition);
}

void ConditionFilter::QueryBlockIntervals(const double t,
    std::vector<CriticalPointPair>* block_intervals) {
  block_intervals->clear();
  for (const auto& critical_condition : critical_conditions_) {
    CriticalPointPair block_interval;
    if (!QueryBlockInterval(t, critical_condition, &block_interval)) {
      continue;
    }
    block_intervals->push_back(std::move(block_interval));
  }
  std::sort(block_intervals->begin(), block_intervals->end(),
      [](const CriticalPointPair& pair_1, const CriticalPointPair& pair_2) {
        return pair_1.first.s() < pair_2.first.s();
      });
}

void ConditionFilter::Init(const ADCNeighborhood& adc_neighborhood) {
  adc_neighborhood.GetCriticalConditions(&critical_conditions_);
}

bool ConditionFilter::TimeWithin(const double t,
    const CriticalCondition& critical_condition) {
  double t_start = critical_condition.bottom_left().t();
  double t_end = critical_condition.upper_right().t();
  return t_start <= t && t <= t_end;
}

}  // namespace planning
}  // namespace apollo
