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

#include <algorithm>
#include <utility>

#include "modules/planning/lattice/trajectory1d_generator/end_condition_sampler.h"

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/util/lattice_params.h"

namespace apollo {
namespace planning {

EndConditionSampler::EndConditionSampler(const std::array<double, 3>& init_s,
                                         const std::array<double, 3>& init_d,
                                         const double s_dot_limit)
    : init_s_(init_s), init_d_(init_d), s_dot_limit_(s_dot_limit) {
  ptr_feasible_region_ = new FeasibleRegion(init_s, s_dot_limit);
}

EndConditionSampler::~EndConditionSampler() { delete ptr_feasible_region_; }

std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLatEndConditions() const {
  std::vector<std::pair<std::array<double, 3>, double>> end_d_conditions;
  // std::array<double, 5> end_d_candidates = {0.0, -0.25, -0.5, 0.25, 0.5};
  // std::array<double, 5> end_s_candidates = {20.0, 30.0, 40.0, 50.0, 60.0};
  std::array<double, 3> end_d_candidates = {0.0, -0.25, 0.25};
  std::array<double, 3> end_s_candidates = {20.0, 35.0, 50.0};

  for (const auto& s : end_s_candidates) {
    for (const auto& d : end_d_candidates) {
      std::array<double, 3> end_d_state = {d, 0.0, 0.0};
      end_d_conditions.emplace_back(end_d_state, s);
    }
  }
  return end_d_conditions;
}

std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLonEndConditionsForCruising(
    const double ref_cruise_speed) const {
  // time interval is one second plus the last one 0.01
  constexpr std::size_t num_time_section = 9;
  std::array<double, num_time_section> time_sections;
  for (std::size_t i = 0; i + 1 < num_time_section; ++i) {
    time_sections[i] = planned_trajectory_time - i;
  }
  time_sections[num_time_section - 1] = 0.01;

  // velocity samples consists of 10 equally distributed samples plus ego's
  // current velocity
  constexpr std::size_t num_velocity_section = 6;

  double velocity_upper = std::max(ref_cruise_speed, init_s_[1]);
  double velocity_lower = 0.0;
  double velocity_seg =
      (velocity_upper - velocity_lower) / (num_velocity_section - 2);

  std::vector<std::pair<std::array<double, 3>, double>> end_s_conditions;
  for (const auto& time : time_sections) {
    for (std::size_t i = 0; i + 1 < num_velocity_section; ++i) {  // velocity
      std::array<double, 3> end_s;
      end_s[0] = 0.0;  // this will not be used in QuarticPolynomial
      end_s[1] = velocity_seg * i;
      end_s[2] = 0.0;
      end_s_conditions.emplace_back(end_s, time);
    }
    std::array<double, 3> end_s = {0.0, init_s_[1], 0.0};
    end_s_conditions.emplace_back(end_s, time);
  }
  return end_s_conditions;
}

std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLonEndConditionsForPathTimeBounds(
    const PlanningTarget& planning_target) const {
  std::vector<std::pair<std::array<double, 3>, double>> end_s_conditions;

  /**
  constexpr std::size_t num_s_section = 5;
  for (const SampleBound& sample_bound : planning_target.sample_bound()) {
    double s_interval = (sample_bound.s_upper() - sample_bound.s_lower())
        / (num_s_section - 1);
    std::array<double, num_s_section> s_samples;
    for (std::size_t i = 0; i < num_s_section; ++i) {
      s_samples[i] = sample_bound.s_lower() + i * s_interval;
    }

    // no longer using sample s_dot
    // get the computed s_dot from sample bound
    double s_dot = sample_bound.v_reference();

    for (const auto s : s_samples) {
        std::array<double, 3> end_state = { s, s_dot, 0.0 };
        end_s_conditions.push_back( { end_state, sample_bound.t() });
    }
  }
  **/

  constexpr std::size_t num_s_section = 4;
  std::array<double, num_s_section> s_offsets = {0.0, -1.0, -2.0, -3.0};

  for (const SampleBound& sample_bound : planning_target.sample_bound()) {
    // no longer using sample s_dot
    // get the computed s_dot from sample bound
    double s_dot = sample_bound.v_reference();

    std::vector<double> s_samples;
    for (const auto& s_offset : s_offsets) {
      s_samples.push_back(std::max(
          init_s_[0], sample_bound.s_upper() - s_dot * 3.0 + s_offset));
    }

    for (const auto s : s_samples) {
      std::array<double, 3> end_state = {s, s_dot, 0.0};
      end_s_conditions.push_back({end_state, sample_bound.t()});
    }
  }

  return end_s_conditions;
}

}  // namespace planning
}  // namespace apollo
