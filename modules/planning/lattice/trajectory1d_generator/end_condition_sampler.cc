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
    const std::array<double, 3>& init_d, const double s_dot_limit) :
    init_s_(init_s), init_d_(init_d), s_dot_limit_(s_dot_limit) {
  ptr_feasible_region_ = new FeasibleRegion(init_s, s_dot_limit);
}

EndConditionSampler::~EndConditionSampler() {
  delete ptr_feasible_region_;
}

std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLatEndConditions() const {
  std::vector<std::pair<std::array<double, 3>, double>> end_d_conditions;
  std::array<double, 5> end_d_candidates = {0.0, -0.25, -0.5, 0.25, 0.5};
  std::array<double, 5> end_s_candidates = {20.0, 30.0, 40.0, 50.0, 60.0};

  for (const auto& s : end_s_candidates) {
    for (const auto& d : end_d_candidates) {
      std::array<double, 3> end_d_state = {d, 0.0, 0.0};
      end_d_conditions.emplace_back(end_d_state, s);
    }
  }
  return end_d_conditions;
}

std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLonEndConditionsForCruising(const double ref_cruise_speed) const {
  // time interval is one second plus the last one 0.01
  constexpr std::size_t num_time_section = 9;
  std::array<double, num_time_section> time_sections;
  for (std::size_t i = 0; i + 1 < num_time_section; ++i) {
    time_sections[i] = planned_trajectory_time - i;
  }
  time_sections[num_time_section - 1] = 0.01;

  // velocity samples consists of 10 equally distributed samples plus ego's
  // current velocity
  constexpr std::size_t num_velocity_section = 11;

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
EndConditionSampler::SampleLonEndConditionsForFollowing(const double obstacle_position,
    const double obstacle_velocity) const {
  std::vector<std::pair<std::array<double, 3>, double>> end_s_conditions;
  constexpr std::size_t num_time_section = 9;
  std::array<double, num_time_section> time_sections;
  for (std::size_t i = 0; i + 1 < num_time_section; ++i) {
    time_sections[i] = planned_trajectory_time - i;
  }
  time_sections[num_time_section - 1] = 0.01;

  // following three-second rule
  // TODO(yajia, kecheng, liyun): fix the case when s on reference line is
  // small enough to be close to init_s[0]
  double ref_position = obstacle_position - obstacle_velocity * 3.0;
  constexpr std::size_t num_position_section = 5;
  std::array<double, num_position_section> s_offsets = {-10.0, -7.5, -5.0, -2.5,
                                                        0.0};

  for (const auto& s_offset : s_offsets) {
    std::array<double, 3> end_s;
    end_s[0] = std::max(ref_position + s_offset, init_s_[0]);
    end_s[1] = obstacle_velocity;
    end_s[2] = 0.0;

    for (const auto& t : time_sections) {
      end_s_conditions.emplace_back(end_s, t);
    }
  }
  return end_s_conditions;
}

std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLonEndConditionsForStopping(const double ref_stop_position) const {
  constexpr size_t num_time_section = 9;
  std::array<double, num_time_section> time_sections;
  for (size_t i = 0; i + 1 < num_time_section; ++i) {
    time_sections[i] = planned_trajectory_time - i;
  }
  time_sections[num_time_section - 1] = 0.01;

  std::vector<std::pair<std::array<double, 3>, double>> end_s_conditions;
  std::array<double, 4> s_offsets = {-1.5, -1.0, -0.5, 0.0};
  for (const auto& s_offset : s_offsets) {
    std::array<double, 3> s = {
        std::max(s_offset + ref_stop_position, init_s_[0]), 0.0, 0.0};

    for (const auto& t : time_sections) {
      end_s_conditions.push_back({s, t});
    }
  }
  return end_s_conditions;
}

std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLonEndConditionsForPathTimeBounds(
    const std::vector<SampleBound>& sample_bounds) const {

  std::vector<std::pair<std::array<double, 3>, double>> end_s_conditions;

  constexpr std::size_t num_s_section = 5;
  constexpr std::size_t num_s_dot_section = 5;
  for (const SampleBound& sample_bound : sample_bounds) {
    double s_interval = (sample_bound.s_upper() - sample_bound.s_lower())
        / (num_s_section - 1);
    std::array<double, num_s_section> s_samples;
    for (std::size_t i = 0; i < num_s_section; ++i) {
      s_samples[i] = sample_bound.s_lower() + i * s_interval;
    }

    double s_dot_interval = (sample_bound.v_upper() - sample_bound.v_lower())
        / (num_s_dot_section - 1);
    std::array<double, num_s_dot_section> s_dot_samples;
    for (std::size_t i = 0; i < num_s_dot_section; ++i) {
      s_dot_samples[i] = sample_bound.v_lower() + i * s_dot_interval;
    }

    for (const auto s : s_samples) {
      for (const auto s_dot : s_dot_samples) {
        std::array<double, 3> end_state = { s, s_dot, 0.0 };
        end_s_conditions.push_back( { end_state, sample_bound.t() });
      }
    }
  }
  return end_s_conditions;
}

}  // namespace planning
}  // namespace apollo
