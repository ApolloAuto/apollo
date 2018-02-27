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
 * @file
 **/

#include "modules/planning/lattice/trajectory_generation/end_condition_sampler.h"

#include <algorithm>
#include <utility>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

EndConditionSampler::EndConditionSampler(
    const std::array<double, 3>& init_s,
    const std::array<double, 3>& init_d,
    const double s_dot_limit,
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_querier)
    : init_s_(init_s), init_d_(init_d),
      feasible_region_(init_s, s_dot_limit),
      ptr_path_time_graph_(ptr_path_time_graph),
      ptr_prediction_querier_(ptr_prediction_querier) {
}

std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLatEndConditions() const {
  std::vector<std::pair<std::array<double, 3>, double>> end_d_conditions;
  std::array<double, 5> end_d_candidates = {0.0, -0.25, 0.25, -0.5, 0.5};
  std::array<double, 4> end_s_candidates = {10.0, 20.0, 30.0, 40.0};

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
    time_sections[i] = FLAGS_trajectory_time_length - i;
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
      if (end_s[1] > feasible_region_.VUpper(time) ||
          end_s[1] < feasible_region_.VLower(time)) {
        continue;
      }
      end_s_conditions.emplace_back(end_s, time);
    }
    std::array<double, 3> end_s = {0.0, init_s_[1], 0.0};
    end_s_conditions.emplace_back(end_s, time);
  }
  return end_s_conditions;
}


std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLonEndConditionsForStopping(
    const double ref_stop_point) const {
  // time interval is one second plus the last one 0.01
  constexpr std::size_t num_time_section = 9;
  std::array<double, num_time_section> time_sections;
  for (std::size_t i = 0; i + 1 < num_time_section; ++i) {
    time_sections[i] = FLAGS_trajectory_time_length - i;
  }
  time_sections[num_time_section - 1] = 0.01;

  constexpr std::size_t num_stop_section = 3;
  std::array<double, num_stop_section> s_offsets;
  for (std::size_t i = 0; i < num_stop_section; ++i) {
    s_offsets[i] = -1 * i;
  }

  std::vector<std::pair<std::array<double, 3>, double>> end_s_conditions;
  for (const auto& time : time_sections) {
    for (const auto& s_offset : s_offsets) {
      std::array<double, 3> end_s;
      end_s[0] = std::max(init_s_[0], ref_stop_point + s_offset);
      end_s[1] = 0.0;
      end_s[2] = 0.0;
      end_s_conditions.emplace_back(end_s, time);
    }
  }
  return end_s_conditions;
}

std::vector<std::pair<std::array<double, 3>, double>>
EndConditionSampler::SampleLonEndConditionsForPathTimePoints() const {
  std::vector<SamplePoint> sample_points = QueryPathTimeObstacleSamplePoints();
  std::vector<std::pair<std::array<double, 3>, double>> end_s_conditions;
  for (const SamplePoint& sample_point : sample_points) {
    double s = sample_point.path_time_point().s();
    double v = sample_point.ref_v();
    double t = sample_point.path_time_point().t();
    if (s > feasible_region_.SUpper(t) ||
        s < feasible_region_.SLower(t)) {
      continue;
    }
    std::array<double, 3> end_state = {s, v, 0.0};
    end_s_conditions.emplace_back(end_state, t);
  }
  return end_s_conditions;
}

std::vector<SamplePoint>
EndConditionSampler::QueryPathTimeObstacleSamplePoints() const {
  std::vector<SamplePoint> sample_points;
  for (const auto& path_time_obstacle :
       ptr_path_time_graph_->GetPathTimeObstacles()) {
    std::string obstacle_id = path_time_obstacle.obstacle_id();

    std::vector<PathTimePoint> overtake_path_time_points =
        ptr_path_time_graph_->GetObstacleSurroundingPoints(
            obstacle_id, FLAGS_lattice_epsilon, FLAGS_time_min_density);
    for (const PathTimePoint& path_time_point : overtake_path_time_points) {
      double v = ptr_prediction_querier_->ProjectVelocityAlongReferenceLine(
          obstacle_id, path_time_point.s(), path_time_point.t());
      SamplePoint sample_point;
      sample_point.mutable_path_time_point()->CopyFrom(path_time_point);
      sample_point.mutable_path_time_point()->set_s(FLAGS_default_lon_buffer);
      sample_point.set_ref_v(v);
      sample_points.push_back(std::move(sample_point));
    }

    std::vector<PathTimePoint> follow_path_time_points =
        ptr_path_time_graph_->GetObstacleSurroundingPoints(
            obstacle_id, -FLAGS_lattice_epsilon, FLAGS_time_min_density);
    for (const PathTimePoint& path_time_point : follow_path_time_points) {
      double v = ptr_prediction_querier_->ProjectVelocityAlongReferenceLine(
          obstacle_id, path_time_point.s(), path_time_point.t());
      SamplePoint sample_point;
      sample_point.mutable_path_time_point()->CopyFrom(path_time_point);
      sample_point.mutable_path_time_point()->set_s(-FLAGS_default_lon_buffer);
      sample_point.set_ref_v(v);
      sample_points.push_back(std::move(sample_point));
    }
  }
  return sample_points;
}

}  // namespace planning
}  // namespace apollo
