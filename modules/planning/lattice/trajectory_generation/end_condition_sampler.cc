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

using State = std::array<double, 3>;
using Condition = std::pair<State, double>;

EndConditionSampler::EndConditionSampler(
    const State& init_s, const State& init_d,
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_querier)
    : init_s_(init_s),
      init_d_(init_d),
      feasible_region_(init_s),
      ptr_path_time_graph_(std::move(ptr_path_time_graph)),
      ptr_prediction_querier_(std::move(ptr_prediction_querier)) {}

std::vector<Condition> EndConditionSampler::SampleLatEndConditions() const {
  std::vector<Condition> end_d_conditions;
  std::array<double, 3> end_d_candidates = {0.0, -0.5, 0.5};
  std::array<double, 4> end_s_candidates = {10.0, 20.0, 40.0, 80.0};

  for (const auto& s : end_s_candidates) {
    for (const auto& d : end_d_candidates) {
      State end_d_state = {d, 0.0, 0.0};
      end_d_conditions.emplace_back(end_d_state, s);
    }
  }
  return end_d_conditions;
}

std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForCruising(
    const double ref_cruise_speed) const {
  CHECK_GT(FLAGS_num_velocity_sample, 1);

  // time interval is one second plus the last one 0.01
  constexpr std::size_t num_of_time_samples = 9;
  std::array<double, num_of_time_samples> time_samples;
  for (std::size_t i = 0; i + 1 < num_of_time_samples; ++i) {
    time_samples[i] = FLAGS_trajectory_time_length - i;
  }
  time_samples[num_of_time_samples - 1] = FLAGS_polynomial_minimal_param;

  std::vector<Condition> end_s_conditions;
  for (const auto& time : time_samples) {
    double v_upper = std::min(feasible_region_.VUpper(time), ref_cruise_speed);
    double v_lower = feasible_region_.VLower(time);

    State lower_end_s = {0.0, v_lower, 0.0};
    end_s_conditions.emplace_back(lower_end_s, time);

    State upper_end_s = {0.0, v_upper, 0.0};
    end_s_conditions.emplace_back(upper_end_s, time);

    double v_range = v_upper - v_lower;
    // Number of sample velocities
    std::size_t num_of_mid_points = std::min(
        static_cast<std::size_t>(FLAGS_num_velocity_sample - 2),
        static_cast<std::size_t>(v_range / FLAGS_min_velocity_sample_gap));

    if (num_of_mid_points > 0) {
      double velocity_seg = v_range / (num_of_mid_points + 1);
      for (std::size_t i = 1; i <= num_of_mid_points; ++i) {
        State end_s = {0.0, v_lower + velocity_seg * i, 0.0};
        end_s_conditions.emplace_back(end_s, time);
      }
    }
  }
  return end_s_conditions;
}

std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForStopping(
    const double ref_stop_point) const {
  // time interval is one second plus the last one 0.01
  constexpr std::size_t num_time_section = 9;
  std::array<double, num_time_section> time_sections;
  for (std::size_t i = 0; i + 1 < num_time_section; ++i) {
    time_sections[i] = FLAGS_trajectory_time_length - i;
  }
  time_sections[num_time_section - 1] = FLAGS_polynomial_minimal_param;

  std::vector<Condition> end_s_conditions;
  for (const auto& time : time_sections) {
    State end_s = {std::max(init_s_[0], ref_stop_point), 0.0, 0.0};
    end_s_conditions.emplace_back(end_s, time);
  }
  return end_s_conditions;
}

std::vector<Condition>
EndConditionSampler::SampleLonEndConditionsForPathTimePoints() const {
  std::vector<Condition> end_s_conditions;

  std::vector<SamplePoint> sample_points = QueryPathTimeObstacleSamplePoints();
  for (const SamplePoint& sample_point : sample_points) {
    if (sample_point.path_time_point().t() < FLAGS_polynomial_minimal_param) {
      continue;
    }
    double s = sample_point.path_time_point().s();
    double v = sample_point.ref_v();
    double t = sample_point.path_time_point().t();
    if (s > feasible_region_.SUpper(t) || s < feasible_region_.SLower(t)) {
      continue;
    }
    State end_state = {s, v, 0.0};
    end_s_conditions.emplace_back(end_state, t);
  }
  return end_s_conditions;
}

std::vector<SamplePoint>
EndConditionSampler::QueryPathTimeObstacleSamplePoints() const {
  const auto& vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  std::vector<SamplePoint> sample_points;
  for (const auto& path_time_obstacle :
       ptr_path_time_graph_->GetPathTimeObstacles()) {
    std::string obstacle_id = path_time_obstacle.obstacle_id();
    QueryFollowPathTimePoints(vehicle_config, obstacle_id, &sample_points);
    QueryOvertakePathTimePoints(vehicle_config, obstacle_id, &sample_points);
  }
  return sample_points;
}

void EndConditionSampler::QueryFollowPathTimePoints(
    const common::VehicleConfig& vehicle_config,
    const std::string& obstacle_id,
    std::vector<SamplePoint>* const sample_points) const {
  std::vector<PathTimePoint> follow_path_time_points =
      ptr_path_time_graph_->GetObstacleSurroundingPoints(
          obstacle_id, -FLAGS_lattice_epsilon, FLAGS_time_min_density);

  for (const auto& path_time_point : follow_path_time_points) {
    double v = ptr_prediction_querier_->ProjectVelocityAlongReferenceLine(
        obstacle_id, path_time_point.s(), path_time_point.t());
    // Generate candidate s
    double s_upper = path_time_point.s() -
                     vehicle_config.vehicle_param().front_edge_to_center();
    double s_lower = s_upper - FLAGS_default_lon_buffer;
    CHECK_GE(FLAGS_num_sample_follow_per_timestamp, 2);
    double s_gap = FLAGS_default_lon_buffer
        / static_cast<double>(FLAGS_num_sample_follow_per_timestamp - 1);
    for (std::size_t i = 0; i < FLAGS_num_sample_follow_per_timestamp; ++i) {
      double s = s_lower + s_gap * static_cast<double>(i);
      SamplePoint sample_point;
      sample_point.mutable_path_time_point()->CopyFrom(path_time_point);
      sample_point.mutable_path_time_point()->set_s(s);
      sample_point.set_ref_v(v);
      sample_points->push_back(std::move(sample_point));
    }
  }
}

void EndConditionSampler::QueryOvertakePathTimePoints(
    const common::VehicleConfig& vehicle_config,
    const std::string& obstacle_id,
    std::vector<SamplePoint>* sample_points) const {
  std::vector<PathTimePoint> overtake_path_time_points =
      ptr_path_time_graph_->GetObstacleSurroundingPoints(
          obstacle_id, FLAGS_lattice_epsilon, FLAGS_time_min_density);

  for (const auto& path_time_point : overtake_path_time_points) {
    double v = ptr_prediction_querier_->ProjectVelocityAlongReferenceLine(
        obstacle_id, path_time_point.s(), path_time_point.t());
    SamplePoint sample_point;
    sample_point.mutable_path_time_point()->CopyFrom(path_time_point);
    sample_point.mutable_path_time_point()->set_s(path_time_point.s() +
                                                  FLAGS_default_lon_buffer);
    sample_point.set_ref_v(v);
    sample_points->push_back(std::move(sample_point));
  }
}

}  // namespace planning
}  // namespace apollo
