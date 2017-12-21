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

#include <algorithm>
#include <cmath>

#include "modules/common/math/linear_interpolation.h"

namespace apollo {
namespace planning {

using CriticalPointPair = std::pair<PathTimePoint, PathTimePoint>;

ConditionFilter::ConditionFilter(
    const Frame* frame,
    const std::array<double, 3>& init_s,
    const double speed_limit,
    const ReferenceLine& reference_line,
    const std::vector<common::PathPoint>& discretized_ref_points) :
    feasible_region_(init_s, speed_limit),
    path_time_neighborhood_(frame, init_s,
      reference_line, discretized_ref_points) {
  Init();
}

std::vector<SampleBound> ConditionFilter::QuerySampleBounds() const {
  std::set<double> critical_timestamps = CriticalTimeStamps();
  std::vector<SampleBound> sample_bounds;
  for (const double t : critical_timestamps) {
    // TODO (kechxu): change the hard-coded 0.001 to some variable
    // TODO (zhangyajia): why is this not zero?
    if (t > 0.001) {
      std::vector<SampleBound> sample_bounds_at_t = QuerySampleBounds(t);
      sample_bounds.insert(sample_bounds.end(), sample_bounds_at_t.begin(),
          sample_bounds_at_t.end());
    }
  }
  return sample_bounds;
}

std::vector<SampleBound> ConditionFilter::QuerySampleBounds(
    const double t) const {
  double feasible_s_lower = feasible_region_.SLower(t);
  double feasible_s_upper = feasible_region_.SUpper(t);
  double feasible_v_lower = feasible_region_.VLower(t);
  double feasible_v_upper = feasible_region_.VUpper(t);

  CHECK(feasible_s_lower <= feasible_s_upper
      && feasible_v_lower <= feasible_v_upper);

  std::vector<CriticalPointPair> path_intervals =
      QueryPathTimeObstacleIntervals(t);
  double s_max_reached = feasible_s_lower;
  const PathTimePoint* point_prev_ptr = nullptr;

  std::vector<SampleBound> sample_bounds;
  for (const auto& path_interval : path_intervals) {
    if (path_interval.second.s() < s_max_reached) {
      continue;
    }

    if (s_max_reached > feasible_s_upper) {
      break;
    }

    // a new interval
    if (s_max_reached < path_interval.first.s()) {
      if (path_interval.first.s() <= feasible_s_upper) {
        SampleBound sample_bound;
        sample_bound.set_t(t);
        sample_bound.set_s_upper(path_interval.first.s());
        sample_bound.set_s_lower(s_max_reached);

        double v_upper = feasible_v_upper;

        if (path_interval.first.has_v()) {
          v_upper = path_interval.first.v();
        }

        double v_lower = feasible_v_lower;
        if (point_prev_ptr) {
          v_lower = point_prev_ptr->v();
        }
        sample_bound.set_v_upper(v_upper);
        sample_bound.set_v_lower(v_lower);

        sample_bounds.push_back(std::move(sample_bound));
      } else {
        SampleBound sample_bound;
        sample_bound.set_t(t);
        sample_bound.set_s_upper(feasible_s_upper);
        sample_bound.set_s_lower(s_max_reached);
        sample_bound.set_v_upper(feasible_v_upper);

        double v_lower = feasible_v_upper;
        /**
        if (point_prev_ptr) {
          v_lower = point_prev_ptr->v();
        }
        **/
        sample_bound.set_v_lower(v_lower);
        sample_bounds.push_back(std::move(sample_bound));
        break;
      }
    }
    if (s_max_reached < path_interval.second.s()) {
      s_max_reached = path_interval.second.s();
      point_prev_ptr = &(path_interval.second);
    }
  }
  return sample_bounds;
}

CriticalPointPair ConditionFilter::QueryPathTimeObstacleIntervals(
    const double t,
    const PathTimeObstacle& path_time_obstacle) const {

  CriticalPointPair block_interval;
  double s_upper = apollo::common::math::lerp(
      path_time_obstacle.upper_left().s(),
      path_time_obstacle.upper_left().t(),
      path_time_obstacle.upper_right().s(),
      path_time_obstacle.upper_right().t(), t);

  double s_lower = apollo::common::math::lerp(
      path_time_obstacle.bottom_left().s(),
      path_time_obstacle.bottom_left().t(),
      path_time_obstacle.bottom_right().s(),
      path_time_obstacle.bottom_right().t(), t);

  const std::string& obstacle_id = path_time_obstacle.obstacle_id();
  double v = path_time_neighborhood_.SpeedAtT(obstacle_id, t);

  block_interval.first.set_t(t);
  block_interval.first.set_s(s_lower);
  block_interval.first.set_v(v);
  block_interval.first.set_obstacle_id(path_time_obstacle.obstacle_id());
  block_interval.second.set_t(t);
  block_interval.second.set_s(s_upper);
  block_interval.second.set_v(v);
  block_interval.second.set_obstacle_id(path_time_obstacle.obstacle_id());

  return block_interval;
}

std::vector<CriticalPointPair> ConditionFilter::QueryPathTimeObstacleIntervals(
    const double t) const {
  std::vector<CriticalPointPair> path_intervals;
  for (const auto& path_time_obstacle : path_time_obstacles_) {
    if (path_time_obstacle.time_lower() > t ||
        path_time_obstacle.time_upper() < t) {
      continue;
    }
    CriticalPointPair path_interval =
        QueryPathTimeObstacleIntervals(t, path_time_obstacle);

    path_intervals.push_back(std::move(path_interval));
  }
  std::sort(path_intervals.begin(), path_intervals.end(),
      [](const CriticalPointPair& pair_1, const CriticalPointPair& pair_2) {
        return pair_1.first.s() < pair_2.first.s();
      });

  return path_intervals;
}

void ConditionFilter::Init() {
  path_time_obstacles_ = path_time_neighborhood_.GetPathTimeObstacles();
}

std::set<double> ConditionFilter::CriticalTimeStamps() const {
  std::set<double> critical_timestamps;
  for (const auto& path_time_obstacle : path_time_obstacles_) {
    double t_start = path_time_obstacle.bottom_left().t();
    double t_end = path_time_obstacle.upper_right().t();
    critical_timestamps.insert(t_start);
    critical_timestamps.insert(t_end);
  }
  return critical_timestamps;
}

}  // namespace planning
}  // namespace apollo
