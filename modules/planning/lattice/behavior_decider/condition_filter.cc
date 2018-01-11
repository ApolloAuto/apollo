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

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"
#include "modules/common/log.h"

namespace apollo {
namespace planning {

using PathTimePointPair = std::pair<PathTimePoint, PathTimePoint>;

ConditionFilter::ConditionFilter(
    const std::array<double, 3>& init_s,
    const double speed_limit,
    std::shared_ptr<PathTimeNeighborhood> path_time_neighborhood)
    : feasible_region_(init_s, speed_limit),
      ptr_path_time_neighborhood_(path_time_neighborhood) {}

std::vector<SampleBound> ConditionFilter::QuerySampleBounds() const {
  // std::set<double> timestamps = CriticalTimeStamps();
  std::vector<double> timestamps = UniformTimeStamps(8);
  std::vector<SampleBound> sample_bounds;
  for (const double t : timestamps) {
    std::vector<SampleBound> sample_bounds_at_t = QuerySampleBounds(t);
    sample_bounds.insert(sample_bounds.end(), sample_bounds_at_t.begin(),
                         sample_bounds_at_t.end());
  }
  return sample_bounds;
}

std::vector<SampleBound> ConditionFilter::QuerySampleBounds(
    const double t) const {
  double feasible_s_lower = feasible_region_.SLower(t);
  double feasible_s_upper = feasible_region_.SUpper(t);
  double feasible_v_lower = feasible_region_.VLower(t);
  double feasible_v_upper = feasible_region_.VUpper(t);

  CHECK(feasible_s_lower <= feasible_s_upper &&
        feasible_v_lower <= feasible_v_upper);

  std::vector<PathTimePointPair> path_intervals =
      QueryPathTimeObstacleIntervals(t);

  double s_max_reached = feasible_s_lower;
  const PathTimePoint* point_prev_ptr = nullptr;

  std::vector<SampleBound> sample_bounds;
  for (const auto& path_interval : path_intervals) {
    // skip inclusive intervals
    if (path_interval.second.s() < s_max_reached) {
      continue;
    }

    // reached the top
    if (s_max_reached > feasible_s_upper) {
      break;
    }

    // a new interval
    //@TODO(liyun): implement reference_v to be
    // (1) front obstacle speed if front obstacle exists
    // (2) rear obstacle speed if no front obstacle exists and only rear
    // obstacle exists
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
        sample_bound.set_v_reference(v_upper);

        sample_bounds.push_back(std::move(sample_bound));
      } else {
        SampleBound sample_bound;
        sample_bound.set_t(t);
        sample_bound.set_s_upper(feasible_s_upper);
        sample_bound.set_s_lower(s_max_reached);
        sample_bound.set_v_upper(feasible_v_upper);
        sample_bound.set_v_reference(feasible_v_upper);

        double v_lower = feasible_v_upper;
        if (point_prev_ptr) {
          v_lower = point_prev_ptr->v();
        }
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

PathTimePointPair ConditionFilter::QueryPathTimeObstacleIntervals(
    const double t, const PathTimeObstacle& path_time_obstacle) const {
  PathTimePointPair block_interval;
  double s_upper = apollo::common::math::lerp(
      path_time_obstacle.upper_left().s(), path_time_obstacle.upper_left().t(),
      path_time_obstacle.upper_right().s(),
      path_time_obstacle.upper_right().t(), t);

  double s_lower =
      apollo::common::math::lerp(path_time_obstacle.bottom_left().s(),
                                 path_time_obstacle.bottom_left().t(),
                                 path_time_obstacle.bottom_right().s(),
                                 path_time_obstacle.bottom_right().t(), t);

  const std::string& obstacle_id = path_time_obstacle.obstacle_id();
  double v = ptr_path_time_neighborhood_->SpeedAtT(obstacle_id, s_lower, t);

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

std::vector<PathTimePointPair> ConditionFilter::QueryPathTimeObstacleIntervals(
    const double t) const {
  std::vector<PathTimePointPair> path_intervals;
  for (const auto& path_time_obstacle : ptr_path_time_neighborhood_->GetPathTimeObstacles()) {
    if (path_time_obstacle.time_lower() > t ||
        path_time_obstacle.time_upper() < t) {
      continue;
    }
    PathTimePointPair path_interval =
        QueryPathTimeObstacleIntervals(t, path_time_obstacle);

    path_intervals.push_back(std::move(path_interval));
  }
  std::sort(
      path_intervals.begin(), path_intervals.end(),
      [](const PathTimePointPair& pair_1, const PathTimePointPair& pair_2) {
        return pair_1.first.s() < pair_2.first.s();
      });

  return path_intervals;
}

std::set<double> ConditionFilter::CriticalTimeStamps() const {
  std::set<double> critical_timestamps;
  for (const auto& path_time_obstacle : ptr_path_time_neighborhood_->GetPathTimeObstacles()) {
    double t_start = path_time_obstacle.bottom_left().t();
    double t_end = path_time_obstacle.upper_right().t();
    critical_timestamps.insert(t_start);
    critical_timestamps.insert(t_end);
  }
  return critical_timestamps;
}

std::vector<double> ConditionFilter::UniformTimeStamps(
    const std::size_t num_of_time_segments) const {
  CHECK(num_of_time_segments > 0);

  double finish_trajectory_length = 0.01;
  CHECK(finish_trajectory_length < (1.0 / (double)FLAGS_planning_loop_rate));

  std::vector<double> timestamps;
  timestamps.push_back(finish_trajectory_length);

  double time_interval = planned_trajectory_time / (double)num_of_time_segments;
  for (std::size_t i = 1; i <= num_of_time_segments; ++i) {
    timestamps.push_back(i * time_interval);
  }

  return timestamps;
}

// Compute pixel img for lattice st
bool ConditionFilter::GenerateLatticeStPixels(
  apollo::planning_internal::LatticeStTraining* st_data,
  double timestamp,
  std::string st_img_name) {
  int num_rows = 250;
  int num_cols = 160;
  double s_step = 100.0 / (double) num_rows;
  double t_step = 8.0 / (double) num_cols;

  if (ptr_path_time_neighborhood_->GetPathTimeObstacles().empty()) {
    AINFO << "No_Path_Time_Neighborhood_Obstacle_in_this_frame";
    return false;
  }
  for (int j = 0; j < num_cols; ++j) {
    double t = t_step * (j + 1);
    double feasible_s_upper = feasible_region_.SUpper(t);
    double feasible_s_lower = feasible_region_.SLower(t);
    for (int i = 0; i < num_rows; ++i) {
      double s = s_step * (num_rows - i + 1);
      if (s <= feasible_s_lower || s >= feasible_s_upper) {
        // Dye gray
        apollo::planning_internal::LatticeStPixel* pixel =
          st_data->add_pixel();
        pixel->set_s(i);
        pixel->set_t(j);
        pixel->set_r(128);
        pixel->set_g(128);
        pixel->set_b(128);
        continue;
      }
      if (WithinObstacleSt(s,t)) {
        // Dye blue
        apollo::planning_internal::LatticeStPixel* pixel =
          st_data->add_pixel();
        pixel->set_s(i);
        pixel->set_t(j);
        pixel->set_r(0);
        pixel->set_g(0);
        pixel->set_b(128);
        continue;
      }
    }
  }
  st_data->set_annotation(st_img_name);
  st_data->set_timestamp(timestamp);
  st_data->set_num_s_grids(num_rows);
  st_data->set_num_t_grids(num_cols);
  st_data->set_s_resolution(s_step);
  st_data->set_t_resolution(t_step);
  return true;
}

bool ConditionFilter::WithinObstacleSt(double s, double t) {
  const auto& path_time_obstacles =
    ptr_path_time_neighborhood_->GetPathTimeObstacles();

  for (const PathTimeObstacle& path_time_obstacle : path_time_obstacles) {
     if (t < path_time_obstacle.upper_left().t() ||
         t > path_time_obstacle.upper_right().t()) {
      continue;
     }

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

    if (s <= s_upper && s >= s_lower) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
