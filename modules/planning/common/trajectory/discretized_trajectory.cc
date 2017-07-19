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
 * @file discretized_trajectory.cpp
 **/

#include "modules/planning/common/trajectory/discretized_trajectory.h"

#include <climits>

#include "glog/logging.h"
#include "modules/planning/common/path/path_point_util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

DiscretizedTrajectory::DiscretizedTrajectory(
    std::vector<TrajectoryPoint> trajectory_points) {
  _trajectory_points = std::move(trajectory_points);
}

double DiscretizedTrajectory::time_length() const {
  if (_trajectory_points.empty()) {
    return 0.0;
  }
  return _trajectory_points.back().relative_time() -
         _trajectory_points.front().relative_time();
}

TrajectoryPoint DiscretizedTrajectory::evaluate(
    const double relative_time) const {
  CHECK(!_trajectory_points.empty());
  CHECK(_trajectory_points.front().relative_time() <= relative_time &&
        _trajectory_points.back().relative_time() <= relative_time)
        << "Invalid relative time input!";

  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower = std::lower_bound(_trajectory_points.begin(),
                                   _trajectory_points.end(),
                                   relative_time, comp);

  if (it_lower == _trajectory_points.begin()) {
    return _trajectory_points.front();
  }
  return util::interpolate(*(it_lower - 1), *it_lower, relative_time);
}

TrajectoryPoint DiscretizedTrajectory::evaluate_linear_approximation(
    const double relative_time) const {
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower = std::lower_bound(_trajectory_points.begin(),
                                   _trajectory_points.end(),
                                   relative_time, comp);

  if (it_lower == _trajectory_points.begin()) {
    return _trajectory_points.front();
  }
  return util::interpolate_linear_approximation(*(it_lower - 1), *it_lower,
                                                relative_time);
}

std::size_t DiscretizedTrajectory::query_nearest_point(
    const double relative_time) const {
  auto func = [](const TrajectoryPoint& tp, const double relative_time) {
    return tp.relative_time() < relative_time;
  };
  auto it_lower = std::lower_bound(_trajectory_points.begin(),
                                   _trajectory_points.end(),
                                   relative_time, func);
  return (std::size_t)(it_lower - _trajectory_points.begin());
}

std::size_t DiscretizedTrajectory::query_nearest_point(
    const Eigen::Vector2d& position) const {
  double dist_min = std::numeric_limits<double>::max();
  std::size_t index_min = 0;
  for (std::size_t i = 0; i < _trajectory_points.size(); ++i) {
    const Eigen::Vector2d coordinate(_trajectory_points[i].path_point().x(),
                                     _trajectory_points[i].path_point().y());
    Eigen::Vector2d dist_vec = coordinate - position;
    double dist = dist_vec.dot(dist_vec);
    if (dist < dist_min) {
      dist_min = dist;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::add_trajectory_point(
    const TrajectoryPoint& trajectory_point) {
  if (!_trajectory_points.empty()) {
    CHECK_GT(trajectory_point.relative_time(),
             _trajectory_points.back().relative_time());
  }
  _trajectory_points.push_back(trajectory_point);
}

const TrajectoryPoint& DiscretizedTrajectory::trajectory_point_at(
    const std::size_t index) const {
  CHECK(index < num_of_points());
  return _trajectory_points[index];
}

TrajectoryPoint DiscretizedTrajectory::start_point() const {
  CHECK(!_trajectory_points.empty());
  return _trajectory_points.front();
}

TrajectoryPoint DiscretizedTrajectory::end_point() const {
  CHECK(!_trajectory_points.empty());
  return _trajectory_points.back();
}

std::size_t DiscretizedTrajectory::num_of_points() const {
  return _trajectory_points.size();
}

}  // namespace planning
}  // namespace apollo
