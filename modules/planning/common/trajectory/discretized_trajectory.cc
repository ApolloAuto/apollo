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
 * @file discretized_trajectory.cc
 **/

#include "modules/planning/common/trajectory/discretized_trajectory.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/common/log.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

DiscretizedTrajectory::DiscretizedTrajectory(
    std::vector<TrajectoryPoint> trajectory_points) {
  _trajectory_points = std::move(trajectory_points);
}

double DiscretizedTrajectory::TimeLength() const {
  if (_trajectory_points.empty()) {
    return 0.0;
  }
  return _trajectory_points.back().relative_time() -
         _trajectory_points.front().relative_time();
}

TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
  CHECK(!_trajectory_points.empty());
  CHECK(_trajectory_points.front().relative_time() <= relative_time &&
        _trajectory_points.back().relative_time() <= relative_time)
      << "Invalid relative time input!";

  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower =
      std::lower_bound(_trajectory_points.begin(), _trajectory_points.end(),
                       relative_time, comp);

  if (it_lower == _trajectory_points.begin()) {
    return _trajectory_points.front();
  }
  return util::interpolate(*(it_lower - 1), *it_lower, relative_time);
}

TrajectoryPoint DiscretizedTrajectory::EvaluateUsingLinearApproximation(
    const double relative_time) const {
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower =
      std::lower_bound(_trajectory_points.begin(), _trajectory_points.end(),
                       relative_time, comp);

  if (it_lower == _trajectory_points.begin()) {
    return _trajectory_points.front();
  }
  return util::interpolate_linear_approximation(*(it_lower - 1), *it_lower,
                                                relative_time);
}

std::uint32_t DiscretizedTrajectory::QueryNearestPoint(
    const double relative_time) const {
  auto func = [](const TrajectoryPoint& tp, const double relative_time) {
    return tp.relative_time() < relative_time;
  };
  auto it_lower =
      std::lower_bound(_trajectory_points.begin(), _trajectory_points.end(),
                       relative_time, func);
  return (std::uint32_t)(it_lower - _trajectory_points.begin());
}

std::uint32_t DiscretizedTrajectory::QueryNearestPoint(
    const common::math::Vec2d& position) const {
  double dist_min = std::numeric_limits<double>::max();
  std::uint32_t index_min = 0;
  for (std::uint32_t i = 0; i < _trajectory_points.size(); ++i) {
    const common::math::Vec2d coordinate(
        _trajectory_points[i].path_point().x(),
        _trajectory_points[i].path_point().y());
    common::math::Vec2d dist_vec = coordinate - position;
    double dist = dist_vec.InnerProd(dist_vec);
    if (dist < dist_min) {
      dist_min = dist;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  if (!_trajectory_points.empty()) {
    CHECK_GT(trajectory_point.relative_time(),
             _trajectory_points.back().relative_time());
  }
  _trajectory_points.push_back(trajectory_point);
}

const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const std::uint32_t index) const {
  CHECK_LT(index, NumOfPoints());
  return _trajectory_points[index];
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  CHECK(!_trajectory_points.empty());
  return _trajectory_points.front();
}

TrajectoryPoint DiscretizedTrajectory::EndPoint() const {
  CHECK(!_trajectory_points.empty());
  return _trajectory_points.back();
}

std::uint32_t DiscretizedTrajectory::NumOfPoints() const {
  return _trajectory_points.size();
}

const std::vector<apollo::common::TrajectoryPoint>&
DiscretizedTrajectory::trajectory_points() const {
  return _trajectory_points;
}

void DiscretizedTrajectory::SetTrajectoryPoints(
    const std::vector<apollo::common::TrajectoryPoint>& points) {
  *this = DiscretizedTrajectory(points);
}

void DiscretizedTrajectory::Clear() { _trajectory_points.clear(); }

}  // namespace planning
}  // namespace apollo
