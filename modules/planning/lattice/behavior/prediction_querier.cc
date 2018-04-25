/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/lattice/behavior/prediction_querier.h"

#include <string>

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/path_matcher.h"

namespace apollo {
namespace planning {

namespace {

int LastIndexBefore(const prediction::Trajectory& trajectory, const double t) {
  int num_traj_point = trajectory.trajectory_point_size();
  if (num_traj_point == 0) {
    return -1;
  }
  if (trajectory.trajectory_point(0).relative_time() > t) {
    return 0;
  }
  int start = 0;
  int end = num_traj_point - 1;
  while (start + 1 < end) {
    int mid = start + (end - start) / 2;
    if (trajectory.trajectory_point(mid).relative_time() <= t) {
      start = mid;
    } else {
      end = mid;
    }
  }
  if (trajectory.trajectory_point(end).relative_time() <= t) {
    return end;
  }
  return start;
}

}  // namespace

PredictionQuerier::PredictionQuerier(
    const std::vector<const Obstacle*>& obstacles,
    std::shared_ptr<std::vector<common::PathPoint>> ptr_reference_line)
    : ptr_reference_line_(ptr_reference_line) {
  for (const auto obstacle : obstacles) {
    if (id_obstacle_map_.find(obstacle->Id()) == id_obstacle_map_.end()) {
      id_obstacle_map_[obstacle->Id()] = obstacle;
      obstacles_.push_back(obstacle);
    } else {
      AWARN << "Duplicated obstacle found [" << obstacle->Id() << "]";
    }
  }
}

std::vector<const Obstacle*> PredictionQuerier::GetObstacles() const {
  return obstacles_;
}

double PredictionQuerier::ProjectVelocityAlongReferenceLine(
    const std::string& obstacle_id, const double s, const double t) const {
  CHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());

  const auto& trajectory = id_obstacle_map_.at(obstacle_id)->Trajectory();
  int num_traj_point = trajectory.trajectory_point_size();
  if (num_traj_point < 2) {
    return 0.0;
  }

  int curr_index = LastIndexBefore(trajectory, t);
  int next_index = curr_index + 1;
  double heading = trajectory.trajectory_point(curr_index).path_point().theta();

  if (curr_index == num_traj_point - 1) {
    curr_index = num_traj_point - 2;
    next_index = num_traj_point - 1;
  }

  double v_curr = trajectory.trajectory_point(curr_index).v();
  double t_curr = trajectory.trajectory_point(curr_index).relative_time();
  double x_curr = trajectory.trajectory_point(curr_index).path_point().x();
  double y_curr = trajectory.trajectory_point(curr_index).path_point().y();
  double v_next = trajectory.trajectory_point(next_index).v();
  double t_next = trajectory.trajectory_point(next_index).relative_time();
  double x_next = trajectory.trajectory_point(next_index).path_point().x();
  double y_next = trajectory.trajectory_point(next_index).path_point().y();
  double v = apollo::common::math::lerp(v_curr, t_curr, v_next, t_next, t);
  if (std::fabs(x_next - x_curr) > 0.1 && std::fabs(y_next - y_curr) > 0.1) {
    heading = std::atan2(y_next - y_curr, x_next - x_curr);
  }
  double v_x = v * std::cos(heading);
  double v_y = v * std::sin(heading);
  common::PathPoint obstacle_point_on_ref_line =
      common::math::PathMatcher::MatchToPath(*ptr_reference_line_, s);
  double ref_theta = obstacle_point_on_ref_line.theta();

  return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}

}  // namespace planning
}  // namespace apollo
