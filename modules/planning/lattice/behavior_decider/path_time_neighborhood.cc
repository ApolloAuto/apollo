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

#include "modules/planning/lattice/behavior_decider/path_time_neighborhood.h"

#include <utility>
#include <vector>
#include <cmath>

#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/planning/lattice/util/reference_line_matcher.h"
#include "modules/planning/lattice/util/lattice_util.h"
#include "modules/common/math/linear_interpolation.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::perception::PerceptionObstacle;

namespace {

int LastIndexBefore(const prediction::Trajectory& trajectory, const double t) {
  int num_traj_point = trajectory.trajectory_point_size();
  if (num_traj_point == 0) {
    return -1;
  }
  if (trajectory.trajectory_point(0).relative_time() > t) {
    return -1;
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

PathTimeNeighborhood::PathTimeNeighborhood(
    const Frame* frame, const double ego_s,
    const ReferenceLine& reference_line,
    const std::vector<common::PathPoint>& discretized_ref_points) {
  ego_s_ = ego_s;
  discretized_ref_points_ = discretized_ref_points;
  SetupObstacles(frame, reference_line, discretized_ref_points);
}

void PathTimeNeighborhood::SetStaticPathTimeObstacle(
    const Obstacle* obstacle,
    const ReferenceLine& reference_line) {
  TrajectoryPoint start_point = obstacle->GetPointAtTime(0.0);
  Box2d box = obstacle->GetBoundingBox(start_point);

  std::string obstacle_id = obstacle->Id();
  SLBoundary sl_boundary;
  reference_line.GetSLBoundary(box, &sl_boundary);
  path_time_obstacle_map_[obstacle_id].set_obstacle_id(obstacle_id);
  path_time_obstacle_map_[obstacle_id].mutable_bottom_left()
      ->CopyFrom(SetPathTimePoint(obstacle_id, sl_boundary.start_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].mutable_bottom_right()
      ->CopyFrom(SetPathTimePoint(obstacle_id, sl_boundary.start_s(),
                                  planned_trajectory_time));
  path_time_obstacle_map_[obstacle_id].mutable_upper_left()
      ->CopyFrom(SetPathTimePoint(obstacle_id, sl_boundary.end_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].mutable_upper_right()
      ->CopyFrom(SetPathTimePoint(obstacle_id, sl_boundary.end_s(),
                                  planned_trajectory_time));
}

void PathTimeNeighborhood::SetupObstacles(
    const Frame* frame, const ReferenceLine& reference_line,
    const std::vector<common::PathPoint>& discretized_ref_points) {
  const auto& obstacles = frame->obstacles();

  for (const Obstacle* obstacle : obstacles) {
    if (prediction_traj_map_.find(obstacle->Id()) ==
        prediction_traj_map_.end()) {
      prediction_traj_map_[obstacle->Id()] = obstacle->Trajectory();
    } else {
      AWARN << "Duplicated obstacle found [" << obstacle->Id() << "]";
    }

    if (!obstacle->HasTrajectory()) {
       SetStaticPathTimeObstacle(obstacle, reference_line);
       continue;
    }

    double relative_time = 0.0;
    while (relative_time < planned_trajectory_time) {
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);

      SLBoundary sl_boundary;
      reference_line.GetSLBoundary(box, &sl_boundary);

      // the obstacle is not shown on the region to be considered.
      if (sl_boundary.end_s() < 0.0 ||
          sl_boundary.start_s() > ego_s_ +  decision_horizon ||
          (std::abs(sl_boundary.start_l()) > lateral_enter_lane_thred &&
           std::abs(sl_boundary.end_l()) > lateral_enter_lane_thred)) {
        if (path_time_obstacle_map_.find(obstacle->Id()) !=
            path_time_obstacle_map_.end()) {
          break;
        } else {
          relative_time += trajectory_time_resolution;
          continue;
        }
      }

      if (path_time_obstacle_map_.find(obstacle->Id()) ==
          path_time_obstacle_map_.end()) {
        path_time_obstacle_map_[obstacle->Id()].set_obstacle_id(obstacle->Id());

        path_time_obstacle_map_[obstacle->Id()].mutable_bottom_left()
            ->CopyFrom(SetPathTimePoint(obstacle->Id(),
                           sl_boundary.start_s(), relative_time));

        path_time_obstacle_map_[obstacle->Id()].mutable_upper_left()
            ->CopyFrom(SetPathTimePoint(obstacle->Id(),
                           sl_boundary.end_s(), relative_time));
      }

      path_time_obstacle_map_[obstacle->Id()].mutable_bottom_right()
          ->CopyFrom(SetPathTimePoint(obstacle->Id(),
                         sl_boundary.start_s(), relative_time));

      path_time_obstacle_map_[obstacle->Id()].mutable_upper_right()
          ->CopyFrom(SetPathTimePoint(obstacle->Id(),
                         sl_boundary.end_s(), relative_time));

      relative_time += trajectory_time_resolution;
    }
  }

  for (auto& path_time_obstacle : path_time_obstacle_map_) {
    double s_upper = std::max(path_time_obstacle.second.bottom_right().s(),
                              path_time_obstacle.second.upper_right().s());

    double s_lower = std::min(path_time_obstacle.second.bottom_left().s(),
                              path_time_obstacle.second.upper_left().s());

    path_time_obstacle.second.set_path_lower(s_lower);

    path_time_obstacle.second.set_path_upper(s_upper);

    double t_upper = std::max(path_time_obstacle.second.bottom_right().t(),
                              path_time_obstacle.second.upper_right().t());

    double t_lower = std::min(path_time_obstacle.second.bottom_left().t(),
                              path_time_obstacle.second.upper_left().t());

    path_time_obstacle.second.set_time_lower(t_lower);

    path_time_obstacle.second.set_time_upper(t_upper);
  }
}

double PathTimeNeighborhood::SpeedAtT(
    const std::string& obstacle_id, const double s, const double t) const {
  bool found =
      prediction_traj_map_.find(obstacle_id) != prediction_traj_map_.end();
  CHECK(found);
  CHECK_GE(t, 0.0);
  auto it_trajectory = prediction_traj_map_.find(obstacle_id);
  CHECK(it_trajectory != prediction_traj_map_.end());

  const prediction::Trajectory& trajectory = it_trajectory->second;
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
  if (std::abs(x_next - x_curr) > 0.1 && std::abs(y_next - y_curr) > 0.1) {
    heading = std::atan2(y_next - y_curr, x_next - x_curr);
  }
  double v_x = v * std::cos(heading);
  double v_y = v * std::sin(heading);
  PathPoint obstacle_point_on_ref_line =
      ReferenceLineMatcher::MatchToReferenceLine(discretized_ref_points_, s);
  double ref_theta = obstacle_point_on_ref_line.theta();

  return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}

PathTimePoint PathTimeNeighborhood::SetPathTimePoint(
    const std::string& obstacle_id, const double s, const double t) const {
  PathTimePoint path_time_point;
  path_time_point.set_s(s);
  path_time_point.set_t(t);
  path_time_point.set_v(0.0);
  path_time_point.set_obstacle_id(obstacle_id);
  return path_time_point;
}

std::vector<PathTimeObstacle> PathTimeNeighborhood::GetPathTimeObstacles()
    const {
  std::vector<PathTimeObstacle> path_time_obstacles;
  for (const auto& path_time_obstacle_element : path_time_obstacle_map_) {
    path_time_obstacles.push_back(path_time_obstacle_element.second);
  }
  return path_time_obstacles;
}

bool PathTimeNeighborhood::GetPathTimeObstacle(
    const std::string& obstacle_id, PathTimeObstacle* path_time_obstacle) {
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return false;
  }
  *path_time_obstacle = path_time_obstacle_map_[obstacle_id];
  return true;
}

}  // namespace planning
}  // namespace apollo
