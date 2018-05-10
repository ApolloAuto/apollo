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

#include "modules/planning/lattice/behavior/path_time_graph.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/path_matcher.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/sl_boundary.pb.h"

namespace apollo {
namespace planning {

using apollo::common::math::lerp;
using apollo::common::math::Box2d;
using apollo::common::math::PathMatcher;
using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::perception::PerceptionObstacle;

PathTimeGraph::PathTimeGraph(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points, const double s_start,
    const double s_end, const double t_start, const double t_end,
    const double path_width) {
  CHECK(s_start < s_end);
  CHECK(t_start < t_end);
  CHECK(path_width > 0.0);

  path_range_.first = s_start;
  path_range_.second = s_end;

  time_range_.first = t_start;
  time_range_.second = t_end;

  half_path_width_ = path_width * 0.5;

  SetupObstacles(obstacles, discretized_ref_points);
}

SLBoundary PathTimeGraph::ComputeObstacleBoundary(
    const Box2d& box,
    const std::vector<PathPoint>& discretized_ref_points) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<common::math::Vec2d> corners;
  box.GetAllCorners(&corners);

  for (const auto& point : corners) {
    auto sl_point = PathMatcher::GetPathFrenetCoordinate(discretized_ref_points,
                                                         point.x(), point.y());
    start_s = std::fmin(start_s, sl_point.first);
    end_s = std::fmax(end_s, sl_point.first);
    start_l = std::fmin(start_l, sl_point.second);
    end_l = std::fmax(end_l, sl_point.second);
  }

  SLBoundary sl_boundary;
  sl_boundary.set_start_s(start_s);
  sl_boundary.set_end_s(end_s);
  sl_boundary.set_start_l(start_l);
  sl_boundary.set_end_l(end_l);

  return sl_boundary;
}

void PathTimeGraph::SetupObstacles(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points) {
  for (const Obstacle* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->HasTrajectory()) {
      SetStaticObstacle(obstacle, discretized_ref_points);
      continue;
    }

    double relative_time = time_range_.first;
    while (relative_time < time_range_.second) {
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      SLBoundary sl_boundary =
          ComputeObstacleBoundary(box, discretized_ref_points);

      // the obstacle is not shown on the region to be considered.
      if (sl_boundary.end_s() < path_range_.first ||
          sl_boundary.start_s() > path_range_.second ||
          (sl_boundary.start_l() > half_path_width_ &&
           sl_boundary.end_l() < -half_path_width_)) {
        if (path_time_obstacle_map_.find(obstacle->Id()) !=
            path_time_obstacle_map_.end()) {
          break;
        } else {
          relative_time += FLAGS_trajectory_time_resolution;
          continue;
        }
      }

      if (path_time_obstacle_map_.find(obstacle->Id()) ==
          path_time_obstacle_map_.end()) {
        path_time_obstacle_map_[obstacle->Id()].set_obstacle_id(obstacle->Id());

        path_time_obstacle_map_[obstacle->Id()].mutable_bottom_left()->CopyFrom(
            SetPathTimePoint(obstacle->Id(), sl_boundary.start_s(),
                             relative_time));

        path_time_obstacle_map_[obstacle->Id()].mutable_upper_left()->CopyFrom(
            SetPathTimePoint(obstacle->Id(), sl_boundary.end_s(),
                             relative_time));
      }

      path_time_obstacle_map_[obstacle->Id()].mutable_bottom_right()->CopyFrom(
          SetPathTimePoint(obstacle->Id(), sl_boundary.start_s(),
                           relative_time));

      path_time_obstacle_map_[obstacle->Id()].mutable_upper_right()->CopyFrom(
          SetPathTimePoint(obstacle->Id(), sl_boundary.end_s(), relative_time));

      relative_time += FLAGS_trajectory_time_resolution;
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

  // store the path_time_obstacles for later access.
  for (const auto& path_time_obstacle_element : path_time_obstacle_map_) {
    path_time_obstacles_.push_back(path_time_obstacle_element.second);
  }
}

void PathTimeGraph::SetStaticObstacle(
    const Obstacle* obstacle,
    const std::vector<PathPoint>& discretized_ref_points) {
  TrajectoryPoint start_point = obstacle->GetPointAtTime(0.0);
  Box2d box = obstacle->GetBoundingBox(start_point);

  std::string obstacle_id = obstacle->Id();
  SLBoundary sl_boundary = ComputeObstacleBoundary(box, discretized_ref_points);

  path_time_obstacle_map_[obstacle_id].set_obstacle_id(obstacle_id);
  path_time_obstacle_map_[obstacle_id].mutable_bottom_left()->CopyFrom(
      SetPathTimePoint(obstacle_id, sl_boundary.start_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].mutable_bottom_right()->CopyFrom(
      SetPathTimePoint(obstacle_id, sl_boundary.start_s(),
                       FLAGS_trajectory_time_length));
  path_time_obstacle_map_[obstacle_id].mutable_upper_left()->CopyFrom(
      SetPathTimePoint(obstacle_id, sl_boundary.end_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].mutable_upper_right()->CopyFrom(
      SetPathTimePoint(obstacle_id, sl_boundary.end_s(),
                       FLAGS_trajectory_time_length));
}

PathTimePoint PathTimeGraph::SetPathTimePoint(const std::string& obstacle_id,
                                              const double s,
                                              const double t) const {
  PathTimePoint path_time_point;
  path_time_point.set_s(s);
  path_time_point.set_t(t);
  path_time_point.set_obstacle_id(obstacle_id);
  return path_time_point;
}

const std::vector<PathTimeObstacle>& PathTimeGraph::GetPathTimeObstacles()
    const {
  return path_time_obstacles_;
}

bool PathTimeGraph::GetPathTimeObstacle(const std::string& obstacle_id,
                                        PathTimeObstacle* path_time_obstacle) {
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return false;
  }
  *path_time_obstacle = path_time_obstacle_map_[obstacle_id];
  return true;
}

std::vector<std::pair<double, double>> PathTimeGraph::GetPathBlockingIntervals(
    const double t) const {
  CHECK(time_range_.first <= t && t <= time_range_.second);
  std::vector<std::pair<double, double>> intervals;
  for (const auto& pt_obstacle : path_time_obstacles_) {
    if (t > pt_obstacle.time_upper() || t < pt_obstacle.time_lower()) {
      continue;
    }
    double s_upper =
        lerp(pt_obstacle.upper_left().s(), pt_obstacle.upper_left().t(),
             pt_obstacle.upper_right().s(), pt_obstacle.upper_right().t(), t);

    double s_lower =
        lerp(pt_obstacle.bottom_left().s(), pt_obstacle.bottom_left().t(),
             pt_obstacle.bottom_right().s(), pt_obstacle.bottom_right().t(), t);

    intervals.emplace_back(s_lower, s_upper);
  }
  return intervals;
}

std::vector<std::vector<std::pair<double, double>>>
PathTimeGraph::GetPathBlockingIntervals(const double t_start,
                                        const double t_end,
                                        const double t_resolution) {
  std::vector<std::vector<std::pair<double, double>>> intervals;
  for (double t = t_start; t <= t_end; t += t_resolution) {
    intervals.push_back(GetPathBlockingIntervals(t));
  }
  return intervals;
}

std::pair<double, double> PathTimeGraph::get_path_range() const {
  return path_range_;
}

std::pair<double, double> PathTimeGraph::get_time_range() const {
  return time_range_;
}

std::vector<PathTimePoint> PathTimeGraph::GetObstacleSurroundingPoints(
    const std::string& obstacle_id, const double s_dist,
    const double t_min_density) const {
  CHECK(t_min_density > 0.0);
  std::vector<PathTimePoint> pt_pairs;
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return pt_pairs;
  }

  const auto& pt_obstacle = path_time_obstacle_map_.at(obstacle_id);

  double s0 = 0.0;
  double s1 = 0.0;

  double t0 = 0.0;
  double t1 = 0.0;
  if (s_dist > 0.0) {
    s0 = pt_obstacle.upper_left().s();
    s1 = pt_obstacle.upper_right().s();

    t0 = pt_obstacle.upper_left().t();
    t1 = pt_obstacle.upper_right().t();
  } else {
    s0 = pt_obstacle.bottom_left().s();
    s1 = pt_obstacle.bottom_right().s();

    t0 = pt_obstacle.bottom_left().t();
    t1 = pt_obstacle.bottom_right().t();
  }

  double time_gap = t1 - t0;
  CHECK(time_gap > -FLAGS_lattice_epsilon);
  time_gap = std::fabs(time_gap);

  std::size_t num_sections = std::size_t(time_gap / t_min_density) + 1;
  double t_interval = time_gap / num_sections;

  for (std::size_t i = 0; i <= num_sections; ++i) {
    double t = t_interval * i + t0;
    double s = lerp(s0, t0, s1, t1, t) + s_dist;

    PathTimePoint ptt;
    ptt.set_obstacle_id(obstacle_id);
    ptt.set_t(t);
    ptt.set_s(s);
    pt_pairs.push_back(std::move(ptt));
  }

  return pt_pairs;
}

}  // namespace planning
}  // namespace apollo
