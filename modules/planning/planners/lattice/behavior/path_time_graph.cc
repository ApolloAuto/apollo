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

#include "modules/planning/planners/lattice/behavior/path_time_graph.h"

#include <algorithm>
#include <limits>

#include "modules/common_msgs/planning_msgs/sl_boundary.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/path_matcher.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::lerp;
using apollo::common::math::PathMatcher;
using apollo::common::math::Polygon2d;

PathTimeGraph::PathTimeGraph(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points,
    const ReferenceLineInfo* ptr_reference_line_info, const double s_start,
    const double s_end, const double t_start, const double t_end,
    const std::array<double, 3>& init_d) {
  CHECK_LT(s_start, s_end);
  CHECK_LT(t_start, t_end);
  path_range_.first = s_start;
  path_range_.second = s_end;
  time_range_.first = t_start;
  time_range_.second = t_end;
  ptr_reference_line_info_ = ptr_reference_line_info;
  init_d_ = init_d;

  SetupObstacles(obstacles, discretized_ref_points);
}

SLBoundary PathTimeGraph::ComputeObstacleBoundary(
    const std::vector<common::math::Vec2d>& vertices,
    const std::vector<PathPoint>& discretized_ref_points) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  for (const auto& point : vertices) {
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
    } else {
      SetDynamicObstacle(obstacle, discretized_ref_points);
    }
  }

  std::sort(static_obs_sl_boundaries_.begin(), static_obs_sl_boundaries_.end(),
            [](const SLBoundary& sl0, const SLBoundary& sl1) {
              return sl0.start_s() < sl1.start_s();
            });

  for (auto& path_time_obstacle : path_time_obstacle_map_) {
    path_time_obstacles_.push_back(path_time_obstacle.second);
  }
}

void PathTimeGraph::SetStaticObstacle(
    const Obstacle* obstacle,
    const std::vector<PathPoint>& discretized_ref_points) {
  const Polygon2d& polygon = obstacle->PerceptionPolygon();

  std::string obstacle_id = obstacle->Id();
  SLBoundary sl_boundary =
      ComputeObstacleBoundary(polygon.GetAllVertices(), discretized_ref_points);

  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;
  ptr_reference_line_info_->reference_line().GetLaneWidth(
      sl_boundary.start_s(), &left_width, &right_width);
  if (sl_boundary.start_s() > path_range_.second ||
      sl_boundary.end_s() < path_range_.first ||
      sl_boundary.start_l() > left_width ||
      sl_boundary.end_l() < -right_width) {
    ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.";
    return;
  }

  path_time_obstacle_map_[obstacle_id].set_id(obstacle_id);
  path_time_obstacle_map_[obstacle_id].set_bottom_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.start_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].set_bottom_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.start_s(), FLAGS_trajectory_time_length));
  path_time_obstacle_map_[obstacle_id].set_upper_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.end_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].set_upper_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.end_s(), FLAGS_trajectory_time_length));
  static_obs_sl_boundaries_.push_back(std::move(sl_boundary));
  ADEBUG << "ST-Graph mapping static obstacle: " << obstacle_id
         << ", start_s : " << sl_boundary.start_s()
         << ", end_s : " << sl_boundary.end_s()
         << ", start_l : " << sl_boundary.start_l()
         << ", end_l : " << sl_boundary.end_l();
}

void PathTimeGraph::SetDynamicObstacle(
    const Obstacle* obstacle,
    const std::vector<PathPoint>& discretized_ref_points) {
  double relative_time = time_range_.first;
  while (relative_time < time_range_.second) {
    TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
    Box2d box = obstacle->GetBoundingBox(point);
    SLBoundary sl_boundary =
        ComputeObstacleBoundary(box.GetAllCorners(), discretized_ref_points);

    double left_width = FLAGS_default_reference_line_width * 0.5;
    double right_width = FLAGS_default_reference_line_width * 0.5;
    ptr_reference_line_info_->reference_line().GetLaneWidth(
        sl_boundary.start_s(), &left_width, &right_width);

    // The obstacle is not shown on the region to be considered.
    if (sl_boundary.start_s() > path_range_.second ||
        sl_boundary.end_s() < path_range_.first ||
        sl_boundary.start_l() > left_width ||
        sl_boundary.end_l() < -right_width) {
      if (path_time_obstacle_map_.find(obstacle->Id()) !=
          path_time_obstacle_map_.end()) {
        break;
      }
      relative_time += FLAGS_trajectory_time_resolution;
      continue;
    }

    if (path_time_obstacle_map_.find(obstacle->Id()) ==
        path_time_obstacle_map_.end()) {
      path_time_obstacle_map_[obstacle->Id()].set_id(obstacle->Id());

      path_time_obstacle_map_[obstacle->Id()].set_bottom_left_point(
          SetPathTimePoint(obstacle->Id(), sl_boundary.start_s(),
                           relative_time));
      path_time_obstacle_map_[obstacle->Id()].set_upper_left_point(
          SetPathTimePoint(obstacle->Id(), sl_boundary.end_s(), relative_time));
    }

    path_time_obstacle_map_[obstacle->Id()].set_bottom_right_point(
        SetPathTimePoint(obstacle->Id(), sl_boundary.start_s(), relative_time));
    path_time_obstacle_map_[obstacle->Id()].set_upper_right_point(
        SetPathTimePoint(obstacle->Id(), sl_boundary.end_s(), relative_time));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

STPoint PathTimeGraph::SetPathTimePoint(const std::string& obstacle_id,
                                        const double s, const double t) const {
  STPoint path_time_point(s, t);
  return path_time_point;
}

const std::vector<STBoundary>& PathTimeGraph::GetPathTimeObstacles() const {
  return path_time_obstacles_;
}

bool PathTimeGraph::GetPathTimeObstacle(const std::string& obstacle_id,
                                        STBoundary* path_time_obstacle) {
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return false;
  }
  *path_time_obstacle = path_time_obstacle_map_[obstacle_id];
  return true;
}

std::vector<std::pair<double, double>> PathTimeGraph::GetPathBlockingIntervals(
    const double t) const {
  ACHECK(time_range_.first <= t && t <= time_range_.second);
  std::vector<std::pair<double, double>> intervals;
  for (const auto& pt_obstacle : path_time_obstacles_) {
    if (t > pt_obstacle.max_t() || t < pt_obstacle.min_t()) {
      continue;
    }
    double s_upper = lerp(pt_obstacle.upper_left_point().s(),
                          pt_obstacle.upper_left_point().t(),
                          pt_obstacle.upper_right_point().s(),
                          pt_obstacle.upper_right_point().t(), t);

    double s_lower = lerp(pt_obstacle.bottom_left_point().s(),
                          pt_obstacle.bottom_left_point().t(),
                          pt_obstacle.bottom_right_point().s(),
                          pt_obstacle.bottom_right_point().t(), t);

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

std::vector<STPoint> PathTimeGraph::GetObstacleSurroundingPoints(
    const std::string& obstacle_id, const double s_dist,
    const double t_min_density) const {
  ACHECK(t_min_density > 0.0);
  std::vector<STPoint> pt_pairs;
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
    s0 = pt_obstacle.upper_left_point().s();
    s1 = pt_obstacle.upper_right_point().s();

    t0 = pt_obstacle.upper_left_point().t();
    t1 = pt_obstacle.upper_right_point().t();
  } else {
    s0 = pt_obstacle.bottom_left_point().s();
    s1 = pt_obstacle.bottom_right_point().s();

    t0 = pt_obstacle.bottom_left_point().t();
    t1 = pt_obstacle.bottom_right_point().t();
  }

  double time_gap = t1 - t0;
  ACHECK(time_gap > -FLAGS_numerical_epsilon);
  time_gap = std::fabs(time_gap);

  size_t num_sections = static_cast<size_t>(time_gap / t_min_density + 1);
  double t_interval = time_gap / static_cast<double>(num_sections);

  for (size_t i = 0; i <= num_sections; ++i) {
    double t = t_interval * static_cast<double>(i) + t0;
    double s = lerp(s0, t0, s1, t1, t) + s_dist;

    STPoint ptt;
    ptt.set_t(t);
    ptt.set_s(s);
    pt_pairs.push_back(std::move(ptt));
  }

  return pt_pairs;
}

bool PathTimeGraph::IsObstacleInGraph(const std::string& obstacle_id) {
  return path_time_obstacle_map_.find(obstacle_id) !=
         path_time_obstacle_map_.end();
}

std::vector<std::pair<double, double>> PathTimeGraph::GetLateralBounds(
    const double s_start, const double s_end, const double s_resolution) {
  CHECK_LT(s_start, s_end);
  CHECK_GT(s_resolution, FLAGS_numerical_epsilon);
  std::vector<std::pair<double, double>> bounds;
  std::vector<double> discretized_path;
  double s_range = s_end - s_start;
  double s_curr = s_start;
  size_t num_bound = static_cast<size_t>(s_range / s_resolution);

  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_width = vehicle_config.vehicle_param().width();

  // Initialize bounds by reference line width
  for (size_t i = 0; i < num_bound; ++i) {
    double left_width = FLAGS_default_reference_line_width / 2.0;
    double right_width = FLAGS_default_reference_line_width / 2.0;
    ptr_reference_line_info_->reference_line().GetLaneWidth(s_curr, &left_width,
                                                            &right_width);
    double ego_d_lower = init_d_[0] - ego_width / 2.0;
    double ego_d_upper = init_d_[0] + ego_width / 2.0;
    bounds.emplace_back(
        std::min(-right_width, ego_d_lower - FLAGS_bound_buffer),
        std::max(left_width, ego_d_upper + FLAGS_bound_buffer));
    discretized_path.push_back(s_curr);
    s_curr += s_resolution;
  }

  for (const SLBoundary& static_sl_boundary : static_obs_sl_boundaries_) {
    UpdateLateralBoundsByObstacle(static_sl_boundary, discretized_path, s_start,
                                  s_end, &bounds);
  }

  for (size_t i = 0; i < bounds.size(); ++i) {
    bounds[i].first += ego_width / 2.0;
    bounds[i].second -= ego_width / 2.0;
    if (bounds[i].first >= bounds[i].second) {
      bounds[i].first = 0.0;
      bounds[i].second = 0.0;
    }
  }
  return bounds;
}

void PathTimeGraph::UpdateLateralBoundsByObstacle(
    const SLBoundary& sl_boundary, const std::vector<double>& discretized_path,
    const double s_start, const double s_end,
    std::vector<std::pair<double, double>>* const bounds) {
  if (sl_boundary.start_s() > s_end || sl_boundary.end_s() < s_start) {
    return;
  }
  auto start_iter = std::lower_bound(
      discretized_path.begin(), discretized_path.end(), sl_boundary.start_s());
  auto end_iter = std::upper_bound(
      discretized_path.begin(), discretized_path.end(), sl_boundary.start_s());
  size_t start_index = start_iter - discretized_path.begin();
  size_t end_index = end_iter - discretized_path.begin();
  if (sl_boundary.end_l() > -FLAGS_numerical_epsilon &&
      sl_boundary.start_l() < FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < end_index; ++i) {
      bounds->operator[](i).first = -FLAGS_numerical_epsilon;
      bounds->operator[](i).second = FLAGS_numerical_epsilon;
    }
    return;
  }
  if (sl_boundary.end_l() < FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      bounds->operator[](i).first =
          std::max(bounds->operator[](i).first,
                   sl_boundary.end_l() + FLAGS_nudge_buffer);
    }
    return;
  }
  if (sl_boundary.start_l() > -FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      bounds->operator[](i).second =
          std::min(bounds->operator[](i).second,
                   sl_boundary.start_l() - FLAGS_nudge_buffer);
    }
    return;
  }
}

}  // namespace planning
}  // namespace apollo
