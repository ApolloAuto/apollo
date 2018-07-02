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
 * @brief This file provides the implementation of the class
 * "NaviObstacleDecider".
 */
#include "modules/planning/navi/decider/navi_obstacle_decider.h"

#include <cfloat>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/path_matcher.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::math::InterpolateUsingLinearApproximation;
using apollo::common::math::LineSegment2d;
using apollo::common::math::PathMatcher;
using apollo::common::math::Vec2d;
using apollo::common::util::MakePathPoint;

namespace {
constexpr double kMaxNudgeDistance = 0.9;
constexpr double kMinNudgeDistance = 0.2;
}  // namespace

NaviObstacleDecider::NaviObstacleDecider() : Task("NaviObstacleDecider") {}

void NaviObstacleDecider::ProcessPathObstacle(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<common::PathPoint>& path_data_points) {
  auto func_distance = [](const PathPoint& point, const double x,
                          const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return sqrt(dx * dx + dy * dy);
  };
  PathPoint projection_point = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  PathPoint point = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Vec2d p1(0.0, 0.0);
  Vec2d p2(0.0, 0.0);
  for (const auto& current_obstacle : obstacles) {
    projection_point = PathMatcher::MatchToPath(
        path_data_points, current_obstacle->Perception().position().x(),
        current_obstacle->Perception().position().y());
    auto dist = func_distance(projection_point,
                              current_obstacle->Perception().position().x(),
                              current_obstacle->Perception().position().y());

    if (dist < (kMaxNudgeDistance + current_obstacle->Perception().width() +
                VehicleParam().left_edge_to_center())) {
      auto proj_len = projection_point.s();
      if ((proj_len == 0) || (proj_len >= path_data_points.back().s())) {
        continue;
      }
      p1.set_x(projection_point.x());
      p1.set_y(projection_point.y());
      if ((proj_len + 1) > path_data_points.back().s()) {
        p2.set_x(path_data_points.back().x());
        p2.set_y(path_data_points.back().y());
      } else {
        point = PathMatcher::MatchToPath(path_data_points, (proj_len + 1));
        p2.set_x(point.x());
        p2.set_y(point.y());
      }
      auto d = ((current_obstacle->Perception().position().x() - p1.x()) *
                (p2.y() - p1.y())) -
               ((current_obstacle->Perception().position().y() - p1.y()) *
                (p2.x() - p1.x()));
      if (d > 0) {
        dist *= -1;
      }
      obstacle_lat_dist_.emplace(std::pair<double, double>(
          current_obstacle->Perception().width(), dist));
    }
  }
}

double NaviObstacleDecider::GetNudgeDistance(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<common::PathPoint>& path_data_points,
    const double min_lane_width) {
  // Calculating the left and right nudgeable distance on the lane
  double left_nudge_lane = 0.0;
  double right_nedge_lane = 0.0;
  double routing_y = path_data_points[0].y();
  if (routing_y <= 0.0) {
    left_nudge_lane = min_lane_width / 2.0 - fabs(routing_y) -
                      VehicleParam().left_edge_to_center();
    right_nedge_lane = -1.0 * (min_lane_width / 2.0 + fabs(routing_y) -
                               VehicleParam().right_edge_to_center());
  } else {
    left_nudge_lane = min_lane_width / 2.0 + fabs(routing_y) -
                      VehicleParam().left_edge_to_center();
    right_nedge_lane = -1.0 * (min_lane_width / 2.0 - fabs(routing_y) -
                               VehicleParam().right_edge_to_center());
  }

  // Calculating the left and right nudgable distance according to the position
  // of the obstacle.
  double left_nudge_obstacle = 0.0;
  double right_nudge_obstacle = 0.0;
  ProcessPathObstacle(obstacles, path_data_points);
  for (auto iter = obstacle_lat_dist_.begin(); iter != obstacle_lat_dist_.end();
       iter++) {
    auto obs_width = iter->first;
    auto lat_dist = iter->second;
    auto actual_dist =
        fabs(lat_dist) - obs_width / 2.0 - VehicleParam().left_edge_to_center();
    if ((actual_dist > kMinNudgeDistance) &&
        (actual_dist < kMaxNudgeDistance)) {
      auto need_nudge_dist = kMaxNudgeDistance - actual_dist;
      if (lat_dist >= 0.0) {
        if (0.0 == right_nudge_obstacle) {
          right_nudge_obstacle = -1 * need_nudge_dist;
        } else if (right_nudge_obstacle > -1 * need_nudge_dist) {
          right_nudge_obstacle = -1 * need_nudge_dist;
        }
      } else {
        if (0.0 == left_nudge_obstacle) {
          left_nudge_obstacle = need_nudge_dist;
        } else if (left_nudge_obstacle < (need_nudge_dist)) {
          left_nudge_obstacle = need_nudge_dist;
        }
      }
    }
  }
  // Get the appropriate value of the nudge distance
  double nudge_dist = 0.0;
  if ((0.0 != left_nudge_obstacle) && (0.0 == right_nudge_obstacle)) {
    if (left_nudge_lane < left_nudge_obstacle) {
      nudge_dist = left_nudge_lane;
    } else {
      nudge_dist = left_nudge_obstacle;
    }
  } else if ((0.0 == left_nudge_obstacle) && (0.0 != right_nudge_obstacle)) {
    if (fabs(right_nedge_lane) > fabs(right_nudge_obstacle)) {
      nudge_dist = right_nedge_lane;
    } else {
      nudge_dist = right_nudge_obstacle;
    }
  }
  return nudge_dist;
}

}  // namespace planning
}  // namespace apollo
