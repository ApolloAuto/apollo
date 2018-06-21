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
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::math::InterpolateUsingLinearApproximation;
using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;

namespace {
constexpr double kMaxNudgeDistance = 0.9;
constexpr double kMinNudgeDistance = 0.2;
}  // namespace

NaviObstacleDecider::NaviObstacleDecider() : Task("NaviObstacleDecider") {
  // TODO(all): Add your other initialization.
}

void NaviObstacleDecider::ProcessPathObstacle(
    const std::vector<const Obstacle*>& obstacles, LocalPath* fpath) {
  if (path_obstacle_processed_) {
    return;
  }
  std::vector<Vec2d> path = fpath->GetXYPoints();
  const LineSegment2d line(path.front(), path.back());
  Vec2d p1(0.0, 0.0);
  Vec2d p2(0.0, 0.0);
  for (const auto& current_obstacle : obstacles) {
    auto current_xypoint = Vec2d(current_obstacle->Perception().position().x(),
                                 current_obstacle->Perception().position().y());
    auto dist = line.DistanceTo(current_xypoint);
    if (dist < (kMaxNudgeDistance + current_obstacle->Perception().width() +
                VehicleParam().left_edge_to_center())) {
      auto proj_len = line.ProjectOntoUnit(current_xypoint);
      if ((proj_len == 0) || (proj_len >= line.length())) {
        continue;
      }
      PathPoint point = InterpolateUsingLinearApproximation(
          fpath->GetPathPoints().front(), fpath->GetPathPoints().back(),
          proj_len);
      p1.set_x(point.x());
      p1.set_y(point.y());
      if ((proj_len + 1) > line.length()) {
        point = InterpolateUsingLinearApproximation(
            fpath->GetPathPoints().front(), fpath->GetPathPoints().back(),
            line.length());
        p2.set_x(point.x());
        p2.set_y(point.y());
      } else {
        point = InterpolateUsingLinearApproximation(
            fpath->GetPathPoints().front(), fpath->GetPathPoints().back(),
            (proj_len + 1));
        p2.set_x(point.x());
        p2.set_y(point.y());
      }
      auto d = ((current_xypoint.x() - p1.x()) * (p2.y() - p1.y())) -
               ((current_xypoint.y() - p1.y()) * (p2.x() - p1.x()));
      if (d > 0) {
        dist *= -1;
      }
      obstacle_lat_dist_.emplace(std::pair<double, double>(
          current_obstacle->Perception().width(), dist));
    }
  }
  path_obstacle_processed_ = true;
}

void NaviObstacleDecider::GetLeftRightNudgableDistance(const double lan_width,
                                                       LocalPath* fpath,
                                                       double* left_nudgable,
                                                       double* right_nudgable) {
  double routing_y = 0.0;
  const auto ret = fpath->GetInitY(&routing_y);
  if (ret == false) {
    return;
  }

  // Calculating the left and right nudgeable distance on the lane
  if (routing_y <= 0.0) {
    *left_nudgable = lan_width / 2.0 - fabs(routing_y) -
                     VehicleParam().left_edge_to_center();
    *right_nudgable = lan_width / 2.0 + fabs(routing_y) -
                      VehicleParam().right_edge_to_center();
  } else {
    *left_nudgable = lan_width / 2.0 + fabs(routing_y) -
                     VehicleParam().left_edge_to_center();
    *right_nudgable = lan_width / 2.0 - fabs(routing_y) -
                      VehicleParam().right_edge_to_center();
  }
}

double NaviObstacleDecider::GetNudgeDistance(const double left_nudgable,
                                             const double right_nudgable) {
  double left_nudge = 0.0;
  double right_nudge = 0.0;

  // Calculate the distance required to get around obstacles.
  const auto& obstacle_lat_dist = MutableObstacleLatDistance();
  for (auto iter = obstacle_lat_dist.begin(); iter != obstacle_lat_dist.end();
       iter++) {
    auto obs_width = iter->first;
    auto lat_dist = iter->second;
    auto actual_dist =
        fabs(lat_dist) - obs_width / 2.0 - VehicleParam().left_edge_to_center();
    if ((actual_dist > kMinNudgeDistance) &&
        (actual_dist < kMaxNudgeDistance)) {
      auto need_nudge_dist = kMaxNudgeDistance - actual_dist;
      if (lat_dist >= 0.0) {
        if (0.0 == right_nudge) {
          right_nudge = -1 * need_nudge_dist;
        } else if (right_nudge > -1 * need_nudge_dist) {
          right_nudge = -1 * need_nudge_dist;
        }
      } else {
        if (0.0 == left_nudge) {
          left_nudge = need_nudge_dist;
        } else if (left_nudge < (need_nudge_dist)) {
          left_nudge = need_nudge_dist;
        }
      }
    }
  }
  double nudge_dist = 0.0;
  if ((0.0 != left_nudge) && (0.0 == right_nudge)) {
    if (left_nudgable < left_nudge) {
      nudge_dist = left_nudgable;
    } else {
      nudge_dist = left_nudge;
    }
  } else if ((0.0 == left_nudge) && (0.0 != right_nudge)) {
    if (right_nudgable > right_nudge) {
      nudge_dist = right_nudgable;
    } else {
      nudge_dist = right_nudge;
    }
  }
  return nudge_dist;
}
}  // namespace planning
}  // namespace apollo
