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
#include <limits>
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

double NaviObstacleDecider::GetMinLaneWidth(
    const std::vector<common::PathPoint>& path_data_points,
    const ReferenceLine& reference_line) {
  double min_lane_width = std::numeric_limits<double>::max();
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  for (const auto& path_data_point : path_data_points) {
    bool bRet = reference_line.GetLaneWidth(
        path_data_point.s(), &lane_left_width, &lane_right_width);
    if (bRet) {
      double lane_width = lane_left_width + lane_right_width;
      if (lane_width < min_lane_width) {
        min_lane_width = lane_width;
      }
    }
  }
  return min_lane_width;
}

void NaviObstacleDecider::JudgePointLeftOrRight(
    const common::PathPoint& projection_point,
    const std::vector<common::PathPoint>& path_data_points,
    const Obstacle* current_obstacle, const double proj_len, double* dist) {
  Vec2d p1(0.0, 0.0);
  Vec2d p2(0.0, 0.0);

  p1.set_x(projection_point.x());
  p1.set_y(projection_point.y());
  if ((proj_len + 1) > path_data_points.back().s()) {
    p2.set_x(path_data_points.back().x());
    p2.set_y(path_data_points.back().y());
  } else {
    auto point = PathMatcher::MatchToPath(path_data_points, (proj_len + 1));
    p2.set_x(point.x());
    p2.set_y(point.y());
  }
  auto d = ((current_obstacle->Perception().position().x() - p1.x()) *
            (p2.y() - p1.y())) -
           ((current_obstacle->Perception().position().y() - p1.y()) *
            (p2.x() - p1.x()));
  if (d > 0) {
    *dist = *dist * -1;
  }
}

int NaviObstacleDecider::ProcessPathObstacle(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<common::PathPoint>& path_data_points,
    const PathDecision& path_decision, const double min_lane_width) {
  auto func_distance = [](const PathPoint& point, const double x,
                          const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return sqrt(dx * dx + dy * dy);
  };
  int obstacles_num = 0;
  PathPoint projection_point = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  PathPoint point = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Vec2d p1(0.0, 0.0);
  Vec2d p2(0.0, 0.0);

  for (const auto& current_obstacle : obstacles) {
    if (current_obstacle->Perception().has_velocity() &&
        current_obstacle->Perception().velocity().has_x()) {
      if (current_obstacle->Perception().velocity().x() > 0.0) {
        continue;
      }
      if (!FLAGS_enable_side_radar) {
        if (current_obstacle->Perception().velocity().x() < 0.0) {
          continue;
        }
      }
    }

    const auto* dest_ptr = path_decision.Find(current_obstacle->Id());

    if (fabs(dest_ptr->PerceptionSLBoundary().start_l()) <
        (min_lane_width / 2)) {
      obstacles_num = obstacles_num + 1;
    }
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
      JudgePointLeftOrRight(projection_point, path_data_points,
                            current_obstacle, proj_len, &dist);
      obstacle_lat_dist_.emplace(std::pair<double, double>(
          current_obstacle->Perception().width(), dist));
    }
  }
  return obstacles_num;
}

double NaviObstacleDecider::GetNudgeDistance(
    const std::vector<const Obstacle*>& obstacles,
    const ReferenceLine& reference_line, const PathDecision& path_decision,
    const std::vector<common::PathPoint>& path_data_points,
    int* lane_obstacles_num) {
  // Calculating the left and right nudgeable distance on the lane
  double left_nudge_lane = 0.0;
  double right_nedge_lane = 0.0;
  double routing_y = path_data_points[0].y();
  double min_lane_width = GetMinLaneWidth(path_data_points, reference_line);
  ADEBUG << "get min_lane_width: " << min_lane_width;
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
  // Calculation of the number of current Lane obstacles
  obstacle_lat_dist_.clear();
  *lane_obstacles_num = ProcessPathObstacle(obstacles, path_data_points,
                                            path_decision, min_lane_width);
  for (auto iter = obstacle_lat_dist_.begin(); iter != obstacle_lat_dist_.end();
       iter++) {
    auto obs_width = iter->first;
    auto lat_dist = iter->second;
    ADEBUG << "get obstacle width : " << obs_width
           << "get lattitude distance : " << lat_dist;
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
        } else if (left_nudge_obstacle < need_nudge_dist) {
          left_nudge_obstacle = need_nudge_dist;
        }
      }
    }
  }
  ADEBUG << "get left_nudge_lane: " << left_nudge_lane
         << "get right_nedge_lane : " << right_nedge_lane
         << "get left_nudge_obstacle: " << left_nudge_obstacle
         << "get right_nudge_obstacle : " << right_nudge_obstacle;
  // Get the appropriate value of the nudge distance
  double nudge_dist = 0.0;
  if ((0.0 != left_nudge_obstacle) && (0.0 == right_nudge_obstacle)) {
    if (left_nudge_lane < left_nudge_obstacle) {
      nudge_dist = left_nudge_lane;
    } else {
      nudge_dist = left_nudge_obstacle;
    }
  } else if ((0.0 == left_nudge_obstacle) && (0.0 != right_nudge_obstacle)) {
    if (right_nedge_lane > right_nudge_obstacle) {
      nudge_dist = right_nedge_lane;
    } else {
      nudge_dist = right_nudge_obstacle;
    }
  }
  ADEBUG << "get nudge distance : " << nudge_dist
         << "get lane_obstacles_num : " << *lane_obstacles_num;
  return nudge_dist;
}

void NaviObstacleDecider::GetUnsafeObstaclesInfo(
    const std::vector<common::PathPoint>& path_data_points,
    const std::vector<const Obstacle*>& obstacles) {
  constexpr double kSafeDistance = 0.2;  // Distance from the edge of the car.

  // Find start point of the reference line.
  double reference_line_y = path_data_points[0].y();

  // Judging unsafed range according to the position of the reference line.
  double unsafe_refline_pos_y = 0.0;
  double unsafe_car_pos_y = 0.0;
  std::pair<double, double> unsafe_range;
  if (reference_line_y < 0.0) {
    unsafe_refline_pos_y = reference_line_y -
                           VehicleParam().right_edge_to_center() -
                           kSafeDistance;
    unsafe_car_pos_y = VehicleParam().right_edge_to_center() + kSafeDistance;
    unsafe_range = std::make_pair(unsafe_refline_pos_y, unsafe_car_pos_y);
  } else {
    unsafe_refline_pos_y =
        reference_line_y + VehicleParam().left_edge_to_center() + kSafeDistance;
    unsafe_car_pos_y =
        -1.0 * (VehicleParam().left_edge_to_center() + kSafeDistance);
    unsafe_range = std::make_pair(unsafe_car_pos_y, unsafe_refline_pos_y);
  }
  // Get obstacles'ID.
  unsafe_obstacle_info_.clear();
  for (const auto& iter : obstacles) {
    double obstacle_y = iter->Perception().position().y();
    if ((obstacle_y > unsafe_range.first) &&
        (obstacle_y < unsafe_range.second)) {
      auto projection_point = PathMatcher::MatchToPath(
          path_data_points, iter->Perception().position().x(),
          iter->Perception().position().y());
      auto ref_theta = projection_point.theta();
      auto project_velocity =
          iter->Perception().velocity().x() * std::cos(ref_theta) +
          iter->Perception().velocity().y() * std::sin(ref_theta);
      unsafe_obstacle_info_.emplace_back(iter->Id(), projection_point.s(),
                                         project_velocity);
    }
  }
}
}  // namespace planning
}  // namespace apollo
