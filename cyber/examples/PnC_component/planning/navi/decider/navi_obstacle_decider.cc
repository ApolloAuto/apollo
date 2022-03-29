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

#include <algorithm>
#include <limits>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/path_matcher.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::math::PathMatcher;
using apollo::common::math::Vec2d;

namespace {
constexpr double kEpislon = 1e-6;
}  // namespace

NaviObstacleDecider::NaviObstacleDecider() : NaviTask("NaviObstacleDecider") {}

bool NaviObstacleDecider::Init(const PlanningConfig& config) {
  PlannerNaviConfig planner_navi_conf =
      config.navigation_planning_config().planner_navi_config();
  config_ = planner_navi_conf.navi_obstacle_decider_config();
  return true;
}

double NaviObstacleDecider::GetMinLaneWidth(
    const std::vector<common::PathPoint>& path_data_points,
    const ReferenceLine& reference_line) {
  double min_lane_width = std::numeric_limits<double>::max();
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  for (const auto& path_data_point : path_data_points) {
    bool ret = reference_line.GetLaneWidth(path_data_point.s(),
                                           &lane_left_width, &lane_right_width);
    if (ret) {
      double lane_width = lane_left_width + lane_right_width;
      if (lane_width < min_lane_width) {
        min_lane_width = lane_width;
      }
    }
  }
  return min_lane_width;
}

void NaviObstacleDecider::AddObstacleOffsetDirection(
    const common::PathPoint& projection_point,
    const std::vector<common::PathPoint>& path_data_points,
    const Obstacle* current_obstacle, const double proj_len, double* dist) {
  Vec2d p1(projection_point.x(), projection_point.y());
  Vec2d p2(0.0, 0.0);
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

bool NaviObstacleDecider::IsNeedFilterObstacle(
    const Obstacle* current_obstacle, const PathPoint& vehicle_projection_point,
    const std::vector<common::PathPoint>& path_data_points,
    const common::VehicleState& vehicle_state,
    PathPoint* projection_point_ptr) {
  bool is_filter = true;
  *projection_point_ptr = PathMatcher::MatchToPath(
      path_data_points, current_obstacle->Perception().position().x(),
      current_obstacle->Perception().position().y());
  ADEBUG << "obstacle distance : " << projection_point_ptr->s()
         << "vehicle distance : " << vehicle_projection_point.s();
  if ((projection_point_ptr->s() - vehicle_projection_point.s()) >
      (config_.judge_dis_coeff() * vehicle_state.linear_velocity() +
       config_.basis_dis_value())) {
    return is_filter;
  }
  double vehicle_frontedge_position =
      vehicle_projection_point.s() + VehicleParam().length();
  double vehicle_backedge_position = vehicle_projection_point.s();

  double obstacle_start_position =
      projection_point_ptr->s() - current_obstacle->Perception().length() / 2.0;
  double obstacle_end_position =
      projection_point_ptr->s() + current_obstacle->Perception().length() / 2.0;
  if ((vehicle_backedge_position - obstacle_end_position) >
      config_.safe_distance()) {
    return is_filter;
  }
  if ((obstacle_start_position - vehicle_frontedge_position) >
      config_.safe_distance()) {
    if (!current_obstacle->IsStatic()) {
      if (current_obstacle->Perception().velocity().x() > 0.0) {
        return is_filter;
      }
    }
  }
  is_filter = false;
  return is_filter;
}

void NaviObstacleDecider::ProcessObstacle(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<common::PathPoint>& path_data_points,
    const PathDecision& path_decision, const double min_lane_width,
    const common::VehicleState& vehicle_state) {
  auto func_distance = [](const PathPoint& point, const double x,
                          const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return sqrt(dx * dx + dy * dy);
  };

  PathPoint projection_point;
  PathPoint vehicle_projection_point =
      PathMatcher::MatchToPath(path_data_points, 0, 0);
  for (const auto& current_obstacle : obstacles) {
    bool is_continue = IsNeedFilterObstacle(
        current_obstacle, vehicle_projection_point, path_data_points,
        vehicle_state, &projection_point);
    if (is_continue) {
      continue;
    }
    auto dist = func_distance(projection_point,
                              current_obstacle->Perception().position().x(),
                              current_obstacle->Perception().position().y());

    if (dist <
        (config_.max_nudge_distance() + current_obstacle->Perception().width() +
         VehicleParam().left_edge_to_center())) {
      auto proj_len = projection_point.s();
      if (std::fabs(proj_len) <= kEpislon ||
          proj_len >= path_data_points.back().s()) {
        continue;
      }
      AddObstacleOffsetDirection(projection_point, path_data_points,
                                 current_obstacle, proj_len, &dist);
      obstacle_lat_dist_.emplace(std::pair<double, double>(
          current_obstacle->Perception().width(), dist));
    }
  }
}

double NaviObstacleDecider::GetObstacleActualOffsetDistance(
    std::map<double, double>::iterator iter, const double right_nudge_lane,
    const double left_nudge_lane, int* lane_obstacles_num) {
  auto obs_width = iter->first;
  auto lat_dist = iter->second;
  ADEBUG << "get obstacle width : " << obs_width
         << "get latitude distance : " << lat_dist;
  auto actual_dist = std::fabs(lat_dist) - obs_width / 2.0 -
                     VehicleParam().left_edge_to_center();

  if (last_nudge_dist_ > 0) {
    if (lat_dist < 0) {
      if (actual_dist < std::fabs(right_nudge_lane) + config_.safe_distance()) {
        *lane_obstacles_num = *lane_obstacles_num + 1;
      }
    }
  } else if (last_nudge_dist_ < 0) {
    if (lat_dist > 0) {
      if (actual_dist < std::fabs(left_nudge_lane) + config_.safe_distance()) {
        *lane_obstacles_num = *lane_obstacles_num + 1;
      }
    }
  }

  if ((last_lane_obstacles_num_ != 0) && (*lane_obstacles_num == 0) &&
      (!is_obstacle_stable_)) {
    is_obstacle_stable_ = true;
    statist_count_ = 0;
    ADEBUG << "begin keep obstacles";
  }

  if (is_obstacle_stable_) {
    ++statist_count_;
    if (statist_count_ > config_.cycles_number()) {
      is_obstacle_stable_ = false;
    } else {
      *lane_obstacles_num = last_lane_obstacles_num_;
    }
    ADEBUG << "statist_count_ : " << statist_count_;
  }
  last_lane_obstacles_num_ = *lane_obstacles_num;
  ADEBUG << "last_nudge_dist : " << last_nudge_dist_
         << "lat_dist : " << lat_dist << "actual_dist : " << actual_dist;
  return actual_dist;
}

void NaviObstacleDecider::RecordLastNudgeDistance(const double nudge_dist) {
  double tolerance = config_.nudge_allow_tolerance();

  if (std::fabs(nudge_dist) > tolerance) {
    if (std::fabs(nudge_dist) > std::fabs(last_nudge_dist_)) {
      last_nudge_dist_ = nudge_dist;
    }
    no_nudge_num_ = 0;
  } else {
    ++no_nudge_num_;
  }

  if (no_nudge_num_ >= config_.cycles_number()) {
    last_nudge_dist_ = 0.0;
  }
}

void NaviObstacleDecider::SmoothNudgeDistance(
    const common::VehicleState& vehicle_state, double* nudge_dist) {
  CHECK_NOTNULL(nudge_dist);
  if (vehicle_state.linear_velocity() < config_.max_allow_nudge_speed()) {
    ++limit_speed_num_;
  } else {
    limit_speed_num_ = 0;
  }
  if (limit_speed_num_ < config_.cycles_number()) {
    *nudge_dist = 0;
  }
  if (std::fabs(*nudge_dist) > config_.nudge_allow_tolerance()) {
    ++eliminate_clutter_num_;
  } else {
    eliminate_clutter_num_ = 0;
  }
  if (eliminate_clutter_num_ < config_.cycles_number()) {
    *nudge_dist = 0;
  }
  ADEBUG << "eliminate_clutter_num_: " << eliminate_clutter_num_;
}

double NaviObstacleDecider::GetNudgeDistance(
    const std::vector<const Obstacle*>& obstacles,
    const ReferenceLine& reference_line, const PathDecision& path_decision,
    const std::vector<common::PathPoint>& path_data_points,
    const common::VehicleState& vehicle_state, int* lane_obstacles_num) {
  CHECK_NOTNULL(lane_obstacles_num);

  // Calculating the left and right nudgeable distance on the lane
  double left_nudge_lane = 0.0;
  double right_nudge_lane = 0.0;
  double routing_y = path_data_points[0].y();
  double min_lane_width = GetMinLaneWidth(path_data_points, reference_line);

  ADEBUG << "get min_lane_width: " << min_lane_width;
  if (routing_y <= 0.0) {
    left_nudge_lane = min_lane_width / 2.0 - std::fabs(routing_y) -
                      VehicleParam().left_edge_to_center();
    right_nudge_lane = -1.0 * (min_lane_width / 2.0 + std::fabs(routing_y) -
                               VehicleParam().right_edge_to_center());
  } else {
    left_nudge_lane = min_lane_width / 2.0 + std::fabs(routing_y) -
                      VehicleParam().left_edge_to_center();
    right_nudge_lane = -1.0 * (min_lane_width / 2.0 - std::fabs(routing_y) -
                               VehicleParam().right_edge_to_center());
  }

  // Calculating the left and right nudgable distance according to the
  // position of the obstacle.
  double left_nudge_obstacle = 0.0;
  double right_nudge_obstacle = 0.0;

  // Calculation of the number of current Lane obstacles
  obstacle_lat_dist_.clear();
  ProcessObstacle(obstacles, path_data_points, path_decision, min_lane_width,
                  vehicle_state);
  for (auto iter = obstacle_lat_dist_.begin(); iter != obstacle_lat_dist_.end();
       ++iter) {
    auto actual_dist = GetObstacleActualOffsetDistance(
        iter, right_nudge_lane, left_nudge_lane, lane_obstacles_num);
    auto lat_dist = iter->second;
    if (actual_dist > config_.min_nudge_distance() &&
        actual_dist < config_.max_nudge_distance()) {
      auto need_nudge_dist = config_.max_nudge_distance() - actual_dist;
      if (lat_dist >= 0.0) {
        right_nudge_obstacle =
            -1.0 * std::max(std::fabs(right_nudge_obstacle), need_nudge_dist);
      } else {
        left_nudge_obstacle =
            std::max(std::fabs(left_nudge_obstacle), need_nudge_dist);
      }
    }
  }
  ADEBUG << "get left_nudge_lane: " << left_nudge_lane
         << "get right_nudge_lane : " << right_nudge_lane
         << "get left_nudge_obstacle: " << left_nudge_obstacle
         << "get right_nudge_obstacle : " << right_nudge_obstacle;
  // Get the appropriate value of the nudge distance
  double nudge_dist = 0.0;
  if (std::fabs(left_nudge_obstacle) > kEpislon &&
      std::fabs(right_nudge_obstacle) <= kEpislon) {
    nudge_dist = std::min(left_nudge_lane, left_nudge_obstacle);
  } else if (std::fabs(right_nudge_obstacle) > kEpislon &&
             std::fabs(left_nudge_obstacle) <= kEpislon) {
    nudge_dist = std::max(right_nudge_lane, right_nudge_obstacle);
  }

  ADEBUG << "get nudge distance : " << nudge_dist
         << "get lane_obstacles_num : " << *lane_obstacles_num;
  RecordLastNudgeDistance(nudge_dist);
  SmoothNudgeDistance(vehicle_state, &nudge_dist);
  KeepNudgePosition(nudge_dist, lane_obstacles_num);
  ADEBUG << "last nudge distance : " << nudge_dist;
  return nudge_dist;
}

void NaviObstacleDecider::KeepNudgePosition(const double nudge_dist,
                                            int* lane_obstacles_num) {
  if (std::fabs(nudge_dist) > config_.nudge_allow_tolerance() &&
      std::fabs(last_nudge_dist_) < config_.nudge_allow_tolerance() &&
      !keep_nudge_flag_) {
    cycles_count_ = 0;
    keep_nudge_flag_ = true;
  }
  if (keep_nudge_flag_) {
    ++cycles_count_;
    if (cycles_count_ > config_.max_keep_nudge_cycles()) {
      *lane_obstacles_num = 0;
      keep_nudge_flag_ = false;
    } else {
      *lane_obstacles_num = 1;
    }
  }
  ADEBUG << "get lane_obstacles_num : " << *lane_obstacles_num;
}

void NaviObstacleDecider::GetUnsafeObstaclesInfo(
    const std::vector<common::PathPoint>& path_data_points,
    const std::vector<const Obstacle*>& obstacles) {
  // Find start point of the reference line.
  double reference_line_y = path_data_points[0].y();

  // Judging unsafed range according to the position of the reference line.
  double unsafe_refline_pos_y = 0.0;
  double unsafe_car_pos_y = 0.0;
  std::pair<double, double> unsafe_range;
  if (reference_line_y < 0.0) {
    unsafe_refline_pos_y = reference_line_y -
                           VehicleParam().right_edge_to_center() -
                           config_.speed_decider_detect_range();
    unsafe_car_pos_y = VehicleParam().right_edge_to_center() +
                       config_.speed_decider_detect_range();
    unsafe_range = std::make_pair(unsafe_refline_pos_y, unsafe_car_pos_y);
  } else {
    unsafe_refline_pos_y = reference_line_y +
                           VehicleParam().left_edge_to_center() +
                           config_.speed_decider_detect_range();
    unsafe_car_pos_y = -1.0 * (VehicleParam().left_edge_to_center() +
                               config_.speed_decider_detect_range());
    unsafe_range = std::make_pair(unsafe_car_pos_y, unsafe_refline_pos_y);
  }
  // Get obstacles'ID.
  unsafe_obstacle_info_.clear();
  PathPoint vehicle_projection_point =
      PathMatcher::MatchToPath(path_data_points, 0, 0);
  for (const auto& iter : obstacles) {
    const double obstacle_y = iter->Perception().position().y();
    if ((obstacle_y > unsafe_range.first && obstacle_y < unsafe_range.second) ||
        std::abs(iter->Perception().velocity().y()) >
            config_.lateral_velocity_value()) {
      auto projection_point = PathMatcher::MatchToPath(
          path_data_points, iter->Perception().position().x(), obstacle_y);
      if (vehicle_projection_point.s() >= projection_point.s()) {
        continue;
      }
      auto front_distance =
          (projection_point.s() - iter->Perception().length() / 2.0) -
          vehicle_projection_point.s();
      auto ref_theta = projection_point.theta();
      auto project_velocity =
          iter->Perception().velocity().x() * std::cos(ref_theta) +
          iter->Perception().velocity().y() * std::sin(ref_theta);
      ADEBUG << "Lateral speed : " << iter->Perception().velocity().y();
      unsafe_obstacle_info_.emplace_back(iter->Id(), front_distance,
                                         project_velocity);
    }
  }
}
}  // namespace planning
}  // namespace apollo
