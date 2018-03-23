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

#include "modules/planning/tasks/traffic_decider/change_lane.h"

#include <algorithm>

#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

namespace {
constexpr double kMinGuardVehicleSpeed = 1.0;
}  // namespace

ChangeLane::ChangeLane(const TrafficRuleConfig& config) : TrafficRule(config) {}

bool ChangeLane::FilterObstacles(ReferenceLineInfo* reference_line_info) {
  const auto& reference_line = reference_line_info->reference_line();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& path_decision = reference_line_info->path_decision();
  const PathObstacle* first_guard_vehicle = nullptr;
  constexpr double kGuardForwardDistance = 60;
  double max_s = 0.0;
  const double min_overtake_time = config_.change_lane().min_overtake_time();
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const auto* obstacle = path_obstacle->obstacle();
    if (!obstacle->HasTrajectory()) {
      continue;
    }
    if (path_obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s()) {
      continue;
    }
    if (path_obstacle->PerceptionSLBoundary().end_s() <
        adc_sl_boundary.start_s() -
            std::max(config_.change_lane().min_overtake_distance(),
                     obstacle->Speed() * min_overtake_time)) {
      overtake_obstacles_.push_back(path_obstacle);
    }
    const auto& last_point =
        *(obstacle->Trajectory().trajectory_point().rbegin());
    if (last_point.v() < config_.change_lane().min_guard_speed()) {
      continue;
    }
    if (!reference_line.IsOnRoad(last_point.path_point())) {
      continue;
    }
    SLPoint last_sl;
    if (!reference_line.XYToSL(last_point.path_point(), &last_sl)) {
      continue;
    }
    if (last_sl.s() < 0 ||
        last_sl.s() > adc_sl_boundary.end_s() + kGuardForwardDistance) {
      continue;
    }
    if (last_sl.s() > max_s) {
      max_s = last_sl.s();
      first_guard_vehicle = path_obstacle;
    }
  }
  if (first_guard_vehicle) {
    guard_obstacles_.push_back(first_guard_vehicle);
  }
  return true;
}

bool ChangeLane::CreateGuardObstacle(
    const ReferenceLineInfo* reference_line_info, Obstacle* obstacle) {
  if (!obstacle || !obstacle->HasTrajectory()) {
    return false;
  }
  const auto& last_point =
      *(obstacle->Trajectory().trajectory_point().rbegin());

  const double kStepDistance = obstacle->PerceptionBoundingBox().length();
  double extend_v =
      std::max(last_point.v(), config_.change_lane().min_guard_speed());
  const double time_delta = kStepDistance / extend_v;
  const auto& reference_line = reference_line_info->reference_line();
  const double end_s = std::min(reference_line.Length(),
                                reference_line_info->AdcSlBoundary().end_s() +
                                    config_.change_lane().guard_distance());
  SLPoint sl_point;
  if (!reference_line.XYToSL(last_point.path_point(), &sl_point)) {
    return false;
  }
  double s = last_point.path_point().s() + kStepDistance;
  double ref_s = sl_point.s() + kStepDistance;
  for (double t = last_point.relative_time() + time_delta; ref_s < end_s;
       ref_s += kStepDistance, s += kStepDistance, t += time_delta) {
    auto ref_point = reference_line.GetNearestReferencePoint(ref_s);

    Vec2d xy_point;
    if (!reference_line.SLToXY(common::util::MakeSLPoint(ref_s, sl_point.l()),
                               &xy_point)) {
      return false;
    }

    auto* tp = obstacle->AddTrajectoryPoint();
    tp->set_a(0.0);
    tp->set_v(extend_v);
    tp->set_relative_time(t);
    tp->mutable_path_point()->set_x(xy_point.x());
    tp->mutable_path_point()->set_y(xy_point.y());
    tp->mutable_path_point()->set_theta(ref_point.heading());

    // this is an approximate estimate since we do not use it.
    tp->mutable_path_point()->set_s(s);
    tp->mutable_path_point()->set_kappa(ref_point.kappa());
  }
  return true;
}

bool ChangeLane::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  // The reference line is not a change lane reference line, skip
  if (reference_line_info->Lanes().IsOnSegment()) {
    return true;
  }
  guard_obstacles_.clear();
  overtake_obstacles_.clear();
  if (!FilterObstacles(reference_line_info)) {
    AERROR << "Failed to filter obstacles";
    return false;
  }
  if (config_.change_lane().enable_guard_obstacle() &&
      !guard_obstacles_.empty()) {
    for (const auto path_obstacle : guard_obstacles_) {
      auto* guard_obstacle = frame->Find(path_obstacle->Id());
      if (guard_obstacle &&
          CreateGuardObstacle(reference_line_info, guard_obstacle)) {
        AINFO << "Created guard obstacle: " << guard_obstacle->Id();
      }
    }
  }

  if (!overtake_obstacles_.empty()) {
    auto* path_decision = reference_line_info->path_decision();
    const auto& reference_line = reference_line_info->reference_line();
    for (const auto* path_obstacle : overtake_obstacles_) {
      auto overtake = CreateOvertakeDecision(reference_line, path_obstacle);
      path_decision->AddLongitudinalDecision(
          TrafficRuleConfig::RuleId_Name(Id()), path_obstacle->Id(), overtake);
    }
  }
  return true;
}

ObjectDecisionType ChangeLane::CreateOvertakeDecision(
    const ReferenceLine& reference_line,
    const PathObstacle* path_obstacle) const {
  ObjectDecisionType overtake;
  overtake.mutable_overtake();
  const double speed = path_obstacle->obstacle()->Speed();
  double distance = std::max(speed * config_.change_lane().min_overtake_time(),
                             config_.change_lane().min_overtake_distance());
  overtake.mutable_overtake()->set_distance_s(distance);
  double fence_s = path_obstacle->PerceptionSLBoundary().end_s() + distance;
  auto point = reference_line.GetReferencePoint(fence_s);
  overtake.mutable_overtake()->set_time_buffer(
      config_.change_lane().min_overtake_time());
  overtake.mutable_overtake()->set_distance_s(distance);
  overtake.mutable_overtake()->set_fence_heading(point.heading());
  overtake.mutable_overtake()->mutable_fence_point()->set_x(point.x());
  overtake.mutable_overtake()->mutable_fence_point()->set_y(point.y());
  overtake.mutable_overtake()->mutable_fence_point()->set_z(0.0);
  return overtake;
}

}  // namespace planning
}  // namespace apollo
