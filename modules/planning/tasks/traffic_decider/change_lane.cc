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

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

namespace {
constexpr double kMinGuardVehicleSpeed = 1.0;
constexpr double kGuardDistance = 100.0;
}

ChangeLane::ChangeLane(const RuleConfig& config) : TrafficRule(config) {}

void ChangeLane::CreateGuardObstacles() {
  const auto& reference_line = reference_line_info_->reference_line();
  const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  const auto& path_decision = reference_line_info_->path_decision();
  constexpr double kGuardForwardDistance = 60;
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const auto* obstacle = path_obstacle->obstacle();
    if (!obstacle->HasTrajectory()) {
      continue;
    }
    if (path_obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s()) {
      continue;
    }
    const auto& last_point =
        *(obstacle->Trajectory().trajectory_point().rbegin());
    if (last_point.v() < kMinGuardVehicleSpeed) {
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
    auto* mutable_obstacle = frame_->Find(obstacle->Id());
    CreateGuardObstacle(mutable_obstacle);
  }
}

void ChangeLane::CreateGuardObstacle(Obstacle* obstacle) {
  if (!obstacle || !obstacle->HasTrajectory()) {
    return;
  }
  const auto& last_point =
      *(obstacle->Trajectory().trajectory_point().rbegin());

  const double kStepDistance = obstacle->PerceptionBoundingBox().length();
  double extend_v = std::max(last_point.v(), kMinGuardVehicleSpeed);
  const double time_delta = kStepDistance / extend_v;
  const auto& reference_line = reference_line_info_->reference_line();
  const double end_s =
      std::min(reference_line.Length(),
               reference_line_info_->AdcSlBoundary().end_s() + kGuardDistance);
  SLPoint sl_point;
  if (!reference_line.XYToSL(last_point.path_point(), &sl_point)) {
    return;
  }
  double s = last_point.path_point().s() + kStepDistance;
  double ref_s = sl_point.s() + kStepDistance;
  for (double t = last_point.relative_time() + time_delta; ref_s < end_s;
       ref_s += kStepDistance, s += kStepDistance, t += time_delta) {
    auto ref_point = reference_line.GetNearestReferencepoint(s);
    auto* tp = obstacle->AddTrajectoryPoint();
    tp->set_a(0.0);
    tp->set_v(extend_v);
    tp->set_relative_time(t);
    tp->mutable_path_point()->set_x(ref_point.x());
    tp->mutable_path_point()->set_y(ref_point.y());
    tp->mutable_path_point()->set_theta(ref_point.heading());
    tp->mutable_path_point()->set_s(s);
    tp->mutable_path_point()->set_kappa(ref_point.kappa());
  }
}

bool ChangeLane::ApplyRule(Frame* frame,
                           ReferenceLineInfo* const reference_line_info) {
  // The reference line is not a change lane reference line, skip
  if (reference_line_info->Lanes().IsOnSegment()) {
    return true;
  }
  reference_line_info_ = reference_line_info;
  frame_ = frame;
  CreateGuardObstacles();
  return true;
}

}  // namespace planning
}  // namespace apollo
