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

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

ChangeLane::ChangeLane(const RuleConfig& config) : TrafficRule(config) {}

const Obstacle* ChangeLane::FindGuardObstacle(
    ReferenceLineInfo* reference_line_info) {
  const auto& reference_line = reference_line_info->reference_line();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& path_decision = reference_line_info->path_decision();
  const Obstacle* first_guard_vehicle = nullptr;
  constexpr double kGuardForwardDistance = 60;
  double max_s = 0.0;
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
      first_guard_vehicle = obstacle;
    }
  }
  return first_guard_vehicle;
}

const Obstacle* ChangeLane::CreateGuardObstacle(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const Obstacle* obstacle) {
  if (!obstacle || !obstacle->HasTrajectory()) {
    return nullptr;
  }
  const auto& last_point =
      *(obstacle->Trajectory().trajectory_point().rbegin());
  std::string id = obstacle->Id() + "_guard";
  auto perception = obstacle->Perception();
  perception.set_id(-(std::hash<std::string>{}(id) >> 1));
  prediction::Trajectory trajectory = obstacle->Trajectory();
  constexpr double kPredictionTimeDelta = 0.1;                 // seconds
  const double kGuardTime = 5.0 + last_point.relative_time();  // seconds
  // trajectory.add_trajectory_point()->CopyFrom(last_point);
  const auto& reference_line = reference_line_info->reference_line();
  SLPoint sl_point;
  if (!reference_line.XYToSL(last_point.path_point(), &sl_point)) {
    return nullptr;
  }
  const double delta_s = kPredictionTimeDelta * last_point.v();
  double s = sl_point.s() + delta_s;
  for (double t = last_point.relative_time() + kPredictionTimeDelta;
       t < kGuardTime && s < reference_line.Length();
       s += delta_s, t += kPredictionTimeDelta) {
    auto ref_point = reference_line.GetNearestReferencepoint(s);
    auto* tp = trajectory.add_trajectory_point();
    tp->set_a(0.0);
    tp->set_v(last_point.v());
    tp->set_relative_time(t);
    tp->mutable_path_point()->set_x(ref_point.x());
    tp->mutable_path_point()->set_y(ref_point.y());
    tp->mutable_path_point()->set_theta(ref_point.heading());
    tp->mutable_path_point()->set_s(s - sl_point.s());
    tp->mutable_path_point()->set_kappa(ref_point.kappa());
  }
  frame->AddObstacle(Obstacle(id, perception, trajectory));
  auto* stored_obstacle = frame->Find(id);
  reference_line_info->AddObstacle(stored_obstacle);
  return stored_obstacle;
}

bool ChangeLane::ApplyRule(Frame* frame,
                           ReferenceLineInfo* const reference_line_info) {
  // The reference line is not a change lane reference line, skip
  if (reference_line_info->Lanes().IsOnSegment()) {
    return true;
  }
  const auto* obstacle = FindGuardObstacle(reference_line_info);
  if (!obstacle) {
    return true;
  } else {
    auto* guard_obstacle =
        CreateGuardObstacle(frame, reference_line_info, obstacle);
    if (guard_obstacle) {
      AINFO << "Created guard obstacle: " << guard_obstacle->Id();
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
