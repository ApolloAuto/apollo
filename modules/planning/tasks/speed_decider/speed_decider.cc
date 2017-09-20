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

#include "modules/planning/tasks/speed_decider/speed_decider.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using common::math::Vec2d;

SpeedDecider::SpeedDecider() : Task("SpeedDecider") {}

bool SpeedDecider::Init(const PlanningConfig& config) {
  dp_st_speed_config_ = config.em_planner_config().dp_st_speed_config();
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config();
  return true;
}

apollo::common::Status SpeedDecider::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  init_point_ = frame_->PlanningStartPoint();
  adc_sl_boundary_ = reference_line_info_->AdcSlBoundary();
  reference_line_ = &reference_line_info_->reference_line();
  if (!MakeObjectDecision(reference_line_info->speed_data(),
                          reference_line_info->path_decision())
           .ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

Status SpeedDecider::MakeObjectDecision(
    const SpeedData& speed_profile, PathDecision* const path_decision) const {
  if (speed_profile.speed_vector().size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const auto& boundary = path_obstacle->st_boundary();
    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0) {
      continue;
    }
    if (path_obstacle->HasLongitudinalDecision()) {
      continue;
    }

    double start_t = boundary.min_t();
    double end_t = boundary.max_t();

    bool go_down = true;
    for (const auto& speed_point : speed_profile.speed_vector()) {
      if (speed_point.t() < start_t) {
        continue;
      }
      if (speed_point.t() > end_t) {
        break;
      }

      STPoint st_point(speed_point.s(), speed_point.t());
      if (boundary.IsPointInBoundary(st_point)) {
        const std::string msg =
            "dp_st_graph failed: speed profile cross st_boundaries.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }

      double s_upper = dp_st_speed_config_.total_path_length();
      double s_lower = 0.0;
      if (boundary.GetBoundarySRange(speed_point.t(), &s_upper, &s_lower)) {
        if (s_lower > speed_point.s()) {
          go_down = true;
        } else if (s_upper < speed_point.s()) {
          go_down = false;
        }
      }
    }
    if (go_down) {
      if (CheckIsFollowByT(boundary)) {
        // FOLLOW decision
        ObjectDecisionType follow_decision;
        if (!CreateFollowDecision(*path_obstacle, boundary, &follow_decision)) {
          AERROR << "Failed to create follow decision for boundary with id "
                 << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to create follow decision");
        }
        if (!path_decision->AddLongitudinalDecision(
                "dp_st_graph", boundary.id(), follow_decision)) {
          AERROR << "Failed to add follow decision to object " << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to add follow decision");
        }
      } else {
        // YIELD decision
        ObjectDecisionType yield_decision;
        if (!CreateYieldDecision(boundary, &yield_decision)) {
          AERROR << "Failed to create yield decision for boundary with id "
                 << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to create yield decision");
        }
        if (!path_decision->AddLongitudinalDecision(
                "dp_st_graph", boundary.id(), yield_decision)) {
          AERROR << "Failed to add yield decision to object " << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to add yield decision");
        }
      }
    } else {
      // OVERTAKE decision
      ObjectDecisionType overtake_decision;
      if (!CreateOvertakeDecision(*path_obstacle, boundary,
                                  &overtake_decision)) {
        AERROR << "Failed to create overtake decision for boundary with id "
               << boundary.id();
        return Status(ErrorCode::PLANNING_ERROR,
                      "faind to create overtake decision");
      }
      if (!path_decision->AddLongitudinalDecision("dp_st_graph", boundary.id(),
                                                  overtake_decision)) {
        AERROR << "Failed to add overtake decision to object " << boundary.id();
        return Status(ErrorCode::PLANNING_ERROR,
                      "faind to add overtake decision");
      }
    }
  }
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    if (!path_obstacle->HasLongitudinalDecision()) {
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      path_decision->AddLongitudinalDecision("dp_st_graph", path_obstacle->Id(),
                                             ignore_decision);
    }
    if (!path_obstacle->HasLateralDecision()) {
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      path_decision->AddLateralDecision("dp_st_graph", path_obstacle->Id(),
                                        ignore_decision);
    }
  }
  return Status::OK();
}

bool SpeedDecider::CreateFollowDecision(
    const PathObstacle& path_obstacle, const StBoundary& boundary,
    ObjectDecisionType* const follow_decision) const {
  DCHECK_NOTNULL(follow_decision);

  auto* follow = follow_decision->mutable_follow();

  const double follow_speed = init_point_.v();
  const double follow_distance_s = -std::fmax(
      follow_speed * FLAGS_follow_time_buffer, FLAGS_follow_min_distance);

  follow->set_distance_s(follow_distance_s);

  const double refence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s;
  auto ref_point = reference_line_->GetReferencePoint(refence_s);
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  return true;
}

bool SpeedDecider::CreateYieldDecision(
    const StBoundary& boundary,
    ObjectDecisionType* const yield_decision) const {
  auto* yield = yield_decision->mutable_yield();

  // in meters
  constexpr double kMinYieldDistance = 10.0;
  const double yield_distance_s =
      std::max(-boundary.min_s(), -1.0 * kMinYieldDistance);
  yield->set_distance_s(yield_distance_s);

  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + yield_distance_s;
  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  yield->mutable_fence_point()->set_x(ref_point.x());
  yield->mutable_fence_point()->set_y(ref_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(ref_point.heading());

  return true;
}

bool SpeedDecider::CreateOvertakeDecision(
    const PathObstacle& path_obstacle, const StBoundary& boundary,
    ObjectDecisionType* const overtake_decision) const {
  DCHECK_NOTNULL(overtake_decision);

  auto* overtake = overtake_decision->mutable_overtake();

  // in seconds
  constexpr double kOvertakeTimeBuffer = 3.0;
  // in meters
  constexpr double kMinOvertakeDistance = 10.0;

  const auto& velocity = path_obstacle.obstacle()->Perception().velocity();
  const double obstacle_speed =
      common::math::Vec2d::CreateUnitVec2d(init_point_.path_point().theta())
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  const double overtake_distance_s = std::fmax(
      std::fmax(init_point_.v(), obstacle_speed) * kOvertakeTimeBuffer,
      kMinOvertakeDistance);
  overtake->set_distance_s(overtake_distance_s);

  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s;

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);
  overtake->mutable_fence_point()->set_x(ref_point.x());
  overtake->mutable_fence_point()->set_y(ref_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(ref_point.heading());

  return true;
}

bool SpeedDecider::CheckIsFollowByT(const StBoundary& boundary) const {
  if (boundary.BottomLeftPoint().s() > boundary.BottomRightPoint().s()) {
    return false;
  }
  const double kFollowTimeEpsilon = 1e-3;
  if (boundary.min_t() > kFollowTimeEpsilon ||
      boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
