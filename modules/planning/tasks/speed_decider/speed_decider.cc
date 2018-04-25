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
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::math::Vec2d;
using apollo::common::Status;
using apollo::perception::PerceptionObstacle;

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

SpeedDecider::StPosition SpeedDecider::GetStPosition(
    const SpeedData& speed_profile, const StBoundary& st_boundary) const {
  StPosition st_position = BELOW;
  if (st_boundary.IsEmpty()) {
    return st_position;
  }
  const double start_t = st_boundary.min_t();
  const double end_t = st_boundary.max_t();
  for (size_t i = 0; i + 1 < speed_profile.speed_vector().size(); ++i) {
    const STPoint curr_st(speed_profile.speed_vector()[i].s(),
                          speed_profile.speed_vector()[i].t());
    const STPoint next_st(speed_profile.speed_vector()[i + 1].s(),
                          speed_profile.speed_vector()[i + 1].t());
    if (curr_st.t() < start_t && next_st.t() < start_t) {
      continue;
    }
    if (curr_st.t() > end_t) {
      break;
    }

    common::math::LineSegment2d speed_line(curr_st, next_st);
    if (st_boundary.HasOverlap(speed_line)) {
      const std::string msg =
          "dp_st_graph failed: speed profile cross st_boundaries.";
      AERROR << msg;
      st_position = CROSS;
      return st_position;
    }

    if (start_t < next_st.t() && curr_st.t() < end_t) {
      STPoint bd_point_front = st_boundary.upper_points().front();
      double side = common::math::CrossProd(bd_point_front, curr_st, next_st);
      if (side < 0.0) {
        st_position = ABOVE;
      } else {
        st_position = BELOW;
      }
      break;
    }
  }
  return st_position;
}

bool SpeedDecider::IsFollowTooClose(const PathObstacle& path_obstacle) const {
  if (!path_obstacle.IsBlockingObstacle()) {
    return false;
  }

  if (path_obstacle.st_boundary().min_t() > 0.0) {
    return false;
  }
  const double obs_speed = path_obstacle.obstacle()->Speed();
  const double ego_speed = init_point_.v();
  if (obs_speed > ego_speed) {
    return false;
  }
  const double distance =
      path_obstacle.st_boundary().min_s() - FLAGS_min_stop_distance_obstacle;
  constexpr double decel = 1.0;
  return distance < std::pow((ego_speed - obs_speed), 2) * 0.5 / decel;
}

Status SpeedDecider::MakeObjectDecision(
    const SpeedData& speed_profile, PathDecision* const path_decision) const {
  if (speed_profile.speed_vector().size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  for (const auto* obstacle : path_decision->path_obstacles().Items()) {
    auto* path_obstacle = path_decision->Find(obstacle->Id());
    const auto& boundary = path_obstacle->st_boundary();

    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0 ||
        boundary.min_t() > dp_st_speed_config_.total_time()) {
      AppendIgnoreDecision(path_obstacle);
      continue;
    }

    if (path_obstacle->HasLongitudinalDecision()) {
      AppendIgnoreDecision(path_obstacle);
      continue;
    }

    auto position = GetStPosition(speed_profile, boundary);
    switch (position) {
      case BELOW:
        if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*path_obstacle, &stop_decision,
                                 -FLAGS_stop_line_stop_distance)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                   stop_decision);
          }
        } else if (CheckIsFollowByT(boundary) &&
                   (boundary.max_t() - boundary.min_t() >
                    FLAGS_follow_min_time_sec)) {
          // stop for low_speed decelerating
          if (IsFollowTooClose(*path_obstacle)) {
            ObjectDecisionType stop_decision;
            if (CreateStopDecision(*path_obstacle, &stop_decision,
                                   -FLAGS_min_stop_distance_obstacle)) {
              path_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                     stop_decision);
            }
          } else {  // high speed or low speed accelerating
            // FOLLOW decision
            ObjectDecisionType follow_decision;
            if (CreateFollowDecision(*path_obstacle, &follow_decision)) {
              path_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                     follow_decision);
            }
          }
        } else {
          // YIELD decision
          ObjectDecisionType yield_decision;
          if (CreateYieldDecision(*path_obstacle, &yield_decision)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                   yield_decision);
          }
        }
        break;
      case ABOVE:
        if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType ignore;
          ignore.mutable_ignore();
          path_obstacle->AddLongitudinalDecision("dp_st_graph", ignore);
        } else {
          // OVERTAKE decision
          ObjectDecisionType overtake_decision;
          if (CreateOvertakeDecision(*path_obstacle, &overtake_decision)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                   overtake_decision);
          }
        }
        break;
      case CROSS:
        if (obstacle->IsBlockingObstacle()) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*path_obstacle, &stop_decision,
                                 -FLAGS_min_stop_distance_obstacle)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph/cross",
                                                   stop_decision);
          }
          const std::string msg =
              "Failed to find a solution for crossing obstacle:" +
              obstacle->Id();
          AERROR << msg;
          return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        break;
      default:
        AERROR << "Unknown position:" << position;
    }
    AppendIgnoreDecision(path_obstacle);
  }
  return Status::OK();
}

void SpeedDecider::AppendIgnoreDecision(PathObstacle* path_obstacle) const {
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  if (!path_obstacle->HasLongitudinalDecision()) {
    path_obstacle->AddLongitudinalDecision("dp_st_graph", ignore_decision);
  }
  if (!path_obstacle->HasLateralDecision()) {
    path_obstacle->AddLateralDecision("dp_st_graph", ignore_decision);
  }
}

bool SpeedDecider::CreateStopDecision(const PathObstacle& path_obstacle,
                                      ObjectDecisionType* const stop_decision,
                                      double stop_distance) const {
  DCHECK_NOTNULL(stop_decision);

  const auto& boundary = path_obstacle.st_boundary();
  const double fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + stop_distance;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < fence_s) {
    ADEBUG << "Stop fence is further away, ignore.";
    return false;
  }

  const auto fence_point = reference_line_->GetReferencePoint(fence_s);

  // set STOP decision
  auto* stop = stop_decision->mutable_stop();
  stop->set_distance_s(stop_distance);
  auto* stop_point = stop->mutable_stop_point();
  stop_point->set_x(fence_point.x());
  stop_point->set_y(fence_point.y());
  stop_point->set_z(0.0);
  stop->set_stop_heading(fence_point.heading());

  PerceptionObstacle::Type obstacle_type =
      path_obstacle.obstacle()->Perception().type();
  ADEBUG << "STOP: obstacle_id[" << path_obstacle.obstacle()->Id()
         << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)
         << "]";

  return true;
}

bool SpeedDecider::CreateFollowDecision(
    const PathObstacle& path_obstacle,
    ObjectDecisionType* const follow_decision) const {
  DCHECK_NOTNULL(follow_decision);

  const double follow_speed = init_point_.v();
  const double follow_distance_s = -std::fmax(
      follow_speed * FLAGS_follow_time_buffer, FLAGS_follow_min_distance);

  const auto& boundary = path_obstacle.st_boundary();
  const double reference_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_s) {
    ADEBUG << "Follow reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_s);

  // set FOLLOW decision
  auto* follow = follow_decision->mutable_follow();
  follow->set_distance_s(follow_distance_s);
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type =
      path_obstacle.obstacle()->Perception().type();
  ADEBUG << "FOLLOW: obstacle_id[" << path_obstacle.obstacle()->Id()
         << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)
         << "]";

  return true;
}

bool SpeedDecider::CreateYieldDecision(
    const PathObstacle& path_obstacle,
    ObjectDecisionType* const yield_decision) const {
  DCHECK_NOTNULL(yield_decision);

  PerceptionObstacle::Type obstacle_type =
      path_obstacle.obstacle()->Perception().type();
  double yield_distance = FLAGS_yield_distance;
  switch (obstacle_type) {
    case PerceptionObstacle::PEDESTRIAN:
    case PerceptionObstacle::BICYCLE:
      yield_distance = FLAGS_yield_distance_pedestrian_bycicle;
      break;
    default:
      yield_distance = FLAGS_yield_distance;
      break;
  }

  const auto& obstacle_boundary = path_obstacle.st_boundary();
  const double yield_distance_s =
      std::max(-obstacle_boundary.min_s(), -yield_distance);

  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + obstacle_boundary.min_s() + yield_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    ADEBUG << "Yield reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set YIELD decision
  auto* yield = yield_decision->mutable_yield();
  yield->set_distance_s(yield_distance_s);
  yield->mutable_fence_point()->set_x(ref_point.x());
  yield->mutable_fence_point()->set_y(ref_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(ref_point.heading());

  ADEBUG << "YIELD: obstacle_id[" << path_obstacle.obstacle()->Id()
         << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)
         << "]";

  return true;
}

bool SpeedDecider::CreateOvertakeDecision(
    const PathObstacle& path_obstacle,
    ObjectDecisionType* const overtake_decision) const {
  DCHECK_NOTNULL(overtake_decision);

  constexpr double kOvertakeTimeBuffer = 3.0;    // in seconds
  constexpr double kMinOvertakeDistance = 10.0;  // in meters

  const auto& velocity = path_obstacle.obstacle()->Perception().velocity();
  const double obstacle_speed =
      common::math::Vec2d::CreateUnitVec2d(init_point_.path_point().theta())
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  const double overtake_distance_s = std::fmax(
      std::fmax(init_point_.v(), obstacle_speed) * kOvertakeTimeBuffer,
      kMinOvertakeDistance);

  const auto& boundary = path_obstacle.st_boundary();
  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    ADEBUG << "Overtake reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set OVERTAKE decision
  auto* overtake = overtake_decision->mutable_overtake();
  overtake->set_distance_s(overtake_distance_s);
  overtake->mutable_fence_point()->set_x(ref_point.x());
  overtake->mutable_fence_point()->set_y(ref_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type =
      path_obstacle.obstacle()->Perception().type();
  ADEBUG << "OVERTAKE: obstacle_id[" << path_obstacle.obstacle()->Id()
         << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)
         << "]";

  return true;
}

bool SpeedDecider::CheckIsFollowByT(const StBoundary& boundary) const {
  if (boundary.BottomLeftPoint().s() > boundary.BottomRightPoint().s()) {
    return false;
  }
  constexpr double kFollowTimeEpsilon = 1e-3;
  constexpr double kFollowCutOffTime = 0.5;
  if (boundary.min_t() > kFollowCutOffTime ||
      boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
