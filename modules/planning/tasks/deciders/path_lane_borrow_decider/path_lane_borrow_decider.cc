/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.h"

#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

constexpr double kLaneBorrowMaxSpeed = 5.0;
constexpr double kIntersectionClearanceDist = 20.0;

PathLaneBorrowDecider::PathLaneBorrowDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathLaneBorrowDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // By default, don't borrow any lane.
  reference_line_info->set_is_path_lane_borrow(false);
  // Check if lane-borrowing is needed, if so, borrow lane.
  if (IsNecessaryToBorrowLane(*frame, *reference_line_info)) {
    reference_line_info->set_is_path_lane_borrow(true);
  }
  return Status::OK();
}

bool PathLaneBorrowDecider::IsNecessaryToBorrowLane(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  if (PlanningContext::is_in_path_lane_borrow_scenario()) {
    // If originally borrowing neighbor lane:
    if (PlanningContext::able_to_use_self_lane_counter() >= 3) {
      // If have been able to use self-lane for some time, then switch to
      // non-lane-borrowing.
      PlanningContext::set_is_in_path_lane_borrow_scenario(false);
      AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
    }
  } else {
    // If originally not borrowing neighbor lane:
    if (HasSingleReferenceLine(frame) &&
        IsWithinSidePassingSpeedADC(frame) &&
        IsBlockingObstacleFarFromIntersection(reference_line_info) &&
        IsLongTermBlockingObstacle() &&
        IsBlockingObstacleWithinDestination(frame, reference_line_info) &&
        IsSidePassableObstacle(frame, reference_line_info)) {
      // Satisfying the above condition will it switch to lane-borrowing.
      PlanningContext::set_is_in_path_lane_borrow_scenario(true);
      AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
    }
  }
  return PlanningContext::is_in_path_lane_borrow_scenario();
}

// This function is to prevent lane-borrowing during lane-changing.
// TODO(jiacheng): depending on our needs, may allow lane-borrowing during
//                 lane-change.
bool PathLaneBorrowDecider::HasSingleReferenceLine(const Frame& frame) {
  return frame.reference_line_info().size() == 1;
}

bool PathLaneBorrowDecider::IsWithinSidePassingSpeedADC(const Frame& frame) {
  return frame.PlanningStartPoint().v() < kLaneBorrowMaxSpeed;
}

bool PathLaneBorrowDecider::IsLongTermBlockingObstacle() {
  return PlanningContext::front_static_obstacle_cycle_counter() >= 3;
}

bool PathLaneBorrowDecider::IsBlockingObstacleWithinDestination(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  std::string blocking_obstacle_id =
      PlanningContext::front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  // TODO(jiacheng): use adc actual s rather than plannign start point's s.
  double adc_frenet_s =
      reference_line_info.reference_line().GetFrenetPoint(
          frame.PlanningStartPoint().path_point()).s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  ADEBUG << "ADC is at s = " << adc_frenet_s;
  ADEBUG << "Destination is at s = "
         << reference_line_info.SDistanceToDestination() + adc_frenet_s;
  if (blocking_obstacle_s - adc_frenet_s >
      reference_line_info.SDistanceToDestination()) {
    return false;
  }
  return true;
}

// TODO(jiacheng): Implement the following two functions.
bool PathLaneBorrowDecider::IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info) {
  return true;
}
bool PathLaneBorrowDecider::IsSidePassableObstacle(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  return true;
}

}  // namespace planning
}  // namespace apollo
