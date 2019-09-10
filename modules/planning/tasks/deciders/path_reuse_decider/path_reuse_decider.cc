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

/**
 * @file
 **/
#include "modules/planning/tasks/deciders/path_reuse_decider/path_reuse_decider.h"

#include <algorithm>
#include <string>
#include <vector>

namespace apollo {
namespace planning {

using apollo::common::Status;

int PathReuseDecider::reusable_path_counter_ = 0;
int PathReuseDecider::total_path_counter_ = 0;

PathReuseDecider::PathReuseDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathReuseDecider::Process(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Check if path is reusable
  if (Decider::config_.path_reuse_decider_config().reuse_path() &&
      CheckPathReusable(frame, reference_line_info)) {
    // count reusable path
    ++reusable_path_counter_;
  }
  ++total_path_counter_;
  ADEBUG << "reusable_path_counter_" << reusable_path_counter_;
  ADEBUG << "total_path_counter_" << total_path_counter_;
  return Status::OK();
}

bool PathReuseDecider::CheckPathReusable(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (history_->GetLastFrame() == nullptr) return false;
  const std::vector<const HistoryObjectDecision*> history_objects_decisions =
      history_->GetLastFrame()->GetStopObjectDecisions();
  const auto& reference_line = reference_line_info->reference_line();
  std::vector<std::pair<const double, const common::PointENU*>*>
      history_stop_positions;
  std::vector<double> current_stop_positions;
  GetCurrentStopObstacleS(frame, &current_stop_positions);
  GetHistoryStopPositions(history_objects_decisions, &history_stop_positions);

  // get current vehicle s
  common::math::Vec2d adc_position = {
      common::VehicleStateProvider::Instance()->x(),
      common::VehicleStateProvider::Instance()->y()};
  common::SLPoint adc_position_sl;
  reference_line.XYToSL(adc_position, &adc_position_sl);
  double nearest_history_stop_s = 300.0;
  double nearest_current_stop_s = 300.0;
  for (auto history_stop_position : history_stop_positions) {
    // history_stop position at current reference line
    common::math::Vec2d stop_position = {history_stop_position->second->x(),
                                         history_stop_position->second->y()};
    common::SLPoint stop_position_sl;
    reference_line.XYToSL(stop_position, &stop_position_sl);
    if (stop_position_sl.s() < adc_position_sl.s()) {
      continue;
    } else {
      // find nearest history stop
      nearest_history_stop_s = stop_position_sl.s();
      break;
    }
  }
  for (auto current_stop_position : current_stop_positions) {
    if (current_stop_position < adc_position_sl.s()) {
      continue;
    } else {
      // find nearest current stop
      nearest_current_stop_s = current_stop_position;
      break;
    }
  }
  return SameStopS(nearest_history_stop_s, nearest_current_stop_s);
}

// compare history stop position vs current stop position
bool PathReuseDecider::SameStopS(const double history_stop_s,
                                 const double current_stop_s) {
  const double KNegative = 0.1;  // (meter) closer
  const double kPositive = 0.5;  // (meter) further
  if ((current_stop_s > history_stop_s &&
       current_stop_s - history_stop_s < kPositive) ||
      (current_stop_s < history_stop_s &&
       history_stop_s - current_stop_s < KNegative))
    return true;
  return false;
}
// get current stop positions
void PathReuseDecider::GetCurrentStopPositions(
    Frame* frame,
    std::vector<const common::PointENU*>* current_stop_positions) {
  auto obstacles = frame->obstacles();
  for (auto obstacle : obstacles) {
    const std::vector<ObjectDecisionType>& current_decisions =
        obstacle->decisions();
    for (auto current_decision : current_decisions) {
      if (current_decision.has_stop())
        current_stop_positions->emplace_back(
            &current_decision.stop().stop_point());
    }
  }
  // sort
  std::sort(current_stop_positions->begin(), current_stop_positions->end(),
            [](const common::PointENU* lhs, const common::PointENU* rhs) {
              return (lhs->x() < rhs->x() ||
                      (lhs->x() == rhs->x() && lhs->y() < rhs->y()));
            });
}

// get current stop obstacle position in s-direction
void PathReuseDecider::GetCurrentStopObstacleS(
    Frame* frame, std::vector<double>* current_stop_obstacle) {
  // get all obstacles
  auto obstacles = frame->obstacles();

  for (auto obstacle : obstacles)
    if (obstacle->IsLaneBlocking())
      current_stop_obstacle->emplace_back(
          obstacle->PerceptionSLBoundary().start_s());

  // sort w.r.t s
  std::sort(current_stop_obstacle->begin(), current_stop_obstacle->end(),
            [](const double lhs, const double rhs) { return lhs < rhs; });
}

// get history stop positions
void PathReuseDecider::GetHistoryStopPositions(
    const std::vector<const HistoryObjectDecision*>& history_objects_decisions,
    std::vector<std::pair<const double, const common::PointENU*>*>*
        history_stop_positions) {
  for (auto history_object_decision : history_objects_decisions) {
    const std::vector<const ObjectDecisionType*> decisions =
        history_object_decision->GetObjectDecision();
    for (const ObjectDecisionType* decision : decisions) {
      if (decision->has_stop()) {
        std::pair<const double, const common::PointENU*> stop_pos =
            std::make_pair(decision->stop().distance_s(),
                           &decision->stop().stop_point());
        history_stop_positions->emplace_back(&stop_pos);
      }
    }
  }
  // sort w.r.t s
  std::sort(history_stop_positions->begin(), history_stop_positions->end(),
            [](const std::pair<const double, const common::PointENU*>* lhs,
               const std::pair<const double, const common::PointENU*>* rhs) {
              return lhs->first < rhs->first;
            });
}

}  // namespace planning
}  // namespace apollo
