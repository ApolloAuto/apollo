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
      CheckPathReusable(frame)) {
    // count reusable path
    ++reusable_path_counter_;
  }
  ++total_path_counter_;
  return Status::OK();
}

bool PathReuseDecider::CheckPathReusable(Frame* const frame) {
  if (history_->GetLastFrame() == nullptr) return false;
  const std::vector<const HistoryObjectDecision*>
      history_stop_objects_decisions =
          history_->GetLastFrame()->GetStopObjectDecisions();
  std::vector<common::PointENU> history_stop_positions;
  std::vector<common::PointENU> current_stop_positions;
  GetCurrentStopPositions(frame, &current_stop_positions);
  GetHistoryStopPositions(history_stop_objects_decisions,
                          &history_stop_positions);
  if (current_stop_positions.size() != history_stop_positions.size()) {
    return false;
  } else {
    for (size_t i = 0; i < current_stop_positions.size(); ++i) {
      if (current_stop_positions[i].x() != history_stop_positions[i].x() &&
          current_stop_positions[i].y() != history_stop_positions[i].y())
        return false;
    }
  }
  return true;
}

// get current stop positions
void PathReuseDecider::GetCurrentStopPositions(
    Frame* frame, std::vector<common::PointENU>* current_stop_positions) {
  auto obstacles = frame->obstacles();
  for (auto obstacle : obstacles) {
    const std::vector<ObjectDecisionType>& current_decisions =
        obstacle->decisions();
    for (auto current_decision : current_decisions) {
      if (current_decision.has_stop())
        current_stop_positions->emplace_back(
            current_decision.stop().stop_point());
    }
  }
  // sort
  std::sort(
      current_stop_positions->begin(), current_stop_positions->end(),
      [](const common::PointENU lhs, const common::PointENU rhs) {
        return (lhs.x() < rhs.x() || (lhs.x() == rhs.x() && lhs.y() < rhs.y()));
      });
}

// get history stop positions
void PathReuseDecider::GetHistoryStopPositions(
    const std::vector<const HistoryObjectDecision*>& history_objects_decisions,
    std::vector<common::PointENU>* history_stop_positions) {
  for (auto history_object_decision : history_objects_decisions) {
    const std::vector<const ObjectDecisionType*> decisions =
        history_object_decision->GetObjectDecision();
    for (const ObjectDecisionType* decision : decisions) {
      if (decision->has_stop())
        history_stop_positions->emplace_back(decision->stop().stop_point());
    }
  }
  // sort
  std::sort(
      history_stop_positions->begin(), history_stop_positions->end(),
      [](const common::PointENU lhs, const common::PointENU rhs) {
        return (lhs.x() < rhs.x() || (lhs.x() == rhs.x() && lhs.y() < rhs.y()));
      });
}

}  // namespace planning
}  // namespace apollo
