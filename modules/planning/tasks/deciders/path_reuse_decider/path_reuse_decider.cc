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

namespace {
static int reusable_path_counter = 0;
}

PathReuseDecider::PathReuseDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathReuseDecider::Process(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  bool enable_path_reuse = true;

  // Check if path is reusable
  if (enable_path_reuse && CheckPathReusable(frame)) {
    // count reusable path
    ++reusable_path_counter;
  }
  return Status::OK();
}

bool PathReuseDecider::CheckPathReusable(Frame* const frame) {
  if (history_->GetLastFrame() == nullptr) {
    return false;
  } else {
    auto obstacles = frame->obstacles();
    for (auto obstacle : obstacles) {
      const std::string& id = obstacle->Id();
      auto history_decisions = history_->GetLastFrame()
                                   ->GetObjectDecisionsById(id)
                                   ->GetObjectDecision();
      // if there are new objects in current frame than previous path is
      // non-reusable
      if (history_decisions.empty())
        return false;
      else
        return CompObjectDecision(obstacle->decisions(), history_decisions);
    }
  }
  return true;
}

bool PathReuseDecider::CompObjectDecision(
    const std::vector<ObjectDecisionType>& current_decisions,
    const std::vector<const ObjectDecisionType*> history_decisions) {
  std::vector<const ObjectDecisionType*> result;
  for (size_t i = 0; i < current_decisions.size(); i++) {
    result.push_back(&(current_decisions[i]));
  }
  std::sort(result.begin(), result.end(),
            [](const ObjectDecisionType* lhs, const ObjectDecisionType* rhs) {
              return lhs->object_tag_case() < rhs->object_tag_case();
            });
  for (size_t i = 0; i < result.size(); ++i) {
    if (result[i]->object_tag_case() != history_decisions[i]->object_tag_case())
      return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
