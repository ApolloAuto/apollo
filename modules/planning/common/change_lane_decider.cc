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

#include "modules/planning/common/change_lane_decider.h"

#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using common::util::Dropbox;
using planning_internal::ChangeLaneState;
using common::time::Clock;

ChangeLaneDecider::ChangeLaneDecider() : state_key_("kChangeLaneStatus") {
  dropbox_ = Dropbox<ChangeLaneState>::Open();
}

void ChangeLaneDecider::UpdateState(ChangeLaneState::State state_code,
                                    const std::string& path_id) {
  UpdateState(Clock::NowInSecond(), state_code, path_id);
}

void ChangeLaneDecider::UpdateState(double timestamp,
                                    ChangeLaneState::State state_code,
                                    const std::string& path_id) {
  ChangeLaneState state;
  state.set_timestamp(timestamp);
  state.set_path_id(path_id);
  state.set_state(state_code);
  dropbox_->Set(state_key_, state);
}

void ChangeLaneDecider::PrioritizeChangeLane(
    std::list<ReferenceLineInfo>* reference_line_info) const {
  if (reference_line_info->empty()) {
    AERROR << "Reference line info empty";
    return;
  }
  auto iter = reference_line_info->begin();
  while (iter != reference_line_info->end()) {
    if (!iter->IsChangeLanePath()) {
      reference_line_info->splice(reference_line_info->begin(),
                                  *reference_line_info, iter);
      break;
    }
    ++iter;
  }
}

void ChangeLaneDecider::RemoveChangeLane(
    std::list<ReferenceLineInfo>* reference_line_info) const {
  auto iter = reference_line_info->begin();
  while (iter != reference_line_info->end()) {
    if (iter->IsChangeLanePath()) {
      iter = reference_line_info->erase(iter);
    } else {
      ++iter;
    }
  }
}

std::string GetCurrentPathId(
    const std::list<ReferenceLineInfo>& reference_line_info) {
  for (const auto& info : reference_line_info) {
    if (!info.IsChangeLanePath()) {
      return info.Lanes().Id();
    }
  }
  return "";
}

bool ChangeLaneDecider::Apply(
    std::list<ReferenceLineInfo>* reference_line_info) {
  if (reference_line_info->empty()) {
    AERROR << "Reference lines empty";
    return false;
  }

  if (FLAGS_reckless_change_lane) {
    PrioritizeChangeLane(reference_line_info);
    return true;
  }

  auto* prev_state = dropbox_->Get(state_key_);
  double now = Clock::NowInSecond();

  if (!prev_state) {
    UpdateState(now, ChangeLaneState::FORWARD,
                reference_line_info->front().Lanes().Id());
    return true;
  }

  bool has_change_lane = reference_line_info->size() > 1;
  if (!has_change_lane) {
    const auto& path_id = reference_line_info->front().Lanes().Id();
    if (prev_state->state() == ChangeLaneState::FORWARD) {
    } else if (prev_state->state() == ChangeLaneState::IN_CHANGE_LANE) {
      UpdateState(now, ChangeLaneState::FORWARD, path_id);
    } else if (prev_state->state() == ChangeLaneState::CHANGE_LANE_FAILED) {
      UpdateState(now, ChangeLaneState::FORWARD, path_id);
    } else {
      AERROR << "Unknown state: " << prev_state->ShortDebugString();
      return false;
    }
    return true;
  } else {  // has change lane in reference lines.
    auto current_path_id = GetCurrentPathId(*reference_line_info);
    if (current_path_id.empty()) {
      AERROR << "The vehicle is not on any reference line";
      return false;
    }
    if (prev_state->state() == ChangeLaneState::IN_CHANGE_LANE) {
      if (prev_state->path_id() == current_path_id) {
        PrioritizeChangeLane(reference_line_info);
      } else {
        RemoveChangeLane(reference_line_info);
        UpdateState(now, ChangeLaneState::FORWARD, current_path_id);
      }
      return true;
    } else if (prev_state->state() == ChangeLaneState::CHANGE_LANE_FAILED) {
      RemoveChangeLane(reference_line_info);
      UpdateState(now, ChangeLaneState::FORWARD, current_path_id);
      return true;
    } else if (prev_state->state() == ChangeLaneState::FORWARD) {
      if (now - prev_state->timestamp() > FLAGS_freeze_change_lane_time) {
        PrioritizeChangeLane(reference_line_info);
        UpdateState(now, ChangeLaneState::IN_CHANGE_LANE, current_path_id);
      } else {
        RemoveChangeLane(reference_line_info);
      }
    } else {
      AERROR << "Unknown state: " << prev_state->ShortDebugString();
      return false;
    }
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
