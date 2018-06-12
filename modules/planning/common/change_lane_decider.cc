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
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

using common::time::Clock;

void ChangeLaneDecider::UpdateStatus(ChangeLaneStatus::Status status_code,
                                     const std::string& path_id) {
  UpdateStatus(Clock::NowInSeconds(), status_code, path_id);
}

void ChangeLaneDecider::UpdateStatus(double timestamp,
                                     ChangeLaneStatus::Status status_code,
                                     const std::string& path_id) {
  auto* change_lane_status = util::GetPlanningStatus()->mutable_change_lane();
  change_lane_status->set_timestamp(timestamp);
  change_lane_status->set_path_id(path_id);
  change_lane_status->set_status(status_code);
}

void ChangeLaneDecider::PrioritizeChangeLane(
    std::list<ReferenceLineInfo>* reference_line_info) const {
  if (reference_line_info->empty()) {
    AERROR << "Reference line info empty";
    return;
  }
  auto iter = reference_line_info->begin();
  while (iter != reference_line_info->end()) {
    if (iter->IsChangeLanePath()) {
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

  auto* prev_status = util::GetPlanningStatus()->mutable_change_lane();
  double now = Clock::NowInSeconds();

  if (!prev_status->has_status()) {
    UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_SUCCESS,
                 GetCurrentPathId(*reference_line_info));
    return true;
  }

  bool has_change_lane = reference_line_info->size() > 1;
  if (!has_change_lane) {
    const auto& path_id = reference_line_info->front().Lanes().Id();
    if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_SUCCESS) {
    } else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_SUCCESS, path_id);
    } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
    } else {
      AERROR << "Unknown state: " << prev_status->ShortDebugString();
      return false;
    }
    return true;
  } else {  // has change lane in reference lines.
    auto current_path_id = GetCurrentPathId(*reference_line_info);
    if (current_path_id.empty()) {
      AERROR << "The vehicle is not on any reference line";
      return false;
    }
    if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      if (prev_status->path_id() == current_path_id) {
        PrioritizeChangeLane(reference_line_info);
      } else {
        RemoveChangeLane(reference_line_info);
        UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_SUCCESS,
                     current_path_id);
      }
      return true;
    } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
      if (now - prev_status->timestamp() < FLAGS_change_lane_fail_freeze_time) {
        RemoveChangeLane(reference_line_info);
      } else {
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);
      }
      return true;
    } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_SUCCESS) {
      if (now - prev_status->timestamp() <
          FLAGS_change_lane_success_freeze_time) {
        RemoveChangeLane(reference_line_info);
      } else {
        PrioritizeChangeLane(reference_line_info);
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);
      }
    } else {
      AERROR << "Unknown state: " << prev_status->ShortDebugString();
      return false;
    }
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
