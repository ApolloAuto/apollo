/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/park_data_center/park_data_center.h"

#include <algorithm>
#include <cmath>

#include "cyber/common/file.h"

namespace apollo {
namespace planning {

bool ParkDataCenter::is_init_ = false;

ParkDataCenter::ParkDataCenter() {
    AINFO << "Park Data Center is ready!";
}

ParkDataCenter::~ParkDataCenter() {
    AINFO << "Destructor ParkDataCenter!";
    history_nudge_info_data_.Clear();
}

void ParkDataCenter::UpdateHistoryNudgeInfo(uint32_t sequence_num) {
    history_nudge_info_data_.Add(sequence_num, std::make_unique<NudgeInfoMap>(current_nudge_info_data_));
    current_nudge_info_data_.clear();
}

bool ParkDataCenter::GetLastFrameNudgeTrackInfo(
        uint32_t sequence_num,
        std::size_t reference_line_key,
        std::unordered_map<std::string, NudgeObstacleInfo> *track_info) {
    auto *last_frame_nugde_info = history_nudge_info_data_.Find(sequence_num);
    if (nullptr == last_frame_nugde_info) {
        return false;
    }
    auto iter = last_frame_nugde_info->find(reference_line_key);
    if (iter == last_frame_nugde_info->end()) {
        return false;
    } else {
        *track_info = iter->second.tracking_nudge_obs_info();
    }
    return true;
}

const NudgeInfo &ParkDataCenter::current_nudge_info(std::size_t reference_line_key) {
    auto iter = current_nudge_info_data_.find(reference_line_key);
    if (iter == current_nudge_info_data_.end()) {
        current_nudge_info_data_[reference_line_key] = NudgeInfo();
    }
    return current_nudge_info_data_[reference_line_key];
}

NudgeInfo *ParkDataCenter::mutable_current_nudge_info(std::size_t reference_line_key) {
    auto itr = current_nudge_info_data_.find(reference_line_key);
    if (itr == current_nudge_info_data_.end()) {
        current_nudge_info_data_[reference_line_key] = NudgeInfo();
    }
    return &current_nudge_info_data_[reference_line_key];
}

}  // namespace planning
}  // namespace apollo
