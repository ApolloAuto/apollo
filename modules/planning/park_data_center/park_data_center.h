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

#pragma once

#include <string>

#include "cyber/common/macros.h"
#include "modules/common/math/box2d.h"
#include "modules/planning/park_data_center/nudge_info.h"
#include "modules/planning/planning_base/common/indexed_queue.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {
using NudgeInfoMap = std::map<std::size_t, NudgeInfo>;
using NudgeInfoIndexedQueue = IndexedQueue<uint32_t, NudgeInfoMap>;

class ParkDataCenter {
public:
    ~ParkDataCenter();

    void UpdateHistoryNudgeInfo(uint32_t sequence_num);
    bool GetLastFrameNudgeTrackInfo(
            uint32_t sequence_num,
            std::size_t reference_line_key,
            std::unordered_map<std::string, NudgeObstacleInfo> *track_info);

    const NudgeInfo &current_nudge_info(std::size_t reference_line_key);
    NudgeInfo *mutable_current_nudge_info(std::size_t reference_line_key);
    void set_need_escape(bool need) { need_escape_ = need; }
    bool is_need_escape() { return need_escape_; }

private:
    static bool is_init_;
    NudgeInfoMap current_nudge_info_data_;
    NudgeInfoIndexedQueue history_nudge_info_data_ = NudgeInfoIndexedQueue(FLAGS_max_frame_history_num);
    bool need_escape_ = false;

    DECLARE_SINGLETON(ParkDataCenter)
};

}  // namespace planning
}  // namespace apollo
