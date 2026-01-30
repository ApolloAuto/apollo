/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file nudge_info.cc
 **/

#include "modules/planning/park_data_center/nudge_info.h"

#include <algorithm>

namespace apollo {
namespace planning {

void NudgeInfo::set_enable(bool is_enable) {
    is_enable_ = is_enable;
}

bool NudgeInfo::is_enable() const {
    return is_enable_;
}

void NudgeInfo::set_is_merge_block_obs(bool is_merge) {
    is_merge_block_obs_ = is_merge;
}
bool NudgeInfo::is_merge_block_obs() const {
    return is_merge_block_obs_;
}

const std::unordered_map<std::string, NudgeObstacleInfo>& NudgeInfo::tracking_nudge_obs_info() const {
    return tracking_nudge_obs_info_;
}

std::unordered_map<std::string, NudgeObstacleInfo>* NudgeInfo::mutable_tracking_nudge_obs_info() {
    return &tracking_nudge_obs_info_;
}

const std::vector<SLPolygon>& NudgeInfo::block_sl_polygons() const {
    return block_sl_polygons_;
}
std::vector<SLPolygon>* NudgeInfo::mutable_block_sl_polygons() {
    return &block_sl_polygons_;
}

const std::set<std::string>& NudgeInfo::update_ids() const {
    return update_ids_;
}
std::set<std::string>* NudgeInfo::mutable_update_ids() {
    return &update_ids_;
}

const std::vector<std::vector<SLPoint>>& NudgeInfo::extra_nudge_key_points() const {
    return extra_nudge_key_points_;
}
std::vector<std::vector<SLPoint>>* NudgeInfo::mutable_extra_nudge_key_points() {
    return &extra_nudge_key_points_;
}

bool NudgeInfo::NeedCheckObsCollision(std::string obs_id) const {
    if (obs_id.empty()) {
        return false;
    }

    if (update_ids_.find(obs_id) != update_ids_.end()) {
        return true;
    }

    return false;
}

bool NudgeInfo::IsObsIgnoreNudgeDecision(std::string obs_id, double obs_start_s, double check_dis) const {
    if (obs_id.empty()) {
        return true;
    }

    if (extra_nudge_key_points_.size() < 2 || extra_nudge_key_points_.at(0).size() < 2) {
        return false;
    }

    if (update_ids_.find(obs_id) == update_ids_.end()
        && obs_start_s - extra_nudge_key_points_.at(0).at(1).s() > check_dis) {
        return true;
    }

    return false;
}

void NudgeInfo::GetFirstBlockSLPolygon(std::string obs_id, SLPolygon* obs_sl_polygon) const {
    if (obs_id.empty()) {
        return;
    }
    if (update_ids_.find(obs_id) != update_ids_.end() && block_sl_polygons_.size() > 0 && is_merge_block_obs_
        && block_sl_polygons_.at(0).id().find(obs_id) != std::string::npos) {
        *obs_sl_polygon = block_sl_polygons_.at(0);
    }
}

void NudgeInfo::SortBlockSLPolygons() {
    sort(block_sl_polygons_.begin(), block_sl_polygons_.end(), [](const SLPolygon& a, const SLPolygon& b) {
        return a.MinS() < b.MinS();
    });
}

void NudgeInfo::PrintDebugInfo() {
    for (auto iter = tracking_nudge_obs_info_.begin(); iter != tracking_nudge_obs_info_.end(); iter++) {
        std::stringstream debug_str;

        debug_str << "NudgeInfo ---- id: " << iter->first << "; " << "tracking time: " << iter->second.tracking_time
                  << "; "
                  // << "nudge type: " << iter->second.nudge_type.str() << "; "
                  << "nudge probability: " << iter->second.nudge_probability << "; ";

        debug_str << "polygon_points: ";

        for (const auto& point : iter->second.origin_polygon.points()) {
            debug_str << "(" << point.x() << ", " << point.y() << "), ";
        }
        AINFO << debug_str.str();
    }
}

bool NudgeInfo::GetInterpolatedNudgeL(bool is_left_nudge, double s, double* nudge_l) const {
    if (extra_nudge_key_points_.size() < 2 || extra_nudge_key_points_.at(0).size() < 2) {
        return false;
    }
    size_t index = is_left_nudge ? 0 : 1;

    if (s <= extra_nudge_key_points_[index].front().s()) {
        *nudge_l = extra_nudge_key_points_[index].front().l();
        return true;
    }
    if (s >= extra_nudge_key_points_[index].back().s()) {
        *nudge_l = extra_nudge_key_points_[index].back().l();
        return true;
    }
    auto iter = std::lower_bound(
            extra_nudge_key_points_[index].begin(),
            extra_nudge_key_points_[index].end(),
            s,
            [](const SLPoint& sl_point, const double s) { return sl_point.s() < s; });
    auto last_iter = std::prev(iter);
    *nudge_l = last_iter->l() + (s - last_iter->s()) * (iter->l() - last_iter->l()) / (iter->s() - last_iter->s());
    return true;
}

}  // namespace planning
}  // namespace apollo
