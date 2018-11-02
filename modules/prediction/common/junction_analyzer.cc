/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <queue>
#include <memory>
#include <utility>
#include <limits>
#include <algorithm>

#include "modules/prediction/common/junction_analyzer.h"

namespace apollo {
namespace prediction {

using apollo::hdmap::LaneInfo;
using ConstLaneInfoPtr = std::shared_ptr<const LaneInfo>;
using apollo::hdmap::JunctionInfo;
using apollo::hdmap::OverlapInfo;

std::shared_ptr<const apollo::hdmap::JunctionInfo>
    JunctionAnalyzer::junction_info_ptr_;
std::unordered_map<std::string, JunctionExit>
    JunctionAnalyzer::junction_exits_;
std::unordered_map<std::string, JunctionFeature>
    JunctionAnalyzer::junction_features_;

void JunctionAnalyzer::Init(const std::string& junction_id) {
  if (junction_info_ptr_ != nullptr &&
      junction_info_ptr_->id().id() == junction_id) {
    return;
  }
  Clear();
  junction_info_ptr_ = PredictionMap::JunctionById(junction_id);
  SetAllJunctionExits();
}

void JunctionAnalyzer::Clear() {
  // Clear all data
  junction_info_ptr_ = nullptr;
  junction_exits_.clear();
  junction_features_.clear();
}

void JunctionAnalyzer::SetAllJunctionExits() {
  CHECK_NOTNULL(junction_info_ptr_);
  for (const auto &overlap_id : junction_info_ptr_->junction().overlap_id()) {
    auto overlap_info_ptr = PredictionMap::OverlapById(overlap_id.id());
    if (overlap_info_ptr == nullptr) {
      continue;
    }
    if (overlap_info_ptr->overlap().object_size() != 2) {
      continue;
    }
    for (const auto &object : overlap_info_ptr->overlap().object()) {
      if (object.has_lane_overlap_info()) {
        std::string lane_id = object.id().id();
        auto lane_info_ptr = PredictionMap::LaneById(lane_id);
        if (object.lane_overlap_info().end_s() + 1e-3 <
            lane_info_ptr->total_length()) {
          JunctionExit junction_exit;
          double s = object.lane_overlap_info().end_s();
          apollo::common::PointENU position = lane_info_ptr->GetSmoothPoint(s);
          junction_exit.set_exit_lane_id(lane_id);
          junction_exit.mutable_exit_position()->set_x(position.x());
          junction_exit.mutable_exit_position()->set_y(position.y());
          junction_exit.set_exit_heading(lane_info_ptr->Heading(s));
          junction_exit.set_exit_width(lane_info_ptr->GetWidth(s));
          // add junction_exit to hashtable
          junction_exits_[lane_id] = junction_exit;
        }
      }
    }
  }
}

std::vector<JunctionExit> JunctionAnalyzer::GetJunctionExits(
    const std::string& start_lane_id) {
  int max_search_level = 3;
  std::vector<JunctionExit> junction_exits;
  std::queue<std::pair<ConstLaneInfoPtr, int>> lane_info_queue;
  lane_info_queue.emplace(PredictionMap::LaneById(start_lane_id), 0);
  while (!lane_info_queue.empty()) {
    ConstLaneInfoPtr curr_lane = lane_info_queue.front().first;
    int level = lane_info_queue.front().second;
    lane_info_queue.pop();
    const std::string& curr_lane_id = curr_lane->id().id();
    if (IsExitLane(curr_lane_id)) {
      junction_exits.push_back(junction_exits_[curr_lane_id]);
      continue;
    }
    if (level >= max_search_level) {
      continue;
    }
    for (const auto& succ_lane_id : curr_lane->lane().successor_id()) {
      ConstLaneInfoPtr succ_lane_ptr =
          PredictionMap::LaneById(succ_lane_id.id());
      lane_info_queue.emplace(succ_lane_ptr, level + 1);
    }
  }
  return junction_exits;
}

const JunctionFeature& JunctionAnalyzer::GetJunctionFeature(
    const std::string& start_lane_id) {
  if (junction_features_.find(start_lane_id) != junction_features_.end()) {
    return junction_features_[start_lane_id];
  }
  JunctionFeature junction_feature;
  // TODO(all) not fill enter_lane yet
  junction_feature.set_junction_id(GetJunctionId());
  junction_feature.set_junction_range(ComputeJunctionRange());
  std::vector<JunctionExit> junction_exits = GetJunctionExits(start_lane_id);

  for (const auto& junction_exit : junction_exits) {
    junction_feature.add_junction_exit()->CopyFrom(junction_exit);
  }
  junction_feature.mutable_enter_lane()->set_lane_id(start_lane_id);
  junction_features_[start_lane_id] = junction_feature;
  return junction_features_[start_lane_id];
}

bool JunctionAnalyzer::IsExitLane(const std::string& lane_id) {
  return junction_exits_.find(lane_id) != junction_exits_.end();
}

const std::string& JunctionAnalyzer::GetJunctionId() {
  CHECK_NOTNULL(junction_info_ptr_);
  return junction_info_ptr_->id().id();
}

double JunctionAnalyzer::ComputeJunctionRange() {
  CHECK_NOTNULL(junction_info_ptr_);
  if (!junction_info_ptr_->junction().has_polygon() ||
      junction_info_ptr_->junction().polygon().point_size() < 3) {
    AERROR << "Junction [" << GetJunctionId()
           << "] has not enough polygon points to compute range";
    // TODO(kechxu) move the default range value to gflags
    return 10.0;
  }
  double x_min = std::numeric_limits<double>::infinity();
  double x_max = -std::numeric_limits<double>::infinity();
  double y_min = std::numeric_limits<double>::infinity();
  double y_max = -std::numeric_limits<double>::infinity();
  for (const auto& point : junction_info_ptr_->junction().polygon().point()) {
    x_min = std::min(x_min, point.x());
    x_max = std::max(x_max, point.x());
    y_min = std::min(y_min, point.y());
    y_max = std::max(y_max, point.y());
  }
  double dx = std::abs(x_max - x_min);
  double dy = std::abs(y_max - y_min);
  double range = std::sqrt(dx * dx + dy * dy);
  return range;
}

}  // namespace prediction
}  // namespace apollo
