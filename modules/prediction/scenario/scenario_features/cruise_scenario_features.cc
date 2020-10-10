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

#include "modules/prediction/scenario/scenario_features/cruise_scenario_features.h"

#include <memory>
#include <queue>
#include <utility>

#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

using apollo::hdmap::LaneInfo;
using ConstLaneInfoPtr = std::shared_ptr<const LaneInfo>;
using apollo::hdmap::Lane;

CruiseScenarioFeatures::CruiseScenarioFeatures() {
  scenario_.set_type(Scenario::CRUISE);
}

CruiseScenarioFeatures::~CruiseScenarioFeatures() {}

bool CruiseScenarioFeatures::IsLaneOfInterest(
    const std::string& lane_id) const {
  return lane_ids_of_interest_.find(lane_id) != lane_ids_of_interest_.end();
}

void CruiseScenarioFeatures::InsertLaneOfInterest(const std::string& lane_id) {
  lane_ids_of_interest_.insert(lane_id);
}

void CruiseScenarioFeatures::BuildCruiseScenarioFeatures(
    const EnvironmentFeatures& environment_features) {
  // Forward lanes
  if (environment_features.has_ego_lane()) {
    auto ego_lane = environment_features.GetEgoLane();
    const std::string& ego_lane_id = ego_lane.first;
    double ego_lane_s = ego_lane.second;
    SearchForwardAndInsert(ego_lane_id, ego_lane_s, 50.0);
  }
  if (environment_features.has_left_neighbor_lane()) {
    auto left_lane = environment_features.GetLeftNeighborLane();
    const std::string& left_lane_id = left_lane.first;
    double left_lane_s = left_lane.second;
    SearchForwardAndInsert(left_lane_id, left_lane_s, 50.0);
  }
  if (environment_features.has_right_neighbor_lane()) {
    auto right_lane = environment_features.GetRightNeighborLane();
    const std::string& right_lane_id = right_lane.first;
    double right_lane_s = right_lane.second;
    SearchForwardAndInsert(right_lane_id, right_lane_s, 50.0);
  }

  // Reverse lanes
  const std::unordered_set<std::string>& reverse_lane_ids =
      environment_features.nonneglectable_reverse_lanes();
  if (reverse_lane_ids.empty()) {
    ADEBUG << "No reverse lane considered";
  }
  for (const std::string& reverse_lane_id : reverse_lane_ids) {
    lane_ids_of_interest_.insert(reverse_lane_id);
  }
  for (const std::string& reverse_lane_id : reverse_lane_ids) {
    std::shared_ptr<const LaneInfo> reverse_lane_info =
        PredictionMap::LaneById(reverse_lane_id);
    for (const auto& prede_id : reverse_lane_info->lane().predecessor_id()) {
      lane_ids_of_interest_.insert(prede_id.id());
    }
  }
}

void CruiseScenarioFeatures::SearchForwardAndInsert(
    const std::string& start_lane_id, const double start_lane_s,
    const double range) {
  ConstLaneInfoPtr start_lane_info = PredictionMap::LaneById(start_lane_id);
  double start_lane_length = start_lane_info->total_length();
  double start_accumulated_s = start_lane_length - start_lane_s;
  std::queue<std::pair<ConstLaneInfoPtr, double>> lane_queue;
  lane_queue.emplace(start_lane_info, start_accumulated_s);

  while (!lane_queue.empty()) {
    ConstLaneInfoPtr lane_info = lane_queue.front().first;
    double accumulated_s = lane_queue.front().second;
    lane_queue.pop();
    InsertLaneOfInterest(lane_info->id().id());
    const Lane& lane = lane_info->lane();
    for (const auto prede_lane_id : lane.predecessor_id()) {
      InsertLaneOfInterest(prede_lane_id.id());
    }
    if (accumulated_s > range) {
      continue;
    }
    for (const auto succ_lane_id : lane.successor_id()) {
      ConstLaneInfoPtr succ_lane_info =
          PredictionMap::LaneById(succ_lane_id.id());
      double succ_lane_length = succ_lane_info->total_length();
      lane_queue.emplace(succ_lane_info, accumulated_s + succ_lane_length);
    }
  }
}

}  // namespace prediction
}  // namespace apollo
