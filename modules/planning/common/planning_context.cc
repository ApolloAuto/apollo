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

#include "modules/planning/common/planning_context.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

PlanningContext::PlanningContext() {}

PlanningContext::RouteLaneInfo PlanningContext::FindLaneStaticS(
    const std::string& lane_id) const {
  auto iter = lane_anchor_s_.find(lane_id);
  if (iter == lane_anchor_s_.end()) {
    RouteLaneInfo info;
    return info;
  } else {
    return iter->second;
  }
}

void PlanningContext::UpdateRouting(
    const apollo::routing::RoutingResponse& routing) {
  // Note: when there are repeating lane ids in routing, the value of
  // lane_anchor_s_ may be overwrite by when there are repeating lane IDs in
  // routing. But for the purpose of assiging a static start point for all the
  // lanes on the route, it is OK.
  lane_anchor_s_.clear();
  float accumulated_s = 0.0;
  for (const auto& road : routing.road()) {
    float road_s = 0.0;
    for (const auto& passage : road.passage()) {
      for (const auto& lane : passage.segment()) {
        auto& info = lane_anchor_s_[lane.id()];
        info.anchor_s = accumulated_s + road_s;
        info.start_s = lane.start_s();
        road_s += lane.end_s() - lane.start_s();
      }
    }
    accumulated_s += road_s;
  }
}

void PlanningContext::Clear() {
  planning_status_.Clear();
  lane_anchor_s_.clear();
}

}  // namespace planning
}  // namespace apollo
