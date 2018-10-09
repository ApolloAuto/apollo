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
 */

#pragma once


#include <string>
#include <unordered_map>

#include "cybertron/common/macros.h"

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_status.pb.h"
#include "modules/routing/proto/routing.pb.h"

/**
 * @brief PlanningContext is the runtime context in planning. It is
 * persistent across multiple frames.
 */
namespace apollo {
namespace planning {

class PlanningContext {
 public:
  void Clear();

  PlanningStatus* mutable_planning_status() { return &planning_status_; }

  void UpdateRouting(const apollo::routing::RoutingResponse& routing);

  // assign a static anchor_s to all lanes in current routing. One purpose is to
  // let path sampler always sample a static point given a lane id;
  struct RouteLaneInfo {
    float anchor_s = 0.0f;
    float start_s = 0.0f;
  };

  RouteLaneInfo FindLaneStaticS(const std::string& lane_id) const;

 private:
  PlanningStatus planning_status_;

  std::unordered_map<std::string, RouteLaneInfo> lane_anchor_s_;

  // this is a singleton class
  DECLARE_SINGLETON(PlanningContext);
};

inline PlanningStatus* mutable_planning_status() {
  return PlanningContext::Instance()->mutable_planning_status();
}

}  // namespace planning
}  // namespace apollo
