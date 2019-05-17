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

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/macros.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/pnc_map/path.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/proto/path_decider_info.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

/**
 * @brief PlanningContext is the runtime context in planning. It is
 * persistent across multiple frames.
 */
namespace apollo {
namespace planning {

class PlanningContext {
 public:
  struct FallBackInfo {
    std::string last_successful_path_label;
  };

  const FallBackInfo& fallback_info() { return fallback_info_; }

  FallBackInfo* mutable_fallback_info() { return &fallback_info_; }

  struct OpenSpaceInfo {
    std::vector<std::string> partitioned_trajectories_index_history;
  };

  const OpenSpaceInfo& open_space_info() { return open_space_info_; }

  OpenSpaceInfo* mutable_open_space_info() { return &open_space_info_; }

  void Clear();

  void Init();

  const PlanningStatus& planning_status() { return planning_status_; }

  PlanningStatus* mutable_planning_status() { return &planning_status_; }

  const PathDeciderInfo& path_decider_info() { return path_decider_info_; }

  PathDeciderInfo* mutable_path_decider_info() { return &path_decider_info_; }

 private:
  PlanningStatus planning_status_;
  FallBackInfo fallback_info_;
  OpenSpaceInfo open_space_info_;
  PathDeciderInfo path_decider_info_;

  // this is a singleton class
  DECLARE_SINGLETON(PlanningContext)
};

}  // namespace planning
}  // namespace apollo
