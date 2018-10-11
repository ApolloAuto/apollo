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

/**
 * @file
 **/

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/proto/planning_status.pb.h"

#include "cybertron/common/macros.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/toolkits/deciders/decider.h"

namespace apollo {
namespace planning {

class DeciderStopSignMonitor : public Decider {
  typedef std::unordered_map<std::string, std::vector<std::string>>
      StopSignLaneVehicles;

 public:
  DeciderStopSignMonitor();
  ~DeciderStopSignMonitor() = default;

  bool Init(const ScenarioConfig::ScenarioTaskConfig &config) override;

 private:
  apollo::common::Status Process(
      Frame* frame,
      ReferenceLineInfo* reference_line_info) override;

  void UpdateWatchVehicles(StopSignLaneVehicles* watch_vehicles);
  int AddWatchVehicle(const PathObstacle& path_obstacle,
                      StopSignLaneVehicles* watch_vehicles);

 private:
  hdmap::PathOverlap next_stop_sign_overlap_;
  hdmap::StopSignInfoConstPtr next_stop_sign_ = nullptr;
  StopSignStatus::Status stop_status_;
  std::vector<std::pair<hdmap::LaneInfoConstPtr, hdmap::OverlapInfoConstPtr>>
      associated_lanes_;
};

}  // namespace planning
}  // namespace apollo
