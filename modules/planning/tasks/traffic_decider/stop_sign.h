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

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_STOP_SIGN_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_STOP_SIGN_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/proto/planning_status.pb.h"
#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

class StopSign : public TrafficRule {
  typedef std::unordered_map<std::string, std::vector<std::string>>
      StopSignLaneVehicles;

 public:
  explicit StopSign(const TrafficRuleConfig& config);
  virtual ~StopSign() = default;

  bool ApplyRule(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);

 private:
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool FindNextStopSign(ReferenceLineInfo* const reference_line_info);
  int GetAssociatedLanes(const hdmap::StopSignInfo& stop_sign_info);
  int ProcessStopStatus(ReferenceLineInfo* const reference_line_info,
                        const hdmap::StopSignInfo& stop_sign_info);
  bool CheckADCkStop(ReferenceLineInfo* const reference_line_info);
  int GetWatchVehicles(const hdmap::StopSignInfo& stop_sign_info,
                       StopSignLaneVehicles* watch_vehicles);
  int UpdateWatchVehicles(StopSignLaneVehicles* watch_vehicles);
  int AddWatchVehicle(const PathObstacle& path_obstacle,
                      StopSignLaneVehicles* watch_vehicles);
  int RemoveWatchVehicle(const PathObstacle& path_obstacle,
                         const std::vector<std::string>& watch_vehicle_ids,
                         StopSignLaneVehicles* watch_vehicles);
  int ClearWatchVehicle(ReferenceLineInfo* const reference_line_info,
                        StopSignLaneVehicles* watch_vehicles);
  bool BuildStopDecision(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info,
                         hdmap::PathOverlap* const overlap,
                         const double stop_distance);

 private:
  static constexpr char const* const STOP_SIGN_VO_ID_PREFIX = "SS_";
  hdmap::PathOverlap* next_stop_sign_overlap_ = nullptr;
  hdmap::StopSignInfo* next_stop_sign_ = nullptr;
  StopSignStatus::Status stop_status_;
  std::vector<std::pair<hdmap::LaneInfoConstPtr, hdmap::OverlapInfoConstPtr>>
      associated_lanes_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_STOP_SIGN_H_
