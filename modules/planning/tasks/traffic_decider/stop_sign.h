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
 * @file stop_sign.h
 **/

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_STOP_SIGN_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_STOP_SIGN_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

class StopSign : public TrafficRule {
  typedef std::unordered_map<std::string, std::vector<std::string>>
      StopSignLaneVehicles;

 public:
  explicit StopSign(const RuleConfig& config);
  virtual ~StopSign() = default;

  bool ApplyRule(Frame* frame, ReferenceLineInfo* const reference_line_info);

  enum class StopSignStopStatus {
    UNKNOWN = 0,
    TO_STOP = 1,
    STOPPING = 2,
    STOP_DONE = 3,
  };

 private:
  void MakeDecisions(Frame* frame,
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
  int ClearWatchVehicle(
      ReferenceLineInfo* const reference_line_info,
      StopSignLaneVehicles* watch_vehicles);
  double GetStopDeceleration(ReferenceLineInfo* const reference_line_info,
                             const hdmap::PathOverlap* stop_sign_overlap);
  bool BuildStopDecision(Frame* frame,
                         ReferenceLineInfo* const reference_line_info,
                         const hdmap::PathOverlap* stop_sign_overlap);
  void ClearDropbox(const std::string& stop_sign_id);
  void ClearDropboxWatchvehicles();

 private:
  constexpr static char const* const db_key_stop_sign_stop_status_prefix_ =
      "kStopSignStopStatus_";
  constexpr static char const* const db_key_stop_sign_stop_starttime_prefix_ =
      "kStopSignStopStarttime_";
  constexpr static char const* const db_key_stop_sign_watch_vehicle_prefix_ =
      "kStopSignWatchVehicle_";

  hdmap::PathOverlap* next_stop_sign_overlap_ = nullptr;
  hdmap::StopSignInfo* next_stop_sign_ = nullptr;
  StopSignStopStatus stop_status_;
  std::vector<std::pair<hdmap::LaneInfoConstPtr, hdmap::OverlapInfoConstPtr>>
      associated_lanes_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_STOP_SIGN_H_
