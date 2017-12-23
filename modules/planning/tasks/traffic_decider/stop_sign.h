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
#include <vector>

#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

using apollo::hdmap::LaneInfo;
using apollo::hdmap::PathOverlap;
using apollo::hdmap::StopSignInfo;
using apollo::perception::PerceptionObstacle;
using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

class StopSign : public TrafficRule {
 public:
  explicit StopSign(const RuleConfig& config);
  virtual ~StopSign() = default;

  bool ApplyRule(Frame* frame, ReferenceLineInfo* const reference_line_info);

 private:
  enum class StopSignStopStatus {
    TO_STOP = 0,
    STOPPING = 1,
    STOP_DONE = 2,
  };

  void MakeDecisions(Frame* frame,
                     ReferenceLineInfo* const reference_line_info);
  bool FindNextStopSign(ReferenceLineInfo* const reference_line_info);
  int GetAssociateLanes(const StopSignInfo& stop_sign_info);
  int ProcessStopStatus(ReferenceLineInfo* const reference_line_info,
                        const StopSignInfo& stop_sign_info);
  bool ChecADCkStop(ReferenceLineInfo* const reference_line_info);
  int GetWatchVehicles(const StopSignInfo& stop_sign_info,
                       StopSignLaneVehicles* watch_vehicles);
  int UpdateWatchVehicles(StopSignLaneVehicles* watch_vehicles);
  int AddWatchVehicle(const PathObstacle& obstacle,
                      StopSignLaneVehicles* watch_vehicles);
  int RemoveWatchVehicle(const PathObstacle& obstacle,
                         StopSignLaneVehicles* watch_vehicles);
  double GetStopDeceleration(ReferenceLineInfo* const reference_line_info,
                             const PathOverlap* stop_sign_overlap);
  void CreateStopObstacle(Frame* frame,
                          ReferenceLineInfo* const reference_line_info,
                          const PathOverlap* stop_sign_overlap);
  void ClearDropbox(const std::string& stop_sign_id);

 private:
  constexpr static char const* const db_key_stop_sign_stop_status_prefix_ =
      "kStopSignStopStatus_";
  constexpr static char const* const db_key_stop_sign_stop_starttime_prefix_ =
      "kStopSignStopStarttime_";
  constexpr static char const* const db_key_stop_sign_watch_vehicle_prefix_ =
      "kStopSignWatchVehicle_";

  PathOverlap* next_stop_sign_overlap_;
  StopSignInfo* next_stop_sign_;
  StopSignStopStatus stop_status_;
  std::vector<const LaneInfo*> associate_lanes_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_STOP_SIGN_H_
