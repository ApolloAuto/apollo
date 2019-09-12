/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <utility>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/history.h"
#include "modules/planning/proto/decider_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class PathReuseDecider : public Decider {
 public:
  explicit PathReuseDecider(const TaskConfig& config);

 private:
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  // check if previous path reusable
  bool CheckPathReusable(Frame* frame, ReferenceLineInfo* reference_line_info);

  void GetCurrentStopPositions(
      Frame* frame,
      std::vector<const common::PointENU*>* current_stop_positions);

  // get current s_projection of history objects which has stop decisions
  void GetHistoryStopPositions(
      ReferenceLineInfo* const reference_line_info,
      const std::vector<const HistoryObjectDecision*>&
          history_objects_decisions,
      std::vector<std::pair<const double, const common::PointENU>>*
          history_stop_positions);

  // get current s_projection of current virtual obstacles
  void GetCurrentStopObstacleS(ReferenceLineInfo* const reference_line_info,
                               std::vector<double>* current_stop_obstacle);

  void GetHistoryStopSPosition(ReferenceLineInfo* const reference_line_info,
                               const std::vector<const HistoryObjectDecision*>&
                                   history_objects_decisions,
                               std::vector<double>* history_stop_positions);

  // compared stop decision in s-direction
  bool SameStopS(const double history_stop_s, const double current_stop_s);

  // check if the nearest virtual obstacle in history is same as current
  bool IsSameVirtualObstacles(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info);

 private:
  History* history_ = History::Instance();
  static int reusable_path_counter_;  // count reused path
  static int total_path_counter_;     // count total path
};

}  // namespace planning
}  // namespace apollo
