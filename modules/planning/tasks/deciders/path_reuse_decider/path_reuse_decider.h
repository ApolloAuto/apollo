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

#include <vector>

#include "modules/planning/tasks/deciders/decider.h"

#include "modules/planning/common/history.h"
#include "modules/planning/proto/decider_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class PathReuseDecider : public Decider {
 public:
  explicit PathReuseDecider(const TaskConfig& config);

 private:
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;
  bool CheckPathReusable(Frame* frame);
  void GetCurrentStopPositions(
      Frame* frame,
      std::vector<const common::PointENU*>* current_stop_positions);
  void GetHistoryStopPositions(
      const std::vector<const HistoryObjectDecision*>&
          history_objects_decisions,
      std::vector<const common::PointENU*>* history_stop_positions);

 private:
  History* history_ = History::Instance();
  static int reusable_path_counter_;  // count reused path
  static int total_path_counter_;     // count total path
};

}  // namespace planning
}  // namespace apollo
