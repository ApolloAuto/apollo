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
 * @file base_stage_cruise.h
 **/

#pragma once

#include "modules/planning/planning_interface_base/scenario_base/stage.h"

namespace apollo {
namespace hdmap {
class PathOverlap;
}  // namespace hdmap
}  // namespace apollo

namespace apollo {
namespace planning {

class PlanningContext;

class BaseStageCruise : public Stage {
 public:
  bool CheckDone(const Frame& frame, const PlanningContext* context,
                 const bool right_of_way_status);

 private:
  /**
   * @brief Get the traffic sign overlap of the stage.
   *
   * @param reference_line_info current reference line information
   * @param context planning context containt realtime planning information.
   * @return traffic sign overlap of the stage
   */
  virtual hdmap::PathOverlap* GetTrafficSignOverlap(
      const ReferenceLineInfo& reference_line_info,
      const PlanningContext* context) const = 0;
};

}  // namespace planning
}  // namespace apollo
