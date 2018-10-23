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

#include "cyber/common/macros.h"

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/pnc_map/path.h"
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
  struct ProceedWithCautionSpeedParam {
    enum class Type {
      FIXED_SPEED,
      FIXED_DISTANCE,
    };
    Type type = Type::FIXED_SPEED;
    double speed = 2.23;  // m/s. (5 mph)
    double distance = 5.0;  // m
  };

  // scenario context
  struct ScenarioInfo {
    apollo::hdmap::PathOverlap next_stop_sign_overlap;
    ProceedWithCautionSpeedParam proceed_with_caution_speed;
  };

  static void Clear();

  static void Init();

  static const PlanningStatus& Planningstatus() { return planning_status_; }

  static PlanningStatus* MutablePlanningStatus() { return &planning_status_; }

  static ScenarioInfo* GetScenarioInfo() { return &scenario_info_; }

 private:
  static PlanningStatus planning_status_;
  static ScenarioInfo scenario_info_;

  // this is a singleton class
  DECLARE_SINGLETON(PlanningContext);
};

}  // namespace planning
}  // namespace apollo
