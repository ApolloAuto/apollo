/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/learning_based_data.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

class DependencyInjector {
 public:
  DependencyInjector() = default;
  ~DependencyInjector() = default;

  PlanningContext* planning_context() {
    return &planning_context_;
  }
  FrameHistory* frame_history() {
    return &frame_history_;
  }
  History* history() {
    return &history_;
  }
  EgoInfo* ego_info() {
    return &ego_info_;
  }
  apollo::common::VehicleStateProvider* vehicle_state() {
    return &vehicle_state_;
  }
  LearningBasedData* learning_based_data() {
    return &learning_based_data_;
  }

 private:
  PlanningContext planning_context_;
  FrameHistory frame_history_;
  History history_;
  EgoInfo ego_info_;
  apollo::common::VehicleStateProvider vehicle_state_;
  LearningBasedData learning_based_data_;
};

}  // namespace planning
}  // namespace apollo
