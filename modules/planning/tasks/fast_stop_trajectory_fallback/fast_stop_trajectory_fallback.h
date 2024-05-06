/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/trajectory_fallback_task.h"

namespace apollo {
namespace planning {

class FastStopTrajectoryFallback : public TrajectoryFallbackTask {
 private:
  SpeedData GenerateFallbackSpeed(const EgoInfo* ego_info,
                                  const double stop_distance = 0.0) override;

  static SpeedData GenerateStopProfile(const double init_speed,
                                       const double init_acc);
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::FastStopTrajectoryFallback, Task)
}  // namespace planning
}  // namespace apollo
