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
 * @file smooth_stop_trajectory_fallback.h
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/trajectory_fallback_task.h"

namespace apollo {
namespace planning {

class SmoothStopTrajectoryFallback : public TrajectoryFallbackTask {
public:
    SpeedData GenerateFallbackSpeed(const EgoInfo* ego_info, const double stop_distance = 0.0) override;

private:
    bool IsCollisionWithSpeedBoundaries(
            const SpeedData& speed_data) const;

    /**
     * @brief Get the lower st boundaries of reference line info.
     * @param lower_bound Output lower st boundaries.
     */
    void GetLowerSTBound(std::vector<std::vector<common::math::Vec2d>>& lower_bounds);
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::SmoothStopTrajectoryFallback, Task)
}  // namespace planning
}  // namespace apollo
