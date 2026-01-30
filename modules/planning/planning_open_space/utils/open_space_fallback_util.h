/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/planning/planning_base/common/open_space_info.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {
class OpenSpaceFallbackUtil {
public:
    static bool IsCollisionFreeTrajectory(
            const TrajGearPair& trajectory_gear_pair,
            const std::vector<std::vector<common::math::Box2d>>& predicted_bounding_rectangles,
            const std::vector<Obstacle*>& static_obstacles,
            int* current_index,
            int* first_collision_index,
            bool& is_collision_with_static_obstacle,
            bool& is_collision_with_dynamic_obstacle,
            const double& collilsion_time_buffer);

    static void BuildPredictedEnvironment(
            const common::math::Vec2d ego_pos,
            const std::vector<const Obstacle*>& obstacles,
            std::vector<std::vector<common::math::Box2d>>& predicted_bounding_rectangles,
            const double& time_period, const double& build_range);

    static void BuildStaticObstacleEnvironment(
            const common::math::Vec2d ego_pos,
            const std::vector<const Obstacle*>& obstacles,
            std::vector<Obstacle*>& static_obstacles, const double& build_range);
};
}  // namespace planning
}  // namespace apollo
