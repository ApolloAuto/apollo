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
 * @file obstacle_blocking_analyzer.h
 */

#pragma once

#include <string>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

bool IsNonmovableObstacle(const ReferenceLineInfo& reference_line_info,
                          const Obstacle& obstacle);

/**
 * @brief Decide whether an obstacle is a blocking one that needs to be
 *        side-passed.
 * @param The frame that contains reference_line and other info.
 * @param The obstacle of interest.
 * @param The speed threshold to tell whether an obstacle is stopped.
 * @param The minimum distance to front blocking obstacle for side-pass.
 *        (if too close, don't side-pass for safety consideration)
 * @param Whether to take into consideration that the blocking obstacle
 *        itself is blocked by others as well. In other words, if the
 *        front blocking obstacle is blocked by others, then don't try
 *        to side-pass it. (Parked obstacles are never blocked by others)
 */
bool IsBlockingObstacleToSidePass(const Frame& frame, const Obstacle* obstacle,
                                  double block_obstacle_min_speed,
                                  double min_front_sidepass_distance,
                                  bool enable_obstacle_blocked_check);

double GetDistanceBetweenADCAndObstacle(const Frame& frame,
                                        const Obstacle* obstacle);

// Check if the obstacle is blocking ADC's driving path (reference_line).
bool IsBlockingDrivingPathObstacle(const ReferenceLine& reference_line,
                                   const Obstacle* obstacle);

bool IsParkedVehicle(const ReferenceLine& reference_line,
                     const Obstacle* obstacle);

bool IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info,
    const std::string& blocking_obstacle_id);

double DistanceBlockingObstacleToIntersection(
    const ReferenceLineInfo& reference_line_info,
    const std::string& blocking_obstacle_id);

double DistanceBlockingObstacleToJunction(
    const ReferenceLineInfo& reference_line_info,
    const std::string& blocking_obstacle_id);

bool IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info,
    const std::string& blocking_obstacle_id, const double threshold);

}  // namespace planning
}  // namespace apollo
