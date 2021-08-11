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

#include "modules/third_party_perception/tools/fusion.h"

#include <vector>

#include "modules/common/math/polygon2d.h"

/**
 * @namespace apollo::third_party_perception::fusion
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {
namespace fusion {

using apollo::common::math::Vec2d;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;

std::vector<Vec2d> PerceptionObstacleToVectorVec2d(
    const PerceptionObstacle& obstacle) {
  std::vector<Vec2d> result;
  for (const auto& vertex : obstacle.polygon_point()) {
    result.emplace_back(vertex.x(), vertex.y());
  }
  return result;
}

bool HasOverlap(const PerceptionObstacle& obstacle_1,
                const PerceptionObstacle& obstacle_2) {
  common::math::Polygon2d polygon_1(
      PerceptionObstacleToVectorVec2d(obstacle_1));
  common::math::Polygon2d polygon_2(
      PerceptionObstacleToVectorVec2d(obstacle_2));
  return polygon_1.HasOverlap(polygon_2);
}

bool HasOverlap(const PerceptionObstacle& obstacle,
                const PerceptionObstacles& obstacles) {
  for (const auto& current_obstacle : obstacles.perception_obstacle()) {
    if (HasOverlap(obstacle, current_obstacle)) {
      return true;
    }
  }
  return false;
}

PerceptionObstacles EyeRadarFusion(const PerceptionObstacles& eye_obstacles,
                                   const PerceptionObstacles& radar_obstacles) {
  PerceptionObstacles eye_obstacles_fusion = eye_obstacles;
  PerceptionObstacles radar_obstacles_fusion = radar_obstacles;

  for (auto& eye_obstacle :
       *(eye_obstacles_fusion.mutable_perception_obstacle())) {
    for (auto& radar_obstacle :
         *(radar_obstacles_fusion.mutable_perception_obstacle())) {
      if (HasOverlap(eye_obstacle, radar_obstacle)) {
        eye_obstacle.set_confidence(0.99);
        eye_obstacle.mutable_velocity()->CopyFrom(radar_obstacle.velocity());
      }
    }
  }

  // mobileye_obstacles_fusion.MergeFrom(radar_obstacles_fusion);
  return eye_obstacles_fusion;
}

}  // namespace fusion
}  // namespace third_party_perception
}  // namespace apollo
