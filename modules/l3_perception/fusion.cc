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

#include "modules/l3_perception/fusion.h"
#include "modules/l3_perception/l3_perception_gflags.h"
#include "modules/l3_perception/l3_perception_util.h"

/**
 * @namespace apollo::l3_perception::convertion
 * @brief apollo::l3_perception
 */
namespace apollo {
namespace l3_perception {
namespace fusion {

using ::apollo::l3_perception::Distance;

bool HasOverlap(const PerceptionObstacle& obstacle, const PerceptionObstacles& obstacles) {
  // TODO(rongqiqiu): compute distances according to bboxes
  for (const auto& current_obstacle : obstacles.perception_obstacle()) {
    if (std::abs(current_obstacle.position().x() - obstacle.position().x()) <
            FLAGS_fusion_x_distance &&
        std::abs(current_obstacle.position().y() - obstacle.position().y()) <
            FLAGS_fusion_y_distance) {
      return true;
    }
  }
  return false;
}

PerceptionObstacles MobileyeRadarFusion(
    const PerceptionObstacles& mobileye_obstacles, const PerceptionObstacles& radar_obstacles) {
  PerceptionObstacles merged_obstacles;
  merged_obstacles.MergeFrom(mobileye_obstacles);
  merged_obstacles.MergeFrom(radar_obstacles);

  PerceptionObstacles obstacles;
  obstacles.mutable_header()->CopyFrom(merged_obstacles.header());
  for (const auto& current_obstacle : merged_obstacles.perception_obstacle()) {
    if (!HasOverlap(current_obstacle, obstacles)) {
      auto* obstacle = obstacles.add_perception_obstacle();
      obstacle->CopyFrom(current_obstacle);
      // TODO(rongqiqiu): compute confidence scores
      obstacle->set_type(PerceptionObstacle::VEHICLE);
    }
  }

  return obstacles;
}

}  // namespace fusion 
}  // namespace l3_perception
}  // namespace apollo
