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

#include "modules/third_party_perception/filter.h"

#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/common/third_party_perception_util.h"

/**
 * @namespace apollo::third_party_perception::filter
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {
namespace filter {

bool IsPreserved(const RadarObstacle& radar_obstacle) {
  if (!radar_obstacle.movable()) {
    return false;
  }
  double nearest_l =
      GetLateralDistanceToNearestLane(radar_obstacle.absolute_position());
  if (std::abs(nearest_l) > FLAGS_filter_y_distance) {
    return false;
  }
  if (radar_obstacle.count() < FLAGS_keep_radar_frames) {
    return false;
  }
  return true;
}

RadarObstacles FilterRadarObstacles(const RadarObstacles& radar_obstacles) {
  RadarObstacles filtered_radar_obstacles;
  for (const auto& iter : radar_obstacles.radar_obstacle()) {
    if (IsPreserved(iter.second)) {
      filtered_radar_obstacles.mutable_radar_obstacle()->insert(iter);
    }
  }
  filtered_radar_obstacles.mutable_header()->CopyFrom(radar_obstacles.header());
  return filtered_radar_obstacles;
}

}  // namespace filter
}  // namespace third_party_perception
}  // namespace apollo
