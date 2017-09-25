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

void MobileyeRadarFusion(
    PerceptionObstacles* const mobileye_obstacles, PerceptionObstacles* const radar_obstacles) {
  for (int mobileye_index = 0; mobileye_index < mobileye_obstacles->perception_obstacle_size(); ++mobileye_index) {
    auto* mob = mobileye_obstacles->mutable_perception_obstacle(mobileye_index);
    for (int radar_index = 0; radar_index < radar_obstacles->perception_obstacle_size(); ++radar_index) {
      auto* rob = radar_obstacles->mutable_perception_obstacle(radar_index);
      if (Distance(mob->position(), rob->position()) < FLAGS_fusion_distance) {
        mob->set_confidence(0.99);
        rob->set_confidence(0.99);
      }
    }
  } 
}

}  // namespace fusion 
}  // namespace l3_perception
}  // namespace apollo




