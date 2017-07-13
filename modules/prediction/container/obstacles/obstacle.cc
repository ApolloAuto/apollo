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

#include "modules/prediction/container/obstacles/obstacle.h"

namespace apollo {
namespace prediction {

using apollo::perception::PerceptionObstacle;

std::mutex Obstacle::_mutex;

Obstacle::Obstacle() : 
    id_(-1),
    type_(PerceptionObstacle::UNKNOWN_MOVABLE),
    feature_history_(0),
    kf_motion_tracker_(),
    is_motion_tracker_enabled_(false),
    kf_lane_tracker_map_(0) {

}

Obstacle::~Obstacle() {
  id_ = -1;
  type_ = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  feature_history_.clear();
  is_motion_tracker_enabled_ = false;
  kf_lane_tracker_map_.clear();
  // TODO(author) current_lanes_.clear();
}

void Obstacle::Insert(
    const apollo::perception::PerceptionObstacle& perception_obstacle,
    const double timestamp) {

}

}  // namespace prediction
}  // namespace apollo
