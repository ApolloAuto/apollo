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
 **/

#include "modules/planning/common/path_obstacle.h"

#include "modules/common/log.h"

namespace apollo {
namespace planning {

const std::string& PathObstacle::Id() const { return id_; }

PathObstacle::PathObstacle(const planning::Obstacle* obstacle,
                           const ReferenceLine* reference_line)
    : obstacle_(obstacle) {
  CHECK_NOTNULL(obstacle) << "obstacle is null";
  id_ = obstacle_->Id();
  CHECK(reference_line != nullptr) << "reference line is null";
  Init(reference_line);
}

bool PathObstacle::Init(const ReferenceLine* reference_line) {}

const planning::Obstacle* PathObstacle::Obstacle() const { return obstacle_; }

void PathObstacle::AddDecision(const ObjectDecisionType& decision) {
  decisions_.push_back(decision);
}

const std::vector<ObjectDecisionType>& PathObstacle::Decisions() const {
  return decisions_;
}

}  // namespace planning
}  // namespace apollo
