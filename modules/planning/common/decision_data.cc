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
 * @file decision_data.cc
 **/

#include "modules/planning/common/decision_data.h"

namespace apollo {
namespace planning {

const DecisionResult &DecisionData::Decision() const { return decision_; }

DecisionResult *DecisionData::MutableDecision() { return &decision_; }

const std::vector<Obstacle *> &DecisionData::Obstacles() const {
  return obstacles_;
}

const std::vector<Obstacle *> &DecisionData::StaticObstacles() const {
  return static_obstacles_;
}

const std::vector<Obstacle *> &DecisionData::DynamicObstacles() const {
  return dynamic_obstacles_;
}

std::vector<Obstacle *> *DecisionData::MutableStaticObstacles() {
  return &static_obstacles_;
}

std::vector<Obstacle *> *DecisionData::MutableDynamicObstacles() {
  return &dynamic_obstacles_;
}

void DecisionData::AddObstacle(Obstacle *obstacle) {
  obstacles_.push_back(obstacle);
  if (obstacle->IsStatic()) {
    static_obstacles_.push_back(obstacle);
  } else {
    dynamic_obstacles_.push_back(obstacle);
  }
}

}  // namespace planning
}  // namespace apollo
