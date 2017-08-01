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
 * @file decision_data.h
 **/

#ifndef MODULES_PLANNING_COMMON_DECISION_DATA_H_
#define MODULES_PLANNING_COMMON_DECISION_DATA_H_

#include <vector>

#include "modules/planning/common/obstacle.h"

namespace apollo {
namespace planning {

class DecisionData {
 public:
  DecisionData() = default;

  const DecisionResult &Decision() const;
  DecisionResult *MutableDecision();

  const std::vector<Obstacle> &Obstacles() const;
  const std::vector<Obstacle *> &StaticObstacles() const ;
  const std::vector<Obstacle *> &DynamicObstacles() const;
  std::vector<Obstacle *> *MutableStaticObstacles();
  std::vector<Obstacle *> *MutableDynamicObstacles();
  void AddObstacle(Obstacle &obstacle);

 private:
  DecisionResult decision_;
  std::vector<Obstacle> obstacles_;
  std::vector<Obstacle *> static_obstacles_;
  std::vector<Obstacle *> dynamic_obstacles_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_DECISION_DATA_H_
