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

#ifndef MODULES_PLANNING_COMMON_PATH_OBSTACLE_H_
#define MODULES_PLANNING_COMMON_PATH_OBSTACLE_H_

#include <list>
#include <string>
#include <vector>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/decision.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

/**
 * @class PathObstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 */
class PathObstacle {
 public:
  PathObstacle() = default;

  PathObstacle(const planning::Obstacle* obstacle,
               const ReferenceLine* reference_line);

  const std::string& Id() const;

  const planning::Obstacle* Obstacle() const;

  /**
   * @class add decision to this obstacle.
   * @param decider_tag identifies which component added this decision
   * @param the decision to be added to this path obstacle.
   */
  void AddDecision(const std::string& decider_tag,
                   const ObjectDecisionType& decision);

  const std::vector<ObjectDecisionType>& Decisions() const;

  const std::string DebugString() const;

 private:
  bool Init(const ReferenceLine* reference_line);

  std::string id_;
  const planning::Obstacle* obstacle_ = nullptr;
  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_OBSTACLE_H_
