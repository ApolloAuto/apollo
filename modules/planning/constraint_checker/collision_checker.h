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

#ifndef MODULES_PLANNING_CONSTRAINT_CHECKER_COLLISION_CHECKER_H_
#define MODULES_PLANNING_CONSTRAINT_CHECKER_COLLISION_CHECKER_H_

#include <array>
#include <vector>

#include "modules/common/math/box2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

class CollisionChecker {
 public:
  explicit CollisionChecker(
      const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<common::PathPoint>& discretized_reference_line);

  bool InCollision(const DiscretizedTrajectory& discretized_trajectory);

 private:
  void BuildPredictedEnvironment(
      const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<common::PathPoint>& discretized_reference_line);

  bool IsEgoVehicleInLane(const double ego_vehicle_d);

  bool ShouldIgnore(
      const Obstacle* obstacle, const double ego_vehicle_s,
      const std::vector<apollo::common::PathPoint>& discretized_reference_line);

  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_CONSTRAINT_CHECKER_COLLISION_CHECKER_H_
