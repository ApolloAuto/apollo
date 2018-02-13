/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/map/relative_map/navigation_lane.h"

#include "modules/common/log.h"

namespace apollo {
namespace relative_map {

using apollo::perception::PerceptionObstacles;

bool NavigationLane::Update(const PerceptionObstacles& perception_obstacles) {
  perception_obstacles_ = perception_obstacles;
  if (!perception_obstacles.has_lane_marker()) {
    AERROR << "No lane marker in perception_obstacles.";
    return false;
  }

  // TODO(All): lane_marker --> navigation path
  return true;
}

double NavigationLane::EvaluateCubicPolynomial(const double c0, const double c1,
                                               const double c2, const double c3,
                                               const double z) const {
  return c3 * std::pow(z, 3) + c2 * std::pow(z, 2) + c1 * z + c0;
}

}  // namespace relative_map
}  // namespace apollo
