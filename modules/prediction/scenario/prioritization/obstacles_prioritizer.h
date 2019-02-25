/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <vector>

#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/scenario/scenario_features/cruise_scenario_features.h"
#include "modules/prediction/scenario/scenario_features/scenario_features.h"

namespace apollo {
namespace prediction {

class ObstaclesPrioritizer {
 public:
  ObstaclesPrioritizer() = delete;

  static void PrioritizeObstacles(
      const EnvironmentFeatures& environment_features,
      const std::shared_ptr<ScenarioFeatures> scenario_features);

 private:
  static void AssignIgnoreLevel(
      const EnvironmentFeatures& environment_features,
      const std::shared_ptr<ScenarioFeatures> scenario_features);

  static void AssignCautionLevelCruiseKeepLane();

  static void AssignCautionLevelCruiseChangeLane();

  static void AssignCautionLevelInJunction(
      const std::shared_ptr<ScenarioFeatures> scenario_features);

  static void AssignCautionLevelByEgoReferenceLine();

  static void AssignCautionByMerge(
      std::shared_ptr<const LaneInfo> lane_info_ptr);

  static void AssignCautionByOverlap(
      std::shared_ptr<const LaneInfo> lane_info_ptr);

  static void SetCautionBackward(
    std::shared_ptr<const LaneInfo> start_lane_info_ptr,
    const double distance);
};

}  // namespace prediction
}  // namespace apollo
