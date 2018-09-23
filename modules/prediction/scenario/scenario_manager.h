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

/**
 * @file
 */

#ifndef MODULES_PREDICTION_SCENARIO_SCENARIO_MANAGER_H_
#define MODULES_PREDICTION_SCENARIO_SCENARIO_MANAGER_H_

#include <vector>
#include <string>
#include <memory>

#include "cybertron/common/macros.h"
#include "modules/prediction/proto/scenario.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/prediction/scenario/feature_extractor/feature_extractor.h"
#include "modules/prediction/scenario/analyzer/scenario_analyzer.h"
#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/scenario/scenario_features/cruise_scenario_features.h"

namespace apollo {
namespace prediction {

class ScenarioManager {
 public:
  /**
   * @brief Run scenario analysis
   */
  void Run();

  /**
   * @brief Get scenario analysis result
   */
  const Scenario& scenario() const;

 private:
  void PrioritizeObstacles();

  void PrioritizeObstacle(
      const EnvironmentFeatures& environment_features,
      std::shared_ptr<ScenarioFeatures> scenario_features,
      Obstacle* obstacle_ptr);

  void PrioritizeObstacleInCruise(
      const EnvironmentFeatures& environment_features,
      std::shared_ptr<CruiseScenarioFeatures> scenario_features,
      Obstacle* obstacle_ptr);

 private:
  FeatureExtractor feature_extractor_;
  ScenarioAnalyzer scenario_analyzer_;

  DECLARE_SINGLETON(ScenarioManager)
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_SCENARIO_SCENARIO_MANAGER_H_
