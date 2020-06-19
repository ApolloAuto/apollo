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

#pragma once

#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/scenario/analyzer/scenario_analyzer.h"
#include "modules/prediction/scenario/feature_extractor/feature_extractor.h"
#include "modules/prediction/scenario/scenario_features/cruise_scenario_features.h"

namespace apollo {
namespace prediction {

class ScenarioManager {
 public:
  /**
   * @brief Constructor
   */
  ScenarioManager() = default;

  /**
   * @brief Destructor
   */
  ~ScenarioManager() = default;

  /**
   * @brief Run scenario analysis
   */
  void Run(ContainerManager* container_manager);

  /**
   * @brief Get scenario analysis result
   */
  const Scenario scenario() const;

 private:
  Scenario current_scenario_;
};

}  // namespace prediction
}  // namespace apollo
