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

#include "modules/prediction/scenario/scenario_manager.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;

ScenarioManager::ScenarioManager() {}

void ScenarioManager::Run() {
  feature_extractor_.ExtractFeatures();
  scenario_analyzer_.Analyze(feature_extractor_.GetEnvironmentFeatures());
  PrioritizeObstacles();
  // TODO(all) other functionalities including lane, junction filters
}

const Scenario& ScenarioManager::scenario() const {
  return scenario_analyzer_.scenario();
}

void ScenarioManager::PrioritizeObstacles() {
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::Instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  obstacles_container->PrioritizeObstacles(scenario_analyzer_.scenario());
}

}  // namespace prediction
}  // namespace apollo
