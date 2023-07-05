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

#include "modules/prediction/scenario/analyzer/scenario_analyzer.h"

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/scenario/scenario_features/cruise_scenario_features.h"
#include "modules/prediction/scenario/scenario_features/junction_scenario_features.h"

namespace apollo {
namespace prediction {

std::shared_ptr<ScenarioFeatures> ScenarioAnalyzer::Analyze(
    const EnvironmentFeatures& environment_features) {
  Scenario::Type scenario_type = Scenario::UNKNOWN;

  if (environment_features.has_front_junction() &&
      environment_features.GetFrontJunction().second <
          FLAGS_junction_distance_threshold) {
    scenario_type = Scenario::JUNCTION;
  } else if (environment_features.has_ego_lane()) {
    scenario_type = Scenario::CRUISE;
  }

  if (scenario_type == Scenario::CRUISE) {
    auto cruise_scenario_features = std::make_shared<CruiseScenarioFeatures>();
    cruise_scenario_features->BuildCruiseScenarioFeatures(environment_features);
    return cruise_scenario_features;
  }

  if (scenario_type == Scenario::JUNCTION) {
    auto junction_scenario_features =
        std::make_shared<JunctionScenarioFeatures>();
    // TODO(all) refactor this part
    junction_scenario_features->BuildJunctionScenarioFeatures(
        environment_features);
    return junction_scenario_features;
  }

  return std::make_shared<ScenarioFeatures>();
}

}  // namespace prediction
}  // namespace apollo
