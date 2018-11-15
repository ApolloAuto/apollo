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

using apollo::common::Scenario;

namespace apollo {
namespace prediction {

void ScenarioAnalyzer::Analyze(const ScenarioFeature& scenario_feature) {
  if (scenario_feature.has_junction_id() &&
      scenario_feature.dist_to_junction() < 10.0) {  // TODO(kechxu) gflags
    scenario_.set_type(Scenario::JUNCTION_UNKNOWN);
  } else if (scenario_feature.has_curr_lane_id()) {
    scenario_.set_type(Scenario::CRUISE_UNKNOWN);
  }
}

const Scenario& ScenarioAnalyzer::scenario() const {
  return scenario_;
}

}  // namespace prediction
}  // namespace apollo
