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

#include "gtest/gtest.h"

#include "modules/prediction/proto/scenario_feature.pb.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using apollo::common::Scenario;

class ScenarioAnalyzerTest : public KMLMapBasedTest {};

TEST_F(ScenarioAnalyzerTest, unknown) {
  ScenarioFeature scenario_feature;
  ScenarioAnalyzer scenario_analyzer;
  scenario_analyzer.Analyze(scenario_feature);
  Scenario scenario = scenario_analyzer.scenario();
  EXPECT_EQ(scenario.type(), Scenario::UNKNOWN);
}

TEST_F(ScenarioAnalyzerTest, junction) {
  ScenarioFeature scenario_feature;
  scenario_feature.set_junction_id("1");
  scenario_feature.set_dist_to_junction(3.0);
  ScenarioAnalyzer scenario_analyzer;
  scenario_analyzer.Analyze(scenario_feature);
  Scenario scenario = scenario_analyzer.scenario();
  EXPECT_EQ(scenario.type(), Scenario::JUNCTION_UNKNOWN);
}

}  // namespace prediction
}  // namespace apollo
