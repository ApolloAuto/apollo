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

#include "modules/prediction/scenario/scenario_features/cruise_scenario_features.h"

#include "gtest/gtest.h"

namespace apollo {
namespace prediction {

class CruiseScenarioFeaturesTest : public ::testing::Test {
 public:
  void SetUp() override {}
};

TEST_F(CruiseScenarioFeaturesTest, LaneId) {
  CruiseScenarioFeatures cruise_scenario_features;
  EXPECT_FALSE(cruise_scenario_features.IsLaneOfInterest("1-1"));
  cruise_scenario_features.InsertLaneOfInterest("1-1");
  EXPECT_TRUE(cruise_scenario_features.IsLaneOfInterest("1-1"));
}

}  // namespace prediction
}  // namespace apollo
