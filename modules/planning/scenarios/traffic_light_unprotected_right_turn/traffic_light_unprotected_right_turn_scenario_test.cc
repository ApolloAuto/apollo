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
 **/

#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"

#include "gtest/gtest.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace planning {

class TrafficLightUnprotectedRightTurnScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<TrafficLightUnprotectedRightTurnScenario> scenario_;
};

TEST_F(TrafficLightUnprotectedRightTurnScenarioTest, Init) {
  std::string scenario_traffic_light_unprotected_right_turn_config_file =
      "/apollo/modules/planning/scenarios/traffic_light_unprotected_right_turn/"
      "conf/scenario_conf.pb.txt";

  ScenarioTrafficLightUnprotectedRightTurnConfig config;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      scenario_traffic_light_unprotected_right_turn_config_file, &config));

  auto injector = std::make_shared<DependencyInjector>();
  scenario_.reset(new TrafficLightUnprotectedRightTurnScenario());
  scenario_->Init(injector, "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN");
  EXPECT_EQ(scenario_->Name(), "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN");
}

}  // namespace planning
}  // namespace apollo
