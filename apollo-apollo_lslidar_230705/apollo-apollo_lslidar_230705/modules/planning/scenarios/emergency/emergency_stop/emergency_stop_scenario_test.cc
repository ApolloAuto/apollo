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

/**
 * @file
 **/

#include "modules/planning/scenarios/emergency/emergency_stop/emergency_stop_scenario.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "gtest/gtest.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace emergency_stop {

class EmergencyStopScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<EmergencyStopScenario> scenario_;
};

TEST_F(EmergencyStopScenarioTest, Init) {
  FLAGS_scenario_emergency_stop_config_file =
      "/apollo/modules/planning/conf/scenario"
      "/emergency_stop_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      FLAGS_scenario_emergency_stop_config_file, &config));
  ScenarioContext context;
  auto injector = std::make_shared<DependencyInjector>();
  scenario_.reset(new EmergencyStopScenario(config, &context, injector));
  EXPECT_EQ(scenario_->scenario_type(), ScenarioType::EMERGENCY_STOP);
}

}  // namespace emergency_stop
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
