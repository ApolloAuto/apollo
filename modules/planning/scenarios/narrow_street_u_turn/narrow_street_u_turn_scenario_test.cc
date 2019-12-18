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

#include "modules/planning/scenarios/narrow_street_u_turn/narrow_street_u_turn_scenario.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace narrow_street_u_turn {

class NarrowStreetUTurnTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<NarrowStreetUTurnScenario> scenario_;
};

TEST_F(NarrowStreetUTurnTest, Init) {
  FLAGS_scenario_narrow_street_u_turn_config_file =
      "/apollo/modules/planning/conf/scenario/"
      "narrow_street_u_turn_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      FLAGS_scenario_narrow_street_u_turn_config_file, &config));
  ScenarioContext context;
  scenario_.reset(new NarrowStreetUTurnScenario(config, &context));
  EXPECT_EQ(scenario_->scenario_type(), ScenarioConfig::NARROW_STREET_U_TURN);
}
}  // namespace narrow_street_u_turn
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
