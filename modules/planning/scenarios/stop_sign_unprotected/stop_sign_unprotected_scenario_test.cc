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
 * @file stop_sign_unprotected_scenario_test.cc
 **/

#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_scenario.h"

#include "gtest/gtest.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/scenarios/stop_sign_unprotected/context.h"

namespace apollo {
namespace planning {

class StopSignUnprotectedScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<StopSignUnprotectedScenario> scenario_;
};

TEST_F(StopSignUnprotectedScenarioTest, Init) {
  std::string scenario_stop_sign_unprotected_config_file =
      "/apollo/modules/planning/scenarios/stop_sign_unprotected/conf/"
      "scenario_conf.pb.txt";

  ScenarioStopSignUnprotectedConfig config;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      scenario_stop_sign_unprotected_config_file, &config));

  auto injector = std::make_shared<DependencyInjector>();
  scenario_.reset(new StopSignUnprotectedScenario());
  scenario_->Init(injector, "STOP_SIGN_UNPROTECTED");
  EXPECT_EQ(scenario_->Name(), "STOP_SIGN_UNPROTECTED");
}

}  // namespace planning
}  // namespace apollo
