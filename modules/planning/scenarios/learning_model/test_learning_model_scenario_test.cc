/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/learning_model/test_learning_model_scenario.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {

class TestLearningModelScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<TestLearningModelScenario> scenario_;
};

TEST_F(TestLearningModelScenarioTest, Init) {
  FLAGS_scenario_test_learning_model_config_file =
    "/apollo/modules/planning/conf/scenario/"
    "test_learning_model_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      FLAGS_scenario_test_learning_model_config_file, &config));
  ScenarioContext context;
  scenario_.reset(new TestLearningModelScenario(config, &context));
  EXPECT_EQ(scenario_->scenario_type(), ScenarioConfig::TEST_LEARNING_MODEL);
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
