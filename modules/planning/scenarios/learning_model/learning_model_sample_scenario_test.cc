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

#include "modules/planning/scenarios/learning_model/learning_model_sample_scenario.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "gtest/gtest.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {

class LearningModelSampleScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<LearningModelSampleScenario> scenario_;
};

TEST_F(LearningModelSampleScenarioTest, Init) {
  FLAGS_scenario_learning_model_sample_config_file =
      "/apollo/modules/planning/conf/scenario/"
      "learning_model_sample_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      FLAGS_scenario_learning_model_sample_config_file, &config));
  ScenarioContext context;
  auto injector = std::make_shared<DependencyInjector>();
  scenario_.reset(new LearningModelSampleScenario(config, &context, injector));
  EXPECT_EQ(scenario_->scenario_type(), ScenarioType::LEARNING_MODEL_SAMPLE);
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
