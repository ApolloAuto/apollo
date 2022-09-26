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

#include "modules/planning/scenarios/learning_model/stage_run.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "gtest/gtest.h"

namespace apollo {
namespace planning {
namespace scenario {

class StageRunTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_stage_type(StageType::LEARNING_MODEL_RUN);
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  ScenarioConfig::StageConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(StageRunTest, Init) {
  LearningModelSampleStageRun learning_model_stage_run(config_, injector_);
  EXPECT_EQ(learning_model_stage_run.Name(),
            StageType_Name(StageType::LEARNING_MODEL_RUN));
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
