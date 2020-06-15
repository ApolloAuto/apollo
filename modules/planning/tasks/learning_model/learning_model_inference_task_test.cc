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

#include "modules/planning/tasks/learning_model/learning_model_inference_task.h"

#include "gtest/gtest.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class LearningModelInferenceTaskTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::LEARNING_MODEL_INFERENCE_TASK);
    auto* inference_config =
        config_.mutable_learning_model_inference_task_config();
    inference_config->set_model_file(
        "/apollo/modules/planning/data/model/test_model_conv_rnn.pt");
    inference_config->set_use_cuda(true);
  }

  virtual void TearDown() {}

 protected:
  TaskConfig config_;
};

TEST_F(LearningModelInferenceTaskTest, Init) {
  LearningModelInferenceTask learning_model_inference_task(config_);
  EXPECT_EQ(learning_model_inference_task.Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

}  // namespace planning
}  // namespace apollo
