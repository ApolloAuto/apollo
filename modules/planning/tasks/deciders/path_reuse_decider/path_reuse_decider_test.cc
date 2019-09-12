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
#define private public
#define protected public

#include "modules/planning/tasks/deciders/path_reuse_decider/path_reuse_decider.h"

#include "gtest/gtest.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class PathReuseDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::PATH_REUSE_DECIDER);
    config_.mutable_path_reuse_decider_config();
  }

  virtual void TearDown() {}

 protected:
  TaskConfig config_;
};

TEST_F(PathReuseDeciderTest, Init) {
  PathReuseDecider path_reuse_decider(config_);
  EXPECT_EQ(path_reuse_decider.Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

}  // namespace planning
}  // namespace apollo
