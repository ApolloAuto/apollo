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
#include "modules/planning/tasks/deciders/side_pass_safety.h"

#include "gtest/gtest.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"

#define private public

namespace apollo {
namespace planning {

class SidePassSafetyTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::SIDE_PASS_SAFETY);
    config_.mutable_side_pass_safety_config()
        ->set_min_obstacle_lateral_distance(1);
  }

  virtual void TearDown() {}

 protected:
  TaskConfig config_;
};

TEST_F(SidePassSafetyTest, Init) {
  const TaskConfig& config = config_;
  SidePassSafety side_pass_safety_(config);
  EXPECT_EQ(side_pass_safety_.Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

}  // namespace planning
}  // namespace apollo
