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
#include "modules/planning/tasks/deciders/side_pass_path_decider.h"

#include "gtest/gtest.h"
#include "modules/planning/proto/planning_config.pb.h"

#define private public

namespace apollo {
namespace planning {

class SidePassPathDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::SIDE_PASS_PATH_DECIDER);
    config_.mutable_side_pass_path_decider_config();
    config_.mutable_side_pass_path_decider_config()->
        set_total_path_length(200.0);
    config_.mutable_side_pass_path_decider_config()->set_path_resolution(0.5);
    config_.mutable_side_pass_path_decider_config()->set_l_weight(1.0);
    config_.mutable_side_pass_path_decider_config()->set_dl_weight(1.0);
    config_.mutable_side_pass_path_decider_config()->set_ddl_weight(1.0);
    config_.mutable_side_pass_path_decider_config()->set_dddl_weight(1.0);
    config_.mutable_side_pass_path_decider_config()->
        set_guiding_line_weight(1.0);

    AINFO << config_.DebugString();
  }

  virtual void TearDown() {}

 protected:
  TaskConfig config_;
};

TEST_F(SidePassPathDeciderTest, Init) {
  SidePassPathDecider side_pass_path_decider(config_);
  EXPECT_EQ(side_pass_path_decider.Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

}  // namespace planning
}  // namespace apollo
