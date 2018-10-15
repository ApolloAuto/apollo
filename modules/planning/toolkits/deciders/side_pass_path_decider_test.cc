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
#include "modules/planning/toolkits/deciders/side_pass_path_decider.h"

#include "gtest/gtest.h"
#include "modules/planning/proto/planning_config.pb.h"

#define private public

namespace apollo {
namespace planning {

class SidePassPathDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    // TODO(all) load task config here
  }

  virtual void TearDown() {}

 protected:
  TaskConfig config_;
};

TEST_F(SidePassPathDeciderTest, Init) {
  const TaskConfig &config = config_;
  SidePassPathDecider side_pass_path_decider(config);
  EXPECT_EQ(side_pass_path_decider.Name(), "SidePassPathDecider");
}

}  // namespace planning
}  // namespace apollo
