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

#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

#include <memory>

#include "gtest/gtest.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

class LaneFollowScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<LaneFollowScenario> scenario_;
};
TEST_F(LaneFollowScenarioTest, Simple) {
  scenario_.reset(new LaneFollowScenario());
  EXPECT_EQ(scenario_->scenario_type(), ScenarioConfig::LANE_FOLLOW);

  FLAGS_planning_config_file = "modules/planning/conf/planning_config.pb.txt";
  PlanningConfig config;
  EXPECT_TRUE(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                                     &config));
  EXPECT_TRUE(scenario_->Init(config));
}

}  // namespace planning
}  // namespace apollo
