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

/**
 * @file
 **/
#include "modules/planning/scenarios/park_and_go/stage_adjust.h"

#include "gtest/gtest.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace park_and_go {

class ParkAndGoStageAdjustTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    PlanningConfig config;
    TaskFactory::Init(config);
    apollo::cyber::common::GetProtoFromFile(
        FLAGS_scenario_park_and_go_config_file, &park_and_go_config_);
  }

 protected:
  ScenarioConfig park_and_go_config_;
};

TEST_F(ParkAndGoStageAdjustTest, Init) {
  ParkAndGoStageAdjust park_and_go_stage_adjust(
      park_and_go_config_.stage_config(1));
  EXPECT_EQ(park_and_go_stage_adjust.stage_type(),
            ScenarioConfig::PARK_AND_GO_ADJUST);
}

}  // namespace park_and_go
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
