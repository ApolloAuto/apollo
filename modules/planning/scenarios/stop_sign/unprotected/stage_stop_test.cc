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
 * @file stage_stop_test.cc
 **/

#include "modules/planning/scenarios/test/stage_test_base.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stage_stop.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

class StopSignUnprotectedStageStopTest : public StageTestBase {
 public:
  virtual void SetUp() {
    scenario_config_file_ = FLAGS_scenario_stop_sign_unprotected_config_file;
    stage_type_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_STOP;
    StageTestBase::SetUp();
  }
};

TEST_F(StopSignUnprotectedStageStopTest, Init) {
  EXPECT_NE(stage_config_map_.find(stage_type_), stage_config_map_.end());
  StopSignUnprotectedStageStop stage(*stage_config_map_[stage_type_]);
  EXPECT_EQ(stage.stage_type(), stage_type_);
  EXPECT_EQ(stage.Name(), ScenarioConfig::StageType_Name(stage_type_));
}

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
