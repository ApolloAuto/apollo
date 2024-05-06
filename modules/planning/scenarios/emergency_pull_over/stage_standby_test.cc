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

#include "modules/planning/scenarios/emergency_pull_over/stage_standby.h"
#include "gtest/gtest.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace planning {

class StageStandbyTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_name("EMERGENCY_PULL_OVER_STANDBY");
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  StagePipeline config_;
  std::shared_ptr<DependencyInjector> injector_;
  EmergencyPullOverContext context_;
};

TEST_F(StageStandbyTest, Init) {
  EmergencyPullOverStageStandby emergency_pull_over_stage_standby;
  emergency_pull_over_stage_standby.Init(
      config_, injector_, "scenarios/emergency_pull_over/conf", &context_);
  EXPECT_EQ(emergency_pull_over_stage_standby.Name(),
            "EMERGENCY_PULL_OVER_STANDBY");
}

}  // namespace planning
}  // namespace apollo
