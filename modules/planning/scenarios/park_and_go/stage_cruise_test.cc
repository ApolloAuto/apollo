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
 * @file stage_cruise_test.cc
 */

#include "modules/planning/scenarios/park_and_go/stage_cruise.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "gtest/gtest.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace park_and_go {

class ParkAndGoStageCruiseTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_stage_type(StageType::PARK_AND_GO_CRUISE);
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  ScenarioConfig::StageConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(ParkAndGoStageCruiseTest, Init) {
  ParkAndGoStageCruise park_and_go_stage_cruise(config_, injector_);
  EXPECT_EQ(park_and_go_stage_cruise.Name(),
            StageType_Name(StageType::PARK_AND_GO_CRUISE));
}

}  // namespace park_and_go
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
