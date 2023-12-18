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

#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"
#include "gtest/gtest.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

class ValetParkingScenarioTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<ValetParkingScenario> scenario_;
};

TEST_F(ValetParkingScenarioTest, Init) {
  ScenarioValetParkingConfig scenario_config;

  auto injector = std::make_shared<DependencyInjector>();
  scenario_.reset(new ValetParkingScenario());
  scenario_->Init(injector, "VALET_PARKING");
  EXPECT_EQ(scenario_->Name(), "VALET_PARKING");
}

}  // namespace planning
}  // namespace apollo
