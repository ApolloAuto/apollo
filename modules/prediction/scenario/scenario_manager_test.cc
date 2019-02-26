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

#include "modules/prediction/scenario/scenario_manager.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;

class ScenarioManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() { manager_ = ScenarioManager::Instance(); }

 protected:
  ScenarioManager* manager_ = nullptr;
};

TEST_F(ScenarioManagerTest, init) {
  auto scenario_manager = ScenarioManager::Instance();
  const auto& scenario = scenario_manager->scenario();
  EXPECT_EQ(scenario.type(), Scenario::UNKNOWN);
}

TEST_F(ScenarioManagerTest, run) {
  // TODO(kechxu) add unit tests with concrete contents
  ContainerManager::Instance()->RegisterContainers();
  std::unique_ptr<Container> adc_traj_container =
      ContainerManager::Instance()->CreateContainer(
          AdapterConfig::PLANNING_TRAJECTORY);
  std::unique_ptr<Container> pose_container =
      ContainerManager::Instance()->CreateContainer(
          AdapterConfig::LOCALIZATION);
  manager_->Run();
  const auto& scenario = manager_->scenario();
  EXPECT_EQ(scenario.type(), Scenario::UNKNOWN);
}

}  // namespace prediction
}  // namespace apollo
