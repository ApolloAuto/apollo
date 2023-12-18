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

#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/stage_creep.h"

#include "gtest/gtest.h"
#include "modules/planning/planning_interface_base/scenario_base/proto/scenario_pipeline.pb.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/context.h"

namespace apollo {
namespace planning {

class TrafficLightUnprotectedLeftTurnStageCreepTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_name("TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP");
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  StagePipeline config_;
  std::shared_ptr<DependencyInjector> injector_;
  TrafficLightUnprotectedLeftTurnContext context_;
};

TEST_F(TrafficLightUnprotectedLeftTurnStageCreepTest, Init) {
  TrafficLightUnprotectedLeftTurnStageCreep
      traffic_light_unprotected_left_turn_stage_creep;
  traffic_light_unprotected_left_turn_stage_creep.Init(
      config_, injector_, "scenarios/traffic_light_unprotected_left_turn/conf",
      &context_);
  EXPECT_EQ(traffic_light_unprotected_left_turn_stage_creep.Name(),
            ("TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP"));
}

}  // namespace planning
}  // namespace apollo
