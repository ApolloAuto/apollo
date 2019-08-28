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
 * @file stage_intersection_cruise_test.cc
 **/

#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/stage_intersection_cruise.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using apollo::cyber::common::GetProtoFromFile;

class TrafficLightUnprotectedRightTurnStageIntersectionCruiseTest :
                                                        public ::testing::Test {
 public:
  virtual void SetUp() {
    PlanningConfig planning_config;
    CHECK(GetProtoFromFile(FLAGS_planning_config_file, &planning_config))
        << "failed to load planning config file " << FLAGS_planning_config_file;
    TaskFactory::Init(planning_config);
    CHECK(GetProtoFromFile(
        FLAGS_scenario_traffic_light_unprotected_right_turn_config_file,
        &traffic_light_unprotected_right_turn_config_))
        << "failed to load traffic_light_unprotected_right_turn config file "
        << FLAGS_scenario_traffic_light_unprotected_right_turn_config_file;
  }

 protected:
  ScenarioConfig traffic_light_unprotected_right_turn_config_;
};

TEST_F(TrafficLightUnprotectedRightTurnStageIntersectionCruiseTest, Init) {
  TrafficLightUnprotectedRightTurnStageIntersectionCruise
      traffic_light_unprotected_right_turn_stage_intersection_cruise(
      traffic_light_unprotected_right_turn_config_.stage_config(2));
  EXPECT_EQ(traffic_light_unprotected_right_turn_stage_intersection_cruise.Name(),
      ScenarioConfig::StageType_Name(
      traffic_light_unprotected_right_turn_config_.stage_config(2)
      .stage_type()));
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
