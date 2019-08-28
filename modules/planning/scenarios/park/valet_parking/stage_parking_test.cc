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
 * @file stage_parking_test.cc
 **/

#include "modules/planning/scenarios/park/valet_parking/stage_parking.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace valet_parking {

using apollo::cyber::common::GetProtoFromFile;

class StageParkingTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    PlanningConfig planning_config;
    CHECK(GetProtoFromFile(FLAGS_planning_config_file, &planning_config))
        << "failed to load planning config file " << FLAGS_planning_config_file;
    TaskFactory::Init(planning_config);
    CHECK(GetProtoFromFile(
        FLAGS_scenario_valet_parking_config_file, &valet_parking_config_))
        << "failed to load valet_parking config file "
        << FLAGS_scenario_valet_parking_config_file;
  }

 protected:
  ScenarioConfig valet_parking_config_;
};

TEST_F(StageParkingTest, Init) {
  StageParking stage_parking(
      valet_parking_config_.stage_config(1));
  EXPECT_EQ(stage_parking.Name(),
      ScenarioConfig::StageType_Name(
      valet_parking_config_.stage_config(1).stage_type()));
}

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
