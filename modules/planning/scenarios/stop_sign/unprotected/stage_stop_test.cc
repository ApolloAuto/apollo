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

#include "modules/planning/scenarios/stop_sign/unprotected/stage_stop.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using apollo::cyber::common::GetProtoFromFile;

class StopSignUnprotectedStageStopTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    PlanningConfig planning_config;
    CHECK(GetProtoFromFile(FLAGS_planning_config_file, &planning_config))
        << "failed to load planning config file " << FLAGS_planning_config_file;
    TaskFactory::Init(planning_config);
    CHECK(GetProtoFromFile(
        FLAGS_scenario_stop_sign_unprotected_config_file,
        &stop_sign_unprotected_config_))
        << "failed to load stop_sign_unprotected config file "
        << FLAGS_scenario_stop_sign_unprotected_config_file;
  }

 protected:
  ScenarioConfig stop_sign_unprotected_config_;
};

TEST_F(StopSignUnprotectedStageStopTest, Init) {
  StopSignUnprotectedStageStop stop_sign_uprotected_stage_stop(
      stop_sign_unprotected_config_.stage_config(1));
  EXPECT_EQ(stop_sign_uprotected_stage_stop.Name(),
      ScenarioConfig::StageType_Name(
      stop_sign_unprotected_config_.stage_config(1).stage_type()));
}

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
