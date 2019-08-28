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

#include "modules/planning/scenarios/bare_intersection/unprotected/stage_intersection_cruise.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace bare_intersection {

using apollo::cyber::common::GetProtoFromFile;

class BareIntersectionUnprotectedStageIntersectionCruiseTest :
                                                    public ::testing::Test {
 public:
  virtual void SetUp() {
    PlanningConfig planning_config;
    CHECK(GetProtoFromFile(FLAGS_planning_config_file, &planning_config))
        << "failed to load planning config file " << FLAGS_planning_config_file;
    TaskFactory::Init(planning_config);
    CHECK(GetProtoFromFile(
        FLAGS_scenario_bare_intersection_unprotected_config_file,
        &bare_intersection_unprotected_config_))
        << "failed to load bare_intersection_unprotected config file "
        << FLAGS_scenario_bare_intersection_unprotected_config_file;
  }

 protected:
  ScenarioConfig bare_intersection_unprotected_config_;
};

TEST_F(BareIntersectionUnprotectedStageIntersectionCruiseTest, Init) {
  BareIntersectionUnprotectedStageIntersectionCruise
      bare_intersection_unprotected_stage_intersection_cruise(
      bare_intersection_unprotected_config_.stage_config(1));
  EXPECT_EQ(bare_intersection_unprotected_stage_intersection_cruise.Name(),
      ScenarioConfig::StageType_Name(
      bare_intersection_unprotected_config_.stage_config(1).
      stage_type()));
}

}  // namespace bare_intersection
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
