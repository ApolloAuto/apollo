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

#include "modules/planning/scenarios/bare_intersection/unprotected/stage_approach.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace bare_intersection {

class StageApproachTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_stage_type(
        ScenarioConfig::BARE_INTERSECTION_UNPROTECTED_APPROACH);
  }

 protected:
  ScenarioConfig::StageConfig config_;
};

TEST_F(StageApproachTest, Init) {
  BareIntersectionUnprotectedStageApproach
      bare_intersection_unprotected_stage_approach(config_);
  EXPECT_EQ(bare_intersection_unprotected_stage_approach.Name(),
            ScenarioConfig::StageType_Name(
                ScenarioConfig::BARE_INTERSECTION_UNPROTECTED_APPROACH));
}

}  // namespace bare_intersection
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
