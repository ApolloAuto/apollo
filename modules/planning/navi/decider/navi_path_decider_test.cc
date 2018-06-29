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

/**
 * @file
 * @brief This file provides several unit tests for the class "NaviPathDecider".
 */

#include "modules/planning/navi/decider/navi_path_decider.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/planning/common/planning_gflags.h"

using apollo::common::TrajectoryPoint;

namespace apollo {
namespace planning {
class NaviPathDeciderTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    AINFO << "NaviPathDeciderTest : SetUpTestCase";
  }
};

TEST_F(NaviPathDeciderTest, Init) {
  NaviPathDecider navi_path_decider;
  PlanningConfig config;
  EXPECT_TRUE(navi_path_decider.Init(config));
}

TEST_F(NaviPathDeciderTest, Execute) {
  NaviPathDecider navi_path_decider;
  PlanningConfig config;
}

TEST_F(NaviPathDeciderTest, SmoothInitY) {
  NaviPathDecider navi_path_decider;
  double smooth_y = navi_path_decider.SmoothInitY(0.002, 0.001);
  EXPECT_DOUBLE_EQ(smooth_y, 0);
}

}  // namespace planning
}  // namespace apollo
