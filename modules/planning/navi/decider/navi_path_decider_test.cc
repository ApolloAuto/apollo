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

TEST_F(NaviPathDeciderTest, Execute) {}

TEST_F(NaviPathDeciderTest, SmoothInitY) {
  NaviPathDecider navi_path_decider;
  PlanningConfig config;
  double min_shift_y = config.navi_planner_config()
                           .navi_path_decider_config()
                           .default_min_smooth_init_y();
  double max_shift_y = config.navi_planner_config()
                           .navi_path_decider_config()
                           .default_max_smooth_init_y();
  navi_path_decider.Init(config);

  // 1. target_path_init_y too small, can not need shift
  double smooth_y = navi_path_decider.SmoothInitY(0.002, min_shift_y - 0.05);
  EXPECT_DOUBLE_EQ(smooth_y, 0.0);

  // 2.need shift
  // 2.1 adc on the right of reference line and shift bettween [min_shift_y,
  // max_shift_y]
  smooth_y = navi_path_decider.SmoothInitY(0.0, 0.30);
  EXPECT_DOUBLE_EQ(smooth_y, 0.30);

  // 2.need shift
  // 2.2 adc on the right of reference line and shift max_shift_y]
  smooth_y = navi_path_decider.SmoothInitY(0.0, max_shift_y + 0.30);
  EXPECT_DOUBLE_EQ(smooth_y, max_shift_y);

  // 2.3 adc on the left of reference line and shift bettween [min_shift_y,
  // max_shift_y]
  smooth_y = navi_path_decider.SmoothInitY(0.0, -0.30);
  EXPECT_DOUBLE_EQ(smooth_y, -0.30);

  // 2.need shift
  // 2.4 adc on the left of reference line and shift max_shift_y]
  smooth_y = navi_path_decider.SmoothInitY(0.0, -max_shift_y - 0.30);
  EXPECT_DOUBLE_EQ(smooth_y, -max_shift_y);
}

}  // namespace planning
}  // namespace apollo
