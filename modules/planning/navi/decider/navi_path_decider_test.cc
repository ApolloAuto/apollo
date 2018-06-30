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
  navi_path_decider.Init(config);

  // 1. target_path_init_y too small, can not need shift
  double smooth_y = navi_path_decider.SmoothInitY(0.002, 0.001);
  EXPECT_DOUBLE_EQ(smooth_y, 0.0);

  // 2. need to shift
  // 2.1 last shift neareast to 0.0, and target on the left of adc set a
  // positive init shift value
  navi_path_decider.last_lane_id_to_start_y_.clear();
  smooth_y = navi_path_decider.SmoothInitY(0.0, 0.03);
  EXPECT_DOUBLE_EQ(smooth_y, config.navi_planner_config()
                                 .navi_path_decider_config()
                                 .min_smooth_init_y());
  // 2.2 last shift neareast to 0.0, and target on the right of adc set a
  // negative init shift value
  smooth_y = navi_path_decider.SmoothInitY(0.0, -0.03);
  EXPECT_DOUBLE_EQ(smooth_y, -config.navi_planner_config()
                                  .navi_path_decider_config()
                                  .min_smooth_init_y());
  // 2.3 last shift direction is the same as target position
  // 2.3.1 last shift was not arrived at target position,need add delta shift
  // and current shift in range [min_smooth_init_y, max_smooth_init_y] and still
  // can't arrived at the target
  navi_path_decider.last_lane_id_to_start_y_["last_adc_position_y"] = 2.5;
  navi_path_decider.cur_reference_line_lane_id_ = "last_adc_position_y";
  smooth_y = navi_path_decider.SmoothInitY(2.4, 2.4);
  EXPECT_DOUBLE_EQ(smooth_y, 2.5 - 2.4 +
                                 config.navi_planner_config()
                                     .navi_path_decider_config()
                                     .lateral_shift_delta());
  // 2.3.2 last shift was not arrived at target position,need add delta shift
  // and current shift exceeded range [min_smooth_init_y, max_smooth_init_y] and
  // still can't arrived at the target
  navi_path_decider.last_lane_id_to_start_y_["last_adc_position_y"] = 2.5;
  navi_path_decider.cur_reference_line_lane_id_ = "last_adc_position_y";
  smooth_y = navi_path_decider.SmoothInitY(2.34, 2.3);
  EXPECT_DOUBLE_EQ(smooth_y, config.navi_planner_config()
                                 .navi_path_decider_config()
                                 .max_smooth_init_y());

  // 2.3.4 last shift was not arrived at target position,need add delta shift
  // and current shift exceeded range [min_smooth_init_y, max_smooth_init_y] and
  // still can't arrived at the target
  navi_path_decider.last_lane_id_to_start_y_["last_adc_position_y"] = 2.5;
  navi_path_decider.cur_reference_line_lane_id_ = "last_adc_position_y";
  smooth_y = navi_path_decider.SmoothInitY(2.34, 2.3);
  EXPECT_DOUBLE_EQ(smooth_y, config.navi_planner_config()
                                 .navi_path_decider_config()
                                 .max_smooth_init_y());

  // 2.3.4 last shift was not arrived at target position,need add delta shift
  // and current shift exceeded range [min_smooth_init_y, max_smooth_init_y] but
  // exceeded the target
  navi_path_decider.last_lane_id_to_start_y_["last_adc_position_y"] = 0.5;
  navi_path_decider.cur_reference_line_lane_id_ = "last_adc_position_y";
  smooth_y = navi_path_decider.SmoothInitY(0.32, 0.15);
  EXPECT_DOUBLE_EQ(smooth_y, 0.15);

  // 2.4 last shift direction is opposite to target position
  // The target position is opposite to the last shift direction
}

}  // namespace planning
}  // namespace apollo
