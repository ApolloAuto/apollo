/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_interface_base/task_base/optimizers/road_graph/trajectory_cost.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(AllTrajectoryTests, GetCostFromObsSL) {
  // left nudge
  TrajectoryCost tc;
  SLBoundary obs_sl_boundary;
  obs_sl_boundary.set_start_s(20.0);
  obs_sl_boundary.set_end_s(25.0);
  obs_sl_boundary.set_start_l(-1.5);
  obs_sl_boundary.set_end_l(-0.2);
  auto cost = tc.GetCostFromObsSL(5.0, 3.5, obs_sl_boundary);
  EXPECT_FLOAT_EQ(cost.safety_cost, 0.0);
  EXPECT_FLOAT_EQ(cost.smoothness_cost, 0.0);
  EXPECT_FALSE(cost.cost_items.at(0));
  EXPECT_FALSE(cost.cost_items.at(1));
  EXPECT_FALSE(cost.cost_items.at(2));

  // collisioned obstacle
  TrajectoryCost tc1;
  SLBoundary obs_sl_boundary1;
  obs_sl_boundary1.set_start_s(20.0);
  obs_sl_boundary1.set_end_s(25.0);
  obs_sl_boundary1.set_start_l(-1.5);
  obs_sl_boundary1.set_end_l(-0.2);
  auto cost1 = tc.GetCostFromObsSL(21.0, -0.5, obs_sl_boundary1);

  EXPECT_FLOAT_EQ(cost1.safety_cost, 825.6347);
  EXPECT_FLOAT_EQ(cost1.smoothness_cost, 0.0);
  EXPECT_TRUE(cost1.cost_items.at(0));
  EXPECT_FALSE(cost1.cost_items.at(1));
  EXPECT_FALSE(cost1.cost_items.at(2));
}

}  // namespace planning
}  // namespace apollo
