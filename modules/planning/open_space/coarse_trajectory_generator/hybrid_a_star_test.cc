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

/*
 * @file
 */

#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

#include "cyber/common/file.h"
#include "gtest/gtest.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

class HybridATest : public ::testing::Test {
 public:
  virtual void SetUp() {
    FLAGS_planner_open_space_config_filename =
        "/apollo/modules/planning/testdata/conf/"
        "open_space_standard_parking_lot.pb.txt";

    ACHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
        << "Failed to load open space config file "
        << FLAGS_planner_open_space_config_filename;

    hybrid_test = std::unique_ptr<HybridAStar>(
        new HybridAStar(planner_open_space_config_));
  }

 protected:
  std::unique_ptr<HybridAStar> hybrid_test;
  PlannerOpenSpaceConfig planner_open_space_config_;
};

TEST_F(HybridATest, test1) {
  double sx = -15.0;
  double sy = 0.0;
  double sphi = 0.0;
  double ex = 15.0;
  double ey = 0.0;
  double ephi = 0.0;
  std::vector<std::vector<Vec2d>> obstacles_list;
  HybridAStartResult result;
  Vec2d obstacle_vertice_a(1.0, 0.0);
  Vec2d obstacle_vertice_b(-1.0, 0.0);
  std::vector<Vec2d> obstacle = {obstacle_vertice_a, obstacle_vertice_b};
  // load xy boundary into the Plan() from configuration(Independent from frame)
  std::vector<double> XYbounds_;
  XYbounds_.push_back(-50.0);
  XYbounds_.push_back(50.0);
  XYbounds_.push_back(-50.0);
  XYbounds_.push_back(50.0);

  obstacles_list.emplace_back(obstacle);
  ASSERT_TRUE(hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_,
                                obstacles_list, &result));
}
}  // namespace planning
}  // namespace apollo
