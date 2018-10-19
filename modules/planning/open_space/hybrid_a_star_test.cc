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

#include <memory>

#include "gtest/gtest.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/open_space/hybrid_a_star.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

class HybridATest : public ::testing::Test {
 public:
  virtual void SetUp() {
    hybrid_test = std::unique_ptr<HybridAStar>(new HybridAStar());
  }

 protected:
  std::unique_ptr<HybridAStar> hybrid_test;
};

TEST_F(HybridATest, test1) {
  double sx = -15.0;
  double sy = 0.0;
  double sphi = 0.0;
  double ex = 15.0;
  double ey = 0.0;
  double ephi = 0.0;
  ThreadSafeIndexedObstacles obstacles_list;
  Result result;
  Vec2d obstacle_center(0.0, 0.0);
  Box2d obstacle_box(obstacle_center, 0.0, 5.0, 5.0);
  std::unique_ptr<Obstacle> obstacle =
      Obstacle::CreateStaticVirtualObstacles("a box in center",
                                                   obstacle_box);
  obstacles_list.Add(obstacle->Id(), *obstacle);
  ASSERT_TRUE(
      hybrid_test->Plan(sx, sy, sphi, ex, ey, ephi, &obstacles_list, &result));
}
}  // namespace planning
}  // namespace apollo
