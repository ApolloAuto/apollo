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

#include "modules/planning/tasks/dp_poly_path/comparable_cost.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(ComparableCost, simple) {
  ComparableCost cc;
  EXPECT_DOUBLE_EQ(cc.safety_cost, 0.0);
  EXPECT_DOUBLE_EQ(cc.smoothness_cost, 0.0);
  EXPECT_FALSE(cc.cost_items[ComparableCost::HAS_COLLISION]);
  EXPECT_FALSE(cc.cost_items[ComparableCost::OUT_OF_BOUNDARY]);
  EXPECT_FALSE(cc.cost_items[ComparableCost::OUT_OF_LANE]);
}

TEST(ComparableCost, add_cost) {
  ComparableCost cc1(true, false, false, 10.12, 2.51);
  ComparableCost cc2(false, false, true, 6.1, 3.45);

  ComparableCost cc = cc1 + cc2;

  EXPECT_TRUE(cc.cost_items[ComparableCost::HAS_COLLISION]);
  EXPECT_FALSE(cc.cost_items[ComparableCost::OUT_OF_BOUNDARY]);
  EXPECT_TRUE(cc.cost_items[ComparableCost::OUT_OF_LANE]);
  EXPECT_DOUBLE_EQ(cc.safety_cost, 16.22);
  EXPECT_DOUBLE_EQ(cc.smoothness_cost, 5.96);

  EXPECT_TRUE(cc1 > cc2);

  cc1 += cc2;

  EXPECT_TRUE(cc1.cost_items[ComparableCost::HAS_COLLISION]);
  EXPECT_FALSE(cc1.cost_items[ComparableCost::OUT_OF_BOUNDARY]);
  EXPECT_TRUE(cc1.cost_items[ComparableCost::OUT_OF_LANE]);
  EXPECT_DOUBLE_EQ(cc1.safety_cost, 16.22);
  EXPECT_DOUBLE_EQ(cc1.smoothness_cost, 5.96);

  ComparableCost cc3(true, false, false, 10.12, 2.51);
  ComparableCost cc4(false, true, true, 6.1, 3.45);

  EXPECT_TRUE(cc3 > cc4);

  ComparableCost cc5(false, false, false, 10.12, 2.51);
  ComparableCost cc6(false, true, true, 6.1, 3.45);

  EXPECT_TRUE(cc5 < cc6);

  ComparableCost cc7 = cc5 + cc6;

  EXPECT_FALSE(cc7.cost_items[ComparableCost::HAS_COLLISION]);
  EXPECT_TRUE(cc7.cost_items[ComparableCost::OUT_OF_BOUNDARY]);
  EXPECT_TRUE(cc7.cost_items[ComparableCost::OUT_OF_LANE]);
  EXPECT_DOUBLE_EQ(cc7.safety_cost, 16.22);
  EXPECT_DOUBLE_EQ(cc7.smoothness_cost, 5.96);

  EXPECT_TRUE(cc5 < cc6);
}

}  // namespace planning
}  // namespace apollo
