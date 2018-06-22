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

#include "modules/common/filters/mean_filter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {

class MeanFilterTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(MeanFilterTest, WindowSizeOne) {
  MeanFilter mean_filter(1);
  EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 4.0);
}

TEST_F(MeanFilterTest, WindowSizeTwo) {
  MeanFilter mean_filter(2);
  EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 4.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 2.5);
}

TEST_F(MeanFilterTest, OnePositiveNonZero) {
  MeanFilter mean_filter(5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 2.0);
}

TEST_F(MeanFilterTest, OneNegativeNonZero) {
  MeanFilter mean_filter(5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(-2.0), -2.0);
}

TEST_F(MeanFilterTest, TwoPositiveNonZeros) {
  MeanFilter mean_filter(5);
  mean_filter.Update(3.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 3.5);
}

TEST_F(MeanFilterTest, TwoNegativeNonZeros) {
  MeanFilter mean_filter(5);
  mean_filter.Update(-3.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(-4.0), -3.5);
}

TEST_F(MeanFilterTest, OnePositiveOneNegative) {
  MeanFilter mean_filter(5);
  mean_filter.Update(-3.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 0.5);
}

TEST_F(MeanFilterTest, NormalThree) {
  MeanFilter mean_filter(5);
  mean_filter.Update(-3.0);
  mean_filter.Update(4.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 3.0);
}

TEST_F(MeanFilterTest, NormalFullFiveExact) {
  MeanFilter mean_filter(5);
  mean_filter.Update(-1.0);
  mean_filter.Update(0.0);
  mean_filter.Update(1.0);
  mean_filter.Update(2.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 1.0);
}

TEST_F(MeanFilterTest, NormalFullFiveOver) {
  MeanFilter mean_filter(5);
  mean_filter.Update(-1.0);
  mean_filter.Update(0.0);
  mean_filter.Update(1.0);
  mean_filter.Update(2.0);
  mean_filter.Update(3.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 2.0);
}

TEST_F(MeanFilterTest, SameNumber) {
  MeanFilter mean_filter(5);
  for (int i = 0; i < 10; ++i) {
    EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 1.0);
  }
}

TEST_F(MeanFilterTest, LargeNumber) {
  MeanFilter mean_filter(10);
  double ret = 0.0;
  for (int i = 0; i < 100; ++i) {
    double input = static_cast<double>(i);
    ret = mean_filter.Update(input);
  }
  EXPECT_DOUBLE_EQ(ret, 94.5);
}

TEST_F(MeanFilterTest, AlmostScale) {
  MeanFilter mean_filter(3);
  EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 1.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(9.0), 2.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(8.0), 8.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(7.0), 8.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(6.0), 7.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(5.0), 6.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 5.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 4.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 3.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 2.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 2.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 3.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(5.0), 4.0);
}

TEST_F(MeanFilterTest, ToyExample) {
  MeanFilter mean_filter(4);
  EXPECT_DOUBLE_EQ(mean_filter.Update(5.0), 5.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 4.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(8.0), 5.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(9.0), 6.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(7.0), 7.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 7.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 4.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 3.0);
}

TEST_F(MeanFilterTest, GoodMinRemoval) {
  MeanFilter mean_filter(2);
  EXPECT_DOUBLE_EQ(mean_filter.Update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(9.0), 5.0);
  EXPECT_DOUBLE_EQ(mean_filter.Update(8.0), 8.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(7.0), 7.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(6.0), 6.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(5.0), 5.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(4.0), 4.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(3.0), 3.5);
  EXPECT_DOUBLE_EQ(mean_filter.Update(2.0), 2.5);
}

}  // namespace common
}  // namespace apollo
