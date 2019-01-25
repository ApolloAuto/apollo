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
#include <vector>

#include "gtest/gtest.h"
#include "modules/common/util/testdata/simple.pb.h"

#include "modules/common/util/util.h"

namespace apollo {
namespace common {
namespace util {

TEST(Util, MaxElement) {
  EXPECT_EQ(3, MaxElement(std::vector<int>{1, 2, 3}));
  EXPECT_FLOAT_EQ(3.3, MaxElement(std::vector<float>{1.1, 2.2, 3.3}));
}

TEST(Util, MinElement) {
  EXPECT_EQ(1, MinElement(std::vector<int>{1, 2, 3}));
  EXPECT_FLOAT_EQ(1.1, MinElement(std::vector<float>{1.1, 2.2, 3.3}));
}

TEST(Util, IsProtoEqual) {
  test::SimpleRepeatedMessage sim1;
  for (int i = 0; i < 10; ++i) {
    auto* t = sim1.add_message();
    t->set_integer(i);
    t->set_text(std::to_string(i));
  }
  auto sim2 = sim1;
  EXPECT_TRUE(IsProtoEqual(sim1, sim2));
  sim2.mutable_message(0)->set_integer(-1);
  EXPECT_FALSE(IsProtoEqual(sim1, sim2));
}

TEST(Util, DistanceXY) {
  class TestPoint {
   public:
    TestPoint(double x, double y) : x_(x), y_(y) {}
    double x() const { return x_; }
    double y() const { return y_; }

   private:
    double x_ = 0.0;
    double y_ = 0.0;
  };
  EXPECT_DOUBLE_EQ(0.0, DistanceXY(TestPoint(0, 0), TestPoint(0, 0)));
  EXPECT_DOUBLE_EQ(1.0, DistanceXY(TestPoint(1, 0), TestPoint(0, 0)));
  EXPECT_DOUBLE_EQ(1.0, DistanceXY(TestPoint(0, 0), TestPoint(1, 0)));
  EXPECT_DOUBLE_EQ(0.0, DistanceXY(TestPoint(1, 0), TestPoint(1, 0)));
}

TEST(Util, uniform_slice) {
  std::vector<double> result;
  uniform_slice(0.0, 10.0, 3, &result);
  ASSERT_EQ(4, result.size());
  EXPECT_DOUBLE_EQ(0.0, result[0]);
  EXPECT_DOUBLE_EQ(3.3333333333333335, result[1]);
  EXPECT_DOUBLE_EQ(6.666666666666667, result[2]);
  EXPECT_DOUBLE_EQ(10.0, result[3]);

  uniform_slice(0.0, -10.0, 3, &result);
  ASSERT_EQ(4, result.size());
  EXPECT_DOUBLE_EQ(0.0, result[0]);
  EXPECT_DOUBLE_EQ(-3.3333333333333335, result[1]);
  EXPECT_DOUBLE_EQ(-6.666666666666667, result[2]);
  EXPECT_DOUBLE_EQ(-10.0, result[3]);
}

TEST(Util, IsFloatEqual) {
  double d1 = 0.2;
  double d2 = 1 / std::sqrt(5) / std::sqrt(5);
  EXPECT_TRUE(IsFloatEqual(d1, d2));
  double d3 = 0.999999999999999999;
  double d4 = 0.999999999999999998;
  EXPECT_TRUE(IsFloatEqual(d3, d4));
  double d5 = 0.11;
  double d6 = 0.12;
  EXPECT_FALSE(IsFloatEqual(d5, d6));
}

}  // namespace util
}  // namespace common
}  // namespace apollo
