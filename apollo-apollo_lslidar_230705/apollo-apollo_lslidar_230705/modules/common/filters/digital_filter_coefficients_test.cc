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

#include "modules/common/filters/digital_filter_coefficients.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {

class DigitalFilterCoefficientsTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(DigitalFilterCoefficientsTest, LpfCoefficients) {
  double ts = 0.01;
  double cutoff_freq = 20;
  std::vector<double> den;
  std::vector<double> num;
  LpfCoefficients(ts, cutoff_freq, &den, &num);
  EXPECT_EQ(den.size(), 3);
  EXPECT_EQ(num.size(), 3);
  EXPECT_NEAR(num[0], 0.1729, 0.01);
  EXPECT_NEAR(num[1], 0.3458, 0.01);
  EXPECT_NEAR(num[2], 0.1729, 0.01);
  EXPECT_NEAR(den[0], 1.0, 0.01);
  EXPECT_NEAR(den[2], 0.2217, 0.01);
}

TEST_F(DigitalFilterCoefficientsTest, LpFirstOrderCoefficients) {
  double ts = 0.01;
  double settling_time = 0.005;
  double dead_time = 0.04;
  std::vector<double> den;
  std::vector<double> num;
  LpFirstOrderCoefficients(ts, settling_time, dead_time, &den, &num);
  EXPECT_EQ(den.size(), 2);
  EXPECT_EQ(num.size(), 5);
  EXPECT_NEAR(den[1], -0.13533, 0.01);
  EXPECT_DOUBLE_EQ(num[0], 0.0);
  EXPECT_DOUBLE_EQ(num[1], 0.0);
  EXPECT_NEAR(num[4], 1 - 0.13533, 0.01);
  dead_time = 0.0;
  LpFirstOrderCoefficients(ts, settling_time, dead_time, &den, &num);
  EXPECT_EQ(den.size(), 2);
  EXPECT_EQ(num.size(), 1);
  EXPECT_NEAR(den[1], -0.13533, 0.01);
  EXPECT_NEAR(num[0], 1 - 0.13533, 0.01);
}

}  // namespace common
}  // namespace apollo
