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

#include "modules/control/filters/digital_filter.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace control {

class DigitalFilterTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(DigitalFilterTest, SetGet) {
  DigitalFilter digital_filter;
  digital_filter.set_coefficients(0.01, 20);
  std::vector<double> denominators_got = digital_filter.denominators();
  std::vector<double> numerators_got = digital_filter.numerators();
  EXPECT_EQ(numerators_got.size(), 3);
  EXPECT_EQ(denominators_got.size(), 3);
  EXPECT_NEAR(numerators_got[0], 0.1729, 0.01);
  EXPECT_NEAR(numerators_got[1], 0.3458, 0.01);
  EXPECT_NEAR(numerators_got[2], 0.1729, 0.01);
  EXPECT_NEAR(denominators_got[0], 1.0, 0.01);
  EXPECT_NEAR(denominators_got[2], 0.2217, 0.01);
  
  double dead_zone = 1.0;
  digital_filter.set_dead_zone(dead_zone);
  EXPECT_FLOAT_EQ(digital_filter.dead_zone(), dead_zone);
}

TEST_F(DigitalFilterTest, FilterOff) {
  DigitalFilter digital_filter(0.1, 0);

  const std::vector<double> step_input(100, 1.0);
  std::vector<double> rand_input(100, 1.0);

  unsigned int seed;
  for (size_t i = 0; i < rand_input.size(); ++i) {
    rand_input[i] = rand_r(&seed);
  }
  // Check setp input
  for (size_t i = 0; i < step_input.size(); ++i) {
    EXPECT_FLOAT_EQ(digital_filter.Filter(step_input[i]), 0.0);
  }
  // Check random input
  for (size_t i = 0; i < rand_input.size(); ++i) {
    EXPECT_FLOAT_EQ(digital_filter.Filter(rand_input[i]), 0.0);
  }
}

TEST_F(DigitalFilterTest, MovingAverage) {
  DigitalFilter digital_filter(0.25, 1);

  const std::vector<double> step_input(100, 1.0);
  // Check step input, transients.
  for (size_t i = 0; i < 4; ++i) {
    double expected_filter_out = (i + 1) * 0.25;
    EXPECT_NEAR(digital_filter.Filter(step_input[i]), expected_filter_out, 0.5);
  }
  // Check step input, steady state
  for (size_t i = 4; i < step_input.size(); ++i) {
    EXPECT_NEAR(digital_filter.Filter(step_input[i]), 1.0, 0.1);
  }
}

}  // namespace control
}  // namespace apollo
