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

#include "modules/common/filters/digital_filter.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace common {

class DigitalFilterTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(DigitalFilterTest, SetGet) {
  DigitalFilter digital_filter;
  std::vector<double> numerators = {1.0, 2.0, 3.0};
  std::vector<double> denominators = {4.0, 5.0, 6.0};
  digital_filter.set_denominators(denominators);
  digital_filter.set_numerators(numerators);
  std::vector<double> denominators_got = digital_filter.denominators();
  std::vector<double> numerators_got = digital_filter.numerators();
  EXPECT_EQ(numerators_got.size(), numerators.size());
  EXPECT_EQ(denominators_got.size(), denominators.size());
  for (size_t i = 0; i < numerators.size(); ++i) {
    EXPECT_DOUBLE_EQ(numerators_got[i], numerators[i]);
  }
  for (size_t i = 0; i < denominators.size(); ++i) {
    EXPECT_DOUBLE_EQ(denominators_got[i], denominators[i]);
  }
  digital_filter.set_coefficients(denominators, numerators);
  denominators_got.clear();
  denominators_got = digital_filter.denominators();
  numerators_got.clear();
  numerators_got = digital_filter.numerators();
  EXPECT_EQ(numerators_got.size(), numerators.size());
  EXPECT_EQ(denominators_got.size(), denominators.size());
  for (size_t i = 0; i < numerators.size(); ++i) {
    EXPECT_DOUBLE_EQ(numerators_got[i], numerators[i]);
  }
  for (size_t i = 0; i < denominators.size(); ++i) {
    EXPECT_DOUBLE_EQ(denominators_got[i], denominators[i]);
  }

  double dead_zone = 1.0;
  digital_filter.set_dead_zone(dead_zone);
  EXPECT_DOUBLE_EQ(digital_filter.dead_zone(), dead_zone);
}

TEST_F(DigitalFilterTest, FilterOff) {
  std::vector<double> numerators = {0.0, 0.0, 0.0};
  std::vector<double> denominators = {1.0, 0.0, 0.0};
  DigitalFilter digital_filter(denominators, numerators);

  const std::vector<double> step_input(100, 1.0);
  std::vector<double> rand_input(100, 1.0);

  unsigned int seed;
  for (size_t i = 0; i < rand_input.size(); ++i) {
    rand_input[i] = rand_r(&seed);
  }
  // Check setp input
  for (size_t i = 0; i < step_input.size(); ++i) {
    EXPECT_DOUBLE_EQ(digital_filter.Filter(step_input[i]), 0.0);
  }
  // Check random input
  for (size_t i = 0; i < rand_input.size(); ++i) {
    EXPECT_DOUBLE_EQ(digital_filter.Filter(rand_input[i]), 0.0);
  }
}

TEST_F(DigitalFilterTest, MovingAverage) {
  std::vector<double> numerators = {0.25, 0.25, 0.25, 0.25};
  std::vector<double> denominators = {1.0, 0.0, 0.0};
  DigitalFilter digital_filter;
  digital_filter.set_numerators(numerators);
  digital_filter.set_denominators(denominators);

  const std::vector<double> step_input(100, 1.0);
  // Check step input, transients.
  for (size_t i = 0; i < 4; ++i) {
    double expected_filter_out = static_cast<double>(i + 1) * 0.25;
    EXPECT_DOUBLE_EQ(digital_filter.Filter(step_input[i]), expected_filter_out);
  }
  // Check step input, steady state
  for (size_t i = 4; i < step_input.size(); ++i) {
    EXPECT_DOUBLE_EQ(digital_filter.Filter(step_input[i]), 1.0);
  }
}

}  // namespace common
}  // namespace apollo
