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

#include "modules/control/control_component/controller_task_base/common/hysteresis_filter.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace control {

TEST(HysteresisFilter, TriangleHysteresis) {
  HysteresisFilter hysteresis_filter;
  std::vector<double> triangle_input(20, 0.0);
  std::vector<double> filter_output(20, 0.0);
  double threshold = 0.0;
  double hysteresis_upper = 1.5;
  double hysteresis_lower = 2.5;
  int state = 0;
  for (int i = 0; i < 10; ++i) {
    triangle_input[i] = i - 5;
  }
  for (int i = 10; i < 20; ++i) {
    triangle_input[i] = -i + 15;
  }

  // First state turning point is threshold + hysteresis_upper
  for (int i = 0; i < 7; ++i) {
    int expected_filter_state = 0;
    hysteresis_filter.filter(triangle_input[i], threshold, hysteresis_upper,
                             hysteresis_lower, &state, &filter_output[i]);
    EXPECT_EQ(state, expected_filter_state);
    if (triangle_input[i] > threshold + hysteresis_upper) {
      EXPECT_EQ(hysteresis_upper + threshold, filter_output[i]);
    } else if (triangle_input[i] < threshold - hysteresis_lower) {
      EXPECT_EQ(threshold - hysteresis_lower, filter_output[i]);
    } else {
      EXPECT_EQ(triangle_input[i], filter_output[i]);
    }
  }

  // Next state turning point is threshold - hysteresis_lower
  for (int i = 7; i < 18; ++i) {
    int expected_filter_state = 1;
    hysteresis_filter.filter(triangle_input[i], threshold, hysteresis_upper,
                             hysteresis_lower, &state, &filter_output[i]);
    EXPECT_EQ(state, expected_filter_state);
    if (triangle_input[i] > threshold + hysteresis_upper) {
      EXPECT_EQ(hysteresis_upper + threshold, filter_output[i]);
    } else if (triangle_input[i] < threshold - hysteresis_lower) {
      EXPECT_EQ(threshold - hysteresis_lower, filter_output[i]);
    } else {
      EXPECT_EQ(triangle_input[i], filter_output[i]);
    }
  }

  // Next state turning point is threshold + hysteresis_upper
  for (int i = 18; i < 20; ++i) {
    int expected_filter_state = 0;
    hysteresis_filter.filter(triangle_input[i], threshold, hysteresis_upper,
                             hysteresis_lower, &state, &filter_output[i]);
    EXPECT_EQ(state, expected_filter_state);
    if (triangle_input[i] > threshold + hysteresis_upper) {
      EXPECT_EQ(hysteresis_upper + threshold, filter_output[i]);
    } else if (triangle_input[i] < threshold - hysteresis_lower) {
      EXPECT_EQ(threshold - hysteresis_lower, filter_output[i]);
    } else {
      EXPECT_EQ(triangle_input[i], filter_output[i]);
    }
  }
}

}  // namespace control
}  // namespace apollo
