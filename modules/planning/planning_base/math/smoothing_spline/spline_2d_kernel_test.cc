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

/**
 * @file
 **/
#include "modules/planning/planning_base/math/smoothing_spline/spline_2d_kernel.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

TEST(Spline2dKernel, add_regularization) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0, 3.0};
  int32_t spline_order = 4;
  Spline2dKernel kernel(x_knots, spline_order);

  kernel.AddRegularization(0.2);

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      if (i == j) {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 0.4);
      } else {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 0.0);
      }
    }
  }

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), 0.0);
    }
  }
}

}  // namespace planning
}  // namespace apollo
