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
#include "modules/planning/math/smoothing_spline/piecewise_linear_kernel.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;

TEST(TestPiecewiseLinearKernel, add_regularization) {
  PiecewiseLinearKernel kernel(10, 0.1);

  kernel.AddRegularization(0.2);

  const auto mat = kernel.kernel_matrix();
  const auto offset = kernel.offset_matrix();

  std::cout << mat << std::endl;
  std::cout << offset << std::endl;

  MatrixXd mat_golden(10, 10);
  // clang-format off
  mat_golden <<
    0.2,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0, 0.2,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0, 0.2,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0, 0.2,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0, 0.2,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0, 0.2,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0, 0.2,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0, 0.2,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0, 0.2,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0, 0.2;
  // clang-format on
  EXPECT_EQ(mat, mat_golden);

  MatrixXd offset_golden(10, 1);
  offset_golden << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  EXPECT_EQ(offset, offset_golden);

  kernel.AddRegularization(0.3);
  mat_golden = MatrixXd::Identity(10, 10) * 0.5;
  EXPECT_EQ(kernel.kernel_matrix(), mat_golden);
}

}  // namespace planning
}  // namespace apollo
