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

TEST(TestPiecewiseLinearKernel, add_reference_line_kernel_matrix_01) {
  PiecewiseLinearKernel kernel(10, 0.1);

  std::vector<uint32_t> index_list;
  std::vector<double> pos_list;
  for (int i = 0; i < 10; ++i) {
    index_list.push_back(i);
    pos_list.push_back(i * 2);
  }

  kernel.AddReferenceLineKernelMatrix(index_list, pos_list, 10.0);

  const auto mat = kernel.kernel_matrix();
  const auto offset = kernel.offset_matrix();

  MatrixXd mat_golden = MatrixXd::Identity(10, 10) * 10.0;
  EXPECT_EQ(mat, mat_golden);

  MatrixXd offset_golden(10, 1);
  offset_golden << 0, -40, -80, -120, -160, -200, -240, -280, -320, -360;

  EXPECT_EQ(offset, offset_golden);
}

TEST(TestPiecewiseLinearKernel, add_reference_line_kernel_matrix_02) {
  PiecewiseLinearKernel kernel(10, 0.1);

  std::vector<uint32_t> index_list;
  std::vector<double> pos_list;
  for (int i = 0; i < 8; i += 2) {
    index_list.push_back(i);
    pos_list.push_back(i * 2);
  }

  kernel.AddReferenceLineKernelMatrix(index_list, pos_list, 10.0);

  const auto mat = kernel.kernel_matrix();
  const auto offset = kernel.offset_matrix();

  MatrixXd mat_golden = MatrixXd::Zero(10, 10);
  // clang-format off
  mat_golden <<
     10,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0, 10,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0, 10,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0, 10,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0;
  // clang-format on
  EXPECT_EQ(mat, mat_golden);

  MatrixXd offset_golden(10, 1);
  offset_golden << 0, 0, -80, 0, -160, 0, -240, 0, 0, 0;

  EXPECT_EQ(offset, offset_golden);
}

TEST(TestPiecewiseLinearKernel, add_second_order_derivative_matrix) {
  PiecewiseLinearKernel kernel(10, 0.1);
  const double init_derivative = 5.0;

  kernel.AddSecondOrderDerivativeMatrix(init_derivative, 1.0);

  const auto mat = kernel.kernel_matrix() / (2.0 * 1.0 / std::pow(0.1, 4));
  const auto offset = kernel.offset_matrix();

  MatrixXd mat_golden(10, 10);
  // clang-format off
  mat_golden <<
      6, -4,  1,  0,  0,  0,  0,  0,  0,  0,
     -4,  6, -4,  1,  0,  0,  0,  0,  0,  0,
      1, -4,  6, -4,  1,  0,  0,  0,  0,  0,
      0,  1, -4,  6, -4,  1,  0,  0,  0,  0,
      0,  0,  1, -4,  6, -4,  1,  0,  0,  0,
      0,  0,  0,  1, -4,  6, -4,  1,  0,  0,
      0,  0,  0,  0,  1, -4,  6, -4,  1,  0,
      0,  0,  0,  0,  0,  1, -4,  6, -4,  1,
      0,  0,  0,  0,  0,  0,  1, -4,  5, -2,
      0,  0,  0,  0,  0,  0,  0,  1, -2,  1;
  // clang-format on
  EXPECT_EQ(mat, mat_golden);

  MatrixXd offset_golden = MatrixXd::Zero(10, 1);
  offset_golden(0, 0) = -10000.0;

  for (int i = 0; i < 10; ++i) {
    EXPECT_DOUBLE_EQ(offset(i, 0), offset_golden(i, 0));
  }
}

TEST(TestPiecewiseLinearKernel, add_third_order_derivative_matrix) {
  PiecewiseLinearKernel kernel(10, 0.1);
  const double init_derivative = 5.0;
  const double init_second_derivative = 2.0;

  kernel.AddThirdOrderDerivativeMatrix(init_derivative, init_second_derivative,
                                       1.0);

  const auto mat = kernel.kernel_matrix() / (2.0 * 1.0 / std::pow(0.1, 6));
  const auto offset = kernel.offset_matrix();

  MatrixXd mat_golden(10, 10);
  // clang-format off
  mat_golden <<
     20, -15,   6,  -1,   0,   0,   0,   0,   0,   0,
    -15,  20, -15,   6,  -1,   0,   0,   0,   0,   0,
      6, -15,  20, -15,   6,  -1,   0,   0,   0,   0,
     -1,   6, -15,  20, -15,   6,  -1,   0,   0,   0,
      0,  -1,   6, -15,  20, -15,   6,  -1,   0,   0,
      0,   0,  -1,   6, -15,  20, -15,   6,  -1,   0,
      0,   0,   0,  -1,   6, -15,  20, -15,   6,  -1,
      0,   0,   0,   0,  -1,   6, -15,  19, -12,   3,
      0,   0,   0,   0,   0,  -1,   6, -12,  10,  -3,
      0,   0,   0,   0,   0,   0,  -1,   3,  -3,   1;
  // clang-format on
  EXPECT_EQ(mat, mat_golden);

  MatrixXd offset_golden = MatrixXd::Zero(10, 1);
  offset_golden(0, 0) = -2.0 * 1.0 *
                            (init_derivative + init_second_derivative * 0.1) /
                            std::pow(0.1, 5) -
                        6.0 * 1.0 * init_derivative / std::pow(0.1, 5);
  offset_golden(1, 0) = 2.0 * 1.0 * init_derivative / std::pow(0.1, 5);

  for (int i = 0; i < 10; ++i) {
    EXPECT_DOUBLE_EQ(offset(i, 0), offset_golden(i, 0));
  }
}

}  // namespace planning
}  // namespace apollo
