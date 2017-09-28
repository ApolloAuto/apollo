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
#include "modules/planning/math/smoothing_spline/piecewise_linear_constraint.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;

TEST(TestPiecewiseLinearConstraint, add_monotone_inequality_constraint) {
  PiecewiseLinearConstraint constraint(10, 0.1);
  constraint.AddMonotoneInequalityConstraint();
  const auto mat = constraint.inequality_constraint_matrix();
  const auto bd = constraint.inequality_constraint_boundary();
  std::cout << mat << std::endl;
  std::cout << bd << std::endl;

  MatrixXd mat_golden(10, 10);
  // clang-format off
  mat_golden <<
   1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  -1,  1,  0,  0,  0,  0,  0,  0,  0,  0,
   0, -1,  1,  0,  0,  0,  0,  0,  0,  0,
   0,  0, -1,  1,  0,  0,  0,  0,  0,  0,
   0,  0,  0, -1,  1,  0,  0,  0,  0,  0,
   0,  0,  0,  0, -1,  1,  0,  0,  0,  0,
   0,  0,  0,  0,  0, -1,  1,  0,  0,  0,
   0,  0,  0,  0,  0,  0, -1,  1,  0,  0,
   0,  0,  0,  0,  0,  0,  0, -1,  1,  0,
   0,  0,  0,  0,  0,  0,  0,  0, -1,  1;
  // clang-format on
  EXPECT_EQ(mat, mat_golden);

  MatrixXd bd_golden(10, 1);
  bd_golden << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  EXPECT_EQ(bd, bd_golden);
}

TEST(TestPiecewiseLinearConstraint, add_boundary) {
  PiecewiseLinearConstraint constraint(10, 0.1);
  std::vector<uint32_t> index_list;
  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
  for (uint32_t i = 0; i < 10; ++i) {
    index_list.push_back(i);
    lower_bound.push_back(1.0);
    upper_bound.push_back(100.0);
  }

  constraint.AddBoundary(index_list, lower_bound, upper_bound);
  const auto mat = constraint.inequality_constraint_matrix();
  const auto bd = constraint.inequality_constraint_boundary();
  std::cout << mat << std::endl;
  std::cout << bd << std::endl;

  MatrixXd mat_golden(20, 10);
  // clang-format off
  mat_golden <<
   -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0, -1,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  1,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0, -1,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0, -1,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  1,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0, -1,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  1,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0, -1,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  1,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0, -1,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  1,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0, -1,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  1,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0, -1,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  1,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0, -1,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  1;
  // clang-format on
  EXPECT_EQ(mat, mat_golden);

  MatrixXd bd_golden(20, 1);
  bd_golden << -100, 1, -100, 1, -100, 1, -100, 1, -100, 1, -100, 1, -100, 1,
      -100, 1, -100, 1, -100, 1;
  EXPECT_EQ(bd, bd_golden);
}

TEST(TestPiecewiseLinearConstraint, add_derivative_boundary) {
  PiecewiseLinearConstraint constraint(10, 0.1);
  std::vector<uint32_t> index_list;
  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
  for (uint32_t i = 0; i < 10; ++i) {
    index_list.push_back(i);
    lower_bound.push_back(1.0);
    upper_bound.push_back(100.0);
  }

  constraint.AddDerivativeBoundary(index_list, lower_bound, upper_bound);
  const auto mat = constraint.inequality_constraint_matrix();
  const auto bd = constraint.inequality_constraint_boundary();
  std::cout << mat << std::endl;
  std::cout << bd << std::endl;

  MatrixXd mat_golden(20, 10);
  // clang-format off
  mat_golden <<
    -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     1, -1,  0,  0,  0,  0,  0,  0,  0,  0,
    -1,  1,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  1, -1,  0,  0,  0,  0,  0,  0,  0,
     0, -1,  1,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  1, -1,  0,  0,  0,  0,  0,  0,
     0,  0, -1,  1,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  1, -1,  0,  0,  0,  0,  0,
     0,  0,  0, -1,  1,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  1, -1,  0,  0,  0,  0,
     0,  0,  0,  0, -1,  1,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  1, -1,  0,  0,  0,
     0,  0,  0,  0,  0, -1,  1,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  1, -1,  0,  0,
     0,  0,  0,  0,  0,  0, -1,  1,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  1, -1,  0,
     0,  0,  0,  0,  0,  0,  0, -1,  1,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  1, -1,
     0,  0,  0,  0,  0,  0,  0,  0, -1,  1;
  // clang-format on
  EXPECT_EQ(mat, mat_golden);

  MatrixXd bd_golden(20, 1);
  bd_golden << -10, 0.1, -10, 0.1, -10, 0.1, -10, 0.1, -10, 0.1, -10, 0.1, -10,
      0.1, -10, 0.1, -10, 0.1, -10, 0.1;

  EXPECT_EQ(bd, bd_golden);
}

}  // namespace planning
}  // namespace apollo
