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
#include "modules/planning/math/smoothing_spline/spline_1d_constraint.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(Spline1dConstraint, add_boundary) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 6;
  Spline1dConstraint constraint(x_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<double> lower_bound = {1.0, 1.0, 1.0};
  std::vector<double> upper_bound = {5.0, 5.0, 5.0};

  constraint.AddBoundary(x_coord, lower_bound, upper_bound);
  const auto mat = constraint.inequality_constraint().constraint_matrix();
  const auto boundary =
      constraint.inequality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(6, 6);
  ref_mat <<
       1,        0,        0,        0,        0,        0,
       1,      0.5,     0.25,    0.125,   0.0625,  0.03125,
       1,        1,        1,        1,        1,        1,
      -1,       -0,       -0,       -0,       -0,       -0,
      -1,     -0.5,    -0.25,   -0.125,  -0.0625, -0.03125,
      -1,       -1,       -1,       -1,       -1,       -1;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_DOUBLE_EQ(mat(i, j), ref_mat(i, j));
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(6, 1);
  ref_boundary << 1.0, 1.0, 1.0, -5.0, -5.0, -5.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_DOUBLE_EQ(boundary(i, 0), ref_boundary(i, 0));
  }
}

TEST(Spline1dConstraint, add_derivative_boundary) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 6;
  Spline1dConstraint constraint(x_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<double> lower_bound = {1.0, 1.0, 1.0};
  std::vector<double> upper_bound = {5.0, 5.0, 5.0};

  constraint.AddDerivativeBoundary(x_coord, lower_bound, upper_bound);
  const auto mat = constraint.inequality_constraint().constraint_matrix();
  const auto boundary =
      constraint.inequality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(6, 6);
  ref_mat <<
      0,       1,       0,       0,       0,       0,
      0,       1,       1,    0.75,     0.5,  0.3125,
      0,       1,       2,       3,       4,       5,
      0,      -1,      -0,      -0,      -0,      -0,
      0,      -1,      -1,   -0.75,    -0.5, -0.3125,
      0,      -1,      -2,      -3,      -4,      -5;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_DOUBLE_EQ(mat(i, j), ref_mat(i, j));
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(6, 1);
  ref_boundary << 1.0, 1.0, 1.0, -5.0, -5.0, -5.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_DOUBLE_EQ(boundary(i, 0), ref_boundary(i, 0));
  }
}

TEST(Spline1dConstraint, add_second_derivative_boundary) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 6;
  Spline1dConstraint constraint(x_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<double> lower_bound = {1.0, 1.0, 1.0};
  std::vector<double> upper_bound = {5.0, 5.0, 5.0};

  constraint.AddSecondDerivativeBoundary(x_coord, lower_bound, upper_bound);
  const auto mat = constraint.inequality_constraint().constraint_matrix();
  const auto boundary =
      constraint.inequality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(6, 6);
  ref_mat <<
   0,    0,    2,    0,    0,    0,
   0,    0,    2,    3,    3,  2.5,
   0,    0,    2,    6,   12,   20,
   0,    0,   -2,   -0,   -0,   -0,
   0,    0,   -2,   -3,   -3, -2.5,
   0,    0,   -2,   -6,  -12,  -20;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_DOUBLE_EQ(mat(i, j), ref_mat(i, j));
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(6, 1);
  ref_boundary << 1.0, 1.0, 1.0, -5.0, -5.0, -5.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_DOUBLE_EQ(boundary(i, 0), ref_boundary(i, 0));
  }
}

TEST(Spline1dConstraint, add_smooth_constraint) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 6;
  Spline1dConstraint constraint(x_knots, spline_order);

  constraint.AddSmoothConstraint();
  const auto mat = constraint.equality_constraint().constraint_matrix();
  const auto boundary = constraint.equality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(1, 12);
  ref_mat << 1, 1, 1, 1, 1, 1, -1, -0, -0, -0, -0, -0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_DOUBLE_EQ(mat(i, j), ref_mat(i, j));
    }
  }

  for (int i = 0; i < boundary.rows(); ++i) {
    EXPECT_DOUBLE_EQ(boundary(i, 0), 0.0);
  }
}

}  // namespace planning
}  // namespace apollo
