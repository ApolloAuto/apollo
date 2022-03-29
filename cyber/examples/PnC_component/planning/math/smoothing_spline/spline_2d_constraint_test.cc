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
#include "modules/planning/math/smoothing_spline/spline_2d_constraint.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

TEST(Spline2dConstraint, add_boundary_01) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  std::vector<double> t_coord = {0.0};
  std::vector<double> angle = {0.0};
  std::vector<Vec2d> ref_point;
  ref_point.emplace_back(Vec2d(0.0, 0.0));
  std::vector<double> lateral_bound = {1.0};
  std::vector<double> longitidinal_bound = {2.0};

  constraint.Add2dBoundary(t_coord, angle, ref_point, longitidinal_bound,
                           lateral_bound);
  const auto mat = constraint.inequality_constraint().constraint_matrix();
  const auto boundary =
      constraint.inequality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(4, 8);
  ref_mat <<
     -0, -0,  -0,  -0,            1,  0,  0,  0,
      0,  0,   0,   0,           -1, -0, -0, -0,
      1,  0,   0,   0,  6.12323e-17,  0,  0,  0,
     -1, -0,  -0,  -0, -6.12323e-17, -0, -0, -0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(4, 1);
  ref_boundary << -1.0, -1.0, -2.0, -2.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

// test add boundary with non-zero angle
TEST(Spline2dConstraint, add_boundary_02) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  std::vector<double> t_coord = {0.0};
  std::vector<double> angle = {0.2};
  std::vector<Vec2d> ref_point;
  ref_point.emplace_back(Vec2d(0.0, 0.0));
  std::vector<double> lateral_bound = {1.0};
  std::vector<double> longitidinal_bound = {2.0};

  constraint.Add2dBoundary(t_coord, angle, ref_point, longitidinal_bound,
                           lateral_bound);
  const auto mat = constraint.inequality_constraint().constraint_matrix();
  const auto boundary =
      constraint.inequality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(4, 8);
  ref_mat <<
    -0.198669,    -0,    -0,    -0,  0.980067,     0,     0,     0,
     0.198669,     0,     0,     0, -0.980067,    -0,    -0,    -0,
     0.980067,     0,     0,     0,  0.198669,     0,     0,     0,
    -0.980067,    -0,    -0,    -0, -0.198669,    -0,    -0,    -0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(4, 1);
  ref_boundary << -1.0, -1.0, -2.0, -2.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

// test add boundary with multiple splines
TEST(Spline2dConstraint, add_boundary_03) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  std::vector<double> t_coord = {1.0};
  std::vector<double> angle = {0.2};
  std::vector<Vec2d> ref_point;
  ref_point.emplace_back(Vec2d(0.0, 0.0));
  std::vector<double> lateral_bound = {1.0};
  std::vector<double> longitidinal_bound = {2.0};

  constraint.Add2dBoundary(t_coord, angle, ref_point, longitidinal_bound,
                           lateral_bound);
  const auto mat = constraint.inequality_constraint().constraint_matrix();
  const auto boundary =
      constraint.inequality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(4, 16);
  ref_mat <<
        0,         0,         0,         0,         0,         0,         0,         0, -0.198669,        -0,        -0,        -0,  0.980067,         0,         0,         0, // NOLINT
        0,         0,         0,         0,         0,         0,         0,         0,  0.198669,         0,         0,         0, -0.980067,        -0,        -0,        -0, // NOLINT
        0,         0,         0,         0,         0,         0,         0,         0,  0.980067,         0,         0,         0,  0.198669,         0,         0,         0, // NOLINT
        0,         0,         0,         0,         0,         0,         0,         0, -0.980067,        -0,        -0,        -0, -0.198669,        -0,        -0,        -0; // NOLINT
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(4, 1);
  ref_boundary << -1.0, -1.0, -2.0, -2.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

TEST(Spline2dConstraint, add_boundary_04) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  std::vector<double> t_coord = {0.0};
  std::vector<double> angle = {-M_PI / 2.0};
  std::vector<Vec2d> ref_point;
  ref_point.emplace_back(Vec2d(0.0, 0.0));
  std::vector<double> lateral_bound = {1.0};
  std::vector<double> longitidinal_bound = {2.0};

  constraint.Add2dBoundary(t_coord, angle, ref_point, longitidinal_bound,
                           lateral_bound);
  const auto mat = constraint.inequality_constraint().constraint_matrix();
  const auto boundary =
      constraint.inequality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(4, 8);
  ref_mat <<
      1, -0,  -0,  -0,  0,  0,  0,  0,
     -1,  0,   0,   0, -0, -0, -0, -0,
      0,  0,   0,   0, -1,  0,  0,  0,
     -0, -0,  -0,  -0,  1, -0, -0, -0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(4, 1);
  ref_boundary << -1.0, -1.0, -2.0, -2.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

TEST(Spline2dConstraint, add_point_angle_constraint_01) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  double t_coord = 0.0;
  double angle = 45.0 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);
  const auto mat = constraint.equality_constraint().constraint_matrix();
  const auto boundary = constraint.equality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(1, 8);
  ref_mat <<
    0, -0.707107, -0, -0, 0, 0.707107, 0, 0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(1, 1);
  ref_boundary << 0.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

TEST(Spline2dConstraint, add_point_angle_constraint_02) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  double t_coord = 0.0;
  double angle = 0.0 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);
  const auto mat = constraint.equality_constraint().constraint_matrix();
  const auto boundary = constraint.equality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(1, 8);
  ref_mat <<
    0, 0, -0, -0, 0, 1, 0, 0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(1, 1);
  ref_boundary << 0.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

TEST(Spline2dConstraint, add_point_angle_constraint_03) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  double t_coord = 0.0;
  double angle = 60.0 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);
  const auto mat = constraint.equality_constraint().constraint_matrix();
  const auto boundary = constraint.equality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(1, 8);
  ref_mat <<
    0, -0.86604136228561401, -0, -0, 0, 0.49997231364250183, 0, 0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(1, 1);
  ref_boundary << 0.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

TEST(Spline2dConstraint, add_point_angle_constraint_04) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  double t_coord = 0.0;
  double angle = 90.0 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);
  const auto mat = constraint.equality_constraint().constraint_matrix();
  const auto boundary = constraint.equality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(1, 8);
  ref_mat <<
    0, -1, -0, -0, 0, 0.0, 0, 0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(1, 1);
  ref_boundary << 0.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

TEST(Spline2dConstraint, add_point_angle_constraint_05) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  double t_coord = 0.0;
  double angle = 135.0 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);
  const auto mat = constraint.equality_constraint().constraint_matrix();
  const auto boundary = constraint.equality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(1, 8);
  ref_mat <<
    0, -0.707107, -0, -0, 0, -0.707107, 0, 0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(1, 1);
  ref_boundary << 0.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

TEST(Spline2dConstraint, add_point_angle_constraint_06) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  double t_coord = 1.0;
  double angle = 60.0 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);
  const auto mat = constraint.equality_constraint().constraint_matrix();
  const auto boundary = constraint.equality_constraint().constraint_boundary();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(1, 8);
  ref_mat << 0, -0.866025, -1.73205, -2.59808, 0, 0.5, 1, 1.5;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  Eigen::MatrixXd ref_boundary = Eigen::MatrixXd::Zero(1, 1);
  ref_boundary << 0.0;

  for (int i = 0; i < ref_boundary.rows(); ++i) {
    EXPECT_NEAR(boundary(i, 0), ref_boundary(i, 0), 1e-4);
  }
}

}  // namespace planning
}  // namespace apollo
