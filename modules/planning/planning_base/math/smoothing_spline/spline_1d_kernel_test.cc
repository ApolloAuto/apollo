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
#include "modules/planning/planning_base/math/smoothing_spline/spline_1d_kernel.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(Spline1dKernel, add_regularization) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0, 3.0};
  int32_t spline_order = 3;
  Spline1dKernel kernel(x_knots, spline_order);

  std::vector<double> x_coord = {0.0, 1.0, 2.0, 3.0};
  kernel.AddRegularization(0.2);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(12, 12);
  // clang-format off
  ref_kernel_matrix <<
      0.2,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0, 0.2,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0, 0.2,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0, 0.2,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0, 0.2,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0, 0.2,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0, 0.2,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0, 0.2,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0, 0.2,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0, 0.2,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0.2,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0.2;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j));
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(12, 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_derivative_kernel_matrix_01) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddDerivativeKernelMatrix(1.0);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(6, 6);

  // clang-format off
  ref_kernel_matrix <<
      0,       0,       0,       0,       0,       0,
      0,       1,       1,       1,       1,       1,
      0,       1, 1.33333,     1.5,     1.6, 1.66667,
      0,       1,     1.5,     1.8,       2, 2.14286,
      0,       1,     1.6,       2, 2.28571,     2.5,
      0,       1, 1.66667, 2.14286,     2.5, 2.77778;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(6, 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_derivative_kernel_matrix_02) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddDerivativeKernelMatrix(1.0);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(
      num_params * (x_knots.size() - 1), num_params * (x_knots.size() - 1));

  // clang-format off
  ref_kernel_matrix <<
      0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1,       1,       1,       1,       1,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1, 1.33333,     1.5,     1.6, 1.66667,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1,     1.5,     1.8,       2, 2.14286,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1,     1.6,       2, 2.28571,     2.5,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1, 1.66667, 2.14286,     2.5, 2.77778,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1,       1,       1,       1,       1,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1, 1.33333,     1.5,     1.6, 1.66667,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1,     1.5,     1.8,       2, 2.14286,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1,     1.6,       2, 2.28571,     2.5,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1, 1.66667, 2.14286,     2.5, 2.77778;  // NOLINT
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(kernel.offset().rows(), 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_derivative_kernel_matrix_03) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 0.5};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddDerivativeKernelMatrix(1.0);

  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), 6 * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(6, 6);

  // clang-format off
  ref_kernel_matrix <<
      0,       0,       0,       0,       0,       0,
      0,       1,       1,       1,       1,       1,
      0,       1, 1.33333,     1.5,     1.6, 1.66667,
      0,       1,     1.5,     1.8,       2, 2.14286,
      0,       1,     1.6,       2, 2.28571,     2.5,
      0,       1, 1.66667, 2.14286,     2.5, 2.77778;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      double param = std::pow(0.5, i + j - 1);
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), param * ref_kernel_matrix(i, j),
                  1e-5);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(6, 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_derivative_kernel_matrix_04) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddDerivativeKernelMatrix(1.0);

  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), 4 * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(4, 4);

  // clang-format off
  ref_kernel_matrix <<
      0,       0,       0,       0,
      0,       1,       1,       1,
      0,       1, 1.33333,     1.5,
      0,       1,     1.5,     1.8;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(6, 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_second_derivative_kernel_matrix_01) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 0.5};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddSecondOrderDerivativeMatrix(1.0);

  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), 6 * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix =
      Eigen::MatrixXd::Zero(6 * (x_knots.size() - 1), 6 * (x_knots.size() - 1));

  // clang-format off
  ref_kernel_matrix <<
      0,       0,       0,       0,       0,       0,
      0,       0,       0,       0,       0,       0,
      0,       0,       4,       6,       8,      10,
      0,       0,       6,      12,      18,      24,
      0,       0,       8,      18,    28.8,      40,
      0,       0,      10,      24,      40, 57.1429;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      const double param = std::pow(0.5, std::max(0, i + j - 3));
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), param * ref_kernel_matrix(i, j),
                  1e-5);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(kernel.offset().rows(), 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_second_derivative_kernel_matrix_02) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 0.5, 1.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddSecondOrderDerivativeMatrix(1.0);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(
      num_params * (x_knots.size() - 1), num_params * (x_knots.size() - 1));

  // clang-format off
  ref_kernel_matrix <<
     0, 0,  0,  0,    0,       0, 0, 0,  0,  0,    0,       0,
     0, 0,  0,  0,    0,       0, 0, 0,  0,  0,    0,       0,
     0, 0,  4,  6,    8,      10, 0, 0,  0,  0,    0,       0,
     0, 0,  6, 12,   18,      24, 0, 0,  0,  0,    0,       0,
     0, 0,  8, 18, 28.8,      40, 0, 0,  0,  0,    0,       0,
     0, 0, 10, 24,   40, 57.1429, 0, 0,  0,  0,    0,       0,
     0, 0,  0,  0,    0,       0, 0, 0,  0,  0,    0,       0,
     0, 0,  0,  0,    0,       0, 0, 0,  0,  0,    0,       0,
     0, 0,  0,  0,    0,       0, 0, 0,  4,  6,    8,      10,
     0, 0,  0,  0,    0,       0, 0, 0,  6, 12,   18,      24,
     0, 0,  0,  0,    0,       0, 0, 0,  8, 18, 28.8,      40,
     0, 0,  0,  0,    0,       0, 0, 0, 10, 24,   40, 57.1429;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      const double param = std::pow(0.5, std::max(0, i % 6 + j % 6 - 3));
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), param * ref_kernel_matrix(i, j),
                  1e-6);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(kernel.offset().rows(), 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_third_derivative_kernel_matrix_01) {
  std::vector<double> x_knots = {0.0, 1.5};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddThirdOrderDerivativeMatrix(1.0);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(
      num_params * (x_knots.size() - 1), num_params * (x_knots.size() - 1));

  // clang-format off
  ref_kernel_matrix <<
      0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,
      0,   0,   0,  36,  72, 120,
      0,   0,   0,  72, 192, 360,
      0,   0,   0, 120, 360, 720;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      const double param = std::pow(1.5, std::max(0, i % 6 + j % 6 - 5));
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), param * ref_kernel_matrix(i, j),
                  1e-6);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(kernel.offset().rows(), 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_third_derivative_kernel_matrix_02) {
  std::vector<double> x_knots = {0.0, 1.5, 3.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddThirdOrderDerivativeMatrix(1.0);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix =
      Eigen::MatrixXd::Zero(num_params, num_params);

  // clang-format off
  ref_kernel_matrix <<
      0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,
      0,   0,   0,  36,  72, 120,
      0,   0,   0,  72, 192, 360,
      0,   0,   0, 120, 360, 720;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      if ((i >= 6 && j < 6) || (i < 6 && j >= 6)) {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 0.0);
      } else {
        const double param = std::pow(1.5, std::max(0, i % 6 + j % 6 - 5));
        EXPECT_NEAR(kernel.kernel_matrix()(i, j),
                    param * ref_kernel_matrix(i % 6, j % 6), 1e-6);
      }
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(kernel.offset().rows(), 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_reference_line_kernel_01) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
                            "]");

  std::vector<double> x_coord = {0.0};
  std::vector<double> ref_x = {0.0};
  kernel.AddReferenceLineKernelMatrix(x_coord, ref_x, 1.0);

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      if (i == 0 && j == 0) {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 2.0);
      } else {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 0.0);
      }
    }
  }

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    EXPECT_DOUBLE_EQ(kernel.offset()(i, 0), 0);
  }
}

TEST(Spline1dKernel, add_reference_line_kernel_02) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);

  std::vector<double> x_coord = {0.0};
  std::vector<double> ref_x = {3.0};
  kernel.AddReferenceLineKernelMatrix(x_coord, ref_x, 1.0);

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      if (i == 0 && j == 0) {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 2.0);
      } else {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 0.0);
      }
    }
  }

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    if (i == 0) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, 0), -6.0);
    } else {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, 0), 0);
    }
  }
}

TEST(Spline1dKernel, add_reference_line_kernel_03) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5};
  std::vector<double> ref_x = {0.0, 2.0};
  kernel.AddReferenceLineKernelMatrix(x_coord, ref_x, 1.0);

  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(1, 6);
  double d = 0.5;
  for (int i = 0; i < 6; ++i) {
    res(0, i) = std::pow(d, i);
  }

  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(6, 6);
  ref_kernel_matrix = 2.0 * res.transpose() * res;
  ref_kernel_matrix(0, 0) += 2.0;

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(6, 1);
  ref_offset = -2.0 * 2.0 * res.transpose();

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j));
    }
  }

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    EXPECT_DOUBLE_EQ(kernel.offset()(i, 0), ref_offset(i, 0));
  }
}

TEST(Spline1dKernel, add_reference_line_kernel_04) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);

  std::vector<double> x_coord = {1.5};
  std::vector<double> ref_x = {2.0};
  kernel.AddReferenceLineKernelMatrix(x_coord, ref_x, 1.0);

  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(1, 6);
  double d = 0.5;
  for (int i = 0; i < 6; ++i) {
    res(0, i) = std::pow(d, i);
  }

  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(6, 6);
  ref_kernel_matrix = 2.0 * res.transpose() * res;

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(6, 1);
  ref_offset = -2.0 * 2.0 * res.transpose();

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      if (i < 6 || j < 6) {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), 0.0);
      } else {
        EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j),
                         ref_kernel_matrix(i % 6, j % 6));
      }
    }
  }

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    if (i < 6) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, 0), 0.0);
    } else {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, 0), ref_offset(i % 6, 0));
    }
  }
}

TEST(Spline1dKernel, add_derivative_kernel_matrix_for_spline_k_01) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddDerivativeKernelMatrixForSplineK(0, 1.0);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(
      num_params * (x_knots.size() - 1), num_params * (x_knots.size() - 1));

  // clang-format off
  ref_kernel_matrix <<
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       2,       2,       2,       2,       2,       0,       0,       0,       0,       0,       0, // NOLINT
0,       2, 2.66667,       3,     3.2, 3.33333,       0,       0,       0,       0,       0,       0, // NOLINT
0,       2,       3,     3.6,       4, 4.28571,       0,       0,       0,       0,       0,       0, // NOLINT
0,       2,     3.2,       4, 4.57143,       5,       0,       0,       0,       0,       0,       0, // NOLINT
0,       2, 3.33333, 4.28571,       5, 5.55556,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0; // NOLINT
  // clang-format on
  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(kernel.offset().rows(), 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel, add_derivative_kernel_matrix_for_spline_k_02) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddDerivativeKernelMatrixForSplineK(1, 1.0);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(
      num_params * (x_knots.size() - 1), num_params * (x_knots.size() - 1));

  // clang-format off
  ref_kernel_matrix <<
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       2,       2,       2,       2,       2, // NOLINT
0,       0,       0,       0,       0,       0,       0,       2, 2.66667,       3,     3.2, 3.33333, // NOLINT
0,       0,       0,       0,       0,       0,       0,       2,       3,     3.6,       4, 4.28571, // NOLINT
0,       0,       0,       0,       0,       0,       0,       2,     3.2,       4, 4.57143,       5, // NOLINT
0,       0,       0,       0,       0,       0,       0,       2, 3.33333, 4.28571,       5, 5.55556; // NOLINT

  // clang-format on
  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(kernel.offset().rows(), 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel,
     add_second_order_derivative_kernel_matrix_for_spline_k_01) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddSecondOrderDerivativeMatrixForSplineK(0, 1.0);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(
      num_params * (x_knots.size() - 1), num_params * (x_knots.size() - 1));

  // clang-format off
  ref_kernel_matrix <<
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       8,      12,      16,      20,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,      12,      24,      36,      48,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,      16,      36,    57.6,      80,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,      20,      48,      80, 114.285714,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0; // NOLINT

  // clang-format on
  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(kernel.offset().rows(), 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}

TEST(Spline1dKernel,
     add_second_order_derivative_kernel_matrix_for_spline_k_02) {
  // please see the document at docs/specs/qp_spline_path_optimizer.md for
  // details.
  std::vector<double> x_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  kernel.AddSecondOrderDerivativeMatrixForSplineK(1, 1.0);

  const uint32_t num_params = spline_order + 1;
  EXPECT_EQ(kernel.kernel_matrix().rows(), kernel.kernel_matrix().cols());
  EXPECT_EQ(kernel.kernel_matrix().rows(), num_params * (x_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(
      num_params * (x_knots.size() - 1), num_params * (x_knots.size() - 1));

  // clang-format off
  ref_kernel_matrix <<
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,       8,      12,      16,      20, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,      12,      24,      36,      48, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,      16,      36,    57.6,      80, // NOLINT
0,       0,       0,       0,       0,       0,       0,       0,      20,      48,      80, 114.285714; // NOLINT

  // clang-format on
  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_NEAR(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(kernel.offset().rows(), 1);

  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}
}  // namespace planning
}  // namespace apollo
