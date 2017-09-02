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
#include "modules/planning/math/smoothing_spline/spline_1d_kernel.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(Spline1dKernel, add_reference_line_kernel) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0, 3.0};
  int32_t spline_order = 5;
  Spline1dKernel kernel(x_knots, spline_order);
  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
                            "]");

  std::vector<double> x_coord = {0.0, 1.0, 2.0, 3.0};
  std::vector<double> ref_x = {0.0, 0.5, 0.6, 2.0};
  kernel.add_reference_line_kernel_matrix(x_coord, ref_x, 1.0);

  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(15, 15);
  ref_kernel_matrix << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 1, 1, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 1, 1, 1, 1;

  for (int i = 0; i < kernel.kernel_matrix().rows(); ++i) {
    for (int j = 0; j < kernel.kernel_matrix().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.kernel_matrix()(i, j), ref_kernel_matrix(i, j));
    }
  }

  Eigen::MatrixXd ref_offset = Eigen::MatrixXd::Zero(15, 1);
  ref_offset << 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, -5.2, -4, -4, -4, -4;
  for (int i = 0; i < kernel.offset().rows(); ++i) {
    for (int j = 0; j < kernel.offset().cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.offset()(i, j), ref_offset(i, j));
    }
  }
}
}  // namespace planning
}  // namespace apollo
