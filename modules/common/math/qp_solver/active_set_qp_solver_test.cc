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
#include "modules/common/math/qp_solver/active_set_qp_solver.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

using Eigen::MatrixXd;

TEST(ActiveSetQpSolver, simple_problem_01) {
  MatrixXd kernel_matrix = MatrixXd::Zero(1, 1);
  kernel_matrix(0, 0) = 1.0;
  MatrixXd offset = MatrixXd::Zero(1, 1);
  offset(0, 0) = -8.0;
  MatrixXd affine_inequality_matrix;
  MatrixXd affine_inequality_boundary;
  MatrixXd affine_equality_matrix;
  MatrixXd affine_equality_boundary;
  ActiveSetQpSolver solver(kernel_matrix, offset, affine_inequality_matrix,
                           affine_inequality_boundary, affine_equality_matrix,
                           affine_equality_boundary);
  solver.Solve();
  EXPECT_NEAR(solver.params()(0, 0), 8.0, 1e-9);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
