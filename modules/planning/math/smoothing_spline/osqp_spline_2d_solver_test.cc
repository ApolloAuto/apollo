/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/planning/math/smoothing_spline/osqp_spline_2d_solver.h"

#include "gtest/gtest.h"

#include "modules/planning/math/curve_math.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;
using Eigen::MatrixXd;

TEST(OSQPSolverTest, basic_test) {
  std::vector<double> t_knots{0, 1, 2, 3, 4, 5};
  std::size_t order = 5;
  OsqpSpline2dSolver spline_solver(t_knots, order);
  EXPECT_TRUE(spline_solver.Solve());

  {
    std::vector<double> data;
    std::vector<double> indices;
    std::vector<double> indptr;
    Eigen::MatrixXd dense_matrix(3, 3);
    dense_matrix << 1, 0, 2, 0, 0, 3, 4, 5, 6;
    spline_solver.ToCSCMatrix(dense_matrix, &data, &indices, &indptr);

    std::vector<double> data_golden = {1, 4, 5, 2, 3, 6};
    std::vector<double> indices_golden = {0, 2, 2, 0, 1, 2};
    std::vector<double> indptr_golden = {0, 2, 3, 6};

    EXPECT_EQ(data.size(), data_golden.size());
    EXPECT_EQ(indices.size(), indices_golden.size());
    EXPECT_EQ(indptr.size(), indptr_golden.size());

    for (size_t i = 0; i < data.size(); ++i) {
      EXPECT_DOUBLE_EQ(data[i], data_golden[i]);
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      EXPECT_DOUBLE_EQ(indices[i], indices_golden[i]);
    }
    for (size_t i = 0; i < indptr.size(); ++i) {
      EXPECT_DOUBLE_EQ(indptr[i], indptr_golden[i]);
    }
  }

  {
    std::vector<double> data;
    std::vector<double> indices;
    std::vector<double> indptr;
    Eigen::MatrixXd dense_matrix(2, 2);
    dense_matrix << 4.0, 1.0, 1.0, 2.0;
    spline_solver.ToCSCMatrix(dense_matrix, &data, &indices, &indptr);

    std::vector<double> data_golden = {4.0, 1.0, 1.0, 2.0};
    std::vector<double> indices_golden = {0, 1, 0, 1};
    std::vector<double> indptr_golden = {0, 2, 4};

    EXPECT_EQ(data.size(), data_golden.size());
    EXPECT_EQ(indices.size(), indices_golden.size());
    EXPECT_EQ(indptr.size(), indptr_golden.size());

    for (size_t i = 0; i < data.size(); ++i) {
      EXPECT_DOUBLE_EQ(data[i], data_golden[i]);
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      EXPECT_DOUBLE_EQ(indices[i], indices_golden[i]);
    }
    for (size_t i = 0; i < indptr.size(); ++i) {
      EXPECT_DOUBLE_EQ(indptr[i], indptr_golden[i]);
    }
  }
}

}  // namespace planning
}  // namespace apollo
