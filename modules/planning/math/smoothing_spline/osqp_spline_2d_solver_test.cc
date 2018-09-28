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
  // TODO(all): fix the test.
  // EXPECT_TRUE(spline_solver.Solve());

  {
    std::vector<double> data;
    std::vector<int> indices;
    std::vector<int> indptr;
    Eigen::MatrixXd dense_matrix(3, 3);
    dense_matrix << 1.2, 0, 2.2, 0, 0, 3.1, 4.8, 5.4, 6.01;
    spline_solver.ToCSCMatrix(dense_matrix, &data, &indices, &indptr);

    std::vector<double> data_golden = {1.2, 4.8, 5.4, 2.2, 3.1, 6.01};
    std::vector<int> indices_golden = {0, 2, 2, 0, 1, 2};
    std::vector<int> indptr_golden = {0, 2, 3, 6};

    EXPECT_EQ(data.size(), data_golden.size());
    EXPECT_EQ(indices.size(), indices_golden.size());
    EXPECT_EQ(indptr.size(), indptr_golden.size());

    for (size_t i = 0; i < data.size(); ++i) {
      EXPECT_DOUBLE_EQ(data[i], data_golden[i]);
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      EXPECT_EQ(indices[i], indices_golden[i]);
    }
    for (size_t i = 0; i < indptr.size(); ++i) {
      EXPECT_EQ(indptr[i], indptr_golden[i]);
    }
  }

  {
    std::vector<double> data;
    std::vector<int> indices;
    std::vector<int> indptr;
    Eigen::MatrixXd dense_matrix(2, 2);
    dense_matrix << 4.0, 1.0, 1.0, 2.0;
    spline_solver.ToCSCMatrix(dense_matrix, &data, &indices, &indptr);

    std::vector<double> data_golden = {4.0, 1.0, 1.0, 2.0};
    std::vector<int> indices_golden = {0, 1, 0, 1};
    std::vector<int> indptr_golden = {0, 2, 4};

    EXPECT_EQ(data.size(), data_golden.size());
    EXPECT_EQ(indices.size(), indices_golden.size());
    EXPECT_EQ(indptr.size(), indptr_golden.size());

    for (size_t i = 0; i < data.size(); ++i) {
      EXPECT_DOUBLE_EQ(data[i], data_golden[i]);
    }
    for (size_t i = 0; i < indices.size(); ++i) {
      EXPECT_EQ(indices[i], indices_golden[i]);
    }
    for (size_t i = 0; i < indptr.size(); ++i) {
      EXPECT_EQ(indptr[i], indptr_golden[i]);
    }
  }
}

TEST(OSQPSolverTest, solver_test_01) {
  std::vector<double> t_knots{0, 1, 2, 3, 4, 5};
  std::size_t order = 5;
  OsqpSpline2dSolver spline_solver(t_knots, order);

  Spline2dConstraint* constraint = spline_solver.mutable_constraint();
  Spline2dKernel* kernel = spline_solver.mutable_kernel();

  std::vector<double> et{0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5};
  std::vector<double> bound(11, 0.2);
  std::vector<std::vector<double>> constraint_data{
      {-1.211566924, 434592.7844, 4437011.568},
      {-1.211572116, 434594.6884, 4437006.498},
      {-1.21157766, 434596.5923, 4437001.428},
      {-1.211571616, 434598.4962, 4436996.358},
      {-1.21155227, 434600.4002, 4436991.288},
      {-1.211532017, 434602.3043, 4436986.218},
      {-1.21155775, 434604.2083, 4436981.148},
      {-1.211634014, 434606.1122, 4436976.077},
      {-1.211698593, 434608.0156, 4436971.007},
      {-1.211576177, 434609.9191, 4436965.937},
      {-1.211256197, 434611.8237, 4436960.867}};
  std::vector<double> angle;
  std::vector<Vec2d> ref_point;

  for (std::size_t i = 0; i < 11; ++i) {
    angle.push_back(constraint_data[i][0]);
    Vec2d prev_point(constraint_data[i][1], constraint_data[i][2]);

    Vec2d new_point = prev_point;
    ref_point.emplace_back(new_point.x(), new_point.y());
  }

  EXPECT_TRUE(constraint->Add2dBoundary(et, angle, ref_point, bound, bound));
  EXPECT_TRUE(constraint->AddThirdDerivativeSmoothConstraint());
  kernel->AddThirdOrderDerivativeMatrix(100);
  // kernel->add_second_order_derivative_matrix(100);
  // kernel->add_derivative_kernel_matrix(100);

  kernel->AddRegularization(0.1);
  // constraint->add_point_angle_constraint(0, -1.21);
  // TODO(all): fix the test.
  // EXPECT_TRUE(spline_solver.Solve());
}

}  // namespace planning
}  // namespace apollo
