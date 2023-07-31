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
#include "modules/planning/planning_base/math/smoothing_spline/osqp_spline_1d_solver.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(OsqpSpline1dSolver, one) {
  // starting point
  std::vector<double> x_knots{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  OsqpSpline1dSolver pg(x_knots, 6);

  auto* spline_constraint = pg.mutable_spline_constraint();
  auto* spline_kernel = pg.mutable_spline_kernel();
  ASSERT_TRUE(spline_constraint != nullptr);
  ASSERT_TRUE(spline_kernel != nullptr);

  std::vector<double> x_coord{0,   0.4, 0.8, 1.2, 1.6, 2,   2.4,
                              2.8, 3.2, 3.6, 4,   4.4, 4.8, 5.2,
                              5.6, 6,   6.4, 6.8, 7.2, 7.6, 8};
  std::vector<double> fx_guide{
      0,       1.8,     3.6,     5.14901, 6.7408,  8.46267, 10.2627,
      12.0627, 13.8627, 15.6627, 17.4627, 19.2627, 21.0627, 22.8627,
      24.6627, 26.4627, 28.2627, 30.0627, 31.8627, 33.6627, 35.4627};
  std::vector<double> lower_bound(x_coord.size(), 0.0);
  std::vector<double> upper_bound(x_coord.size(), 68.4432);
  spline_constraint->AddBoundary(x_coord, lower_bound, upper_bound);
  std::vector<double> speed_lower_bound(x_coord.size(), 0.0);
  std::vector<double> speed_upper_bound(x_coord.size(), 4.5);
  spline_constraint->AddDerivativeBoundary(x_coord, speed_lower_bound,
                                           speed_upper_bound);
  // add jointness smooth constraint, up to jerk level continuous
  spline_constraint->AddThirdDerivativeSmoothConstraint();
  spline_constraint->AddMonotoneInequalityConstraintAtKnots();
  spline_constraint->AddPointConstraint(0.0, 0.0);
  spline_constraint->AddPointDerivativeConstraint(0.0, 4.2194442749023438);
  spline_constraint->AddPointSecondDerivativeConstraint(0.0,
                                                        1.2431812867484089);
  spline_constraint->AddPointSecondDerivativeConstraint(8.0, 0.0);

  // add kernel (optimize kernel);
  // jerk cost

  spline_kernel->AddThirdOrderDerivativeMatrix(1000.0);
  spline_kernel->AddReferenceLineKernelMatrix(x_coord, fx_guide, 0.4);
  spline_kernel->AddRegularization(1.0);

  EXPECT_TRUE(pg.Solve());
  // extract parameters
  auto params = pg.spline();
}

TEST(OsqpSpline1dSolver, two) {
  // starting point
  std::vector<double> x_knots{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  OsqpSpline1dSolver pg(x_knots, 6);

  auto* spline_constraint = pg.mutable_spline_constraint();
  auto* spline_kernel = pg.mutable_spline_kernel();
  ASSERT_TRUE(spline_constraint != nullptr);
  ASSERT_TRUE(spline_kernel != nullptr);

  std::vector<double> x_coord{0,   0.4, 0.8, 1.2, 1.6, 2,   2.4,
                              2.8, 3.2, 3.6, 4,   4.4, 4.8, 5.2,
                              5.6, 6,   6.4, 6.8, 7.2, 7.6, 8};
  std::vector<double> fx_guide{
      0,       1.8,     3.6,     5.14901, 6.7408,  8.46267, 10.2627,
      12.0627, 13.8627, 15.6627, 17.4627, 19.2627, 21.0627, 22.8627,
      24.6627, 26.4627, 28.2627, 30.0627, 31.8627, 33.6627, 35.4627};
  std::vector<double> lower_bound(x_coord.size(), 0.0);
  std::vector<double> upper_bound(x_coord.size(), 68.4432);
  spline_constraint->AddBoundary(x_coord, lower_bound, upper_bound);
  std::vector<double> speed_lower_bound(x_coord.size(), 0.0);
  std::vector<double> speed_upper_bound(x_coord.size(), 4.5);
  spline_constraint->AddDerivativeBoundary(x_coord, speed_lower_bound,
                                           speed_upper_bound);
  // add jointness smooth constraint, up to jerk level continuous
  spline_constraint->AddThirdDerivativeSmoothConstraint();
  spline_constraint->AddMonotoneInequalityConstraintAtKnots();
  spline_constraint->AddPointConstraint(0.0, 0.0);
  spline_constraint->AddPointDerivativeConstraint(0.0, 0);
  spline_constraint->AddPointSecondDerivativeConstraint(0.0, 0.0);
  spline_constraint->AddPointSecondDerivativeConstraint(8.0, 0.0);

  // add kernel (optimize kernel);
  // jerk cost

  spline_kernel->AddThirdOrderDerivativeMatrix(1000.0);
  spline_kernel->AddReferenceLineKernelMatrix(x_coord, fx_guide, 0.4);
  spline_kernel->AddRegularization(1.0);

  EXPECT_TRUE(pg.Solve());
  // extract parameters
  auto params = pg.spline();
}

TEST(OsqpSpline1dSolver, three) {
  std::vector<double> x_knots{0, 1, 2};
  OsqpSpline1dSolver pg(x_knots, 5);
  QuadraticProgrammingProblem qp_proto;
  auto* spline_constraint = pg.mutable_spline_constraint();
  auto* spline_kernel = pg.mutable_spline_kernel();

  spline_constraint->AddThirdDerivativeSmoothConstraint();
  spline_constraint->AddMonotoneInequalityConstraintAtKnots();

  spline_constraint->AddPointConstraint(0.0, 0.0);
  spline_constraint->AddPointDerivativeConstraint(0.0, 0.0);
  spline_constraint->AddPointSecondDerivativeConstraint(0.0, 0.0);

  std::vector<double> x_coord = {0, 0.5};
  std::vector<double> l_bound = {1.8, 2};
  std::vector<double> u_bound = {3, 7};

  spline_constraint->AddBoundary(x_coord, l_bound, u_bound);

  double intercept = 5;
  double slope = 4;

  spline_kernel->AddRegularization(1.0);
  spline_kernel->AddThirdOrderDerivativeMatrix(10);

  std::vector<double> t_knots(21, 0.0);
  std::vector<double> ft_knots(21, 0.0);

  for (size_t i = 0; i < t_knots.size(); ++i) {
    t_knots[i] = static_cast<double>(i) * 0.1;
    ft_knots[i] = t_knots[i] * slope + intercept;
  }

  spline_kernel->AddReferenceLineKernelMatrix(t_knots, ft_knots, 1);
  EXPECT_TRUE(pg.Solve());

  pg.GenerateProblemProto(&qp_proto);

  EXPECT_EQ(qp_proto.param_size(), 12);
  EXPECT_EQ(qp_proto.quadratic_matrix().row_size(), 12);
  EXPECT_EQ(qp_proto.quadratic_matrix().col_size(), 12);
  EXPECT_EQ(qp_proto.equality_matrix().row_size(), 7);
  EXPECT_EQ(qp_proto.inequality_matrix().row_size(), 6);
}

}  // namespace planning
}  // namespace apollo
