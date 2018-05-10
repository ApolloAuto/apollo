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

#include "modules/common/math/mpc_solver.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(MPCSolverTest, MPC) {
  const int STATES = 4;
  int CONTROLS = 2;
  const int HORIZON = 10;
  const double EPS = 0.01;
  const int MAX_ITER = 100;

  Eigen::MatrixXd A(STATES, STATES);
  A << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::MatrixXd B(STATES, CONTROLS);
  B << 0, 1, 0, 0, 1, 0, 0, 1;

  Eigen::MatrixXd C(STATES, 1);
  C << 0, 0, 0, 0.1;

  Eigen::MatrixXd Q(STATES, STATES);
  Q << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  Eigen::MatrixXd R(CONTROLS, CONTROLS);
  R << 1, 0, 0, 1;

  Eigen::MatrixXd lower_bound(CONTROLS, 1);
  lower_bound << -10, -10;

  Eigen::MatrixXd upper_bound(CONTROLS, 1);
  upper_bound << 10, 10;

  Eigen::MatrixXd initial_state(STATES, 1);
  initial_state << 0, 0, 0, 0;

  Eigen::MatrixXd reference_state(STATES, 1);
  reference_state << 200, 200, 0, 0;

  std::vector<Eigen::MatrixXd> reference(HORIZON, reference_state);

  Eigen::MatrixXd control_matrix(CONTROLS, 1);
  control_matrix << 0, 0;
  std::vector<Eigen::MatrixXd> control(HORIZON, control_matrix);

  for (unsigned int i = 0; i < control.size(); ++i) {
    for (unsigned int i = 1; i < control.size(); ++i) {
      control[i - 1] = control[i];
    }
    control[HORIZON - 1] = control_matrix;
    SolveLinearMPC(A, B, C, Q, R, lower_bound, upper_bound, initial_state,
                   reference, EPS, MAX_ITER, &control);
    EXPECT_FLOAT_EQ(upper_bound(0), control[0](0));
  }
  CONTROLS = 1;

  Eigen::MatrixXd B1(STATES, CONTROLS);
  B1 << 0, 0, 1, 0;

  Eigen::MatrixXd R1(CONTROLS, CONTROLS);
  R1 << 1;

  Eigen::MatrixXd lower_bound1(CONTROLS, 1);
  lower_bound1 << -5;

  Eigen::MatrixXd upper_bound1(CONTROLS, 1);
  upper_bound1 << 5;

  Eigen::MatrixXd initial_state1(STATES, 1);
  initial_state1 << 30, 30, 0, 0;

  Eigen::MatrixXd reference_state1(STATES, 1);
  reference_state1 << 0, 0, 0, 0;

  std::vector<Eigen::MatrixXd> reference1(HORIZON, reference_state1);

  Eigen::MatrixXd control_matrix1(CONTROLS, 1);
  control_matrix1 << 0;
  std::vector<Eigen::MatrixXd> control1(HORIZON, control_matrix1);
  for (unsigned int i = 0; i < control1.size(); ++i) {
    for (unsigned int i = 1; i < control1.size(); ++i) {
      control1[i - 1] = control1[i];
    }
    SolveLinearMPC(A, B1, C, Q, R1, lower_bound1, upper_bound1, initial_state1,
                   reference1, EPS, MAX_ITER, &control1);
    EXPECT_FLOAT_EQ(lower_bound1(0), control1[0](0));
  }

  Eigen::MatrixXd B2(STATES, CONTROLS);
  B2 << 0, 0, 1, 0;

  Eigen::MatrixXd R2(CONTROLS, CONTROLS);
  R2 << 1;

  Eigen::MatrixXd lower_bound2(CONTROLS, 1);
  lower_bound2 << -10;

  Eigen::MatrixXd upper_bound2(CONTROLS, 1);
  upper_bound2 << 10;

  Eigen::MatrixXd initial_state2(STATES, 1);
  initial_state2 << 30, 30, 0, 0;

  Eigen::MatrixXd reference_state2(STATES, 1);
  reference_state2 << 30, 30, 0, 0;

  std::vector<Eigen::MatrixXd> reference2(HORIZON, reference_state2);

  Eigen::MatrixXd control_matrix2(CONTROLS, 1);
  control_matrix2 << 0;
  std::vector<Eigen::MatrixXd> control2(HORIZON, control_matrix2);

  for (unsigned int i = 0; i < control2.size(); ++i) {
    for (unsigned int i = 1; i < control2.size(); ++i) {
      control2[i - 1] = control2[i];
    }
    SolveLinearMPC(A, B2, C, Q, R2, lower_bound2, upper_bound2, initial_state2,
                   reference2, EPS, MAX_ITER, &control2);
    EXPECT_NEAR(0.0, control2[0](0), 1e-7);
  }
}
}  // namespace math
}  // namespace common
}  // namespace apollo
