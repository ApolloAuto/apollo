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
  const int states = 4;
  int controls = 2;
  const int horizon = 10;
  const double eps = 0.01;
  const int max_iter = 100;

  Eigen::MatrixXd A(states, states);
  A << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::MatrixXd B(states, controls);
  B << 0, 1, 0, 0, 1, 0, 0, 1;

  Eigen::MatrixXd C(states, 1);
  C << 0, 0, 0, 0.1;

  Eigen::MatrixXd Q(states, states);
  Q << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  Eigen::MatrixXd R(controls, controls);
  R << 1, 0, 0, 1;

  Eigen::MatrixXd lower_bound(controls, 1);
  lower_bound << -10, -10;

  Eigen::MatrixXd upper_bound(controls, 1);
  upper_bound << 10, 10;

  Eigen::MatrixXd initial_state(states, 1);
  initial_state << 0, 0, 0, 0;

  Eigen::MatrixXd reference_state(states, 1);
  reference_state << 200, 200, 0, 0;

  std::vector<Eigen::MatrixXd> reference(horizon, reference_state);

  Eigen::MatrixXd control_matrix(controls, 1);
  control_matrix << 0, 0;
  std::vector<Eigen::MatrixXd> control(horizon, control_matrix);

  Eigen::MatrixXd control_gain_matrix(controls, states);
  control_gain_matrix << 0, 0, 0, 0, 0, 0, 0, 0;
  std::vector<Eigen::MatrixXd> control_gain(horizon, control_gain_matrix);

  Eigen::MatrixXd addition_gain_matrix(controls, 1);
  addition_gain_matrix << 0, 0;
  std::vector<Eigen::MatrixXd> addition_gain(horizon, addition_gain_matrix);

  for (unsigned int i = 0; i < control.size(); ++i) {
    for (unsigned int i = 1; i < control.size(); ++i) {
      control[i - 1] = control[i];
    }
    control[horizon - 1] = control_matrix;
    SolveLinearMPC(A, B, C, Q, R, lower_bound, upper_bound, initial_state,
                   reference, eps, max_iter, &control, &control_gain,
                   &addition_gain);
    EXPECT_FLOAT_EQ(upper_bound(0), control[0](0));
  }
  controls = 1;

  Eigen::MatrixXd B1(states, controls);
  B1 << 0, 0, 1, 0;

  Eigen::MatrixXd R1(controls, controls);
  R1 << 1;

  Eigen::MatrixXd lower_bound1(controls, 1);
  lower_bound1 << -5;

  Eigen::MatrixXd upper_bound1(controls, 1);
  upper_bound1 << 5;

  Eigen::MatrixXd initial_state1(states, 1);
  initial_state1 << 30, 30, 0, 0;

  Eigen::MatrixXd reference_state1(states, 1);
  reference_state1 << 0, 0, 0, 0;

  std::vector<Eigen::MatrixXd> reference1(horizon, reference_state1);

  Eigen::MatrixXd control_matrix1(controls, 1);
  control_matrix1 << 0;
  std::vector<Eigen::MatrixXd> control1(horizon, control_matrix1);

  Eigen::MatrixXd control_gain_matrix1(controls, states);
  control_gain_matrix1 << 0, 0, 0, 0;
  std::vector<Eigen::MatrixXd> control_gain1(horizon, control_gain_matrix1);

  Eigen::MatrixXd addition_gain_matrix1(controls, 1);
  addition_gain_matrix1 << 0;
  std::vector<Eigen::MatrixXd> addition_gain1(horizon, addition_gain_matrix1);

  for (unsigned int i = 0; i < control1.size(); ++i) {
    for (unsigned int i = 1; i < control1.size(); ++i) {
      control1[i - 1] = control1[i];
    }
    SolveLinearMPC(A, B1, C, Q, R1, lower_bound1, upper_bound1, initial_state1,
                   reference1, eps, max_iter, &control1, &control_gain1,
                   &addition_gain1);
    EXPECT_FLOAT_EQ(lower_bound1(0), control1[0](0));
  }

  Eigen::MatrixXd B2(states, controls);
  B2 << 0, 0, 1, 0;

  Eigen::MatrixXd R2(controls, controls);
  R2 << 1;

  Eigen::MatrixXd lower_bound2(controls, 1);
  lower_bound2 << -10;

  Eigen::MatrixXd upper_bound2(controls, 1);
  upper_bound2 << 10;

  Eigen::MatrixXd initial_state2(states, 1);
  initial_state2 << 30, 30, 0, 0;

  Eigen::MatrixXd reference_state2(states, 1);
  reference_state2 << 30, 30, 0, 0;

  std::vector<Eigen::MatrixXd> reference2(horizon, reference_state2);

  Eigen::MatrixXd control_matrix2(controls, 1);
  control_matrix2 << 0;
  std::vector<Eigen::MatrixXd> control2(horizon, control_matrix2);

  Eigen::MatrixXd control_gain_matrix2(controls, states);
  control_gain_matrix2 << 0, 0, 0, 0;
  std::vector<Eigen::MatrixXd> control_gain2(horizon, control_gain_matrix2);

  Eigen::MatrixXd addition_gain_matrix2(controls, 1);
  addition_gain_matrix2 << 0;
  std::vector<Eigen::MatrixXd> addition_gain2(horizon, addition_gain_matrix2);

  for (unsigned int i = 0; i < control2.size(); ++i) {
    for (unsigned int i = 1; i < control2.size(); ++i) {
      control2[i - 1] = control2[i];
    }
    SolveLinearMPC(A, B2, C, Q, R2, lower_bound2, upper_bound2, initial_state2,
                   reference2, eps, max_iter, &control2, &control_gain2,
                   &addition_gain2);
    EXPECT_NEAR(0.0, control2[0](0), 1e-7);
  }
}
}  // namespace math
}  // namespace common
}  // namespace apollo
