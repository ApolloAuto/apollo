/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/math/mpc_osqp.h"

#include "cyber/common/log.h"
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(MPCOSQPSolverTest, MPCOSQP) {
  const int states = 4;
  int controls = 2;
  const int horizon = 10;
  const int max_iter = 100;
  const double eps = 0.01;

  Eigen::MatrixXd A(states, states);
  A << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::MatrixXd B(states, controls);
  B << 0, 1, 0, 0, 1, 0, 0, 1;

  Eigen::MatrixXd Q(states, states);
  Q << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  Eigen::MatrixXd R(controls, controls);
  R << 1, 0, 0, 1;

  Eigen::MatrixXd lower_bound(controls, 1);
  lower_bound << -5, -5;

  Eigen::MatrixXd upper_bound(controls, 1);
  upper_bound << 5, 5;

  Eigen::MatrixXd initial_state(states, 1);
  initial_state << 20, 0, 0, 0;
  std::vector<double> control_cmd(2, 0);
  MpcOsqp mpc_osqp_solver(A, B, Q, R, lower_bound, upper_bound, initial_state,
                          max_iter, horizon, eps);
  mpc_osqp_solver.Solve(&control_cmd);
  EXPECT_FLOAT_EQ(0, control_cmd[0]);
}

TEST(MPCOSQPSolverTest, NonFullRankMatrix) {
  const int states = 2;
  int controls = 1;
  const int horizon = 2;
  const int max_iter = 100;
  const double eps = 0.01;

  Eigen::MatrixXd A(states, states);
  A << 0, 2, 0, 0;

  Eigen::MatrixXd B(states, controls);
  B << 0, 3;

  Eigen::MatrixXd Q(states, states);
  Q << 1, 0, 0, 0;

  Eigen::MatrixXd R(controls, controls);
  R << 1;

  Eigen::MatrixXd lower_bound(controls, 1);
  lower_bound << -10;

  Eigen::MatrixXd upper_bound(controls, 1);
  upper_bound << 10;

  Eigen::MatrixXd initial_state(states, 1);
  initial_state << 3, 0;

  std::vector<double> control_cmd(2, 0);

  MpcOsqp mpc_osqp_solver(A, B, Q, R, lower_bound, upper_bound, initial_state,
                          max_iter, horizon, eps);
  mpc_osqp_solver.Solve(&control_cmd);
  EXPECT_FLOAT_EQ(0, control_cmd[0]);
}

TEST(MPCOSQPSolverTest, NullMatrix) {
  const int states = 2;
  int controls = 1;
  const int horizon = 2;
  const int max_iter = 100;
  const double eps = 0.01;

  Eigen::MatrixXd A(states, states);
  A << 0, 0, 0, 0;

  Eigen::MatrixXd B(states, controls);
  B << 0, 3;

  Eigen::MatrixXd Q(states, states);
  Q << 1, 0, 0, 0;

  Eigen::MatrixXd R(controls, controls);
  R << 1;

  Eigen::MatrixXd lower_bound(controls, 1);
  lower_bound << -10;

  Eigen::MatrixXd upper_bound(controls, 1);
  upper_bound << 10;

  Eigen::MatrixXd initial_state(states, 1);
  initial_state << 3, 0;

  std::vector<double> control_cmd(2, 0);

  MpcOsqp mpc_osqp_solver(A, B, Q, R, lower_bound, upper_bound, initial_state,
                          max_iter, horizon, eps);
  mpc_osqp_solver.Solve(&control_cmd);
  EXPECT_NEAR(0.0, control_cmd[0], 1e-7);
}
}  // namespace math
}  // namespace common
}  // namespace apollo
