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
    const int CONTROLS = 1;
    const int PREVIEW_HORIZON = 30;
    const int CONTROL_HORIZON = 30;
    const double EPS = 0.01;
    const int MAX_ITER = 100;

    Eigen::MatrixXd A(STATES, STATES);
    A << 1, 0, 1, 0,
         0, 1, 0, 1,
         0, 0, 1, 0,
         0, 0, 0, 1;

    Eigen::MatrixXd B(STATES, CONTROLS);
    B << 0, 
         0,
         1,
         0;

    Eigen::MatrixXd C(STATES, 1);
    C << 0,
         0,
         0,
         0.1;

    Eigen::MatrixXd Q(STATES, STATES);
    Q << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

    Eigen::MatrixXd R(CONTROLS, 1);
    R << 1;

    Eigen::MatrixXd lower_bound(CONTROLS, 1);
    lower_bound << -0.2;

    std::cout << "lower_bound size: " << lower_bound.size() << std::endl;
    Eigen::MatrixXd upper_bound(CONTROLS, 1);
    upper_bound << 0.6;

    Eigen::MatrixXd initial_state(STATES, 1);
    initial_state << 0,
                     0,
                     0,
                     0;

    Eigen::MatrixXd reference_state(STATES, 1);
    reference_state << 200,
                       200,
                       0,
                       0;

    std::vector<Eigen::MatrixXd> reference(PREVIEW_HORIZON, reference_state);

    Eigen::MatrixXd control_matrix(CONTROLS, 1);
    control_matrix << 0;

    std::vector<Eigen::MatrixXd> control(CONTROL_HORIZON, control_matrix);
    solve_linear_mpc(A, B, C, Q, R, lower_bound, upper_bound, initial_state, reference, EPS, MAX_ITER, &control);
    for (int i = 0; i < control.size(); ++i) {
        EXPECT_FLOAT_EQ(0.6, control[0](0));
    }
}

}  // namespace math
}  // namespace common
}  // namespace apollo

