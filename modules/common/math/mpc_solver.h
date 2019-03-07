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
 * @file mpc_solver.h
 * @brief Convert mpc problem to qp based problem and solve.
 */

#pragma once

#include <vector>

#include "Eigen/Core"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */

namespace apollo {
namespace common {
namespace math {
/**
 * @brief Solver for discrete-time model predictive control problem.
 * @param matrix_a The system dynamic matrix
 * @param matrix_b The control matrix
 * @param matrix_c The disturbance matrix
 * @param matrix_q The cost matrix for control state
 * @param matrix_lower The lower bound control constrain matrix
 * @param matrix_upper The upper bound control constrain matrix
 * @param matrix_initial_state The initial state matrix
 * @param reference The control reference vector with respect to time
 * @param eps The control convergence tolerance
 * @param max_iter The maximum iterations for solving ARE
 * @param control The feedback control matrix (pointer)
 */
bool SolveLinearMPC(
    const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b,
    const Eigen::MatrixXd &matrix_c, const Eigen::MatrixXd &matrix_q,
    const Eigen::MatrixXd &matrix_r, const Eigen::MatrixXd &matrix_lower,
    const Eigen::MatrixXd &matrix_upper,
    const Eigen::MatrixXd &matrix_initial_state,
    const std::vector<Eigen::MatrixXd> &reference, const double eps,
    const int max_iter, std::vector<Eigen::MatrixXd> *control,
    std::vector<Eigen::MatrixXd> *control_gain,
    std::vector<Eigen::MatrixXd> *addition_gain);

}  // namespace math
}  // namespace common
}  // namespace apollo
