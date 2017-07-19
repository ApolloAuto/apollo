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

#ifndef MODULES_CONTROL_COMMON_MPC_SOLVER_H_
#define MODULES_CONTROL_COMMON_MPC_SOLVER_H_

#include <vector>

#include "Eigen/Core"
#include "Eigen/LU"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */

namespace apollo {
namespace common {
namespace math {
/**
 * @brief Solver for discrete-time model predictive control problem.
 * @param A The system dynamic matrix
 * @param B The control matrix
 * @param Q The cost matrix for system state
 * @param R The cost matrix for control output
 * @param tolerance The numerical tolerance for solving
 *        Algebraic Riccati equation (ARE)
 * @param max_num_iteration The maximum iterations for solving ARE
 * @param ptr_K The feedback control matrix (pointer)
 */
void SolveLinearMPC(const Eigen::MatrixXd &matrix_a,
                    const Eigen::MatrixXd &matrix_b,
                    const Eigen::MatrixXd &matrix_c,
                    const Eigen::MatrixXd &matrix_q,
                    const Eigen::MatrixXd &matrix_r,
                    const Eigen::MatrixXd &matrix_lower,
                    const Eigen::MatrixXd &matrix_upper,
                    const Eigen::MatrixXd &_matrix_initial_state,
                    const std::vector<Eigen::MatrixXd> &reference,
                    const double eps,
                    const int max_iter,
                    std::vector<Eigen::MatrixXd> *control);

void SolveQPSMO(const Eigen::MatrixXd& matrix_q,
                const Eigen::MatrixXd& matrix_b,
                const Eigen::MatrixXd& matrix_lower,
                const Eigen::MatrixXd& matrix_upper,
                const double& eps,
                const int& max_iter,
                Eigen::MatrixXd* matrix_alpha);

}  // namespace math
}  // namespace common
}  // namespace apollo

#endif  // MODULES_CONTROL_COMMON_MPC_SOLVER_H_
