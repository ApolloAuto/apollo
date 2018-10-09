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
 * @brief Solver for discrete-time linear quadratic problem.
 */

#pragma once

#include "Eigen/Core"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @brief Solver for discrete-time linear quadratic problem.
 * @param A The system dynamic matrix
 * @param B The control matrix
 * @param Q The cost matrix for system state
 * @param R The cost matrix for control output
 * @param tolerance The numerical tolerance for solving
 *        Algebraic Riccati equation (ARE)
 * @param max_num_iteration The maximum iterations for solving ARE
 * @param ptr_K The feedback control matrix (pointer)
 */
void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                     const double tolerance, const uint max_num_iteration,
                     Eigen::MatrixXd *ptr_K);

}  // namespace math
}  // namespace common
}  // namespace apollo
