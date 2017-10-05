/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/common/math/mpc_solver.h"

#include <algorithm>
#include <iostream>
#include <memory>

#include "modules/common/log.h"
#include "modules/common/math/qp_solver/active_set_qp_solver.h"
#include "modules/common/math/qp_solver/qp_solver.h"

namespace apollo {
namespace common {
namespace math {

using Matrix = Eigen::MatrixXd;

// discrete linear predictive control solver, with control format
// x(i + 1) = A * x(i) + B * u (i) + C
void SolveLinearMPC(Matrix &matrix_a, Matrix &matrix_b, Matrix &matrix_c,
                    Matrix &matrix_q, Matrix &matrix_r, Matrix &matrix_lower,
                    Matrix &matrix_upper, Matrix &matrix_initial_state,
                    std::vector<Matrix> &reference, double eps, int max_iter,
                    std::vector<Matrix> *control) {
  if (matrix_a.rows() != matrix_a.cols() ||
      matrix_b.rows() != matrix_a.rows() ||
      matrix_lower.rows() != matrix_upper.rows()) {
    AERROR << "One or more matrices have incompatible dimensions. Aborting.";
    return;
  }

  unsigned int horizon = reference.size();

  // Update augment reference matrix_t
  Matrix matrix_t = Matrix::Zero(matrix_b.rows() * horizon, 1);
  for (unsigned int j = 0; j < horizon; ++j) {
    matrix_t.block(j * reference[0].size(), 0, reference[0].size(), 1) =
        reference[j];
  }

  AERROR << "pdate augment reference matrix_t done.";
  // Update augment control matrix_v
  Matrix matrix_v = Matrix::Zero((*control)[0].rows() * horizon, 1);
  for (unsigned int j = 0; j < horizon; ++j) {
    matrix_v.block(j * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =
        (*control)[j];
  }

  AERROR << "pdate augment control matrix_v done.";

  std::vector<Matrix> matrix_a_power(horizon);
  matrix_a_power[0] = matrix_a;
  for (unsigned int i = 1; i < matrix_a_power.size(); ++i) {
    matrix_a_power[i] = matrix_a * matrix_a_power[i - 1];
  }

  Matrix matrix_k =
      Matrix::Zero(matrix_b.rows() * horizon,
                   matrix_b.cols() * control->size() * control->at(0).rows());
  for (unsigned int r = 0; r < horizon; ++r) {
    for (unsigned int c = 0; c <= r; ++c) {
      matrix_k.block(r * matrix_b.rows(), c * matrix_b.cols(), matrix_b.rows(),
                     matrix_b.cols()) = matrix_a_power[r - c] * matrix_b;
    }
  }
  // Initialize matrix_k, matrix_m, matrix_t and matrix_v, matrix_qq, matrix_rr,
  // vector of matrix A power
  Matrix matrix_m = Matrix::Zero(matrix_b.rows() * horizon, 1);
  Matrix matrix_qq = Matrix::Zero(matrix_k.rows(), matrix_k.rows());
  Matrix matrix_rr = Matrix::Zero(matrix_k.cols(), matrix_k.cols());
  Matrix matrix_ll = Matrix::Zero(horizon * matrix_lower.rows(), 1);
  Matrix matrix_uu = Matrix::Zero(horizon * matrix_upper.rows(), 1);

  AERROR << "pdate augment multiple matrix done.";
  AERROR << "matrix_a size: ." << matrix_a.rows() << " " << matrix_a.cols();
  AERROR << "matrix_initial_state size: ." << matrix_initial_state.rows() << " "
         << matrix_initial_state.cols();
  AERROR << "matrix_c size: ." << matrix_c.rows() << " " << matrix_c.cols();
  AERROR << "matrix_m size: ." << matrix_m.rows() << " " << matrix_m.cols();
  // Compute matrix_m
  matrix_m.block(0, 0, matrix_a.rows(), 1) =
      matrix_a * matrix_initial_state + matrix_c;
  for (unsigned int i = 1; i < horizon; ++i) {
    matrix_m.block(i * matrix_a.rows(), 0, matrix_a.rows(), 1) =
        matrix_a *
            matrix_m.block((i - 1) * matrix_a.rows(), 0, matrix_a.rows(), 1) +
        matrix_c;
  }
  AERROR << "start ll, uu, qq, rr.";
  AERROR << "matrix_ll size: ." << matrix_ll.rows() << " " << matrix_ll.cols();
  AERROR << "matrix_uu size: ." << matrix_uu.rows() << " " << matrix_uu.cols();
  AERROR << "matrix_qq size: ." << matrix_qq.rows() << " " << matrix_qq.cols();
  AERROR << "matrix_rr size: ." << matrix_rr.rows() << " " << matrix_rr.cols();
  // Compute matrix_ll, matrix_uu, matrix_qq, matrix_rr
  for (unsigned int i = 0; i < horizon; ++i) {
    matrix_ll.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =
        matrix_lower;
    matrix_uu.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =
        matrix_upper;
    matrix_qq.block(i * matrix_q.rows(), i * matrix_q.rows(), matrix_q.rows(),
                    matrix_q.rows()) = matrix_q;
    matrix_rr.block(i * matrix_r.rows(), i * matrix_r.rows(), matrix_r.cols(),
                    matrix_r.cols()) = matrix_r;
  }

  // Update matrix_m1, matrix_m2, convert MPC problem to QP problem done
  Matrix matrix_m1 = matrix_k.transpose() * matrix_qq * matrix_k + matrix_rr;
  Matrix matrix_m2 = matrix_k.transpose() * matrix_qq * (matrix_m - matrix_t);

  // Method 1: QPOASES
  Matrix matrix_inequality_constrain_ll =
      -Matrix::Identity(matrix_ll.rows(), matrix_ll.rows());
  Matrix matrix_inequality_constrain_uu =
      Matrix::Identity(matrix_uu.rows(), matrix_uu.rows());
  Matrix matrix_inequality_constrain =
      Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.rows());
  matrix_inequality_constrain << -matrix_inequality_constrain_ll,
      -matrix_inequality_constrain_uu;
  Matrix matrix_inequality_boundary =
      Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.cols());
  matrix_inequality_boundary << matrix_ll, -matrix_uu;
  Matrix matrix_equality_constrain =
      Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.rows());
  Matrix matrix_equality_boundary =
      Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.cols());

  std::unique_ptr<QpSolver> qp_solver(new ActiveSetQpSolver(
      matrix_m1, matrix_m2, matrix_inequality_constrain,
      matrix_inequality_boundary, matrix_equality_constrain,
      matrix_equality_boundary));

  auto result = qp_solver->Solve();
  if (!result) {
    AWARN << "Linear MPC solver failed";
  }
  matrix_v = qp_solver->params();

  for (unsigned int i = 0; i < horizon; ++i) {
    (*control)[i] =
        matrix_v.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1);
  }
}

}  // namespace math
}  // namespace common
}  // namespace apollo
