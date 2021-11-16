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

#include "cyber/common/log.h"
#include "modules/common/math/matrix_operations.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace {
constexpr double kRoadBound = 1e10;
}

using apollo::common::math::DenseToCSCMatrix;
using Eigen::MatrixXd;

OsqpSpline2dSolver::OsqpSpline2dSolver(const std::vector<double>& t_knots,
                                       const uint32_t order)
    : Spline2dSolver(t_knots, order) {}

void OsqpSpline2dSolver::Reset(const std::vector<double>& t_knots,
                               const uint32_t order) {
  spline_ = Spline2d(t_knots, order);
  kernel_ = Spline2dKernel(t_knots, order);
  constraint_ = Spline2dConstraint(t_knots, order);
}

// customize setup
Spline2dConstraint* OsqpSpline2dSolver::mutable_constraint() {
  return &constraint_;
}

Spline2dKernel* OsqpSpline2dSolver::mutable_kernel() { return &kernel_; }

Spline2d* OsqpSpline2dSolver::mutable_spline() { return &spline_; }

bool OsqpSpline2dSolver::Solve() {
  // Namings here are following osqp convention.
  // For details, visit: https://osqp.org/docs/examples/demo.html

  // change P to csc format
  const MatrixXd& P = kernel_.kernel_matrix();
  ADEBUG << "P: " << P.rows() << ", " << P.cols();
  if (P.rows() == 0) {
    return false;
  }

  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  DenseToCSCMatrix(P, &P_data, &P_indices, &P_indptr);

  // change A to csc format
  const MatrixXd& inequality_constraint_matrix =
      constraint_.inequality_constraint().constraint_matrix();
  const MatrixXd& equality_constraint_matrix =
      constraint_.equality_constraint().constraint_matrix();
  MatrixXd A(
      inequality_constraint_matrix.rows() + equality_constraint_matrix.rows(),
      inequality_constraint_matrix.cols());
  A << inequality_constraint_matrix, equality_constraint_matrix;
  ADEBUG << "A: " << A.rows() << ", " << A.cols();
  if (A.rows() == 0) {
    return false;
  }

  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  DenseToCSCMatrix(A, &A_data, &A_indices, &A_indptr);

  // set q, l, u: l < A < u
  const MatrixXd& q_eigen = kernel_.offset();
  c_float q[q_eigen.rows()];  // NOLINT
  for (int i = 0; i < q_eigen.size(); ++i) {
    q[i] = q_eigen(i);
  }

  const MatrixXd& inequality_constraint_boundary =
      constraint_.inequality_constraint().constraint_boundary();
  const MatrixXd& equality_constraint_boundary =
      constraint_.equality_constraint().constraint_boundary();

  auto constraint_num = inequality_constraint_boundary.rows() +
                        equality_constraint_boundary.rows();

  static constexpr float kEpsilon = 1e-9f;
  static constexpr float kUpperLimit = 1e9f;
  c_float l[constraint_num];  // NOLINT
  c_float u[constraint_num];  // NOLINT
  for (int i = 0; i < constraint_num; ++i) {
    if (i < inequality_constraint_boundary.rows()) {
      l[i] = inequality_constraint_boundary(i, 0);
      u[i] = kUpperLimit;
    } else {
      const auto idx = i - inequality_constraint_boundary.rows();
      l[i] = equality_constraint_boundary(idx, 0) - kEpsilon;
      u[i] = equality_constraint_boundary(idx, 0) + kEpsilon;
    }
  }

  // Problem settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  // Populate data
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data->n = P.rows();
  data->m = constraint_num;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = l;
  data->u = u;

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.0;  // Change alpha parameter
  settings->eps_abs = 1.0e-05;
  settings->eps_rel = 1.0e-05;
  settings->max_iter = 5000;
  settings->polish = true;
  settings->verbose = FLAGS_enable_osqp_debug;

  // Setup workspace
  OSQPWorkspace* work = nullptr;
  work = osqp_setup(data, settings);
  // osqp_setup(&work, data, settings);

  // Solve Problem
  osqp_solve(work);

  MatrixXd solved_params = MatrixXd::Zero(P.rows(), 1);
  for (int i = 0; i < P.rows(); ++i) {
    solved_params(i, 0) = work->solution->x[i];
  }

  last_num_param_ = static_cast<int>(P.rows());
  last_num_constraint_ = static_cast<int>(constraint_num);

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return spline_.set_splines(solved_params, spline_.spline_order());
}

// extract
const Spline2d& OsqpSpline2dSolver::spline() const { return spline_; }

}  // namespace planning
}  // namespace apollo
