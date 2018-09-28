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

#include <algorithm>

#include "cybertron/common/log.h"

#include "modules/common/math/qp_solver/qp_solver_gflags.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace {

constexpr double kRoadBound = 1e10;
}

using apollo::common::time::Clock;
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
  std::vector<double> P_data;
  std::vector<int> P_indices;
  std::vector<int> P_indptr;
  ToCSCMatrix(P, &P_data, &P_indices, &P_indptr);

  c_int P_nnz = P_data.size();
  c_float P_x[P_nnz];  // NOLINT
  for (int i = 0; i < P_nnz; ++i) {
    P_x[i] = P_data[i];
  }
  c_int P_i[P_indices.size()];  // NOLINT
  for (size_t i = 0; i < P_indices.size(); ++i) {
    P_i[i] = P_indices[i];
  }
  c_int P_p[P_indptr.size()];  // NOLINT
  for (size_t i = 0; i < P_indptr.size(); ++i) {
    P_p[i] = P_indptr[i];
  }

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

  std::vector<double> A_data;
  std::vector<int> A_indices;
  std::vector<int> A_indptr;
  ToCSCMatrix(A, &A_data, &A_indices, &A_indptr);

  c_int A_nnz = A_data.size();
  c_float A_x[A_nnz];  // NOLINT
  for (int i = 0; i < A_nnz; ++i) {
    A_x[i] = A_data[i];
  }
  c_int A_i[A_indices.size()];  // NOLINT
  for (size_t i = 0; i < A_indices.size(); ++i) {
    A_i[i] = A_indices[i];
  }
  c_int A_p[A_indptr.size()];  // NOLINT
  for (size_t i = 0; i < A_indptr.size(); ++i) {
    A_p[i] = A_indptr[i];
  }

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

  int constraint_num = inequality_constraint_boundary.rows() +
                       equality_constraint_boundary.rows();

  c_float l[constraint_num];  // NOLINT
  for (int i = 0; i < constraint_num; ++i) {
    if (i < inequality_constraint_boundary.rows()) {
      l[i] = inequality_constraint_boundary(i, 0);
    } else {
      l[i] = equality_constraint_boundary(
          i - inequality_constraint_boundary.rows(), 0);
    }
  }

  c_float u[constraint_num];  // NOLINT
  constexpr float kEpsilon = 1e-9;
  constexpr float kUpperLimit = 1e9;
  for (int i = 0; i < constraint_num; ++i) {
    if (i < inequality_constraint_boundary.rows()) {
      u[i] = kUpperLimit;
    } else {
      u[i] = equality_constraint_boundary(
                 i - inequality_constraint_boundary.rows(), 0) +
             kEpsilon;
    }
  }
  // Problem settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  // Structures
  OSQPWorkspace* work;  // Workspace
  OSQPData* data;       // OSQPData

  // Populate data
  data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data->n = P.rows();
  data->m = constraint_num;
  data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
  data->l = l;
  data->u = u;

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.0;  // Change alpha parameter

  // Setup workspace
  work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(work);

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

// extract
const Spline2d& OsqpSpline2dSolver::spline() const { return spline_; }

void OsqpSpline2dSolver::ToCSCMatrix(const MatrixXd& dense_matrix,
                                     std::vector<double>* data,
                                     std::vector<int>* indices,
                                     std::vector<int>* indptr) const {
  constexpr double epsilon = 1e-9;
  int data_count = 0;
  for (int c = 0; c < dense_matrix.cols(); ++c) {
    indptr->emplace_back(data_count);
    for (int r = 0; r < dense_matrix.cols(); ++r) {
      if (std::fabs(dense_matrix(r, c)) < epsilon) {
        continue;
      }
      data->emplace_back(dense_matrix(r, c));
      ++data_count;
      indices->emplace_back(r);
    }
  }
  indptr->emplace_back(data_count);
}

}  // namespace planning
}  // namespace apollo
