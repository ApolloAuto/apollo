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
  const MatrixXd& kernel_matrix = kernel_.kernel_matrix();
  const MatrixXd& offset = kernel_.offset();
  const MatrixXd& inequality_constraint_matrix =
      constraint_.inequality_constraint().constraint_matrix();
  const MatrixXd& inequality_constraint_boundary =
      constraint_.inequality_constraint().constraint_boundary();
  const MatrixXd& equality_constraint_matrix =
      constraint_.equality_constraint().constraint_matrix();
  const MatrixXd& equality_constraint_boundary =
      constraint_.equality_constraint().constraint_boundary();
  // TODO(All): implement here.
  return true;
}

// extract
const Spline2d& OsqpSpline2dSolver::spline() const { return spline_; }

void OsqpSpline2dSolver::ToCSCMatrix(const MatrixXd& dense_matrix,
                                     std::vector<double>* data,
                                     std::vector<double>* indices,
                                     std::vector<double>* indptr) const {
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
