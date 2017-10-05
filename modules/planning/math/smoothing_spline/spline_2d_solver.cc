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
 * @file spline_smoother_solver.cc
 **/

#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"

#include "modules/common/math/qp_solver/active_set_qp_solver.h"

namespace apollo {
namespace planning {

Spline2dSolver::Spline2dSolver(const std::vector<double>& t_knots,
                               const uint32_t order)
    : spline_(t_knots, order),
      kernel_(t_knots, order),
      constraint_(t_knots, order) {}

// customize setup
Spline2dConstraint* Spline2dSolver::mutable_constraint() {
  return &constraint_;
}

Spline2dKernel* Spline2dSolver::mutable_kernel() { return &kernel_; }

Spline2d* Spline2dSolver::mutable_spline() { return &spline_; }

// solve
bool Spline2dSolver::Solve() {
  const Eigen::MatrixXd& kernel_matrix = kernel_.kernel_matrix();
  const Eigen::MatrixXd& offset = kernel_.offset();
  const Eigen::MatrixXd& inequality_constraint_matrix =
      constraint_.inequality_constraint().constraint_matrix();
  const Eigen::MatrixXd& inequality_constraint_boundary =
      constraint_.inequality_constraint().constraint_boundary();
  const Eigen::MatrixXd& equality_constraint_matrix =
      constraint_.equality_constraint().constraint_matrix();
  const Eigen::MatrixXd& equality_constraint_boundary =
      constraint_.equality_constraint().constraint_boundary();

  qp_solver_.reset(new apollo::common::math::ActiveSetQpSolver(
      kernel_matrix, offset, inequality_constraint_matrix,
      inequality_constraint_boundary, equality_constraint_matrix,
      equality_constraint_boundary));

  qp_solver_->EnableCholeskyRefactorisation(1);
  qp_solver_->set_pos_semi_definite_hessian();

  if (!qp_solver_->Solve()) {
    return false;
  }

  const uint32_t spline_order = spline_.spline_order();
  const Eigen::MatrixXd solved_params = qp_solver_->params();
  return spline_.set_splines(solved_params, spline_order);
}

// extract
const Spline2d& Spline2dSolver::spline() const { return spline_; }
}  // namespace planning
}  // namespace apollo
