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
 * @file : spline_1d_generator.cc
 * @brief: piecewise_smoothing_spline (pss) generator class
 *           solve pss by qp algorithm, include adding constraint, adding
 *kernel, and solver solve
 **/

#include "modules/planning/math/smoothing_spline/spline_1d_generator.h"

#include "modules/common/math/qp_solver/active_set_qp_solver.h"

namespace apollo {
namespace planning {

Spline1dGenerator::Spline1dGenerator(const std::vector<double>& x_knots,
                                     const std::uint32_t spline_order)
    : spline_(x_knots, spline_order),
      spline_constraint_(x_knots, spline_order),
      spline_kernel_(x_knots, spline_order) {}

Spline1dConstraint* Spline1dGenerator::mutable_spline_constraint() {
  return &spline_constraint_;
}

Spline1dKernel* Spline1dGenerator::mutable_spline_kernel() {
  return &spline_kernel_;
}

void Spline1dGenerator::SetupInitQpPoint(const Eigen::MatrixXd& x,
                                         const Eigen::MatrixXd& y,
                                         const Eigen::MatrixXd& z,
                                         const Eigen::MatrixXd& s) {
  init_x_ = x;
  init_y_ = y;
  init_z_ = z;
  init_s_ = s;
}

bool Spline1dGenerator::Solve() {
  const Eigen::MatrixXd& kernel_matrix = spline_kernel_.kernel_matrix();
  const Eigen::MatrixXd& offset = spline_kernel_.offset();
  const Eigen::MatrixXd& inequality_constraint_matrix =
      spline_constraint_.inequality_constraint().constraint_matrix();
  const Eigen::MatrixXd& inequality_constraint_boundary =
      spline_constraint_.inequality_constraint().constraint_boundary();
  const Eigen::MatrixXd& equality_constraint_matrix =
      spline_constraint_.equality_constraint().constraint_matrix();
  const Eigen::MatrixXd& equality_constraint_boundary =
      spline_constraint_.equality_constraint().constraint_boundary();

  qp_solver_.reset(new apollo::common::math::ActiveSetQpSolver(
      kernel_matrix, offset, inequality_constraint_matrix,
      inequality_constraint_boundary, equality_constraint_matrix,
      equality_constraint_boundary));

  if (!qp_solver_->Solve()) {
    return false;
  }

  const std::uint32_t spline_order = spline_.spline_order();
  const Eigen::MatrixXd solved_params = qp_solver_->params();
  return spline_.SetSplineSegs(solved_params, spline_order);
}

const Spline1d& Spline1dGenerator::spline() const { return spline_; }

}  // namespace planning
}  // namespace apollo
