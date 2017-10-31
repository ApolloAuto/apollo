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

#include <algorithm>

#include "Eigen/Core"

#include "modules/common/log.h"
#include "modules/common/math/qp_solver/active_set_qp_solver.h"
#include "modules/common/math/qp_solver/qp_solver_gflags.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;
using Eigen::MatrixXd;

Spline2dSolver::Spline2dSolver(const std::vector<double>& t_knots,
                               const uint32_t order)
    : spline_(t_knots, order),
      kernel_(t_knots, order),
      constraint_(t_knots, order) {}

void Spline2dSolver::Reset(const std::vector<double>& t_knots,
                           const uint32_t order) {
  spline_ = Spline2d(t_knots, order);
  kernel_ = Spline2dKernel(t_knots, order);
  constraint_ = Spline2dConstraint(t_knots, order);
}

// customize setup
Spline2dConstraint* Spline2dSolver::mutable_constraint() {
  return &constraint_;
}

Spline2dKernel* Spline2dSolver::mutable_kernel() { return &kernel_; }

Spline2d* Spline2dSolver::mutable_spline() { return &spline_; }

bool Spline2dSolver::Solve() {
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

  if (kernel_matrix.rows() != kernel_matrix.cols()) {
    AERROR << "kernel_matrix.rows() [" << kernel_matrix.rows()
           << "] and kernel_matrix.cols() [" << kernel_matrix.cols()
           << "] should be identical.";
    return false;
  }

  int num_param = kernel_matrix.rows();
  int num_constraint =
      equality_constraint_matrix.rows() + inequality_constraint_matrix.rows();
  ADEBUG << "num_param: " << num_param
         << ", last_num_param_: " << last_num_param_;
  ADEBUG << "num_constraint: " << num_constraint
         << ", last_num_constraint_: " << last_num_constraint_;

  bool use_hotstart =
      (FLAGS_enable_sqp_solver && sqp_solver_ != nullptr &&
       num_param == last_num_param_ && num_constraint == last_num_constraint_);

  if (!use_hotstart) {
    sqp_solver_.reset(new ::qpOASES::SQProblem(num_param, num_constraint,
                                               ::qpOASES::HST_UNKNOWN));
    ::qpOASES::Options my_options;
    my_options.enableCholeskyRefactorisation = 10;
    my_options.epsNum = FLAGS_default_active_set_eps_num;
    my_options.epsDen = FLAGS_default_active_set_eps_den;
    my_options.epsIterRef = FLAGS_default_active_set_eps_iter_ref;
    sqp_solver_->setOptions(my_options);
    if (!FLAGS_default_enable_active_set_debug_info) {
      sqp_solver_->setPrintLevel(qpOASES::PL_NONE);
    }
  }

  // definition of qpOASESproblem
  const int kNumOfMatrixElements = kernel_matrix.rows() * kernel_matrix.cols();
  double h_matrix[kNumOfMatrixElements];  // NOLINT

  const int kNumOfOffsetRows = offset.rows();
  double g_matrix[kNumOfOffsetRows];  // NOLINT
  int index = 0;

  for (int r = 0; r < kernel_matrix.rows(); ++r) {
    g_matrix[r] = offset(r, 0);
    for (int c = 0; c < kernel_matrix.cols(); ++c) {
      h_matrix[index++] = kernel_matrix(r, c);
    }
  }
  DCHECK_EQ(index, kernel_matrix.rows() * kernel_matrix.cols());

  // search space lower bound and uppper bound
  double lower_bound[num_param];  // NOLINT
  double upper_bound[num_param];  // NOLINT

  const double l_lower_bound_ = -1e10;
  const double l_upper_bound_ = 1e10;
  for (int i = 0; i < num_param; ++i) {
    lower_bound[i] = l_lower_bound_;
    upper_bound[i] = l_upper_bound_;
  }

  // constraint matrix construction
  double affine_constraint_matrix[num_param * num_constraint];  // NOLINT
  double constraint_lower_bound[num_constraint];                // NOLINT
  double constraint_upper_bound[num_constraint];                // NOLINT
  index = 0;

  for (int r = 0; r < equality_constraint_matrix.rows(); ++r) {
    constraint_lower_bound[r] = equality_constraint_boundary(r, 0);
    constraint_upper_bound[r] = equality_constraint_boundary(r, 0);

    for (int c = 0; c < num_param; ++c) {
      affine_constraint_matrix[index++] = equality_constraint_matrix(r, c);
    }
  }

  DCHECK_EQ(index, equality_constraint_matrix.rows() * num_param);

  const double constraint_upper_bound_ = 1e10;
  for (int r = 0; r < inequality_constraint_matrix.rows(); ++r) {
    constraint_lower_bound[r + equality_constraint_boundary.rows()] =
        inequality_constraint_boundary(r, 0);
    constraint_upper_bound[r + equality_constraint_boundary.rows()] =
        constraint_upper_bound_;

    for (int c = 0; c < num_param; ++c) {
      affine_constraint_matrix[index++] = inequality_constraint_matrix(r, c);
    }
  }
  DCHECK_EQ(index, equality_constraint_matrix.rows() * num_param +
                       inequality_constraint_boundary.rows() * num_param);

  // initialize problem
  int max_iteration_ = 1000;
  int max_iter = std::max(max_iteration_, num_constraint);

  ::qpOASES::returnValue ret;
  const double start_timestamp = Clock::NowInSecond();
  if (use_hotstart) {
    ADEBUG << "Spline2dSolver is using SQP hotstart.";
    ret = sqp_solver_->hotstart(
        h_matrix, g_matrix, affine_constraint_matrix, lower_bound, upper_bound,
        constraint_lower_bound, constraint_upper_bound, max_iter);
  } else {
    ADEBUG << "Spline2dSolver is NOT using SQP hotstart.";
    ret = sqp_solver_->init(h_matrix, g_matrix, affine_constraint_matrix,
                            lower_bound, upper_bound, constraint_lower_bound,
                            constraint_upper_bound, max_iter);
  }
  const double end_timestamp = Clock::NowInSecond();
  ADEBUG << "Spline2dSolver QP time: "
         << (end_timestamp - start_timestamp) * 1000 << " ms.";

  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      AERROR << "qpOASES solver failed due to reached max iteration";
    } else {
      AERROR << "qpOASES solver failed due to infeasibility or other internal "
                "reasons:"
             << ret;
    }
    return false;
  }

  double result[num_param];  // NOLINT
  sqp_solver_->getPrimalSolution(result);

  MatrixXd solved_params = MatrixXd::Zero(num_param, 1);
  for (int i = 0; i < num_param; ++i) {
    solved_params(i, 0) = result[i];
  }

  const uint32_t spline_order = spline_.spline_order();

  last_num_param_ = num_param;
  last_num_constraint_ = num_constraint;
  return spline_.set_splines(solved_params, spline_order);
}

// extract
const Spline2d& Spline2dSolver::spline() const { return spline_; }
}  // namespace planning
}  // namespace apollo
