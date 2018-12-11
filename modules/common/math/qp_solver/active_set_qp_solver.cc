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
 * @file: active_set_qp_solver.cc
 **/
#include "modules/common/math/qp_solver/active_set_qp_solver.h"

#include <algorithm>
#include <climits>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/math/qp_solver/qp_solver_gflags.h"

namespace apollo {
namespace common {
namespace math {

ActiveSetQpSolver::ActiveSetQpSolver(
    const Eigen::MatrixXd& kernel_matrix, const Eigen::MatrixXd& offset,
    const Eigen::MatrixXd& affine_inequality_matrix,
    const Eigen::MatrixXd& affine_inequality_boundary,
    const Eigen::MatrixXd& affine_equality_matrix,
    const Eigen::MatrixXd& affine_equality_boundary)
    : QpSolver(kernel_matrix, offset, affine_inequality_matrix,
               affine_inequality_boundary, affine_equality_matrix,
               affine_equality_boundary),
      num_constraint_(static_cast<int>(affine_equality_matrix_.rows() +
                                       affine_inequality_matrix_.rows())),
      num_param_(static_cast<int>(kernel_matrix.rows())),
      qp_eps_num_(FLAGS_default_active_set_eps_num),
      qp_eps_den_(FLAGS_default_active_set_eps_den),
      qp_eps_iter_ref_(FLAGS_default_active_set_eps_iter_ref),
      debug_info_(FLAGS_default_enable_active_set_debug_info) {}

bool ActiveSetQpSolver::Solve() {
  ::qpOASES::QProblem qp_problem(num_param_, num_constraint_, hessian_type_);
  ::qpOASES::Options my_options;

  my_options.enableCholeskyRefactorisation = cholesky_refactorisation_freq_;
  if (hessian_type_ == ::qpOASES::HST_POSDEF ||
      hessian_type_ == ::qpOASES::HST_SEMIDEF) {
    my_options.enableRegularisation = ::qpOASES::BT_TRUE;
  }
  my_options.epsNum = qp_eps_num_;
  my_options.epsDen = qp_eps_den_;
  my_options.epsIterRef = qp_eps_iter_ref_;
  my_options.terminationTolerance = termination_tolerance_;
  qp_problem.setOptions(my_options);
  if (!debug_info_) {
    qp_problem.setPrintLevel(qpOASES::PL_NONE);
  }
  if (kernel_matrix_.rows() != kernel_matrix_.cols()) {
    AERROR << "kernel_matrix_.rows() [" << kernel_matrix_.rows()
           << "] and kernel_matrix_.cols() [" << kernel_matrix_.cols()
           << "] should be identical.";
    return false;
  }
  // definition of qpOASESproblem
  const auto kNumOfMatrixElements =
      kernel_matrix_.rows() * kernel_matrix_.cols();
  double h_matrix[kNumOfMatrixElements];  // NOLINT

  const auto kNumOfOffsetRows = offset_.rows();
  double g_matrix[kNumOfOffsetRows];  // NOLINT
  int index = 0;

  for (int r = 0; r < kernel_matrix_.rows(); ++r) {
    g_matrix[r] = offset_(r, 0);
    for (int c = 0; c < kernel_matrix_.cols(); ++c) {
      h_matrix[index++] = kernel_matrix_(r, c);
    }
  }
  DCHECK_EQ(index, kernel_matrix_.rows() * kernel_matrix_.cols());

  // search space lower bound and upper bound
  double lower_bound[num_param_];  // NOLINT
  double upper_bound[num_param_];  // NOLINT

  // TODO(All): change this to a configurable version
  for (int i = 0; i < num_param_; ++i) {
    lower_bound[i] = l_lower_bound_;
    upper_bound[i] = l_upper_bound_;
  }

  // constraint matrix construction
  double affine_constraint_matrix[num_param_ * num_constraint_];  // NOLINT
  double constraint_lower_bound[num_constraint_];                 // NOLINT
  double constraint_upper_bound[num_constraint_];                 // NOLINT
  index = 0;

  for (int r = 0; r < affine_equality_matrix_.rows(); ++r) {
    // TODO(All): change to a configurable version
    constraint_lower_bound[r] = affine_equality_boundary_(r, 0);
    constraint_upper_bound[r] = affine_equality_boundary_(r, 0);

    for (int c = 0; c < num_param_; ++c) {
      affine_constraint_matrix[index++] = affine_equality_matrix_(r, c);
    }
  }

  DCHECK_EQ(index, affine_equality_matrix_.rows() * num_param_);

  for (int r = 0; r < affine_inequality_matrix_.rows(); ++r) {
    constraint_lower_bound[r + affine_equality_boundary_.rows()] =
        affine_inequality_boundary_(r, 0);
    constraint_upper_bound[r + affine_equality_boundary_.rows()] =
        constraint_upper_bound_;

    // TODO(All): change to a configurable version
    for (int c = 0; c < num_param_; ++c) {
      affine_constraint_matrix[index++] = affine_inequality_matrix_(r, c);
    }
  }
  DCHECK_EQ(index, affine_equality_matrix_.rows() * num_param_ +
                       affine_inequality_boundary_.rows() * num_param_);

  // initialize problem
  int max_iter = std::max(max_iteration_, num_constraint_);

  auto ret = qp_problem.init(h_matrix, g_matrix, affine_constraint_matrix,
                             lower_bound, upper_bound, constraint_lower_bound,
                             constraint_upper_bound, max_iter);
  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      AERROR << "qpOASES solver failed due to reached max iteration";
    } else {
      AERROR << "qpOASES solver failed due to infeasibility or other internal "
                "reasons";
    }
    std::stringstream ss;
    ss << "ActiveSetQpSolver inputs: " << std::endl;
    ss << "kernel_matrix:\n" << kernel_matrix_ << std::endl;
    ss << "offset:\n" << offset_ << std::endl;
    ss << "affine_inequality_matrix:\n"
       << affine_inequality_matrix_ << std::endl;
    ss << "affine_inequality_boundary:\n"
       << affine_inequality_boundary_ << std::endl;
    ss << "affine_equality_matrix:\n" << affine_equality_matrix_ << std::endl;
    ss << "affine_equality_boundary:\n"
       << affine_equality_boundary_ << std::endl;

    ADEBUG << ss.str();

    return false;
  }

  double result[num_param_];  // NOLINT
  qp_problem.getPrimalSolution(result);

  params_ = Eigen::MatrixXd::Zero(num_param_, 1);
  for (int i = 0; i < num_param_; ++i) {
    params_(i, 0) = result[i];
  }
  return qp_problem.isSolved() == qpOASES::BT_TRUE;
}

void ActiveSetQpSolver::set_qp_eps_num(const double eps) { qp_eps_num_ = eps; }

void ActiveSetQpSolver::set_qp_eps_den(const double eps) { qp_eps_den_ = eps; }

void ActiveSetQpSolver::set_qp_eps_iter_ref(const double eps) {
  qp_eps_iter_ref_ = eps;
}

void ActiveSetQpSolver::set_debug_info(const bool enable) {
  debug_info_ = enable;
}

void ActiveSetQpSolver::set_l_lower_bound(const double l_lower_bound) {
  l_lower_bound_ = l_lower_bound;
}

void ActiveSetQpSolver::set_l_upper_bound(const double l_upper_bound) {
  l_upper_bound_ = l_upper_bound;
}

void ActiveSetQpSolver::set_constraint_upper_bound(
    const double la_upper_bound) {
  constraint_upper_bound_ = la_upper_bound;
}

void ActiveSetQpSolver::set_max_iteration(const int max_iter) {
  max_iteration_ = max_iter;
}

double ActiveSetQpSolver::qp_eps_num() const { return qp_eps_num_; }

double ActiveSetQpSolver::qp_eps_den() const { return qp_eps_den_; }

double ActiveSetQpSolver::qp_eps_iter_ref() const { return qp_eps_iter_ref_; }

bool ActiveSetQpSolver::debug_info() const { return debug_info_; }

double ActiveSetQpSolver::l_lower_bound() const { return l_lower_bound_; }

double ActiveSetQpSolver::l_upper_bound() const { return l_upper_bound_; }

double ActiveSetQpSolver::constraint_upper_bound() const {
  return constraint_upper_bound_;
}

int ActiveSetQpSolver::max_iteration() const { return max_iteration_; }

// pure virtual
bool ActiveSetQpSolver::sanity_check() {
  return kernel_matrix_.rows() == kernel_matrix_.cols() &&
         kernel_matrix_.rows() == affine_inequality_matrix_.cols() &&
         kernel_matrix_.rows() == affine_equality_matrix_.cols() &&
         affine_equality_matrix_.rows() == affine_equality_boundary_.rows() &&
         affine_inequality_matrix_.rows() == affine_inequality_boundary_.rows();
}

}  // namespace math
}  // namespace common
}  // namespace apollo
