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
 * @file: active_set_qp_solver.h
 * @brief: wrapper class for active set method in qpOases
 **/

#pragma once

#include "qpOASES.hpp"

#include "modules/common/math/qp_solver/qp_solver.h"

namespace apollo {
namespace common {
namespace math {

class ActiveSetQpSolver : public QpSolver {
 public:
  ActiveSetQpSolver(const Eigen::MatrixXd& kernel_matrix,
                    const Eigen::MatrixXd& offset,
                    const Eigen::MatrixXd& affine_inequality_matrix,
                    const Eigen::MatrixXd& affine_inequality_boundary,
                    const Eigen::MatrixXd& affine_equality_matrix,
                    const Eigen::MatrixXd& affine_equality_boundary);
  virtual ~ActiveSetQpSolver() = default;

  bool Solve() override;

  void set_qp_eps_num(const double eps);
  void set_qp_eps_den(const double eps);
  void set_qp_eps_iter_ref(const double eps);
  void set_debug_info(const bool enable);
  void set_max_iteration(const int max_iter);

  void set_l_lower_bound(const double l_lower_bound);
  void set_l_upper_bound(const double l_upper_bound);
  void set_constraint_upper_bound(const double la_upper_bound);

  double qp_eps_num() const;
  double qp_eps_den() const;
  double qp_eps_iter_ref() const;
  bool debug_info() const;
  int max_iteration() const;

  double l_lower_bound() const;
  double l_upper_bound() const;
  double constraint_upper_bound() const;

  void set_pos_semi_definite_hessian() override {
    // If hessianType is set to HST_SEMIDEF, the built-in regularisation scheme
    // is switched on at no additional computational cost
    hessian_type_ = ::qpOASES::HST_SEMIDEF;
  }
  void set_pos_definite_hessian() override {
    hessian_type_ = ::qpOASES::HST_POSDEF;
  }

  void EnableCholeskyRefactorisation(const int num) override {
    // Specifies the frequency of full factorisation refactorisations of the
    // projected Hessian: 0 turns them off, 1 uses them at each iteration etc
    cholesky_refactorisation_freq_ = num;
  }

  void SetTerminationTolerance(const double tolerance) override {
    termination_tolerance_ = tolerance;
  }

 private:
  bool sanity_check() override;

 private:
  // equality constraint + inequality constraint
  int num_constraint_ = 0;
  // number of parameters
  int num_param_ = 0;

  double qp_eps_num_ = 0.0;
  double qp_eps_den_ = 0.0;
  double qp_eps_iter_ref_ = 0.0;
  bool debug_info_ = false;

  // parameter search bound
  double l_lower_bound_ = -1e10;
  double l_upper_bound_ = 1e10;

  // constraint search upper bound
  double constraint_upper_bound_ = 1e10;
  int max_iteration_ = 1000;

  ::qpOASES::HessianType hessian_type_ = ::qpOASES::HST_UNKNOWN;
  int cholesky_refactorisation_freq_ = 0;
  double termination_tolerance_ = 1.0e-9;
};

}  // namespace math
}  // namespace common
}  // namespace apollo
