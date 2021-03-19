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
 * @file: qp_solver.h
 * @brief: quadratic programming base class
 *
 *        min_x  : q(x) = 0.5 * x^T * Q * x  + x^T c
 *        with respect to:  A * x = b (equality constraint)
 *                          C * x >= d (inequality constraint)
 **/

#pragma once

#include "Eigen/Core"
#include "Eigen/LU"

namespace apollo {
namespace common {
namespace math {

class QpSolver {
 public:
  QpSolver(const Eigen::MatrixXd& kernel_matrix, const Eigen::MatrixXd& offset,
           const Eigen::MatrixXd& affine_inequality_matrix,
           const Eigen::MatrixXd& affine_inequality_boundary,
           const Eigen::MatrixXd& affine_equality_matrix,
           const Eigen::MatrixXd& affine_equality_boundary);
  virtual ~QpSolver() = default;

  virtual void set_pos_semi_definite_hessian() {}
  virtual void set_pos_definite_hessian() {}
  virtual void EnableCholeskyRefactorisation(const int) {}
  virtual void SetTerminationTolerance(const double) {}
  virtual bool Solve() = 0;

  const Eigen::MatrixXd& params() const;
  const Eigen::MatrixXd& kernel_matrix() const;
  const Eigen::MatrixXd& offset() const;
  const Eigen::MatrixXd& affine_equality_matrix() const;
  const Eigen::MatrixXd& affine_equality_boundary() const;
  const Eigen::MatrixXd& affine_inequality_matrix() const;
  const Eigen::MatrixXd& affine_inequality_boundary() const;

 protected:
  virtual bool sanity_check() = 0;
  Eigen::MatrixXd params_;
  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd offset_;
  Eigen::MatrixXd affine_inequality_matrix_;
  Eigen::MatrixXd affine_inequality_boundary_;
  Eigen::MatrixXd affine_equality_matrix_;
  Eigen::MatrixXd affine_equality_boundary_;
};

}  // namespace math
}  // namespace common
}  // namespace apollo
