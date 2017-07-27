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
 * @file: qp_solver.cc
 **/
#include "modules/common/math/qp_solver/qp_solver.h"

namespace apollo {
namespace common {
namespace math {

QpSolver::QpSolver(const Eigen::MatrixXd& kernel_matrix,
                   const Eigen::MatrixXd& offset,
                   const Eigen::MatrixXd& affine_inequality_matrix,
                   const Eigen::MatrixXd& affine_inequality_boundary,
                   const Eigen::MatrixXd& affine_equality_matrix,
                   const Eigen::MatrixXd& affine_equality_boundary)
    : kernel_matrix_(kernel_matrix),
      offset_(offset),
      affine_inequality_matrix_(affine_inequality_matrix),
      affine_inequality_boundary_(affine_inequality_boundary),
      affine_equality_matrix_(affine_equality_matrix),
      affine_equality_boundary_(affine_equality_boundary) {}

const Eigen::MatrixXd& QpSolver::params() const { return params_; }

const Eigen::MatrixXd& QpSolver::kernel_matrix() const {
  return kernel_matrix_;
}

const Eigen::MatrixXd& QpSolver::offset() const { return offset_; }

const Eigen::MatrixXd& QpSolver::affine_equality_matrix() const {
  return affine_equality_matrix_;
}

const Eigen::MatrixXd& QpSolver::affine_equality_boundary() const {
  return affine_equality_boundary_;
}

const Eigen::MatrixXd& QpSolver::affine_inequality_matrix() const {
  return affine_inequality_matrix_;
}

const Eigen::MatrixXd& QpSolver::affine_inequality_boundary() const {
  return affine_inequality_boundary_;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
