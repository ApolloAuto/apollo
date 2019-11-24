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
 * @file
 **/
#include "modules/planning/math/smoothing_spline/spline_1d_solver.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;

// converte qp problem to proto
void Spline1dSolver::GenerateProblemProto(
    QuadraticProgrammingProblem* const qp_proto) const {
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

  qp_proto->set_param_size(static_cast<int32_t>(kernel_matrix.rows()));
  ConvertMatrixXdToProto(kernel_matrix, qp_proto->mutable_quadratic_matrix());

  for (int i = 0; i < offset.rows(); ++i) {
    qp_proto->add_bias(offset(i, 0));
  }

  ConvertMatrixXdToProto(inequality_constraint_matrix,
                         qp_proto->mutable_inequality_matrix());

  for (int i = 0; i < inequality_constraint_boundary.rows(); ++i) {
    qp_proto->add_inequality_value(inequality_constraint_boundary(i, 0));
  }

  ConvertMatrixXdToProto(equality_constraint_matrix,
                         qp_proto->mutable_equality_matrix());

  for (int i = 0; i < equality_constraint_boundary.rows(); ++i) {
    qp_proto->add_equality_value(equality_constraint_boundary(i, 0));
  }

  // add the optimal solution
  const auto& splines = spline_.splines();
  for (const auto& spline_seg : splines) {
    const auto& param_seg = spline_seg.spline_func().params();
    for (const auto value : param_seg) {
      qp_proto->add_optimal_param(value);
    }
  }
}

void Spline1dSolver::ConvertMatrixXdToProto(const Eigen::MatrixXd& matrix,
                                            QPMatrix* const proto) const {
  int row_size = static_cast<int>(matrix.rows());
  int col_size = static_cast<int>(matrix.cols());

  proto->set_row_size(row_size);
  proto->set_col_size(col_size);

  for (int r = 0; r < row_size; ++r) {
    for (int c = 0; c < col_size; ++c) {
      proto->add_element(matrix(r, c));
    }
  }
}

}  // namespace planning
}  // namespace apollo
