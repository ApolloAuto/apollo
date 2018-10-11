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

#include "modules/planning/math/finite_element_qp/fem_1d_jerk_qp_problem.h"

#include "Eigen/Core"
#include "cybertron/common/log.h"

#include "modules/common/math/matrix_operations.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;
using apollo::common::math::DenseToCSCMatrix;

bool Fem1dJerkQpProblem::Optimize() {
  // TODO(All): implement here.
  return true;
}

void Fem1dJerkQpProblem::CalcualteKernel(std::vector<c_float>* P_data,
                                         std::vector<c_int>* P_indices,
                                         std::vector<c_int>* P_indptr) {
  CHECK_NOTNULL(P_data);
  CHECK_NOTNULL(P_indices);
  CHECK_NOTNULL(P_indptr);
  return;
}

void Fem1dJerkQpProblem::CalcualteOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  // TODO(All): implement here.
  return;
}

void Fem1dJerkQpProblem::CalcualteAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  CHECK_NOTNULL(A_data);
  CHECK_NOTNULL(A_indices);
  CHECK_NOTNULL(A_indptr);
  // TODO(All): implement here.
  return;
}

}  // namespace planning
}  // namespace apollo
