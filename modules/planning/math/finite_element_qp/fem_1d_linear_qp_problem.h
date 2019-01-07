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

#pragma once

#include <vector>

#include "modules/planning/math/finite_element_qp/fem_1d_qp_problem.h"

namespace apollo {
namespace planning {

/*
 * @brief:
 * construct a N x N matrix of x
 */

class Fem1dLinearQpProblem : public Fem1dQpProblem {
 public:
  Fem1dLinearQpProblem() = default;

  virtual ~Fem1dLinearQpProblem() = default;

  bool Optimize() override;

  void PreSetKernel() override;

  void PreSetAffineConstraintMatrix();

 private:
  // naming convention follows osqp solver.
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr) override;

  void CalculateOffset(std::vector<c_float>* q) override;

  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                 std::vector<c_int>* A_indices,
                                 std::vector<c_int>* A_indptr,
                                 std::vector<c_float>* lower_bounds,
                                 std::vector<c_float>* upper_bounds) override;

 private:
  bool is_const_kernel_ = false;
  bool is_const_affine_constraint_matrix_ = false;

  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;

  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
};

}  // namespace planning
}  // namespace apollo
