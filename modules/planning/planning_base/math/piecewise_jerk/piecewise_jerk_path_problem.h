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

#include <utility>
#include <vector>

#include "modules/planning/planning_base/common/path_boundary.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_problem.h"
namespace apollo {
namespace planning {

/*
 * @brief:
 * FEM stands for finite element method.
 * This class solve an optimization problem:
 * x
 * |
 * |                       P(s1, x1)  P(s2, x2)
 * |            P(s0, x0)                       ... P(s(k-1), x(k-1))
 * |P(start)
 * |
 * |________________________________________________________ s
 *
 * we suppose s(k+1) - s(k) == s(k) - s(k-1)
 *
 * Given the x, x', x'' at P(start),  The goal is to find x0, x1, ... x(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class PiecewiseJerkPathProblem : public PiecewiseJerkProblem {
 public:
  PiecewiseJerkPathProblem(const size_t num_of_knots, const double delta_s,
                           const std::array<double, 3>& x_init);

  virtual ~PiecewiseJerkPathProblem() = default;
  void set_extra_constraints(const ObsCornerConstraints& extra_constraints) {
    extra_constraints_ = extra_constraints;
  }

  void set_vertex_constraints(const ADCVertexConstraints& vertexs) {
    vertex_constraints_ = vertexs;
  }

 protected:
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr) override;

  void CalculateOffset(std::vector<c_float>* q) override;

  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                 std::vector<c_int>* A_indices,
                                 std::vector<c_int>* A_indptr,
                                 std::vector<c_float>* lower_bounds,
                                 std::vector<c_float>* upper_bounds) override;
  
  OSQPSettings* SolverDefaultSettings() override;

 private:
  ObsCornerConstraints extra_constraints_;
  ADCVertexConstraints vertex_constraints_;
};

}  // namespace planning
}  // namespace apollo
