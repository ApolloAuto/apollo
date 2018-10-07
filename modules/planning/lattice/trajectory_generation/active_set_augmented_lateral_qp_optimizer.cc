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

#include "modules/planning/lattice/trajectory_generation/active_set_augmented_lateral_qp_optimizer.h"

#include "cybertron/common/log.h"
#include "qpOASES.hpp"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

bool ActiverSetAugmentedLateralQPOptimizer::optimize(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds) {
  opt_d_.clear();
  opt_d_prime_.clear();
  opt_d_pprime_.clear();

  delta_s_ = delta_s;
  const int kNumVariable = d_bounds.size();

  const int kNumConstraint = (kNumVariable - 3);
  ::qpOASES::HessianType hessian_type = ::qpOASES::HST_POSDEF;
  ::qpOASES::QProblem qp_problem(kNumVariable, kNumConstraint, hessian_type);
  ::qpOASES::Options my_options;
  my_options.enableRegularisation = ::qpOASES::BT_TRUE;

  my_options.epsNum = -1e-3;
  my_options.epsDen = 1e-3;
  my_options.epsIterRef = 1e-3;
  my_options.terminationTolerance = 1e-3;
  qp_problem.setOptions(my_options);
  qp_problem.setPrintLevel(qpOASES::PL_NONE);
  int max_iter = 1000;

  // Construct kernel matrix
  const int kNumOfMatrixElement = kNumVariable * kNumVariable;
  double h_matrix[kNumOfMatrixElement];
  std::fill(h_matrix, h_matrix + kNumOfMatrixElement, 0.0);

  // pre-calculate some const
  const double delta_s_sq = delta_s * delta_s;
  const double delta_s_quad = delta_s_sq * delta_s_sq;
  const double one_over_delta_s_sq_coeff =
      1.0 / delta_s_sq * FLAGS_weight_lateral_derivative;
  const double one_over_delta_s_quad_coeff =
      1.0 / delta_s_quad * FLAGS_weight_lateral_second_order_derivative;

  for (int i = 0; i < kNumVariable; ++i) {
    h_matrix[i * kNumVariable + i] += 2.0 * FLAGS_weight_lateral_offset;
    h_matrix[i * kNumVariable + i] +=
        2.0 * FLAGS_weight_lateral_obstacle_distance;

    // first order derivative
    int idx = (i + 1) * kNumVariable + (i + 1);
    if (idx < kNumOfMatrixElement) {
      h_matrix[idx] += 2 * one_over_delta_s_sq_coeff;
      h_matrix[i * kNumVariable + i] += 2 * one_over_delta_s_sq_coeff;
      h_matrix[i * kNumVariable + i + 1] += 2.0 * one_over_delta_s_sq_coeff;
      h_matrix[(i + 1) * kNumVariable + i] += 2.0 * one_over_delta_s_sq_coeff;
    }

    // second order derivative
    idx = (i + 2) * kNumVariable + (i + 2);
    if (idx < kNumOfMatrixElement) {
      h_matrix[idx] += 2.0 * one_over_delta_s_quad_coeff;
      h_matrix[(i + 1) * kNumVariable + i + 1] +=
          8.0 * one_over_delta_s_quad_coeff;
      h_matrix[i * kNumVariable + i] += 2.0 * one_over_delta_s_quad_coeff;

      h_matrix[(i)*kNumVariable + (i + 1)] +=
          -4.0 * one_over_delta_s_quad_coeff;
      h_matrix[(i + 1) * kNumVariable + i] +=
          -4.0 * one_over_delta_s_quad_coeff;

      h_matrix[(i + 1) * kNumVariable + (i + 2)] +=
          -4.0 * one_over_delta_s_quad_coeff;
      h_matrix[(i + 2) * kNumVariable + (i + 1)] +=
          -4.0 * one_over_delta_s_quad_coeff;

      h_matrix[i * kNumVariable + (i + 2)] += 2.0 * one_over_delta_s_quad_coeff;
      h_matrix[(i + 2) * kNumVariable + i] += 2.0 * one_over_delta_s_quad_coeff;
    }
  }

  // Construct offset vector
  double g_matrix[kNumVariable];
  for (int i = 0; i < kNumVariable; ++i) {
    g_matrix[i] = -2.0 * FLAGS_weight_lateral_obstacle_distance *
                  (d_bounds[i].first + d_bounds[i].second);
  }

  // Construct variable bounds
  double param_lower_bounds[kNumVariable];
  double param_upper_bounds[kNumVariable];
  for (int i = 0; i < kNumVariable; ++i) {
    param_lower_bounds[i] = d_bounds[i].first;
    param_upper_bounds[i] = d_bounds[i].second;
  }

  // Construct constraint matrix
  const int kNumOfConstraint = kNumVariable * kNumConstraint;
  double affine_constraint_matrix[kNumOfConstraint];
  std::fill(affine_constraint_matrix,
            affine_constraint_matrix + kNumOfConstraint, 0.0);
  double constraint_lower_bounds[kNumConstraint];
  double constraint_upper_bounds[kNumConstraint];

  int constraint_index = 0;

  const double third_order_derivative_max_coff =
      FLAGS_lateral_third_order_derivative_max * delta_s * delta_s * delta_s;
  for (int i = 0; i + 3 < kNumVariable; ++i) {
    int idx = i * kNumVariable + i;
    affine_constraint_matrix[idx] = -1.0;
    affine_constraint_matrix[idx + 1] = 3.0;
    affine_constraint_matrix[idx + 2] = -3.0;
    affine_constraint_matrix[idx + 3] = 1.0;
    constraint_lower_bounds[constraint_index] =
        -third_order_derivative_max_coff;
    constraint_upper_bounds[constraint_index] = third_order_derivative_max_coff;
    ++constraint_index;
  }

  const double kDeltaL = 1.0e-7;
  int var_index = constraint_index * kNumVariable;
  bool set_init_status_constraints = false;

  // TODO(lianglia-apollo):
  // Add initial status constraints here
  if (set_init_status_constraints) {
    affine_constraint_matrix[var_index] = 1.0;
    constraint_lower_bounds[constraint_index] = d_state[0] - kDeltaL;
    constraint_upper_bounds[constraint_index] = d_state[0] + kDeltaL;
    ++constraint_index;

    var_index = constraint_index * kNumVariable;
    affine_constraint_matrix[var_index] = -1.0 / delta_s;
    affine_constraint_matrix[var_index + 1] = 1.0 / delta_s;
    constraint_lower_bounds[constraint_index] = d_state[1] - kDeltaL;
    constraint_upper_bounds[constraint_index] = d_state[1] + kDeltaL;
    ++constraint_index;

    var_index = constraint_index * kNumVariable;
    affine_constraint_matrix[var_index] = 1.0 / delta_s_sq;
    affine_constraint_matrix[var_index + 1] = -2.0 / delta_s_sq;
    affine_constraint_matrix[var_index + 2] = 1.0 / delta_s_sq;
    constraint_lower_bounds[constraint_index] = d_state[2] - kDeltaL;
    constraint_upper_bounds[constraint_index] = d_state[2] + kDeltaL;
    ++constraint_index;
  }

  // TODO(lianglia-apollo):
  // Verify whether it is necessary to keep final dl and ddl to 0.0

  CHECK_EQ(constraint_index, kNumConstraint);

  auto ret = qp_problem.init(h_matrix, g_matrix, affine_constraint_matrix,
                             param_lower_bounds, param_upper_bounds,
                             constraint_lower_bounds, constraint_upper_bounds,
                             max_iter);
  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      AERROR << "qpOASES solver failed due to reached max iteration";
    } else {
      AERROR << "qpOASES solver failed due to infeasibility or other internal "
                "reasons";
    }
  }
  double result[kNumVariable];
  qp_problem.getPrimalSolution(result);
  for (int i = 0; i + 2 < kNumVariable; ++i) {
    opt_d_.push_back(result[i]);
    if (i > 0) {
      opt_d_prime_.push_back((result[i] - result[i - 1]) / delta_s);
    }
    if (i > 1) {
      opt_d_pprime_.push_back((result[i + 2] - 2 * result[i + 1] + result[i]) /
                              delta_s_sq);
    }
  }
  opt_d_prime_.push_back(0.0);
  opt_d_pprime_.push_back(0.0);
  return qp_problem.isSolved() == qpOASES::BT_TRUE;
}

}  // namespace planning
}  // namespace apollo
