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
  delta_s_ = delta_s;
  const int num_var = d_bounds.size();

  const int kNumConstraint = (num_var - 3);
  ::qpOASES::HessianType hessian_type = ::qpOASES::HST_POSDEF;
  ::qpOASES::QProblem qp_problem(num_var, kNumConstraint, hessian_type);
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
  const int kNumOfMatrixElement = num_var * num_var;
  double h_matrix[kNumOfMatrixElement];
  std::fill(h_matrix, h_matrix + kNumOfMatrixElement, 0.0);

  // pre-calculate some const
  const double delta_s_sq = delta_s * delta_s;
  const double delta_s_quad = delta_s_sq * delta_s_sq;
  const double one_over_delta_s_sq_coeff =
      1.0 / delta_s_sq * FLAGS_weight_lateral_derivative;
  const double one_over_delta_s_quad_coeff =
      1.0 / delta_s_quad * FLAGS_weight_lateral_second_order_derivative;

  for (int i = 0; i < num_var; ++i) {
    h_matrix[i * num_var + i] += 2.0 * FLAGS_weight_lateral_offset;
    h_matrix[i * num_var + i] += 2.0 * FLAGS_weight_lateral_obstacle_distance;

    // first order derivative
    int idx = (i + 1) * num_var + (i + 1);
    if (idx < kNumOfMatrixElement) {
      h_matrix[idx] += 2 * one_over_delta_s_sq_coeff;
      h_matrix[i * num_var + i] += 2 * one_over_delta_s_sq_coeff;
      h_matrix[i * num_var + i + 1] += 2.0 * one_over_delta_s_sq_coeff;
      h_matrix[(i + 1) * num_var + i] += 2.0 * one_over_delta_s_sq_coeff;
    }

    // second order derivative
    idx = (i + 2) * num_var + (i + 2);
    if (idx < kNumOfMatrixElement) {
      h_matrix[idx] += 2.0 * one_over_delta_s_quad_coeff;
      h_matrix[(i + 1) * num_var + i + 1] += 8.0 * one_over_delta_s_quad_coeff;
      h_matrix[i * num_var + i] += 2.0 * one_over_delta_s_quad_coeff;

      h_matrix[(i)*num_var + (i + 1)] += -4.0 * one_over_delta_s_quad_coeff;
      h_matrix[(i + 1) * num_var + i] += -4.0 * one_over_delta_s_quad_coeff;

      h_matrix[(i + 1) * num_var + (i + 2)] +=
          -4.0 * one_over_delta_s_quad_coeff;
      h_matrix[(i + 2) * num_var + (i + 1)] +=
          -4.0 * one_over_delta_s_quad_coeff;

      h_matrix[i * num_var + (i + 2)] += 2.0 * one_over_delta_s_quad_coeff;
      h_matrix[(i + 2) * num_var + i] += 2.0 * one_over_delta_s_quad_coeff;
    }
  }

  // Construct offset vector
  double g_matrix[num_var];
  for (int i = 0; i < num_var; ++i) {
    g_matrix[i] = -2.0 * FLAGS_weight_lateral_obstacle_distance *
                  (d_bounds[i].first + d_bounds[i].second);
  }

  // Construct variable bounds
  double param_lower_bounds[num_var];
  double param_upper_bounds[num_var];
  for (int i = 0; i < num_var; ++i) {
    param_lower_bounds[i] = d_bounds[i].first;
    param_upper_bounds[i] = d_bounds[i].second;
  }

  // Construct constraint matrix
  const int kNumOfConstraint = num_var * kNumConstraint;
  double affine_constraint_matrix[kNumOfConstraint];
  std::fill(affine_constraint_matrix,
            affine_constraint_matrix + kNumOfConstraint, 0.0);
  double constraint_lower_bounds[kNumConstraint];
  double constraint_upper_bounds[kNumConstraint];

  int constraint_index = 0;

  const double third_order_derivative_max_coff =
      FLAGS_lateral_third_order_derivative_max * delta_s * delta_s * delta_s;
  for (int i = 0; i + 3 < num_var; ++i) {
    int idx = i * num_var + i;
    affine_constraint_matrix[idx] = -1.0;
    affine_constraint_matrix[idx + 1] = 3.0;
    affine_constraint_matrix[idx + 2] = -3.0;
    affine_constraint_matrix[idx + 3] = 1.0;
    constraint_lower_bounds[constraint_index] =
        -third_order_derivative_max_coff;
    constraint_upper_bounds[constraint_index] = third_order_derivative_max_coff;
    ++constraint_index;
  }

  // TODO(lianglia-apollo):
  // (1) Add initial status constraints
  // (2) Verify whether it is necessary to keep final dl and ddl to 0.0
  /*
  const double kDeltaL = 1.0e-4;
  int var_index = constraint_index * num_var;
  affine_constraint_matrix[var_index] = 1.0;
  constraint_lower_bounds[constraint_index] = d_state[0] - kDeltaL;
  constraint_upper_bounds[constraint_index] = d_state[0] + kDeltaL;
  ++constraint_index;

  const double kDeltaDL = 1.0e-2;
  var_index = constraint_index;
  affine_constraint_matrix[var_index] = -1.0 / delta_s;
  affine_constraint_matrix[var_index + 1] = 1.0 / delta_s;
  constraint_lower_bounds[constraint_index] = d_state[1] - kDeltaDL;
  constraint_upper_bounds[constraint_index] = d_state[1] + kDeltaDL;
  ++constraint_index;

  var_index = constraint_index;
  affine_constraint_matrix[var_index] = 1.0 / delta_s_sq;
  affine_constraint_matrix[var_index + 1] = -2.0 / delta_s_sq;
  affine_constraint_matrix[var_index + 2] = 1.0 / delta_s_sq;
  constraint_lower_bounds[constraint_index] = d_state[2];
  constraint_upper_bounds[constraint_index] = d_state[2];
  ++constraint_index;
  */

  /*
  var_index = constraint_index * num_var - 2;
  affine_constraint_matrix[var_index] = -1.0 / delta_s;
  affine_constraint_matrix[var_index + 1] = 1.0 / delta_s;
  constraint_lower_bounds[constraint_index] = 0.0;
  constraint_upper_bounds[constraint_index] = 0.0;
  ++constraint_index;

  var_index = constraint_index * num_var - 3;
  affine_constraint_matrix[var_index] = 1.0 / delta_s_sq;
  affine_constraint_matrix[var_index + 1] = -2.0 / delta_s_sq;
  affine_constraint_matrix[var_index + 2] = 1.0 / delta_s_sq;
  constraint_lower_bounds[constraint_index] = 0.0;
  constraint_upper_bounds[constraint_index] = 0.0;
  ++constraint_index;
  */

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
  double result[num_var];
  qp_problem.getPrimalSolution(result);
  for (int i = 0; i + 2 < num_var; ++i) {
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
