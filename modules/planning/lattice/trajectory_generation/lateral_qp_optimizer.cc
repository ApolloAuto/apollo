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

#include "modules/planning/lattice/trajectory_generation/lateral_qp_optimizer.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common/log.h"
namespace apollo {
namespace planning {

bool LateralQPOptimizer::optimize(
    const std::array<double, 3>& d_state,
    const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds) {
  delta_s_ = delta_s;
  const int num_var = static_cast<int>(d_bounds.size());
  const int kNumParam = 3 * num_var;
  const int kNumConstraint = 3 * (num_var - 1) + 3;
  ::qpOASES::HessianType hessian_type = ::qpOASES::HST_POSDEF;
  ::qpOASES::QProblem qp_problem(kNumParam, kNumConstraint, hessian_type);
  ::qpOASES::Options my_options;
  my_options.enableRegularisation = ::qpOASES::BT_TRUE;
  // TODO(kechxu) move to gflags
  my_options.epsNum = -1e-3;
  my_options.epsDen = 1e-3;
  my_options.epsIterRef = 1e-3;
  my_options.terminationTolerance = 1e-3;
  qp_problem.setOptions(my_options);
  int max_iter = 1000;

  // Construct kernel matrix
  const int kNumOfMatrixElement = kNumParam * kNumParam;
  double h_matrix[kNumOfMatrixElement];
  std::fill(h_matrix, h_matrix + kNumOfMatrixElement, 0.0);
  for (int i = 0; i < kNumParam; ++i) {
    if (i < num_var) {
      h_matrix[i * kNumParam + i] = 2.0 * FLAGS_weight_lateral_offset +
          2.0 * FLAGS_weight_lateral_obstacle_distance;
    } else if (i < 2 * num_var) {
      h_matrix[i * kNumParam + i] = 2.0 * FLAGS_weight_lateral_derivative;
    } else {
      h_matrix[i * kNumParam + i] =
          2.0 * FLAGS_weight_lateral_second_order_derivative;
    }
  }

  // Construct offset vector
  const int kNumOfOffsetRow = kNumParam;
  double g_matrix[kNumOfOffsetRow];
  for (int i = 0; i < kNumParam; ++i) {
    if (i < num_var) {
      g_matrix[i] = -2.0 * FLAGS_weight_lateral_obstacle_distance *
                    (d_bounds[i].first + d_bounds[i].second);
    } else {
      g_matrix[i] = 0.0;
    }
  }

  // Construct variable bounds
  double param_lower_bounds[kNumParam];
  double param_upper_bounds[kNumParam];
  const double LARGE_VALUE = 2.0;
  for (int i = 0; i < kNumParam; ++i) {
    if (i < num_var) {
      param_lower_bounds[i] = d_bounds[i].first;
      param_upper_bounds[i] = d_bounds[i].second;
    } else {
      param_lower_bounds[i] = -LARGE_VALUE;
      param_upper_bounds[i] = LARGE_VALUE;
    }
  }

  // Construct constraint matrix
  const int kNumOfConstraint = kNumParam * kNumConstraint;
  double affine_constraint_matrix[kNumOfConstraint];
  std::fill(affine_constraint_matrix,
            affine_constraint_matrix + kNumOfConstraint, 0.0);
  double constraint_lower_bounds[kNumConstraint];
  double constraint_upper_bounds[kNumConstraint];
  const int prime_offset = num_var;
  const int pprime_offset = 2 * num_var;
  int constraint_index = 0;
  // d_i+1'' - d_i''
  for (int i = 0; i + 1 < num_var; ++i) {
    int pprime_index = constraint_index * kNumParam + pprime_offset + i;
    affine_constraint_matrix[pprime_index + 1] = 1.0;
    affine_constraint_matrix[pprime_index] = -1.0;
    constraint_lower_bounds[constraint_index] =
        -FLAGS_lateral_third_order_derivative_max * delta_s;
    constraint_upper_bounds[constraint_index] =
        FLAGS_lateral_third_order_derivative_max * delta_s;
    ++constraint_index;
  }
  // d_i+1' - d_i' - 0.5 * ds * (d_i'' + d_i+1'')
  for (int i = 0; i + 1 < num_var; ++i) {
    int pprime_index = constraint_index * kNumParam + pprime_offset + i;
    int prime_index = constraint_index * kNumParam + prime_offset + i;
    affine_constraint_matrix[prime_index + 1] = 1.0;
    affine_constraint_matrix[prime_index] = -1.0;
    affine_constraint_matrix[pprime_index + 1] = -0.5 * delta_s;
    affine_constraint_matrix[pprime_index] = -0.5 * delta_s;
    constraint_lower_bounds[constraint_index] = 0.0;
    constraint_upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }
  // d_i+1 - d_i - d_i' * ds - 1/3 * d_i'' * ds^2 - 1/6 * d_i+1'' * ds^2
  for (int i = 0; i + 1 < num_var; ++i) {
    int var_index = constraint_index * kNumParam + i;
    int pprime_index = constraint_index * kNumParam + pprime_offset + i;
    int prime_index = constraint_index * kNumParam + prime_offset + i;
    affine_constraint_matrix[var_index + 1] = 1.0;
    affine_constraint_matrix[var_index] = -1.0;
    affine_constraint_matrix[prime_index] = -delta_s;
    affine_constraint_matrix[pprime_index] = -delta_s * delta_s / 3.0;
    affine_constraint_matrix[pprime_index + 1] = -delta_s * delta_s / 6.0;
    constraint_lower_bounds[constraint_index] = 0.0;
    constraint_upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }

  int var_index = constraint_index * kNumParam;
  affine_constraint_matrix[var_index] = 1.0;
  constraint_lower_bounds[constraint_index] = d_state[0];
  constraint_upper_bounds[constraint_index] = d_state[0];
  ++constraint_index;

  int prime_index = constraint_index * kNumParam + prime_offset;
  affine_constraint_matrix[prime_index] = 1.0;
  constraint_lower_bounds[constraint_index] = d_state[1];
  constraint_upper_bounds[constraint_index] = d_state[1];
  ++constraint_index;

  int pprime_index = constraint_index * kNumParam + pprime_offset;
  affine_constraint_matrix[pprime_index] = 1.0;
  constraint_lower_bounds[constraint_index] = d_state[2];
  constraint_upper_bounds[constraint_index] = d_state[2];
  ++constraint_index;

  CHECK_EQ(constraint_index, kNumConstraint);

  auto ret = qp_problem.init(
      h_matrix, g_matrix, affine_constraint_matrix,
      param_lower_bounds, param_upper_bounds,
      constraint_lower_bounds, constraint_upper_bounds, max_iter);
  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      AERROR << "qpOASES solver failed due to reached max iteration";
    } else {
      AERROR << "qpOASES solver failed due to infeasibility or other internal "
                "reasons";
    }
  }
  double result[kNumParam];
  qp_problem.getPrimalSolution(result);
  for (int i = 0; i < num_var; ++i) {
    opt_d_.push_back(result[i]);
    opt_d_prime_.push_back(result[num_var + i]);
    opt_d_pprime_.push_back(result[2 * num_var + i]);
  }
  opt_d_prime_[num_var - 1] = 0.0;
  opt_d_pprime_[num_var - 1] = 0.0;
  return qp_problem.isSolved() == qpOASES::BT_TRUE;
}

PiecewiseJerkTrajectory1d
LateralQPOptimizer::GetOptimalTrajectory() const {
  CHECK(!opt_d_.empty() && !opt_d_prime_.empty() && !opt_d_pprime_.empty());

  PiecewiseJerkTrajectory1d optimal_trajectory(
      opt_d_.front(), opt_d_prime_.front(), opt_d_pprime_.front());

  for (std::size_t i = 1; i < opt_d_.size(); ++i) {
    double j = (opt_d_pprime_[i] - opt_d_pprime_[i - 1]) / delta_s_;
    optimal_trajectory.AppendSegment(j, delta_s_);
  }
  return optimal_trajectory;
}

}  // namespace planning
}  // namespace apollo
