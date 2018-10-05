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

#include "modules/planning/lattice/trajectory_generation/osqp_lateral_qp_optimizer.h"

#include "Eigen/Core"
#include "osqp/include/osqp.h"

#include "cybertron/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using Eigen::MatrixXd;

bool OsqpLateralQPOptimizer::optimize(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds) {
  delta_s_ = delta_s;
  const int num_var = static_cast<int>(d_bounds.size());
  const int kNumParam = 3 * num_var;
  const int kNumConstraint = 3 * (num_var - 1) + 5;

  // const int kNumOfMatrixElement = kNumParam * kNumParam;
  MatrixXd kernel = MatrixXd::Zero(kNumParam, kNumParam);  // dense matrix

  for (int i = 0; i < kNumParam; ++i) {
    if (i < num_var) {
      kernel(i, i) = 2.0 * FLAGS_weight_lateral_offset +
                     2.0 * FLAGS_weight_lateral_obstacle_distance;
    } else if (i < 2 * num_var) {
      kernel(i, i) = 2.0 * FLAGS_weight_lateral_derivative;
    } else {
      kernel(i, i) = 2.0 * FLAGS_weight_lateral_second_order_derivative;
    }
  }

  MatrixXd affine_constraint = MatrixXd::Zero(kNumParam, kNumConstraint);
  // const int kNumOfConstraint = kNumParam * kNumConstraint;
  double lower_bounds[kNumConstraint];
  double upper_bounds[kNumConstraint];

  const int prime_offset = num_var;
  const int pprime_offset = 2 * num_var;
  int constraint_index = 0;
  // d_i+1'' - d_i''
  for (int i = 0; i + 1 < num_var; ++i) {
    const int row = constraint_index;
    const int col = pprime_offset + i;
    affine_constraint(row, col) = -1.0;
    affine_constraint(row, col + 1) = 1.0;

    lower_bounds[constraint_index] =
        -FLAGS_lateral_third_order_derivative_max * delta_s;
    upper_bounds[constraint_index] =
        FLAGS_lateral_third_order_derivative_max * delta_s;
    ++constraint_index;
  }

  // d_i+1' - d_i' - 0.5 * ds * (d_i'' + d_i+1'')
  for (int i = 0; i + 1 < num_var; ++i) {
    affine_constraint(constraint_index, prime_offset + i) = -1.0;
    affine_constraint(constraint_index, prime_offset + i + 1) = 1.0;

    affine_constraint(constraint_index, pprime_offset + i) = -0.5 * delta_s;
    affine_constraint(constraint_index, pprime_offset + i + 1) = -0.5 * delta_s;

    lower_bounds[constraint_index] = 0.0;
    upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }

  // d_i+1 - d_i - d_i' * ds - 1/3 * d_i'' * ds^2 - 1/6 * d_i+1'' * ds^2
  for (int i = 0; i + 1 < num_var; ++i) {
    affine_constraint(constraint_index, i) = -1.0;
    affine_constraint(constraint_index, i + 1) = 1.0;

    affine_constraint(constraint_index, prime_offset + i) = -delta_s;

    affine_constraint(constraint_index, pprime_offset + i) =
        -delta_s * delta_s / 3.0;
    affine_constraint(constraint_index, pprime_offset + i + 1) =
        -delta_s * delta_s / 6.0;

    lower_bounds[constraint_index] = 0.0;
    upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }

  affine_constraint(constraint_index, 0) = 1.0;
  lower_bounds[constraint_index] = d_state[0];
  upper_bounds[constraint_index] = d_state[0];
  ++constraint_index;

  affine_constraint(constraint_index, prime_offset) = 1.0;
  lower_bounds[constraint_index] = d_state[1];
  upper_bounds[constraint_index] = d_state[1];
  ++constraint_index;

  affine_constraint(constraint_index, pprime_offset) = 1.0;
  lower_bounds[constraint_index] = d_state[2];
  upper_bounds[constraint_index] = d_state[2];
  ++constraint_index;

  affine_constraint(constraint_index, prime_offset + num_var - 1) = 1.0;
  lower_bounds[constraint_index] = 0.0;
  upper_bounds[constraint_index] = 0.0;
  ++constraint_index;

  affine_constraint(constraint_index, pprime_offset + num_var - 1) = 1.0;
  lower_bounds[constraint_index] = 0.0;
  upper_bounds[constraint_index] = 0.0;
  ++constraint_index;

  return true;
}

}  // namespace planning
}  // namespace apollo
