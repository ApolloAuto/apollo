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

/*
 * @file
 */

#pragma once

#include <vector>

#include "Eigen/Dense"

#include "modules/planning/open_space/dual_variable_warm_start_ipopt_interface.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

class DualVariableWarmStartProblem {
 public:
  explicit DualVariableWarmStartProblem(
      const PlannerOpenSpaceConfig& planner_open_space_config);

  virtual ~DualVariableWarmStartProblem() = default;

  bool Solve(const std::size_t& num_of_variables,
             const std::size_t& num_of_constraints, const std::size_t& horizon,
             const float& ts, const Eigen::MatrixXd& x0,
             const Eigen::MatrixXd& xF, const Eigen::MatrixXd& XYbounds,
             Eigen::MatrixXd* l_warm_up, Eigen::MatrixXd* n_warm_up);

 private:
  PlannerOpenSpaceConfig planner_open_space_config_;
};

}  // namespace planning
}  // namespace apollo
