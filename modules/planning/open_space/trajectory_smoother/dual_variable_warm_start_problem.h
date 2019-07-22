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
#include <algorithm>
#include "Eigen/Dense"

#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_ipopt_interface.h"
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_ipopt_qp_interface.h"
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_osqp_interface.h"
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_slack_osqp_interface.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

class DualVariableWarmStartProblem {
 public:
  explicit DualVariableWarmStartProblem(
      const PlannerOpenSpaceConfig& planner_open_space_config);

  virtual ~DualVariableWarmStartProblem() = default;

  bool Solve(const size_t horizon, const double ts, const Eigen::MatrixXd& ego,
             const size_t obstacles_num,
             const Eigen::MatrixXi& obstacles_edges_num,
             const Eigen::MatrixXd& obstacles_A,
             const Eigen::MatrixXd& obstacles_b, const Eigen::MatrixXd& xWS,
             Eigen::MatrixXd* l_warm_up, Eigen::MatrixXd* n_warm_up,
             Eigen::MatrixXd* s_warm_up);

 private:
  PlannerOpenSpaceConfig planner_open_space_config_;
};

}  // namespace planning
}  // namespace apollo
