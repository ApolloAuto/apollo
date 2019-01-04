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

#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_problem.h"

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

DualVariableWarmStartProblem::DualVariableWarmStartProblem(
    const PlannerOpenSpaceConfig& planner_open_space_config) {
  planner_open_space_config_.CopyFrom(planner_open_space_config);
}

bool DualVariableWarmStartProblem::Solve(
    const size_t& horizon, const double& ts, const Eigen::MatrixXd& ego,
    size_t obstacles_num, const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const Eigen::MatrixXd& xWS, Eigen::MatrixXd* l_warm_up,
    Eigen::MatrixXd* n_warm_up) {
  auto t_start = cyber::Time::Now().ToSecond();
  DualVariableWarmStartIPOPTInterface* ptop =
      new DualVariableWarmStartIPOPTInterface(
          horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
          obstacles_b, xWS, planner_open_space_config_);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", planner_open_space_config_.
      dual_variable_warm_start_config().ipopt_config().ipopt_print_level());
  app->Options()->SetIntegerValue("mumps_mem_percent",
      planner_open_space_config_.dual_variable_warm_start_config().
      ipopt_config().mumps_mem_percent());
  app->Options()->SetNumericValue("mumps_pivtol", planner_open_space_config_.
      dual_variable_warm_start_config().ipopt_config().mumps_pivtol());
  app->Options()->SetIntegerValue("max_iter", planner_open_space_config_.
      dual_variable_warm_start_config().ipopt_config().ipopt_max_iter());
  app->Options()->SetNumericValue("tol", planner_open_space_config_.
      dual_variable_warm_start_config().ipopt_config().ipopt_tol());
  app->Options()->SetNumericValue("acceptable_constr_viol_tol",
      planner_open_space_config_.dual_variable_warm_start_config().
      ipopt_config().ipopt_acceptable_constr_viol_tol());
  app->Options()->SetNumericValue("min_hessian_perturbation",
      planner_open_space_config_.dual_variable_warm_start_config().
      ipopt_config().ipopt_min_hessian_perturbation());
  app->Options()->SetNumericValue("jacobian_regularization_value",
      planner_open_space_config_.dual_variable_warm_start_config().
      ipopt_config().ipopt_jacobian_regularization_value());
  app->Options()->SetStringValue("print_timing_statistics",
      planner_open_space_config_.dual_variable_warm_start_config().
      ipopt_config().ipopt_print_timing_statistics());
  app->Options()->SetStringValue("alpha_for_y",
      planner_open_space_config_.dual_variable_warm_start_config().
      ipopt_config().ipopt_alpha_for_y());
  app->Options()->SetStringValue("recalc_y", planner_open_space_config_.
      dual_variable_warm_start_config().ipopt_config().ipopt_recalc_y());

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    AERROR
        << "*** Dual variable wart start problem error during initialization!";
    return false;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    AINFO << "*** The problem solved in " << iter_count << " iterations!";

    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    AINFO << "*** The final value of the objective function is " << final_obj
          << '.';
    auto t_end = cyber::Time::Now().ToSecond();

    AINFO << "Dual vairable warm start solving time in second : "
          << t_end - t_start;
  } else {
    AINFO << "Solve not succeeding, return status: " << int(status);
  }

  ptop->get_optimization_results(l_warm_up, n_warm_up);

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

}  // namespace planning
}  // namespace apollo
