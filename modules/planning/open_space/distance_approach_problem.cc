/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/open_space/distance_approach_problem.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

DistanceApproachProblem::DistanceApproachProblem(
    Eigen::MatrixXd x0, Eigen::MatrixXd xF, Eigen::MatrixXd last_time_u,
    std::size_t horizon, float ts, Eigen::MatrixXd ego, Eigen::MatrixXd xWS,
    Eigen::MatrixXd uWS, Eigen::MatrixXd XYbounds, std::size_t obstacles_num,
    Eigen::MatrixXd obstacles_edges_num, Eigen::MatrixXd obstacles_A,
    Eigen::MatrixXd obstacles_b,
    const PlannerOpenSpaceConfig& planner_open_space_config)
    : x0_(x0),
      xF_(xF),
      last_time_u_(last_time_u),
      horizon_(horizon),
      ts_(ts),
      ego_(ego),
      xWS_(xWS),
      uWS_(uWS),
      XYbounds_(XYbounds),
      obstacles_num_(obstacles_num),
      obstacles_edges_num_(obstacles_edges_num),
      obstacles_A_(obstacles_A),
      obstacles_b_(obstacles_b) {
  planner_open_space_config_.CopyFrom(planner_open_space_config);
}

bool DistanceApproachProblem::Solve(Eigen::MatrixXd* state_result,
                                    Eigen::MatrixXd* control_result,
                                    Eigen::MatrixXd* time_result) {
  // TODO(QiL) : set up number of variables and number of constaints, and rego
  // so constants do not get set repeatedly

  // n1 : states variables, 4 * (N+1)
  int n1 = 4 * (horizon_ + 1);

  // n2 : control inputs variables
  int n2 = 2 * horizon_;

  // n3 : sampling time variables
  int n3 = horizon_ + 1;

  // n4 : dual multiplier associated with obstacleShape
  int n4 = obstacles_edges_num_.sum() * (horizon_ + 1);

  // n5 : dual multipier associated with car shape, obstacles_num*4 * (N+1)
  int n5 = obstacles_num_ * 4 * (horizon_ + 1);

  // m1 : dynamics constatins
  int m1 = 4 * horizon_;

  // m2 : control rate constraints (only steering)
  int m2 = horizon_;

  // m3 : sampling time equality constraints
  int m3 = horizon_;

  // m4 : obstacle constraints
  int m4 = 4 * obstacles_num_ * (horizon_ + 1);

  int num_of_variables = n1 + n2 + n3 + n4 + n5;
  int num_of_constraints = m1 + m2 + m3 + m4;

  // TODO(QiL) : evaluate whether need to new it everytime

  auto t_start = cyber::Time::Now().ToSecond();
  DistanceApproachIPOPTInterface* ptop = new DistanceApproachIPOPTInterface(
      num_of_variables, num_of_constraints, horizon_, ts_, ego_, xWS_, uWS_,
      timeWS_, x0_, xF_, last_time_u_, XYbounds_, obstacles_edges_num_,
      obstacles_num_, obstacles_A_, obstacles_b_, planner_open_space_config_);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  // app->Options()->SetStringValue("jacobian_approximation",
  // "finite-difference-values");
  // app->Options()->SetStringValue("derivative_test", "first-order");
  // app->Options()->SetNumericValue("derivative_test_tol", 1.0e-3);
  // TODO(QiL) : Change IPOPT settings to flag or configs
  app->Options()->SetIntegerValue("print_level", 0);
  app->Options()->SetIntegerValue("mumps_mem_percent", 6000);
  app->Options()->SetNumericValue("mumps_pivotol", 1e-6);
  app->Options()->SetIntegerValue("max_iter", 1000);
  app->Options()->SetNumericValue("tol", 1e-4);
  app->Options()->SetNumericValue("min_hessian_perturbation", 1e-12);
  app->Options()->SetNumericValue("jacobian_regularization_value", 1e-7);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    AINFO << "*** Distiance Approach problem error during initialization!";
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

    AINFO << "DistanceApproachProblem solving time in second : "
          << t_end - t_start;
  } else {
    AINFO << "Return status: " << int(status);
  }

  ptop->get_optimization_results(state_result, control_result, time_result);

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

}  // namespace planning
}  // namespace apollo
