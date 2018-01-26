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
 * distance_approach_problem.cc
 */

#include "modules/planning/planner/open_space/distance_approach_problem.h"

#include <algorithm>
#include <iomanip>
#include <utility>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

DistanceApproachProblem::DistanceApproachProblem(
    Eigen::MatrixXd x0, Eigen::MatrixXd xF, std::size_t horizon, float ts,
    Eigen::MatrixXd ego, Eigen::MatrixXd xWS, Eigen::MatrixXd uWS,
    Eigen::MatrixXd timeWS, Eigen::MatrixXd XYbounds, std::size_t nOb,
    Eigen::MatrixXd vOb, Eigen::MatrixXd AOb, Eigen::MatrixXd bOb)
    : x0_(x0),
      xF_(xF),
      horizon_(horizon),
      ts_(ts),
      ego_(ego),
      xWS_(xWS),
      uWS_(uWS),
      timeWS_(timeWS),
      XYbounds_(XYbounds),
      nOb_(nOb),
      vOb_(vOb),
      AOb_(AOb),
      bOb_(bOb) {}

bool DistanceApproachProblem::Solve(Eigen::MatrixXd* state_result,
                                    Eigen::MatrixXd* control_result,
                                    Eigen::MatrixXd* time_result) {
  // TODO(QiL) : set up number of variables and number of constaints, and rego
  // so constants do not get set repeatedly

  // n1 : states variables
  int n1 = 4 * (horizon_ + 1);

  // n2 : control inputs variables
  int n2 = 2 * horizon_;

  // n3 : sampling time variables
  int n3 = horizon_ + 1;

  // n4 : dual multiplier associated with obstacleShape
  int n4 = vOb_.sum() * (horizon_ + 1);

  // m1 : state equality constatins
  int m1 = 4 * horizon_;

  // m2 : sampling time equality constraints
  int m2 = horizon_;

  // m3 : state inequality constraints
  int m3 = 4 * horizon_;

  // m4 : control inequality constraints
  int m4 = 2 * horizon_;

  // m5 : sampling time inequality constraints
  int m5 = horizon_;

  int num_of_variables = n1 + n2 + n3 + n4;
  int num_of_constraints = m1 + m2 + m3 + m4 + m5;

  // TODO(QiL) : evaluate whether need to new it everytime
  DistanceApproachIPOPTInterface* ptop = new DistanceApproachIPOPTInterface(
      num_of_variables, num_of_constraints, horizon_, ts_, ego_, x0_, xF_,
      XYbounds_, vOb_, nOb_);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetStringValue("hessian_approximation", "exact");
  // TODO(QiL) : Change IPOPT settings to flag or configs
  int print_level = 0;
  app->Options()->SetIntegerValue("print_level", print_level);
  int num_iterations = 0;
  app->Options()->SetIntegerValue("max_iter", num_iterations);
  int mumps_mem_percent = 6000;
  app->Options()->SetIntegerValue("mumps_mem_percent", mumps_mem_percent);
  int max_iter = 750;
  app->Options()->SetIntegerValue("max_iter", max_iter);
  double tol = 1e-5;
  app->Options()->SetNumericValue("tol", tol);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    AERROR << "*** Distiance Approach problem error during initialization!";
    return static_cast<int>(status);
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    ADEBUG << "*** The problem solved in " << iter_count << " iterations!";

    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    ADEBUG << "*** The final value of the objective function is " << final_obj
           << '.';
  } else {
    ADEBUG << "Return status: " << int(status);
  }

  ptop->get_optimization_results(state_result, control_result, time_result);

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

}  // namespace planning
}  // namespace apollo
