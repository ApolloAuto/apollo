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
 * warm_start_problem.cc
 */

#include "modules/planning/planner/open_space/warm_start_problem.h"

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

WarmStartProblem::WarmStartProblem() {}

bool WarmStartProblem::Solve() const {
  WarmUpIPOPTInterface* ptop = new WarmUpIPOPTInterface();

  ptop->set_start_point();

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  //  app->Options()->SetStringValue("jacobian_approximation",
  //  "finite-difference-values");
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  //  app->Options()->SetStringValue("derivative_test", "first-order");
  //  app->Options()->SetNumericValue("derivative_test_perturbation", 1.0e-7);
  //  app->Options()->SetStringValue("derivative_test", "second-order");
  app->Options()->SetIntegerValue("print_level", 0);
  int num_iterations = 0;
  app->Options()->SetIntegerValue("max_iter", num_iterations);

  //  app->Options()->SetNumericValue("acceptable_tol", 0.5);
  //  app->Options()->SetNumericValue("acceptable_obj_change_tol", 0.5);
  //  app->Options()->SetNumericValue("constr_viol_tol", 0.01);
  //  app->Options()->SetIntegerValue("acceptable_iter", 10);
  //  app->Options()->SetIntegerValue("print_level", 0);
  //  app->Options()->SetStringValue("fast_step_computation", "yes");

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    ADEBUG << "*** Error during initialization!";
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

  ptop->get_optimization_results();

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}  // namespace planning

}  // namespace planning
}  // namespace apollo
