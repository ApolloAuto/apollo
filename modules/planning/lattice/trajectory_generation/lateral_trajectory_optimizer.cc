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

/**
 * @file
 **/

#include "modules/planning/lattice/trajectory_generation/lateral_trajectory_optimizer.h"

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/trajectory_generation/lateral_trajectory_optimizer_interface.h"
#include "modules/planning/lattice/trajectory1d/constant_jerk_trajectory1d.h"

namespace apollo {
namespace planning {

bool LateralTrajectoryOptimizer::optimize(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& lateral_bounds) {
  delta_s_ = delta_s;

  auto ptr_interface = new LateralTrajectoryOptimizerInterface(
      d_state[0], d_state[1], d_state[2], delta_s,
      FLAGS_lateral_third_order_derivative_max, lateral_bounds);

  ptr_interface->set_objective_weights(
      FLAGS_weight_lateral_offset,
      FLAGS_weight_lateral_derivative,
      FLAGS_weight_lateral_second_order_derivative,
      FLAGS_weight_lateral_obstacle_distance);


  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptr_interface;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  // app->Options()->SetIntegerValue("print_level", 0);
  app->Options()->SetIntegerValue("max_iter", 100);
  app->Options()->SetIntegerValue("acceptable_iter", 5);
  app->Options()->SetNumericValue("tol", 1.0e-3);
  app->Options()->SetNumericValue("acceptable_tol", 1.0e-3);
//  app->Options()->SetStringValue("mehrotra_algorithm", "yes");

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    ADEBUG << "*** Error during initialization!";
    return false;
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

  ptr_interface->GetOptimizationResult(&opt_d_, &opt_d_prime_, &opt_d_pprime_);

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

PiecewiseJerkTrajectory1d
LateralTrajectoryOptimizer::GetOptimalTrajectory() const {
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
