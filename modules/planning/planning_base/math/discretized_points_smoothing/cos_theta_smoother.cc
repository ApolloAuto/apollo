/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_base/math/discretized_points_smoothing/cos_theta_smoother.h"

#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/math/discretized_points_smoothing/cos_theta_ipopt_interface.h"

namespace apollo {
namespace planning {
CosThetaSmoother::CosThetaSmoother(const CosThetaSmootherConfig& config)
    : config_(config) {}

bool CosThetaSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  const double weight_cos_included_angle = config_.weight_cos_included_angle();
  const double weight_anchor_points = config_.weight_anchor_points();
  const double weight_length = config_.weight_length();
  const size_t print_level = config_.print_level();
  const size_t max_num_of_iterations = config_.max_num_of_iterations();
  const size_t acceptable_num_of_iterations =
      config_.acceptable_num_of_iterations();
  const double tol = config_.tol();
  const double acceptable_tol = config_.acceptable_tol();
  const bool use_automatic_differentiation =
      config_.ipopt_use_automatic_differentiation();

  CosThetaIpoptInterface* smoother =
      new CosThetaIpoptInterface(raw_point2d, bounds);

  smoother->set_weight_cos_included_angle(weight_cos_included_angle);
  smoother->set_weight_anchor_points(weight_anchor_points);
  smoother->set_weight_length(weight_length);
  smoother->set_automatic_differentiation_flag(use_automatic_differentiation);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", static_cast<int>(print_level));
  app->Options()->SetIntegerValue("max_iter",
                                  static_cast<int>(max_num_of_iterations));
  app->Options()->SetIntegerValue(
      "acceptable_iter", static_cast<int>(acceptable_num_of_iterations));
  app->Options()->SetNumericValue("tol", tol);
  app->Options()->SetNumericValue("acceptable_tol", acceptable_tol);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    AERROR << "*** Error during initialization!";
    return false;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    ADEBUG << "*** The problem solved in " << iter_count << " iterations!";
  } else {
    AERROR << "Solver fails with return code: " << static_cast<int>(status);
    return false;
  }
  smoother->get_optimization_results(opt_x, opt_y);
  return true;
}
}  // namespace planning
}  // namespace apollo
