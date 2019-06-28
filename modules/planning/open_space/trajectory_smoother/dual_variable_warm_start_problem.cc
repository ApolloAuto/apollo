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
  planner_open_space_config_ = planner_open_space_config;
}

bool DualVariableWarmStartProblem::Solve(
    const size_t horizon, const double ts, const Eigen::MatrixXd& ego,
    size_t obstacles_num, const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const Eigen::MatrixXd& xWS, Eigen::MatrixXd* l_warm_up,
    Eigen::MatrixXd* n_warm_up) {
  auto t_start = cyber::Time::Now().ToSecond();
  bool solver_flag = false;

  if (planner_open_space_config_.dual_variable_warm_start_config()
          .qp_format() == OSQP) {
    DualVariableWarmStartOSQPInterface* ptop =
        new DualVariableWarmStartOSQPInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, planner_open_space_config_);
    bool succ = ptop->optimize();

    if (succ) {
      ADEBUG << "dual warm up done.";
      ptop->get_optimization_results(l_warm_up, n_warm_up);

      auto t_end = cyber::Time::Now().ToSecond();
      ADEBUG << "Dual variable warm start solving time in second : "
             << t_end - t_start;

      solver_flag = true;
    } else {
      AWARN << "dual warm up fail.";
      ptop->get_optimization_results(l_warm_up, n_warm_up);
      solver_flag = false;
    }
  } else if (planner_open_space_config_.dual_variable_warm_start_config()
                 .qp_format() == IPOPTQP) {
    DualVariableWarmStartIPOPTQPInterface* ptop =
        new DualVariableWarmStartIPOPTQPInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, planner_open_space_config_);

    Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;
    // Create an instance of the IpoptApplication
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetIntegerValue(
        "print_level",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_print_level());
    app->Options()->SetIntegerValue(
        "mumps_mem_percent",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .mumps_mem_percent());
    app->Options()->SetNumericValue(
        "mumps_pivtol",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .mumps_pivtol());
    app->Options()->SetIntegerValue(
        "max_iter", planner_open_space_config_.dual_variable_warm_start_config()
                        .ipopt_config()
                        .ipopt_max_iter());
    app->Options()->SetNumericValue(
        "tol", planner_open_space_config_.dual_variable_warm_start_config()
                   .ipopt_config()
                   .ipopt_tol());
    app->Options()->SetNumericValue(
        "acceptable_constr_viol_tol",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_acceptable_constr_viol_tol());
    app->Options()->SetNumericValue(
        "min_hessian_perturbation",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_min_hessian_perturbation());
    app->Options()->SetNumericValue(
        "jacobian_regularization_value",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_jacobian_regularization_value());
    app->Options()->SetStringValue(
        "print_timing_statistics",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_print_timing_statistics());
    app->Options()->SetStringValue(
        "alpha_for_y",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_alpha_for_y());
    app->Options()->SetStringValue(
        "recalc_y", planner_open_space_config_.dual_variable_warm_start_config()
                        .ipopt_config()
                        .ipopt_recalc_y());
    // for qp problem speed up
    app->Options()->SetStringValue("mehrotra_algorithm", "yes");

    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
      AERROR << "*** Dual variable wart start problem error during "
                "initialization!";
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
      auto t_end = cyber::Time::Now().ToSecond();

      ADEBUG << "Dual variable warm start solving time in second : "
             << t_end - t_start;
    } else {
      ADEBUG << "Solve not succeeding, return status: " << int(status);
    }

    ptop->get_optimization_results(l_warm_up, n_warm_up);

    solver_flag = (status == Ipopt::Solve_Succeeded ||
                   status == Ipopt::Solved_To_Acceptable_Level);
  } else if (planner_open_space_config_.dual_variable_warm_start_config()
                 .qp_format() == IPOPT) {
    DualVariableWarmStartIPOPTInterface* ptop =
        new DualVariableWarmStartIPOPTInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, planner_open_space_config_);

    Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;
    // Create an instance of the IpoptApplication
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetIntegerValue(
        "print_level",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_print_level());
    app->Options()->SetIntegerValue(
        "mumps_mem_percent",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .mumps_mem_percent());
    app->Options()->SetNumericValue(
        "mumps_pivtol",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .mumps_pivtol());
    app->Options()->SetIntegerValue(
        "max_iter", planner_open_space_config_.dual_variable_warm_start_config()
                        .ipopt_config()
                        .ipopt_max_iter());
    app->Options()->SetNumericValue(
        "tol", planner_open_space_config_.dual_variable_warm_start_config()
                   .ipopt_config()
                   .ipopt_tol());
    app->Options()->SetNumericValue(
        "acceptable_constr_viol_tol",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_acceptable_constr_viol_tol());
    app->Options()->SetNumericValue(
        "min_hessian_perturbation",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_min_hessian_perturbation());
    app->Options()->SetNumericValue(
        "jacobian_regularization_value",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_jacobian_regularization_value());
    app->Options()->SetStringValue(
        "print_timing_statistics",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_print_timing_statistics());
    app->Options()->SetStringValue(
        "alpha_for_y",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_alpha_for_y());
    app->Options()->SetStringValue(
        "recalc_y", planner_open_space_config_.dual_variable_warm_start_config()
                        .ipopt_config()
                        .ipopt_recalc_y());

    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
      AERROR << "*** Dual variable wart start problem error during "
                "initialization!";
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
      auto t_end = cyber::Time::Now().ToSecond();

      ADEBUG << "Dual variable warm start solving time in second : "
             << t_end - t_start;
    } else {
      ADEBUG << "Solve not succeeding, return status: " << int(status);
    }

    ptop->get_optimization_results(l_warm_up, n_warm_up);

    solver_flag = (status == Ipopt::Solve_Succeeded ||
                   status == Ipopt::Solved_To_Acceptable_Level);
  } else {  // debug mode
    DualVariableWarmStartOSQPInterface* ptop_osqp =
        new DualVariableWarmStartOSQPInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, planner_open_space_config_);
    bool succ = ptop_osqp->optimize();

    if (!succ) {
      AERROR << "dual warm up fail.";
      ptop_osqp->get_optimization_results(l_warm_up, n_warm_up);
    }

    ADEBUG << "dual warm up done.";
    ptop_osqp->get_optimization_results(l_warm_up, n_warm_up);

    auto t_end = cyber::Time::Now().ToSecond();
    ADEBUG << "Dual variable warm start solving time in second : "
           << t_end - t_start;

    // ipoptqp result
    Eigen::MatrixXd l_warm_up_ipoptqp(l_warm_up->rows(), l_warm_up->cols());
    Eigen::MatrixXd n_warm_up_ipoptqp(n_warm_up->rows(), n_warm_up->cols());

    DualVariableWarmStartIPOPTQPInterface* ptop_ipoptqp =
        new DualVariableWarmStartIPOPTQPInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, planner_open_space_config_);

    Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop_ipoptqp;
    // Create an instance of the IpoptApplication
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetIntegerValue(
        "print_level",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_print_level());
    app->Options()->SetIntegerValue(
        "mumps_mem_percent",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .mumps_mem_percent());
    app->Options()->SetNumericValue(
        "mumps_pivtol",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .mumps_pivtol());
    app->Options()->SetIntegerValue(
        "max_iter", planner_open_space_config_.dual_variable_warm_start_config()
                        .ipopt_config()
                        .ipopt_max_iter());
    app->Options()->SetNumericValue(
        "tol", planner_open_space_config_.dual_variable_warm_start_config()
                   .ipopt_config()
                   .ipopt_tol());
    app->Options()->SetNumericValue(
        "acceptable_constr_viol_tol",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_acceptable_constr_viol_tol());
    app->Options()->SetNumericValue(
        "min_hessian_perturbation",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_min_hessian_perturbation());
    app->Options()->SetNumericValue(
        "jacobian_regularization_value",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_jacobian_regularization_value());
    app->Options()->SetStringValue(
        "print_timing_statistics",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_print_timing_statistics());
    app->Options()->SetStringValue(
        "alpha_for_y",
        planner_open_space_config_.dual_variable_warm_start_config()
            .ipopt_config()
            .ipopt_alpha_for_y());
    app->Options()->SetStringValue(
        "recalc_y", planner_open_space_config_.dual_variable_warm_start_config()
                        .ipopt_config()
                        .ipopt_recalc_y());
    // for qp problem speed up
    app->Options()->SetStringValue("mehrotra_algorithm", "yes");

    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
      AERROR << "*** Dual variable wart start problem error during "
                "initialization!";
      return false;
    }

    status = app->OptimizeTNLP(problem);

    if (status == Ipopt::Solve_Succeeded ||
        status == Ipopt::Solved_To_Acceptable_Level) {
      // Retrieve some statistics about the solve
      Ipopt::Index iter_count = app->Statistics()->IterationCount();
      ADEBUG << "*** IPOPTQP: The problem solved in " << iter_count
             << " iterations!";
      Ipopt::Number final_obj = app->Statistics()->FinalObjective();
      ADEBUG << "*** IPOPTQP: The final value of the objective function is "
             << final_obj << '.';
    } else {
      ADEBUG << "Solve not succeeding, return status: " << int(status);
    }

    ptop_ipoptqp->get_optimization_results(&l_warm_up_ipoptqp,
                                           &n_warm_up_ipoptqp);

    // ipopt result
    Eigen::MatrixXd l_warm_up_ipopt(l_warm_up->rows(), l_warm_up->cols());
    Eigen::MatrixXd n_warm_up_ipopt(n_warm_up->rows(), n_warm_up->cols());

    DualVariableWarmStartIPOPTInterface* ptop_ipopt =
        new DualVariableWarmStartIPOPTInterface(
            horizon, ts, ego, obstacles_edges_num, obstacles_num, obstacles_A,
            obstacles_b, xWS, planner_open_space_config_);

    problem = ptop_ipopt;
    // Create an instance of the IpoptApplication

    status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
      AERROR << "*** Dual variable wart start problem error during "
                "initialization!";
      return false;
    }

    status = app->OptimizeTNLP(problem);
    if (status == Ipopt::Solve_Succeeded ||
        status == Ipopt::Solved_To_Acceptable_Level) {
      // Retrieve some statistics about the solve
      Ipopt::Index iter_count = app->Statistics()->IterationCount();
      ADEBUG << "*** IPOPT: The problem solved in " << iter_count
             << " iterations!";
      Ipopt::Number final_obj = app->Statistics()->FinalObjective();
      ADEBUG << "*** IPOPT: The final value of the objective function is "
             << final_obj << '.';
    } else {
      ADEBUG << "Solve not succeeding, return status: " << int(status);
    }

    ptop_ipopt->get_optimization_results(&l_warm_up_ipopt, &n_warm_up_ipopt);

    // compare three results
    double l_max_diff1 = 0.0;
    double l_max_diff2 = 0.0;
    double l_max_diff3 = 0.0;
    for (int c = 0; c < l_warm_up->cols(); ++c) {
      for (int r = 0; r < l_warm_up->rows(); ++r) {
        l_max_diff1 = std::max(l_max_diff1, std::abs(l_warm_up->coeff(r, c) -
                                                     l_warm_up_ipopt(r, c)));
        l_max_diff2 = std::max(l_max_diff2, std::abs(l_warm_up->coeff(r, c) -
                                                     l_warm_up_ipoptqp(r, c)));
        l_max_diff3 = std::max(l_max_diff3, std::abs(l_warm_up_ipoptqp(r, c) -
                                                     l_warm_up_ipopt(r, c)));
      }
    }
    ADEBUG << "max l warm up diff between osqp & ipopt: " << l_max_diff1;
    ADEBUG << "max l warm up diff between osqp & ipoptqp: " << l_max_diff2;
    ADEBUG << "max l warm up diff between ipopt & ipoptqp: " << l_max_diff3;

    double n_max_diff1 = 0.0;
    double n_max_diff2 = 0.0;
    double n_max_diff3 = 0.0;
    for (int c = 0; c < n_warm_up->cols(); ++c) {
      for (int r = 0; r < n_warm_up->rows(); ++r) {
        n_max_diff1 = std::max(n_max_diff1, std::abs(n_warm_up->coeff(r, c) -
                                                     n_warm_up_ipopt(r, c)));
        n_max_diff2 = std::max(n_max_diff2, std::abs(n_warm_up->coeff(r, c) -
                                                     n_warm_up_ipoptqp(r, c)));
        n_max_diff3 = std::max(n_max_diff3, std::abs(n_warm_up_ipoptqp(r, c) -
                                                     n_warm_up_ipopt(r, c)));
      }
    }
    ADEBUG << "max n warm up diff between osqp & ipopt: " << n_max_diff1;
    ADEBUG << "max n warm up diff between osqp & ipoptqp: " << n_max_diff2;
    ADEBUG << "max n warm up diff between ipopt & ipoptqp: " << n_max_diff3;

    return true;
  }

  if (solver_flag == false) {
    // if solver fails during dual warm up, insert zeros instead
    for (int r = 0; r < l_warm_up->rows(); ++r) {
      for (int c = 0; c < l_warm_up->cols(); ++c) {
        (*l_warm_up)(r, c) = 0.0;
      }
    }

    for (int r = 0; r < n_warm_up->rows(); ++r) {
      for (int c = 0; c < n_warm_up->cols(); ++c) {
        (*n_warm_up)(r, c) = 0.0;
      }
    }
  }

  return true;
}
}  // namespace planning
}  // namespace apollo
