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
 * @file distance_approach_problem_interface.cc
 */

#include "modules/planning/open_space/distance_approach_ipopt_interface.h"

#include <math.h>
#include <utility>

#include "cybertron/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

constexpr double dmin = 0.05;

DistanceApproachIPOPTInterface::DistanceApproachIPOPTInterface(
    const int num_of_variables, const int num_of_constraints,
    std::size_t horizon, float ts, Eigen::MatrixXd ego,
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const Eigen::MatrixXd& timeWS, Eigen::MatrixXd x0, Eigen::MatrixXd xf,
    Eigen::MatrixXd XYbounds, Eigen::MatrixXd obstacles_vertices_num,
    std::size_t obstacles_num)
    : num_of_variables_(num_of_variables),
      num_of_constraints_(num_of_constraints),
      horizon_(horizon),
      ts_(ts),
      ego_(ego),
      xWS_(xWS),
      uWS_(uWS),
      timeWS_(timeWS),
      x0_(x0),
      xf_(xf),
      XYbounds_(XYbounds),
      obstacles_vertices_num_(obstacles_vertices_num),
      obstacles_num_(obstacles_num) {
  w_ev_ = ego_(1, 0) + ego_(3, 0);
  l_ev_ = ego_(0, 0) + ego_(2, 0);

  g_ = {l_ev_ / 2, w_ev_ / 2, l_ev_ / 2, w_ev_ / 2};
  offset_ = (ego_(0, 0) + ego_(2, 0)) / 2 - ego_(2, 0);
  obstacles_vertices_sum_ = std::size_t(obstacles_vertices_num_.sum());
  state_result_ = Eigen::MatrixXd::Zero(horizon_ + 1, 4);
  control_result_ = Eigen::MatrixXd::Zero(horizon_ + 1, 2);
  time_result_ = Eigen::MatrixXd::Zero(horizon_ + 1, 1);
}

/*
  num_of_variables, num_of_constraints, horizon_, x0_, xF_, XYbounds_
  */

void DistanceApproachIPOPTInterface::set_objective_weights(
    const DistanceApproachConfig& distance_approach_config) {
  distance_approach_config_.CopyFrom(distance_approach_config);
}

bool DistanceApproachIPOPTInterface::get_nlp_info(int& n, int& m,
                                                  int& nnz_jac_g,
                                                  int& nnz_h_lag,
                                                  IndexStyleEnum& index_style) {
  // number of variables
  n = num_of_variables_;

  // number of constraints
  m = num_of_constraints_;

  // number of nonzero hessian and lagrangian.
  nnz_jac_g = 0;

  nnz_h_lag = 0;

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool DistanceApproachIPOPTInterface::get_bounds_info(int n, double* x_l,
                                                     double* x_u, int m,
                                                     double* g_l, double* g_u) {
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  CHECK(n == num_of_variables_) << "num_of_variables_ mismatch, n: " << n
                                << ", num_of_variables_: " << num_of_variables_;
  CHECK(m == num_of_constraints_)
      << "num_of_constraints_ mismatch, n: " << n
      << ", num_of_constraints_: " << num_of_constraints_;

  // Variables: includes state, u, sample time and lagrange multipliers
  // 1. state variables, 4 * (N+1)
  // start point pose
  std::size_t variable_index = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    x_l[i] = x0_(i, 0);
    x_u[i] = x0_(i, 0);
  }
  variable_index += 4;

  // During horizons, 2 ~ N-1
  for (std::size_t i = 2; i <= horizon_ - 1; ++i) {
    // x
    x_l[variable_index] = XYbounds_(0, 0);
    x_u[variable_index] = XYbounds_(1, 0);

    // y
    x_l[variable_index + 1] = XYbounds_(2, 0);
    x_u[variable_index + 1] = XYbounds_(3, 0);

    // phi
    // TODO(QiL): Change this to configs
    x_l[variable_index + 2] = -7;
    x_u[variable_index + 2] = 7;

    // v
    // TODO(QiL) : Change this to configs
    x_l[variable_index + 3] = -1;
    x_u[variable_index + 3] = 2;

    variable_index += 4;
  }

  // end point pose
  for (std::size_t i = 0; i < 4; ++i) {
    x_l[variable_index + i] = xf_(i, 0);
    x_u[variable_index + i] = xf_(i, 0);
  }
  variable_index += 4;
  ADEBUG << "variable_index after adding state variables : " << variable_index;

  // 2. control varialbles, 2 * (1 ~ N)
  for (std::size_t i = 1; i < horizon_; ++i) {
    // u1
    x_l[variable_index] = -0.6;
    x_u[variable_index] = 0.6;

    // u2
    x_l[variable_index + 1] = -1;
    x_u[variable_index + 1] = 1;

    variable_index += 2;
  }
  ADEBUG << "variable_index after adding control variables : "
         << variable_index;

  // 3. sampling time variables, 1 ~ N + 1
  for (std::size_t i = 1; i <= horizon_ + 1; ++i) {
    x_l[variable_index] = -0.6;
    x_u[variable_index] = 0.6;

    ++variable_index;
  }

  ADEBUG << "variable_index after adding sample time : " << variable_index;

  // 4. lagrange constraint l, obstacles_vertices_sum_ * (N+1)
  for (std::size_t i = 1; i <= horizon_ + 1; ++i) {
    for (std::size_t j = 1; j <= obstacles_vertices_sum_; ++j) {
      x_l[i * obstacles_vertices_sum_ + j] = 0.0;
      // TODO(QiL): refine this variables limits
      x_u[i * obstacles_vertices_sum_ + j] = 100.0;

      ++variable_index;
    }
  }

  ADEBUG << "variable_index after adding lagrange l : " << variable_index;

  // 4. lagrange constraint m, 4*obstacles_num * (horizon_+1)
  for (std::size_t i = 1; i <= horizon_ + 1; ++i) {
    for (std::size_t j = 1; j <= 4 * obstacles_num_; ++j) {
      x_l[i * 4 * obstacles_num_ + j] = 0.0;
      // TODO(QiL): refine this variables limits
      x_u[i * 4 * obstacles_num_ + j] = 100.0;

      ++variable_index;
    }
  }

  ADEBUG << "variable_index after adding lagrange m : " << variable_index;

  // Constraints: includes state, u, sample time and lagrange multipliers
  // constraints

  // 1. state constraints 4 * (N+1)
  // start point pose
  std::size_t constraint_index = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    g_l[i] = x0_(i, 0);
    g_u[i] = x0_(i, 0);
  }
  constraint_index += 4;

  // During horizons 2 ~ (N-1)
  for (std::size_t i = 1; i < horizon_ - 1; ++i) {
    // x
    g_l[constraint_index] = XYbounds_(0, 0);
    g_u[constraint_index] = XYbounds_(1, 0);

    // y
    g_l[constraint_index + 1] = XYbounds_(2, 0);
    g_u[constraint_index + 1] = XYbounds_(3, 0);

    // phi
    // TODO(QiL): Change this to configs
    g_l[constraint_index + 2] = -7;
    g_u[constraint_index + 2] = 7;

    // v
    // TODO(QiL) : Change this to configs
    g_l[constraint_index + 3] = -1;
    g_u[constraint_index + 3] = 2;

    constraint_index += 4;
  }

  // end point pose
  for (std::size_t i = 0; i < 4; ++i) {
    g_l[constraint_index + i] = xf_(i, 0);
    g_u[constraint_index + i] = xf_(i, 0);
  }
  constraint_index += 4;
  ADEBUG << "constraint_index after adding state constraints : "
         << constraint_index;

  // 2. input constraints 2 * (1~N)
  for (std::size_t i = 1; i < horizon_; ++i) {
    // u1
    g_l[constraint_index] = -0.6;
    g_u[constraint_index] = 0.6;

    // u2
    g_l[constraint_index + 1] = -1;
    g_u[constraint_index + 1] = 1;

    constraint_index += 2;
  }
  ADEBUG << "constraint_index after adding input constraints : "
         << constraint_index;

  // 3. sampling time constraints  (1~N+1)
  for (std::size_t i = 1; i < horizon_ + 1; ++i) {
    g_l[constraint_index] = -0.6;
    g_u[constraint_index] = 0.6;

    ++constraint_index;
  }
  ADEBUG << "constraint_index after adding sampling time constraints : "
         << constraint_index;

  // 3. lagrangian constraints l, obstacles_vertices_sum_ * (N+1)
  for (std::size_t i = 1; i < horizon_ + 1; ++i) {
    for (std::size_t j = 1; j <= obstacles_vertices_sum_; ++j) {
      g_l[i * obstacles_vertices_sum_ + j] = 0.0;
      // TODO(QiL) : Check constraint index 100 after running senarios.
      g_u[i * obstacles_vertices_sum_ + j] = 100.0;

      ++constraint_index;
    }
  }

  ADEBUG << "constraint_index after adding lagrangian constraint l: "
         << constraint_index;

  // 4. lagrangian constraints n, 4 * obstacles_num_ * (N+1)
  for (std::size_t i = 1; i < horizon_ + 1; ++i) {
    for (std::size_t j = 1; j <= 4 * obstacles_num_; ++j) {
      g_l[i * 4 * obstacles_num_ + j] = 0.0;
      // TODO(QiL) : Check constraint index 100 back after running the
      // senarios.
      g_u[i * 4 * obstacles_num_ + j] = 100.0;

      ++constraint_index;
    }
  }

  ADEBUG << "constraint_index after adding lagrangian constraints n : "
         << constraint_index;
  return true;
}

bool DistanceApproachIPOPTInterface::eval_g(int n, const double* x, bool new_x,
                                            int m, double* g) {
  // state start index.
  std::size_t state_start_index = 0;

  // control start index.
  std::size_t control_start_index = 4 * (horizon_ + 1);

  // sampling time start index.
  std::size_t time_start_index = 4 * (horizon_ + 1) + 2 * horizon_;
  for (std::size_t i = 0; i < horizon_; ++i) {
    // x1
    // TODO(QiL) : change to sin table
    g[state_start_index + 4] =
        g[state_start_index] + g[time_start_index] * ts_ *
                                   g[state_start_index + 3] *
                                   std::cos(g[state_start_index + 2]);
    // x2
    g[state_start_index + 5] =
        g[state_start_index + 1] + g[time_start_index] * ts_ *
                                       g[state_start_index + 3] *
                                       std::sin(g[state_start_index + 2]);
    // x3
    g[state_start_index + 6] =
        g[state_start_index + 2] +
        g[time_start_index] * ts_ * g[state_start_index + 3] *
            std::tan(g[control_start_index] / wheelbase_);

    // x4
    g[state_start_index + 7] =
        g[state_start_index + 3] +
        g[time_start_index] * ts_ * g[control_start_index + 1];

    // sampling time
    g[time_start_index + 1] = g[time_start_index];

    state_start_index += 4;
    control_start_index += 2;
    ++time_start_index;
  }

  // Next evaluate and iterate over time steps & obstacles
  // TODO(QiL): two iterative loops
  return true;
}

bool DistanceApproachIPOPTInterface::get_starting_point(
    int n, bool init_x, double* x, bool init_z, double* z_L, double* z_U, int m,
    bool init_lambda, double* lambda) {
  CHECK(n == num_of_variables_)
      << "No. of variables wrong in get_starting_point. n : " << n;
  CHECK(init_x == true) << "Warm start init_x setting failed";
  CHECK(init_z == false) << "Warm start init_z setting failed";
  CHECK(init_lambda == false) << "Warm start init_lambda setting failed";

  std::size_t variable_index = 0;

  // 1. state variables 4 * (horizon_ + 1)
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < 4; ++j) {
      x[i * 4 + j] = xWS_(j, i);
    }
    variable_index += 4;
  }

  // 2. control variable initialization, 2 * N
  for (std::size_t i = 1; i <= horizon_; ++i) {
    x[variable_index + i] = uWS_(0, i - 1);
    x[variable_index + i + 1] = uWS_(1, i - 1);
    variable_index += 2;
  }

  // TODO(QiL) : better hot start l
  // 3. lagrange constraint l, obstacles_vertices_sum_ * (N+1)
  for (std::size_t i = 1; i <= horizon_ + 1; ++i) {
    for (std::size_t j = 1; j <= obstacles_vertices_sum_; ++j) {
      x[i * obstacles_vertices_sum_ + j] = 0.2;
      ++variable_index;
    }
  }

  // TODO(QiL) : better hot start m
  // 4. lagrange constraint m, 4*obstacles_num * (horizon_+1)
  for (std::size_t i = 1; i <= horizon_ + 1; ++i) {
    for (std::size_t j = 1; j <= 4 * obstacles_num_; ++j) {
      x[i * 4 * obstacles_num_ + j] = 0.2;
      ++variable_index;
    }
  }

  ADEBUG << "variable index after initialization done : " << variable_index;

  return true;
}

bool DistanceApproachIPOPTInterface::eval_jac_g(int n, const double* x,
                                                bool new_x, int m, int nele_jac,
                                                int* iRow, int* jCol,
                                                double* values) {
  return true;
}

bool DistanceApproachIPOPTInterface::eval_h(int n, const double* x, bool new_x,
                                            double obj_factor, int m,
                                            const double* lambda,
                                            bool new_lambda, int nele_hess,
                                            int* iRow, int* jCol,
                                            double* values) {
  return true;
}

bool DistanceApproachIPOPTInterface::eval_f(int n, const double* x, bool new_x,
                                            double& obj_value) {
  // Objective is :
  // min control inputs
  // min input rate
  // min time
  // regularization wrt warm start traectory

  std::size_t control_start_index = (horizon_ + 1) * 4;
  // std::size_t time_start_index = control_start_index + horizon_ * 2;

  // TODO(QiL): Initial impelementation towards earier understanding and debug
  // purpose, later code refine towards improving efficiency

  DCHECK(ts_ != 0) << "ts in distance_approach_ is 0";
  // 1. objective to minimize u square
  for (std::size_t i = 0; i < horizon_; ++i) {
    obj_value += distance_approach_config_.weight_u(0) *
                     x[control_start_index + i] * x[control_start_index + i] +
                 distance_approach_config_.weight_u(1) *
                     x[control_start_index + i] *
                     x[control_start_index + i + 1];
  }

  // 2. ojective to monimize input change rate, first horizon
  obj_value += distance_approach_config_.weight_u_rate(0) *
                   (x[control_start_index] / ts_) *
                   (x[control_start_index] / ts_) +
               distance_approach_config_.weight_u_rate(1) *
                   (x[control_start_index + 1] / ts_) *
                   (x[control_start_index + 1] / ts_);

  // 3. objective to minimize input change rates, 1 ~ horizone -1
  for (std::size_t i = 1; i < horizon_ - 1; ++i) {
    double u1_rate =
        (x[control_start_index + i + 2] - x[control_start_index + i]) / ts_;
    double u2_rate =
        (x[control_start_index + i + 3] - x[control_start_index + i + 1]) / ts_;
    obj_value +=
        distance_approach_config_.weight_u_rate(0) * u1_rate * u1_rate +
        distance_approach_config_.weight_u_rate(1) * u2_rate * u2_rate;
  }

  // 4. objective to minimize state diff to warm up
  for (std::size_t i = 0; i < horizon_; ++i) {
    double x1_diff = x[4 * i] - xWS_(0, i);
    double x2_diff = x[4 * i + 1] - xWS_(1, i);
    double x3_diff = x[4 * i + 2] - xWS_(2, i);
    obj_value += distance_approach_config_.weight_state(0) * x1_diff * x1_diff +
                 distance_approach_config_.weight_state(1) * x2_diff * x2_diff +
                 distance_approach_config_.weight_state(2) * x3_diff * x3_diff;
  }

  return true;
}

bool DistanceApproachIPOPTInterface::eval_grad_f(int n, const double* x,
                                                 bool new_x, double* grad_f) {
  return true;
}

void DistanceApproachIPOPTInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  // std::size_t state_start_index = 0;
  // std::size_t input_start_index = (horizon_ + 1) * 4;
  // std::size_t time_start_index = input_start_index + horizon_ * 2;

  for (std::size_t i = 0; i < horizon_; ++i) {
    std::size_t state_index = i * 4;

    state_result_(i, 0) = x[state_index];
    state_result_(i, 1) = x[state_index + 1];
    state_result_(i, 2) = x[state_index + 2];
    state_result_(i, 3) = x[state_index + 3];

    std::size_t control_index = (horizon_ + 1) * 4 + i * 2;
    control_result_(i, 0) = x[control_index];
    control_result_(i, 1) = x[control_index + 1];

    std::size_t time_index = (horizon_ + 1) * 4 + horizon_ * 2 + i;
    time_result_(i, 0) = x[time_index];
  }
  // push back state for N+1
  state_result_(4 * horizon_, 0) = x[4 * horizon_];
  state_result_(4 * horizon_, 1) = x[4 * horizon_ + 1];
  state_result_(4 * horizon_, 2) = x[4 * horizon_ + 2];
  state_result_(4 * horizon_, 3) = x[4 * horizon_ + 3];
}

void DistanceApproachIPOPTInterface::get_optimization_results(
    Eigen::MatrixXd* state_result, Eigen::MatrixXd* control_result,
    Eigen::MatrixXd* time_result) const {
  *state_result = state_result_;
  *control_result = control_result_;
  *time_result = time_result_;
}
}  // namespace planning
}  // namespace apollo
