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

#include "modules/planning/open_space/distance_approach_ipopt_interface.h"

namespace apollo {
namespace planning {

DistanceApproachIPOPTInterface::DistanceApproachIPOPTInterface(
    const int num_of_variables, const int num_of_constraints,
    std::size_t horizon, float ts, Eigen::MatrixXd ego,
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const Eigen::MatrixXd& timeWS, Eigen::MatrixXd x0, Eigen::MatrixXd xf,
    Eigen::MatrixXd last_time_u, Eigen::MatrixXd XYbounds,
    Eigen::MatrixXd obstacles_edges_num, std::size_t obstacles_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    bool use_fix_time)
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
      last_time_u_(last_time_u),
      XYbounds_(XYbounds),
      obstacles_edges_num_(obstacles_edges_num),
      obstacles_num_(obstacles_num),
      obstacles_A_(obstacles_A),
      obstacles_b_(obstacles_b),
      use_fix_time_(use_fix_time) {
  w_ev_ = ego_(1, 0) + ego_(3, 0);
  l_ev_ = ego_(0, 0) + ego_(2, 0);

  g_ = {l_ev_ / 2, w_ev_ / 2, l_ev_ / 2, w_ev_ / 2};
  offset_ = (ego_(0, 0) + ego_(2, 0)) / 2 - ego_(2, 0);
  obstacles_vertices_sum_ = std::size_t(obstacles_edges_num_.sum());
  state_result_ = Eigen::MatrixXd::Zero(horizon_ + 1, 4);
  control_result_ = Eigen::MatrixXd::Zero(horizon_ + 1, 2);
  time_result_ = Eigen::MatrixXd::Zero(horizon_ + 1, 1);
  state_start_index_ = 0;
  control_start_index_ = 4 * (horizon_ + 1);
  time_start_index_ = control_start_index_ + 2 * horizon_;
  l_start_index_ = time_start_index_ + (horizon_ + 1);
  n_start_index_ = l_start_index_ + obstacles_vertices_sum_ * (horizon_ + 1);

  // TODO(QiL): integrate open_space planner into task config when refactor done
  CHECK(common::util::GetProtoFromFile(FLAGS_planner_open_space_config_filename,
                                       &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;

  distance_approach_config_ =
      planner_open_space_config_.distance_approach_config();
  weight_state_x_ = distance_approach_config_.weight_state(0);
  weight_state_y_ = distance_approach_config_.weight_state(1);
  weight_state_phi_ = distance_approach_config_.weight_state(2);
  weight_input_steer_ = distance_approach_config_.weight_u(0);
  weight_input_a_ = distance_approach_config_.weight_u(1);
  weight_rate_steer_ = distance_approach_config_.weight_u_rate(0);
  weight_rate_a_ = distance_approach_config_.weight_u_rate(1);
  weight_stitching_steer_ = distance_approach_config_.weight_stitching(0);
  weight_stitching_a_ = distance_approach_config_.weight_stitching(1);
  weight_first_order_time_ = distance_approach_config_.weight_time(0);
  weight_second_order_time_ = distance_approach_config_.weight_time(1);
  min_safety_distance_ = distance_approach_config_.min_safety_distance();

  wheelbase_ = vehicle_param_.wheel_base();
}

bool DistanceApproachIPOPTInterface::get_nlp_info(int& n, int& m,
                                                  int& nnz_jac_g,
                                                  int& nnz_h_lag,
                                                  IndexStyleEnum& index_style) {
  AINFO << "get_nlp_info";
  // number of variables
  n = num_of_variables_;
  AINFO << "num_of_variables_ " << num_of_variables_;
  // number of constraints
  m = num_of_constraints_;
  AINFO << "num_of_constraints_ " << num_of_constraints_;

  // number of nonzero hessian and lagrangian.
  int tmp = 0;

  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_num_; ++j) {
      std::size_t current_edges_num = obstacles_edges_num_(j, 0);
      tmp += current_edges_num * 4 + 9 + 4;
    }
  }

  nnz_jac_g = 23 * horizon_ + 3 * horizon_ + 2 * horizon_ + tmp;

  AINFO << "nnz_jac_g_" << nnz_jac_g;

  nnz_h_lag = 14 * num_of_variables_ + 5;

  index_style = IndexStyleEnum::C_STYLE;
  AINFO << "get_nlp_info out";
  return true;
}

bool DistanceApproachIPOPTInterface::get_bounds_info(int n, double* x_l,
                                                     double* x_u, int m,
                                                     double* g_l, double* g_u) {
  AINFO << "get_bounds_info";
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  CHECK(n == num_of_variables_) << "num_of_variables_ mismatch, n: " << n
                                << ", num_of_variables_: " << num_of_variables_;
  CHECK(m == num_of_constraints_)
      << "num_of_constraints_ mismatch, m: " << m
      << ", num_of_constraints_: " << num_of_constraints_;

  // Variables: includes state, u, sample time and lagrange multipliers
  // 1. state variables, 4 * [0, horizon]
  // start point pose
  std::size_t variable_index = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    x_l[i] = x0_(i, 0);
    x_u[i] = x0_(i, 0);
  }
  variable_index += 4;

  // During horizons, 2 ~ N-1
  for (std::size_t i = 1; i < horizon_; ++i) {
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
  AINFO << "variable_index after adding state variables : " << variable_index;

  // 2. control variables, 2 * [0, horizon_-1]
  for (std::size_t i = 0; i < horizon_; ++i) {
    // u1
    x_l[variable_index] = -0.6;
    x_u[variable_index] = 0.6;

    // u2
    x_l[variable_index + 1] = -1;
    x_u[variable_index + 1] = 1;

    variable_index += 2;
  }
  AINFO << "variable_index after adding control variables : " << variable_index;

  // 3. sampling time variables, 1 * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    if (!use_fix_time_) {
      x_l[variable_index] = 0.0;
      x_u[variable_index] = 10.0;
    } else {
      x_l[variable_index] = 1.0;
      x_u[variable_index] = 1.0;
    }

    ++variable_index;
  }
  AINFO << "variable_index after adding sample time : " << variable_index;
  AINFO << "sample time fix time status is : " << use_fix_time_;

  // 4. lagrange constraint l, [0, obstacles_vertices_sum_ - 1] * [0,
  // horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_vertices_sum_; ++j) {
      x_l[variable_index] = 0.0;
      // TODO(QiL): refine this variables limits
      x_u[variable_index] = 100.0;
      ++variable_index;
    }
  }
  AINFO << "variable_index after adding lagrange l : " << variable_index;

  // 4. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < 4 * obstacles_num_; ++j) {
      x_l[variable_index] = 0.0;
      // TODO(QiL): refine this variables limits
      x_u[variable_index] = 100.0;

      ++variable_index;
    }
  }

  AINFO << "variable_index after adding lagrange n : " << variable_index;

  // Constraints: includes four state Euler forward constraints, three
  // Obstacle related constraints

  // 1. dynamics constraints 4 * [0, horizons-1]
  std::size_t constraint_index = 0;
  for (std::size_t i = 0; i < 4 * horizon_; ++i) {
    g_l[i] = 0.0;
    g_u[i] = 0.0;
  }
  constraint_index += 4 * horizon_;

  AINFO << "constraint_index after adding Euler forward dynamics constraints: "
        << constraint_index;

  // 2. Control rate limit constraints, 1 * [0, horizons-1], only apply
  // steering rate as of now
  for (std::size_t i = 0; i < horizon_; ++i) {
    // TODO(QiL) : change this to configs
    g_l[constraint_index] = -0.6;
    g_u[constraint_index] = 0.6;
    ++constraint_index;
  }

  AINFO << "constraint_index after adding steering rate constraints: "
        << constraint_index;

  // 3. Time constraints 1 * [0, horizons-1]
  for (std::size_t i = 0; i < horizon_; ++i) {
    g_l[constraint_index] = 0.0;
    g_u[constraint_index] = 0.0;
    ++constraint_index;
  }

  AINFO << "constraint_index after adding time constraints: "
        << constraint_index;

  // 4. Three obstacles related equal constraints, one equality constraints,
  // [0, horizon_] * [0, obstacles_num_-1] * 4
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_num_; ++j) {
      // a. norm(A'*lambda) = 1
      g_l[constraint_index] = 1.0;
      g_u[constraint_index] = 1.0;

      // b. G'*mu + R'*A*lambda = 0
      g_l[constraint_index + 1] = 0.0;
      g_u[constraint_index + 1] = 0.0;
      g_l[constraint_index + 2] = 0.0;
      g_u[constraint_index + 2] = 0.0;

      // c. -g'*mu + (A*t - b)*lambda > 0
      g_l[constraint_index + 3] = 0.0;
      g_u[constraint_index + 3] = 10000.0;
      constraint_index += 4;
    }
  }

  AINFO << "constraint_index after adding obstacles constraints: "
        << constraint_index;
  AINFO << "get_bounds_info_ out";
  return true;
}

bool DistanceApproachIPOPTInterface::eval_g(int n, const double* x, bool new_x,
                                            int m, double* g) {
  AINFO << "eval_g";
  // state start index
  std::size_t state_index = state_start_index_;

  // control start index.
  std::size_t control_index = control_start_index_;

  // time start index
  std::size_t time_index = time_start_index_;

  std::size_t constraint_index = 0;

  // // 1. state constraints 4 * [0, horizons-1]
  for (std::size_t i = 0; i < horizon_; ++i) {
    // x1
    // TODO(QiL) : optimize and remove redundant calculation in next
    // iteration.
    g[constraint_index] =
        x[state_index + 4] -
        (x[state_index] +
         ts_ * x[time_index] *
             (x[state_index + 3] +
              ts_ * x[time_index] * 0.5 * x[control_index + 1]) *
             (std::cos(x[state_index + 2] +
                       ts_ * x[time_index] * 0.5 * x[state_index + 3] *
                           std::tan(x[control_index] / wheelbase_))));
    // x2
    g[constraint_index + 1] =
        x[state_index + 5] -
        (x[state_index + 1] +
         ts_ * x[time_index] *
             (x[state_index + 3] +
              ts_ * x[time_index] * 0.5 * x[control_index + 1]) *
             std::sin(x[state_index + 3]) *
             std::tan(x[control_index] / wheelbase_));

    // x3
    g[constraint_index + 2] =
        x[state_index + 6] -
        (x[state_index + 2] +
         ts_ * x[time_index] *
             (x[state_index + 3] +
              ts_ * x[time_index] * 0.5 * x[control_index + 1]) *
             std::tan(x[control_index] / wheelbase_));

    // x4
    g[constraint_index + 3] =
        x[state_index + 7] -
        (x[state_index + 3] + ts_ * x[time_index] * x[control_index + 1]);

    control_index += 2;
    constraint_index += 4;
    time_index += 1;
    state_index += 4;
  }

  AINFO << "constraint_index after adding Euler forward dynamics constraints "
           "updated: "
        << constraint_index;

  // 2. Control rate limit constraints, 1 * [0, horizons-1], only apply
  // steering rate as of now
  control_index = control_start_index_;
  time_index = time_start_index_;

  // First rate is compare first with stitch point
  g[constraint_index] =
      (x[control_index] - last_time_u_(0, 0)) / x[time_index] / ts_;
  control_index += 1;
  constraint_index += 1;
  time_index += 1;

  for (std::size_t i = 1; i < horizon_; ++i) {
    g[constraint_index] =
        (x[control_index] - x[control_index - 1]) / x[time_index] / ts_;

    constraint_index += 1;
    control_index += 1;
    time_index += 1;
  }

  // 3. Time constraints 1 * [0, horizons-1]
  time_index = time_start_index_;
  for (std::size_t i = 0; i < horizon_; ++i) {
    g[constraint_index] = x[time_index + 1] - x[time_index];

    constraint_index += 1;
    time_index += 1;
  }

  AINFO << "constraint_index after adding time constraints "
           "updated: "
        << constraint_index;

  // 4. Three obstacles related equal constraints, one equality constraints,
  // [0, horizon_] * [0, obstacles_num_-1] * 4

  state_index = state_start_index_;
  std::size_t l_index = l_start_index_;
  std::size_t n_index = n_start_index_;

  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    int edges_counter = 0;
    for (std::size_t j = 0; j < obstacles_num_; ++j) {
      std::size_t current_edges_num = obstacles_edges_num_(i, 0);
      Eigen::MatrixXd Aj =
          obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
      std::vector<int> lj(&x[l_index], &x[l_index + current_edges_num]);
      std::vector<int> nj(&x[n_index], &x[n_index + 3]);
      Eigen::MatrixXd bj =
          obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

      // norm(A* lambda == 1)
      double tmp1 = 0;
      double tmp2 = 0;
      for (std::size_t k = 0; k < current_edges_num; ++k) {
        // TODO(QiL) : replace this one directly with x
        tmp1 += Aj(k, 0) * x[l_index + k];
        tmp2 += Aj(k, 1) * x[l_index + k];
      }
      g[constraint_index] = tmp1 * tmp1 + tmp2 * tmp2 - 1.0;

      // G' * mu + R' * lambda == 0
      g[constraint_index + 1] = nj[0] - nj[2] +
                                std::cos(x[state_index + 2]) * tmp1 +
                                std::sin(x[state_index + 2]) * tmp2;

      g[constraint_index + 2] = nj[1] - nj[3] -
                                std::sin(x[state_index + 2]) * tmp1 +
                                std::cos(x[state_index + 2]) * tmp2;

      //  -g'*mu + (A*t - b)*lambda > 0
      double tmp3 = 0.0;
      for (std::size_t k = 0; k < 4; ++k) {
        // TODO(QiL) : replace this one directly with x
        tmp3 += -g_[k] * nj[k];
      }

      double tmp4 = 0.0;
      for (std::size_t k = 0; k < current_edges_num; ++k) {
        tmp4 += bj(k, 0) * x[l_index + k];
      }

      g[constraint_index + 3] =
          tmp3 +
          (x[state_index] + std::cos(x[state_index + 2]) * offset_) * tmp1 +
          (x[state_index + 1] + std::sin(x[state_index + 2]) * offset_) * tmp2 -
          tmp4 - min_safety_distance_;

      // Update index
      edges_counter += current_edges_num;
      l_index += current_edges_num;
      n_index += 4;
      constraint_index += 4;
    }
  }

  AINFO << "constraint_index after obstacles avoidance constraints "
           "updated: "
        << constraint_index;

  return true;
}

bool DistanceApproachIPOPTInterface::get_starting_point(
    int n, bool init_x, double* x, bool init_z, double* z_L, double* z_U, int m,
    bool init_lambda, double* lambda) {
  AINFO << "get_starting_point";
  CHECK(n == num_of_variables_)
      << "No. of variables wrong in get_starting_point. n : " << n;
  CHECK(init_x == true) << "Warm start init_x setting failed";
  CHECK(init_z == false) << "Warm start init_z setting failed";
  CHECK(init_lambda == false) << "Warm start init_lambda setting failed";

  // 1. state variables 4 * (horizon_ + 1)
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    std::size_t index = i * 4;
    for (std::size_t j = 0; j < 4; ++j) {
      x[index + j] = xWS_(j, i);
    }
  }

  // 2. control variable initialization, 2 * horizon_
  for (std::size_t i = 0; i < horizon_; ++i) {
    std::size_t index = i * 2;
    x[control_start_index_ + index] = uWS_(0, i);
    x[control_start_index_ + index + 1] = uWS_(1, i);
  }

  // 2. time scale variable initialization, horizon_ + 1
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    x[time_start_index_ + i] = 1.0;
  }

  // TODO(QiL) : better hot start l
  // 3. lagrange constraint l, obstacles_vertices_sum_ * (horizon_+1)
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    std::size_t index = i * obstacles_vertices_sum_;
    for (std::size_t j = 0; j < obstacles_vertices_sum_; ++j) {
      x[index + j] = 0.2;
    }
  }

  // TODO(QiL) : better hot start m
  // 4. lagrange constraint m, 4*obstacles_num * (horizon_+1)
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    std::size_t index = i * 4 * obstacles_num_;
    for (std::size_t j = 0; j < 4 * obstacles_num_; ++j) {
      x[index + j] = 0.2;
    }
  }
  AINFO << "get_starting_point out";
  return true;
}

bool DistanceApproachIPOPTInterface::eval_jac_g(int n, const double* x,
                                                bool new_x, int m, int nele_jac,
                                                int* iRow, int* jCol,
                                                double* values) {
  AINFO << "eval_jac_g";
  CHECK_EQ(n, num_of_variables_)
      << "No. of variables wrong in eval_jac_g. n : " << n;
  CHECK_EQ(m, num_of_constraints_)
      << "No. of constraints wrong in eval_jac_g. n : " << m;

  if (values == nullptr) {
    int nz_index = 0;
    int constraint_index = 0;
    int state_index = state_start_index_;
    int control_index = control_start_index_;
    int time_index = time_start_index_;

    // 1. State Constraint with respect to variables
    for (std::size_t i = 0; i < horizon_; ++i) {
      // g(0)' with respect to x0 ~ x7
      iRow[nz_index] = state_index;
      jCol[nz_index] = state_index;
      ++nz_index;

      iRow[nz_index] = state_index;
      jCol[nz_index] = state_index + 2;
      ++nz_index;

      iRow[nz_index] = state_index;
      jCol[nz_index] = state_index + 3;
      ++nz_index;

      iRow[nz_index] = state_index;
      jCol[nz_index] = state_index + 4;
      ++nz_index;

      // g(0)' with respect to u0 ~ u1'
      iRow[nz_index] = state_index;
      jCol[nz_index] = control_index;
      ++nz_index;

      iRow[nz_index] = state_index;
      jCol[nz_index] = control_index + 1;
      ++nz_index;

      // g(0)' with respect to time
      iRow[nz_index] = state_index;
      jCol[nz_index] = time_index;
      ++nz_index;

      // g(1)' with respect to x0 ~ x7
      iRow[nz_index] = state_index + 1;
      jCol[nz_index] = state_index + 1;
      ++nz_index;

      iRow[nz_index] = state_index + 1;
      jCol[nz_index] = state_index + 2;
      ++nz_index;

      iRow[nz_index] = state_index + 1;
      jCol[nz_index] = state_index + 3;
      ++nz_index;

      iRow[nz_index] = state_index + 1;
      jCol[nz_index] = state_index + 5;
      ++nz_index;

      // g(1)' with respect to u0 ~ u1'
      iRow[nz_index] = state_index + 1;
      jCol[nz_index] = control_index;
      ++nz_index;

      iRow[nz_index] = state_index + 1;
      jCol[nz_index] = control_index + 1;
      ++nz_index;

      // g(1)' with respect to time
      iRow[nz_index] = state_index + 1;
      jCol[nz_index] = time_index;
      ++nz_index;

      // g(2)' with respect to x0 ~ x7
      iRow[nz_index] = state_index + 2;
      jCol[nz_index] = state_index + 2;
      ++nz_index;

      iRow[nz_index] = state_index + 2;
      jCol[nz_index] = state_index + 3;
      ++nz_index;

      iRow[nz_index] = state_index + 2;
      jCol[nz_index] = state_index + 6;
      ++nz_index;

      // g(2)' with respect to u0 ~ u1'
      iRow[nz_index] = state_index + 2;
      jCol[nz_index] = control_index;
      ++nz_index;

      iRow[nz_index] = state_index + 2;
      jCol[nz_index] = control_index + 1;
      ++nz_index;

      // g(2)' with respect to time
      iRow[nz_index] = state_index + 2;
      jCol[nz_index] = time_index;
      ++nz_index;

      // g(3)'  with x0 ~ x7
      iRow[nz_index] = state_index + 3;
      jCol[nz_index] = state_index + 3;
      ++nz_index;

      iRow[nz_index] = state_index + 3;
      jCol[nz_index] = state_index + 7;
      ++nz_index;

      // g(3)' with respect to time
      iRow[nz_index] = state_index + 2;
      jCol[nz_index] = time_index;
      ++nz_index;

      state_index += 4;
      control_index += 2;
      time_index += 1;
      constraint_index += 4;
    }

    AINFO << "After adding dynamics constraints derivative, nz_index : "
          << nz_index << " nele_jac : " << nele_jac;

    // 2. only have control rate constraints on u0 , range [0, horizon_-1]
    control_index = control_start_index_;
    state_index = state_start_index_;
    time_index = time_start_index_;

    for (std::size_t i = 0; i < horizon_; ++i) {
      // with respect to u(0, i-1)
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = control_index;
      ++nz_index;

      // with respect to u(0, i)
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = control_index + 2;
      ++nz_index;

      // with respect to time
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = time_index;
      ++nz_index;

      // only consider rate limits on u0
      control_index += 2;
      constraint_index += 1;
    }

    AINFO << "After adding control rate constraints derivative, nz_index : "
          << nz_index << " nele_jac : " << nele_jac;

    // 3. Time constraints [0, horizon_ -1]
    time_index = time_start_index_;

    for (std::size_t i = 0; i < horizon_; ++i) {
      // with respect to timescale(0, i-1)
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = time_index;
      ++nz_index;

      // with respect to timescale(0, i)
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = time_index + 1;
      ++nz_index;

      time_index += 1;
      constraint_index += 1;
    }

    AINFO << "After adding time constraints derivative, nz_index : " << nz_index
          << " nele_jac : " << nele_jac;

    // 4. Three obstacles related equal constraints, one equality constraints,
    // [0, horizon_] * [0, obstacles_num_-1] * 4

    state_index = state_start_index_;
    std::size_t l_index = l_start_index_;
    std::size_t n_index = n_start_index_;

    for (std::size_t i = 0; i < horizon_ + 1; ++i) {
      for (std::size_t j = 0; j < obstacles_num_; ++j) {
        std::size_t current_edges_num = obstacles_edges_num_(j, 0);

        // 1. norm(A* lambda == 1)
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          // with respect to l
          iRow[nz_index] = constraint_index;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }
        constraint_index += 1;

        // 2. G' * mu + R' * lambda == 0, part 1
        // With respect to x
        iRow[nz_index] = constraint_index;
        jCol[nz_index] = state_index + 2;
        ++nz_index;

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          iRow[nz_index] = constraint_index;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }

        // With respect to n
        iRow[nz_index] = constraint_index;
        jCol[nz_index] = n_index;
        ++nz_index;

        iRow[nz_index] = constraint_index;
        jCol[nz_index] = n_index + 2;
        ++nz_index;

        constraint_index += 1;

        // 2. G' * mu + R' * lambda == 0, part 2
        // With respect to x
        iRow[nz_index] = constraint_index;
        jCol[nz_index] = state_index + 2;
        ++nz_index;

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          iRow[nz_index] = constraint_index;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }

        // With respect to n
        iRow[nz_index] = constraint_index;
        jCol[nz_index] = n_index + 1;
        ++nz_index;

        iRow[nz_index] = constraint_index;
        jCol[nz_index] = n_index + 3;
        ++nz_index;

        constraint_index += 1;

        //  -g'*mu + (A*t - b)*lambda > 0

        // With respect to x
        iRow[nz_index] = constraint_index;
        jCol[nz_index] = state_index;
        ++nz_index;

        iRow[nz_index] = constraint_index;
        jCol[nz_index] = state_index + 1;
        ++nz_index;

        iRow[nz_index] = constraint_index;
        jCol[nz_index] = state_index + 2;
        ++nz_index;

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          iRow[nz_index] = constraint_index;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }

        // with respect to n
        for (std::size_t k = 0; k < 4; ++k) {
          iRow[nz_index] = constraint_index;
          jCol[nz_index] = n_index + k;
          ++nz_index;
        }

        // Update inde
        l_index += current_edges_num;
        n_index += 4;
        constraint_index += 1;
      }
    }

    AINFO << "After adding obstacles constraint derivatie indexes, nz_index : "
          << nz_index << " nele_jac : " << nele_jac;
    CHECK_EQ(nz_index, static_cast<std::size_t>(nele_jac));
    CHECK_EQ(constraint_index, static_cast<std::size_t>(m));
  } else {
    AINFO << "eval_jac_g, second time";
    std::fill(values, values + nele_jac, 0.0);
    std::size_t nz_index = 0;

    std::size_t time_index = time_start_index_;
    std::size_t state_index = state_start_index_;
    std::size_t control_index = control_start_index_;

    // TODO(QiL) : initially implemented to be debug friendly, later iterate
    // towards better efficiency
    // 1. state constraints 4 * [0, horizons-1]
    for (std::size_t i = 0; i < horizon_ + 1; ++i) {
      values[nz_index] = -1.0;
      ++nz_index;

      values[nz_index] =
          x[time_index] * ts_ *
          (x[state_index + 3] +
           x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
          std::sin(x[state_index + 2] +
                   x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                       std::tan(x[control_index] / wheelbase_));  // a.
      ++nz_index;

      values[nz_index] =
          -1.0 *
          (x[time_index] * ts_ *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)) +
           x[time_index] * ts_ * x[time_index] * ts_ * 0.5 *
               x[control_index + 1] * (-1) * x[time_index] * ts_ * 0.5 *
               std::tan(x[control_index] / wheelbase_) *
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)));  // b
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          x[time_index] * ts_ *
          (x[state_index] + x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
          std::sin(x[state_index + 2] +
                   x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                       std::tan(x[control_index] / wheelbase_)) *
          x[time_index] * ts_ * 0.5 * wheelbase_ /
          (std::cos(x[control_index] / wheelbase_) *
           std::cos(x[control_index] / wheelbase_));  // c
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ * x[time_index] * ts_ * 0.5 *
                  std::cos(x[state_index + 2] +
                           x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                               std::tan(x[control_index] / wheelbase_)));  // d
      ++nz_index;

      values[nz_index] =
          -1.0 *
          (ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)) +
           x[time_index] * ts_ * ts_ * 0.5 * x[control_index + 1] *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)) -
           x[time_index] * ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               x[state_index + 3] * ts_ * 0.5 *
               std::tan(x[control_index] / wheelbase_) *
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)));  // e
      ++nz_index;

      values[nz_index] = -1.0;
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ *
                  (x[state_index + 3] +
                   x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
                  std::cos(x[state_index + 2] +
                           x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                               std::tan(x[control_index] / wheelbase_)));  // f.
      ++nz_index;

      values[nz_index] =
          -1.0 *
          (x[time_index] * ts_ *
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)) +
           x[time_index] * ts_ * x[time_index] * ts_ * 0.5 *
               x[control_index + 1] * (-1) * x[time_index] * ts_ * 0.5 *
               std::tan(x[control_index] / wheelbase_) *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)));  // g
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          -1.0 *
          (x[time_index] * ts_ *
           (x[state_index] + x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
           std::cos(x[state_index + 2] +
                    x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                        std::tan(x[control_index] / wheelbase_)) *
           x[time_index] * ts_ * 0.5 * wheelbase_ /
           (std::cos(x[control_index] / wheelbase_) *
            std::cos(x[control_index] / wheelbase_)));  // h
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ * x[time_index] * ts_ * 0.5 *
                  std::sin(x[state_index + 2] +
                           x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                               std::tan(x[control_index] / wheelbase_)));  // i
      ++nz_index;

      values[nz_index] =
          -1.0 *
          (ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)) +
           x[time_index] * ts_ * ts_ * 0.5 * x[control_index + 1] *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)) +
           x[time_index] * ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               x[state_index + 3] * ts_ * 0.5 *
               std::tan(x[control_index] / wheelbase_) *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)));  // j
      ++nz_index;

      values[nz_index] = -1.0;
      ++nz_index;

      values[nz_index] = -1.0 * x[time_index] * ts_;  // k.
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ *
                  (x[state_index + 3] +
                   x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
                  wheelbase_ /
                  (std::cos(x[control_index] / wheelbase_) *
                   std::cos(x[control_index] / wheelbase_)));  // l.
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ * x[time_index] * ts_ *
                  std::tan(x[control_index] / wheelbase_));  // m.
      ++nz_index;

      values[nz_index] =
          -1.0 * (ts_ *
                      (x[state_index + 3] +
                       x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
                      std::tan(x[control_index] / wheelbase_) +
                  x[time_index] * ts_ * ts_ * 0.5 * x[control_index + 1] *
                      std::tan(x[control_index] / wheelbase_));  // n.
      ++nz_index;

      values[nz_index] = -1.0;
      ++nz_index;

      values[nz_index] = -1.0 * ts_ * x[control_index + 1];  // p.
      ++nz_index;

      state_index += 4;
      control_index += 2;
      time_index += 1;
    }

    AINFO << "eval_jac_g, fulfilled state constraint values";

    // 2. control rate constraints 1 * [0, horizons-1]
    control_index = control_start_index_;
    state_index = state_start_index_;
    time_index = time_start_index_;
    // std::size_t control_rate_constraint_index = control_start_index_;

    for (std::size_t i = 0; i < horizon_; ++i) {
      // with respect to u(0, i-1)
      if (i == 0) {
        values[nz_index] = 0.0;
      } else {
        values[nz_index] = -1.0 / x[time_index] / ts_;
      }
      ++nz_index;

      // with respect to u(0, i)
      values[nz_index] = 1.0 / x[time_index] / ts_;
      ++nz_index;

      // with respect to time
      values[nz_index] = 1.0 / x[time_index] / ts_;
      ++nz_index;

      // only consider rate limits on u0
      if (i == 0) {
        values[nz_index] = -1.0 * (x[control_index] - last_time_u_(0, 0)) /
                           x[time_index] / x[time_index] / ts_;
      } else {
        values[nz_index] = -1.0 * (x[control_index + 2] - x[control_index]) /
                           x[time_index] / x[time_index] / ts_;
      }
      control_index += 2;
      time_index += 1;
    }

    AINFO << "eval_jac_g, fulfilled control rate constraint values";

    // 3. Time constraints [0, horizon_ -1]
    time_index = time_start_index_;

    for (std::size_t i = 0; i < horizon_; ++i) {
      // with respect to timescale(0, i-1)
      values[nz_index] = -1.0;
      ++nz_index;

      // with respect to timescale(0, i)
      values[nz_index] = 1.0;
      ++nz_index;

      time_index += 1;
    }

    AINFO << "eval_jac_g, fulfilled time constraint values";

    // 4. Three obstacles related equal constraints, one equality constraints,
    // [0, horizon_] * [0, obstacles_num_-1] * 4

    state_index = state_start_index_;
    std::size_t l_index = l_start_index_;
    std::size_t n_index = n_start_index_;

    for (std::size_t i = 0; i < horizon_ + 1; ++i) {
      std::size_t edges_counter = 0;
      for (std::size_t j = 0; j < obstacles_num_; ++j) {
        std::size_t current_edges_num = obstacles_edges_num_(j, 0);
        AINFO << "eval_jac_g, obstacle constraint values, current "
                 "vertice_num : "
              << current_edges_num << " i :  " << i << " j : " << j;
        AINFO << "obstacles_A_ size : " << obstacles_A_.rows() << " and "
              << obstacles_A_.cols();
        AINFO << "edges_counter : " << edges_counter;
        Eigen::MatrixXd Aj =
            obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
        AINFO << "before after Aj";
        std::vector<int> lj(&x[l_index], &x[l_index + current_edges_num]);
        AINFO << "before nj";
        std::vector<int> nj(&x[n_index], &x[n_index + 3]);
        Eigen::MatrixXd bj =
            obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

        AINFO
            << "eval_jac_g, obstacle constraint values, extraction number : i "
            << i << " j : " << j;
        // TODO(QiL) : Remove redudant calculation
        double tmp1 = 0;
        double tmp2 = 0;
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          // TODO(QiL) : replace this one directly with x
          tmp1 += Aj(k, 0) * x[l_index + k];
          tmp2 += Aj(k, 1) * x[l_index + k];
        }

        double tmp3 = 0.0;
        double tmp4 = 0.0;
        for (std::size_t k = 0; k < 4; ++k) {
          // TODO(QiL) : replace this one directly with x
          tmp3 += -g_[k] * nj[k];
        }

        for (std::size_t k = 0; k < current_edges_num; ++k) {
          tmp4 += bj(k, 0) * x[l_index + k];
        }

        AINFO << "eval_jac_g, bstacle constraint values, tmp prepare : i " << i
              << " j : " << j;

        // 1. norm(A* lambda == 1)
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          // with respect to l
          values[nz_index] = 2 * Aj(k, 0) * Aj(k, 0) * x[l_index + k] +
                             2 * Aj(k, 0) * Aj(k, 1) * x[l_index + k];  // t0~tk
          ++nz_index;
        }

        AINFO << "eval_jac_g, bstacle constraint values, 1 : i " << i
              << " j : " << j;

        // 2. G' * mu + R' * lambda == 0, part 1
        // With respect to x
        values[nz_index] = -std::sin(x[state_index + 2]) * tmp1 +
                           std::cos(x[state_index + 2]) * tmp2;  // u
        ++nz_index;

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          values[nz_index] = std::cos(x[state_index + 2]) * Aj(k, 0) +
                             std::sin(x[state_index + 2]) * Aj(k, 1);  // v0~vn
          ++nz_index;
        }

        // With respect to n
        values[nz_index] = 1.0;  // w0
        ++nz_index;

        values[nz_index] = -1.0;  // w2
        ++nz_index;

        // 2. G' * mu + R' * lambda == 0, part 2
        // With respect to x
        values[nz_index] = -std::cos(x[state_index + 2]) * tmp1 -
                           std::sin(x[state_index + 2]) * tmp2;  // x
        ++nz_index;

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          values[nz_index] = std::cos(x[state_index + 2]) * Aj(k, 0) +
                             std::sin(x[state_index + 2]) * Aj(k, 1);  // y0~yn
          ++nz_index;
        }

        // With respect to n
        values[nz_index] = 1.0;  // z1
        ++nz_index;

        values[nz_index] = -1.0;  // z3
        ++nz_index;

        AINFO << "eval_jac_g, bstacle constraint values, 2 : i " << i
              << " j : " << j;

        //  3. -g'*mu + (A*t - b)*lambda > 0

        // With respect to x
        values[nz_index] = tmp1;  // aa1
        ++nz_index;

        values[nz_index] = tmp2;  // bb1
        ++nz_index;

        values[nz_index] =
            -std::sin(x[state_index + 2]) * offset_ * tmp1 +
            std::cos(x[state_index + 2]) * offset_ * tmp2;  // cc1
        ++nz_index;

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          values[nz_index] =
              (x[state_index] + std::cos(x[state_index + 2]) * offset_) *
                  Aj(k, 0) +
              (x[state_index + 1] + std::sin(x[state_index + 2]) * offset_) *
                  Aj(k, 1) -
              bj(k, 0);  // ddk
          ++nz_index;
        }

        // with respect to n
        for (std::size_t k = 0; k < 4; ++k) {
          values[nz_index] = -g_[k];  // eek
          ++nz_index;
        }

        AINFO << "eval_jac_g, bstacle constraint values, 4 : i " << i
              << " j : " << j;

        // Update index
        edges_counter += current_edges_num;
        l_index += current_edges_num;
        n_index += 4;
      }
    }

    AINFO << "eval_jac_g, fulfilled obstacle constraint values";
    //  CHECK_EQ(nz_index, static_cast<std::size_t>(nele_jac));
  }
  return true;
}

bool DistanceApproachIPOPTInterface::eval_h(int n, const double* x, bool new_x,
                                            double obj_factor, int m,
                                            const double* lambda,
                                            bool new_lambda, int nele_hess,
                                            int* iRow, int* jCol,
                                            double* values) {
  AINFO << "eval_h";
  return false;
}

bool DistanceApproachIPOPTInterface::eval_f(int n, const double* x, bool new_x,
                                            double& obj_value) {
  AINFO << "eval_f";
  // Objective is :
  // min control inputs
  // min input rate
  // min time (if the time step is not fixed)
  // regularization wrt warm start trajectory
  DCHECK(ts_ != 0) << "ts in distance_approach_ is 0";
  std::size_t control_start_index = (horizon_ + 1) * 4;
  std::size_t time_start_index = (horizon_ + 1) * 4 + horizon_ * 2;

  // TODO(QiL): Initial implementation towards earlier understanding and debug
  // purpose, later code refine towards improving efficiency

  // 1. objective to minimize u square
  for (std::size_t i = 0; i < horizon_; ++i) {
    std::size_t index = 2 * i;
    obj_value += weight_input_steer_ * x[control_start_index + index] *
                     x[control_start_index + index] +
                 weight_input_a_ * x[control_start_index + index + 1] *
                     x[control_start_index + index + 1];
  }

  // 2. objective to minimize input change rates, 1 ~ horizone -1
  for (std::size_t i = 0; i < horizon_ - 1; ++i) {
    std::size_t index = 2 * i;
    double steering_rate =
        (x[control_start_index + index] - x[control_start_index + index + 2]) /
        x[time_start_index + i] / ts_;
    double a_rate = (x[control_start_index + index + 1] -
                     x[control_start_index + index + 3]) /
                    x[time_start_index + i] / ts_;
    obj_value += weight_rate_steer_ * steering_rate * steering_rate +
                 weight_rate_a_ * a_rate * a_rate;
  }

  // 3. objective to minimize input volume for first horizon
  double last_time_steer_rate =
      (x[control_start_index] - last_time_u_(0, 0)) / x[time_start_index] / ts_;
  double last_time_a_rate = (x[control_start_index + 1] - last_time_u_(1, 0)) /
                            x[time_start_index + 1] / ts_;
  obj_value +=
      weight_stitching_steer_ * last_time_steer_rate * last_time_steer_rate +
      weight_stitching_a_ * last_time_a_rate * last_time_a_rate;

  // 4. objective to minimize total time
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    double first_order_penalty =
        weight_first_order_time_ * x[time_start_index + i];
    double second_order_penalty = weight_second_order_time_ *
                                  x[time_start_index + i] *
                                  x[time_start_index + i];
    obj_value += first_order_penalty + second_order_penalty;
  }

  // 5. objective to minimize state diff to warm up
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    std::size_t index = 4 * i;
    double x1_diff = x[index] - xWS_(0, i);
    double x2_diff = x[index + 1] - xWS_(1, i);
    double x3_diff = x[index + 2] - xWS_(2, i);
    obj_value += weight_state_x_ * x1_diff * x1_diff +
                 weight_state_y_ * x2_diff * x2_diff +
                 weight_state_phi_ * x3_diff * x3_diff;
  }

  return true;
}

bool DistanceApproachIPOPTInterface::eval_grad_f(int n, const double* x,
                                                 bool new_x, double* grad_f) {
  AINFO << "eval_grad_f";
  std::fill(grad_f, grad_f + n, 0.0);
  // gradients on states(No.5 in eval_f())
  for (std::size_t i = 0; i < horizon_ + 1; i++) {
    std::size_t index = i * 4;
    grad_f[index] += weight_state_x_ * (2 * x[index] - 2 * xWS_(0, i));
    grad_f[index + 1] += weight_state_y_ * (2 * x[index + 1] - 2 * xWS_(1, i));
    grad_f[index + 2] +=
        weight_state_phi_ * (2 * x[index + 2] - 2 * xWS_(2, i));
    grad_f[index + 3] += 0.0;
  }

  // gradients on input absolute value(No.1 in eval_f())
  for (std::size_t i = 0; i < horizon_; i++) {
    std::size_t index = i << 1;
    grad_f[control_start_index_ + index] +=
        weight_input_steer_ * 2 * x[control_start_index_ + index];
    grad_f[control_start_index_ + index + 1] +=
        weight_input_a_ * 2 * x[control_start_index_ + index + 1];
  }

  // gradients on input change rate(No.2 in eval_f())
  for (std::size_t i = 1; i < horizon_ - 1; i++) {
    std::size_t index = i << 1;
    grad_f[control_start_index_ + index] +=
        weight_rate_steer_ *
        ((2 * x[control_start_index_ + index] -
          2 * x[control_start_index_ + index - 2]) /
             (ts_ * ts_ * x[time_start_index_ + i - 1] *
              x[time_start_index_ + i - 1]) +
         (2 * x[control_start_index_ + index] -
          2 * x[control_start_index_ + index + 2]) /
             (ts_ * ts_ * x[time_start_index_ + i] * x[time_start_index_ + i]));
    grad_f[control_start_index_ + index + 1] +=
        weight_rate_a_ *
        ((2 * x[control_start_index_ + index + 1] -
          2 * x[control_start_index_ + index + 1 - 2]) /
             (ts_ * ts_ * x[time_start_index_ + i - 1] *
              x[time_start_index_ + i - 1]) +
         (2 * x[control_start_index_ + index + 1] -
          2 * x[control_start_index_ + index + 1 + 2]) /
             (ts_ * ts_ * x[time_start_index_ + i] * x[time_start_index_ + i]));
  }
  grad_f[control_start_index_] +=
      weight_rate_steer_ *
      ((2 * x[control_start_index_] - 2 * x[control_start_index_ + 2]) /
       (ts_ * ts_ * x[time_start_index_] * x[time_start_index_]));
  grad_f[control_start_index_ + 1] +=
      weight_rate_a_ *
      ((2 * x[control_start_index_ + 1] - 2 * x[control_start_index_ + 1 + 2]) /
       (ts_ * ts_ * x[time_start_index_] * x[time_start_index_]));
  grad_f[control_start_index_ + 2 * (horizon_ - 1)] +=
      weight_rate_steer_ *
      ((2 * x[control_start_index_ + 2 * (horizon_ - 1)] -
        2 * x[control_start_index_ + 2 * (horizon_ - 1) - 2]) /
       (ts_ * ts_ * x[time_start_index_ + (horizon_ - 1) - 1] *
        x[time_start_index_ + (horizon_ - 1) - 1]));
  grad_f[control_start_index_ + 2 * (horizon_ - 1) + 1] +=
      weight_rate_a_ *
      ((2 * x[control_start_index_ + 2 * (horizon_ - 1) + 1] -
        2 * x[control_start_index_ + 2 * (horizon_ - 1) + 1 - 2]) /
       (ts_ * ts_ * x[time_start_index_ + (horizon_ - 1) - 1] *
        x[time_start_index_ + (horizon_ - 1) - 1]));
  for (std::size_t i = 0; i < horizon_ - 1; i++) {
    std::size_t index = i << 1;
    grad_f[time_start_index_ + i] +=
        -2 * weight_rate_steer_ *
            ((x[control_start_index_ + index] *
                  x[control_start_index_ + index] +
              x[control_start_index_ + index + 2] *
                  x[control_start_index_ + index + 2] -
              2 * x[control_start_index_ + index] *
                  x[control_start_index_ + index + 2]) /
             (ts_ * ts_ * x[time_start_index_ + i] * x[time_start_index_ + i] *
              x[time_start_index_ + i])) +
        -2 * weight_rate_a_ *
            ((x[control_start_index_ + index + 1] *
                  x[control_start_index_ + index + 1] +
              x[control_start_index_ + index + 1 + 2] *
                  x[control_start_index_ + index + 1 + 2] -
              2 * x[control_start_index_ + index + 1] *
                  x[control_start_index_ + index + 1 + 2]) /
             (ts_ * ts_ * x[time_start_index_ + i] * x[time_start_index_ + i] *
              x[time_start_index_ + i]));
  }

  // gradients on stitching input (No.3)
  grad_f[control_start_index_] +=
      weight_stitching_steer_ *
      ((2 * x[control_start_index_] - 2 * last_time_u_(0, 0)) /
       (ts_ * ts_ * x[time_start_index_] * x[time_start_index_]));
  grad_f[control_start_index_ + 1] +=
      weight_stitching_a_ *
      ((2 * x[control_start_index_ + 1] - 2 * last_time_u_(1, 0)) /
       (ts_ * ts_ * x[time_start_index_] * x[time_start_index_]));

  // gradients on timestep(No.4 in eval_f())
  for (std::size_t i = 0; i < horizon_ + 1; i++) {
    grad_f[time_start_index_ + i] =
        weight_first_order_time_ +
        2 * weight_second_order_time_ * x[time_start_index_ + i];
  }

  // gradients on dual variables
  for (std::size_t i = 0; i < horizon_ + 1; i++) {
    std::size_t index = i << 1;
    grad_f[l_start_index_ + index] = 0.0;
    grad_f[l_start_index_ + index + 1] = 0.0;
  }

  return true;
}

void DistanceApproachIPOPTInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  AINFO << "finalize_solution";
  for (std::size_t i = 0; i < horizon_; ++i) {
    std::size_t state_index = i * 4;
    std::size_t control_index = i * 2;

    state_result_(0, i) = x[state_index];
    state_result_(1, i) = x[state_index + 1];
    state_result_(2, i) = x[state_index + 2];
    state_result_(3, i) = x[state_index + 3];

    control_result_(0, i) = x[control_start_index_ + control_index];
    control_result_(1, i) = x[control_start_index_ + control_index + 1];

    time_result_(0, i) = x[time_start_index_ + i];
  }
  // push back state for N+1
  state_result_(0, 4 * horizon_) = x[4 * horizon_];
  state_result_(1, 4 * horizon_ + 1) = x[4 * horizon_ + 1];
  state_result_(2, 4 * horizon_ + 2) = x[4 * horizon_ + 2];
  state_result_(3, 4 * horizon_ + 3) = x[4 * horizon_ + 3];
  time_result_(0, time_start_index_ + horizon_) =
      x[time_start_index_ + horizon_];
}

void DistanceApproachIPOPTInterface::get_optimization_results(
    Eigen::MatrixXd* state_result, Eigen::MatrixXd* control_result,
    Eigen::MatrixXd* time_result) const {
  AINFO << "get_optimization_results";
  *state_result = state_result_;
  *control_result = control_result_;
  *time_result = time_result_;
}

}  // namespace planning
}  // namespace apollo
