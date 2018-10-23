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
  obstacles_edges_sum_ = std::size_t(obstacles_edges_num_.sum());
  state_result_ = Eigen::MatrixXd::Zero(4, horizon_ + 1);
  control_result_ = Eigen::MatrixXd::Zero(2, horizon_ + 1);
  time_result_ = Eigen::MatrixXd::Zero(1, horizon_ + 1);
  state_start_index_ = 0;
  control_start_index_ = 4 * (horizon_ + 1);
  time_start_index_ = control_start_index_ + 2 * horizon_;
  l_start_index_ = time_start_index_ + (horizon_ + 1);
  n_start_index_ = l_start_index_ + obstacles_edges_sum_ * (horizon_ + 1);

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
  max_steer_angle_ = distance_approach_config_.max_steer_angle();
  max_speed_forward_ = distance_approach_config_.max_speed_forward();
  max_speed_reverse_ = distance_approach_config_.max_speed_reverse();
  max_acceleration_forward_ =
      distance_approach_config_.max_acceleration_forward();
  max_acceleration_reverse_ =
      distance_approach_config_.max_acceleration_reverse();
  min_time_sample_scaling_ =
      distance_approach_config_.min_time_sample_scaling();
  max_time_sample_scaling_ =
      distance_approach_config_.max_time_sample_scaling();
  max_steer_rate_ = distance_approach_config_.max_steer_rate();

  wheelbase_ = vehicle_param_.wheel_base();
}

bool DistanceApproachIPOPTInterface::get_nlp_info(int& n, int& m,
                                                  int& nnz_jac_g,
                                                  int& nnz_h_lag,
                                                  IndexStyleEnum& index_style) {
  ADEBUG << "get_nlp_info";
  // number of variables
  n = num_of_variables_;
  ADEBUG << "num_of_variables_ " << num_of_variables_;
  // number of constraints
  m = num_of_constraints_;
  ADEBUG << "num_of_constraints_ " << num_of_constraints_;

  // number of nonzero hessian and lagrangian.
  int tmp = 0;

  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_num_; ++j) {
      std::size_t current_edges_num = obstacles_edges_num_(j, 0);
      tmp += current_edges_num * 4 + 9 + 4;
    }
  }

  nnz_jac_g = 24 * horizon_ + 3 * horizon_ + 2 * horizon_ + tmp - 1;

  ADEBUG << "nnz_jac_g_" << nnz_jac_g;

  nnz_h_lag = 0;

  index_style = IndexStyleEnum::C_STYLE;
  ADEBUG << "get_nlp_info out";
  return true;
}

bool DistanceApproachIPOPTInterface::get_bounds_info(int n, double* x_l,
                                                     double* x_u, int m,
                                                     double* g_l, double* g_u) {
  ADEBUG << "get_bounds_info";
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
    x_l[variable_index + 2] = -max_steer_angle_;
    x_u[variable_index + 2] = max_steer_angle_;

    // v
    // TODO(QiL) : Change this to configs
    x_l[variable_index + 3] = -max_speed_reverse_;
    x_u[variable_index + 3] = max_speed_forward_;

    variable_index += 4;
  }

  // end point pose
  for (std::size_t i = 0; i < 4; ++i) {
    x_l[variable_index + i] = xf_(i, 0);
    x_u[variable_index + i] = xf_(i, 0);
  }
  variable_index += 4;
  ADEBUG << "variable_index after adding state variables : " << variable_index;

  // 2. control variables, 2 * [0, horizon_-1]
  for (std::size_t i = 0; i < horizon_; ++i) {
    // u1
    x_l[variable_index] = -max_steer_angle_;
    x_u[variable_index] = max_steer_angle_;

    // u2
    x_l[variable_index + 1] = -max_acceleration_forward_;
    x_u[variable_index + 1] = max_acceleration_forward_;

    variable_index += 2;
  }
  ADEBUG << "variable_index after adding control variables : "
         << variable_index;

  // 3. sampling time variables, 1 * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    if (!use_fix_time_) {
      x_l[variable_index] = min_time_sample_scaling_;
      x_u[variable_index] = max_time_sample_scaling_;
    } else {
      x_l[variable_index] = 1.0;
      x_u[variable_index] = 1.0;
    }

    ++variable_index;
  }
  ADEBUG << "variable_index after adding sample time : " << variable_index;
  ADEBUG << "sample time fix time status is : " << use_fix_time_;

  // 4. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_edges_sum_; ++j) {
      x_l[variable_index] = 0.0;
      // TODO(QiL): refine this variables limits
      x_u[variable_index] = 100.0;
      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange l : " << variable_index;

  // 5. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < 4 * obstacles_num_; ++j) {
      x_l[variable_index] = 0.0;
      // TODO(QiL): refine this variables limits
      x_u[variable_index] = 10000.0;

      ++variable_index;
    }
  }

  ADEBUG << "variable_index after adding lagrange n : " << variable_index;

  // Constraints: includes four state Euler forward constraints, three
  // Obstacle related constraints

  // 1. dynamics constraints 4 * [0, horizons-1]
  std::size_t constraint_index = 0;
  for (std::size_t i = 0; i < 4 * horizon_; ++i) {
    g_l[i] = 0.0;
    g_u[i] = 0.0;
  }
  constraint_index += 4 * horizon_;

  ADEBUG << "constraint_index after adding Euler forward dynamics constraints: "
         << constraint_index;

  // 2. Control rate limit constraints, 1 * [0, horizons-1], only apply
  // steering rate as of now
  for (std::size_t i = 0; i < horizon_; ++i) {
    // TODO(QiL) : change this to configs
    g_l[constraint_index] = -max_steer_rate_;
    g_u[constraint_index] = max_steer_rate_;
    ++constraint_index;
  }

  ADEBUG << "constraint_index after adding steering rate constraints: "
         << constraint_index;

  // 3. Time constraints 1 * [0, horizons-1]
  for (std::size_t i = 0; i < horizon_; ++i) {
    g_l[constraint_index] = 0.0;
    g_u[constraint_index] = 0.0;
    ++constraint_index;
  }

  ADEBUG << "constraint_index after adding time constraints: "
         << constraint_index;

  // 4. Three obstacles related equal constraints, one equality constraints,
  // [0, horizon_] * [0, obstacles_num_-1] * 4
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_num_; ++j) {
      // a. norm(A'*lambda) = 1
      g_l[constraint_index] = -1.0;
      g_u[constraint_index] = 0.0;

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

  ADEBUG << "constraint_index after adding obstacles constraints: "
         << constraint_index;
  ADEBUG << "get_bounds_info_ out";
  return true;
}

bool DistanceApproachIPOPTInterface::eval_g(int n, const double* x, bool new_x,
                                            int m, double* g) {
  ADEBUG << "eval_g";
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
             std::cos(x[state_index + 2] +
                      ts_ * x[time_index] * 0.5 * x[state_index + 3] *
                          std::tan(x[control_index] / wheelbase_)));
    // x2
    g[constraint_index + 1] =
        x[state_index + 5] -
        (x[state_index + 1] +
         ts_ * x[time_index] *
             (x[state_index + 3] +
              ts_ * x[time_index] * 0.5 * x[control_index + 1]) *
             std::sin(x[state_index + 2] +
                      ts_ * x[time_index] * 0.5 * x[state_index + 3] *
                          std::tan(x[control_index] / wheelbase_)));

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

  ADEBUG << "constraint_index after adding Euler forward dynamics constraints "
            "updated: "
         << constraint_index;

  // 2. Control rate limit constraints, 1 * [0, horizons-1], only apply
  // steering rate as of now
  control_index = control_start_index_;
  time_index = time_start_index_;

  // First rate is compare first with stitch point
  g[constraint_index] =
      (x[control_index] - last_time_u_(0, 0)) / x[time_index] / ts_;
  control_index += 2;
  constraint_index += 1;
  time_index += 1;

  for (std::size_t i = 1; i < horizon_; ++i) {
    g[constraint_index] =
        (x[control_index] - x[control_index - 2]) / x[time_index] / ts_;
    constraint_index += 1;
    control_index += 2;
    time_index += 1;
  }

  // 3. Time constraints 1 * [0, horizons-1]
  time_index = time_start_index_;
  for (std::size_t i = 0; i < horizon_; ++i) {
    g[constraint_index] = x[time_index + 1] - x[time_index];
    constraint_index += 1;
    time_index += 1;
  }

  ADEBUG << "constraint_index after adding time constraints "
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
      std::size_t current_edges_num = obstacles_edges_num_(j, 0);
      Eigen::MatrixXd Aj =
          obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
      std::vector<int> lj(&x[l_index], &x[l_index + current_edges_num]);
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
      g[constraint_index + 1] = x[n_index] - x[n_index + 2] +
                                std::cos(x[state_index + 2]) * tmp1 +
                                std::sin(x[state_index + 2]) * tmp2;

      g[constraint_index + 2] = x[n_index + 1] - x[n_index + 3] -
                                std::sin(x[state_index + 2]) * tmp1 +
                                std::cos(x[state_index + 2]) * tmp2;

      //  -g'*mu + (A*t - b)*lambda > 0
      double tmp3 = 0.0;
      for (std::size_t k = 0; k < 4; ++k) {
        tmp3 += -g_[k] * x[n_index + k];
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
    state_index += 4;
  }

  ADEBUG << "constraint_index after obstacles avoidance constraints "
            "updated: "
         << constraint_index;

  return true;
}

bool DistanceApproachIPOPTInterface::get_starting_point(
    int n, bool init_x, double* x, bool init_z, double* z_L, double* z_U, int m,
    bool init_lambda, double* lambda) {
  ADEBUG << "get_starting_point";
  CHECK(n == num_of_variables_)
      << "No. of variables wrong in get_starting_point. n : " << n;
  CHECK(init_x == true) << "Warm start init_x setting failed";
  CHECK(init_z == false) << "Warm start init_z setting failed";
  CHECK(init_lambda == false) << "Warm start init_lambda setting failed";

  CHECK_EQ(horizon_, uWS_.cols());
  CHECK_EQ(horizon_ + 1, xWS_.cols());

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
  // 3. lagrange constraint l, obstacles_edges_sum_ * (horizon_+1)
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    std::size_t index = i * obstacles_edges_sum_;
    for (std::size_t j = 0; j < obstacles_edges_sum_; ++j) {
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
  ADEBUG << "get_starting_point out";
  return true;
}

bool DistanceApproachIPOPTInterface::eval_jac_g(int n, const double* x,
                                                bool new_x, int m, int nele_jac,
                                                int* iRow, int* jCol,
                                                double* values) {
  ADEBUG << "eval_jac_g";
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

      // g(3)' with respect to u0 ~ u1'
      iRow[nz_index] = state_index + 2;
      jCol[nz_index] = control_index + 1;
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

    // 2. only have control rate constraints on u0 , range [0, horizon_-1]
    control_index = control_start_index_;
    state_index = state_start_index_;
    time_index = time_start_index_;

    // First one, with respect to u(0, 0)
    iRow[nz_index] = constraint_index;
    jCol[nz_index] = control_index;
    ++nz_index;

    // First element, with respect to time
    iRow[nz_index] = constraint_index;
    jCol[nz_index] = time_index;
    ++nz_index;

    control_index += 2;
    time_index += 1;
    constraint_index += 1;

    for (std::size_t i = 1; i < horizon_; ++i) {
      // with respect to u(0, i-1)
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = control_index - 2;
      ++nz_index;

      // with respect to u(0, i)
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = control_index;
      ++nz_index;

      // with respect to time
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = time_index;
      ++nz_index;

      // only consider rate limits on u0
      control_index += 2;
      constraint_index += 1;
      time_index += 1;
    }

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

        // 2. G' * mu + R' * lambda == 0, part 1
        // With respect to x
        iRow[nz_index] = constraint_index + 1;
        jCol[nz_index] = state_index + 2;
        ++nz_index;

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          iRow[nz_index] = constraint_index + 1;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }

        // With respect to n
        iRow[nz_index] = constraint_index + 1;
        jCol[nz_index] = n_index;
        ++nz_index;

        iRow[nz_index] = constraint_index + 1;
        jCol[nz_index] = n_index + 2;
        ++nz_index;

        // 2. G' * mu + R' * lambda == 0, part 2
        // With respect to x
        iRow[nz_index] = constraint_index + 2;
        jCol[nz_index] = state_index + 2;
        ++nz_index;  // FIXME()

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          iRow[nz_index] = constraint_index + 2;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }

        // With respect to n
        iRow[nz_index] = constraint_index + 2;
        jCol[nz_index] = n_index + 1;
        ++nz_index;

        iRow[nz_index] = constraint_index + 2;
        jCol[nz_index] = n_index + 3;
        ++nz_index;

        /*
                CHECK_NE(constraint_index + 2, 300)
                    << "index i : " << i << "index j : " << j
                    << ", state_index + 2 : " << state_index + 2
                    << ", l_index : " << l_index << ", n_index : " << n_index;
        */
        //  -g'*mu + (A*t - b)*lambda > 0

        // With respect to x
        iRow[nz_index] = constraint_index + 3;
        jCol[nz_index] = state_index;
        ++nz_index;

        iRow[nz_index] = constraint_index + 3;
        jCol[nz_index] = state_index + 1;
        ++nz_index;  // FIXME()

        iRow[nz_index] = constraint_index + 3;
        jCol[nz_index] = state_index + 2;
        ++nz_index;  // FIXME()

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          iRow[nz_index] = constraint_index + 3;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }

        // with respect to n
        for (std::size_t k = 0; k < 4; ++k) {
          iRow[nz_index] = constraint_index + 3;
          jCol[nz_index] = n_index + k;
          ++nz_index;
        }

        // Update inde
        l_index += current_edges_num;
        n_index += 4;
        constraint_index += 4;
      }
      state_index += 4;
    }

    CHECK_EQ(nz_index, static_cast<std::size_t>(nele_jac));
    CHECK_EQ(constraint_index, static_cast<std::size_t>(m));
  } else {
    ADEBUG << "eval_jac_g, second time";
    std::fill(values, values + nele_jac, 0.0);
    std::size_t nz_index = 0;

    std::size_t time_index = time_start_index_;
    std::size_t state_index = state_start_index_;
    std::size_t control_index = control_start_index_;

    // TODO(QiL) : initially implemented to be debug friendly, later iterate
    // towards better efficiency
    // 1. state constraints 4 * [0, horizons-1]
    for (std::size_t i = 0; i < horizon_; ++i) {
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
           x[time_index] * ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               (-1) * x[time_index] * ts_ * 0.5 *
               std::tan(x[control_index] / wheelbase_) *
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)));  // b
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          x[time_index] * ts_ *
          (x[state_index + 3] +
           x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
          std::sin(x[state_index + 2] +
                   x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                       std::tan(x[control_index] / wheelbase_)) *
          x[time_index] * ts_ * 0.5 * x[state_index + 3] /
          (std::cos(x[control_index] / wheelbase_) *
           std::cos(x[control_index] / wheelbase_)) /
          wheelbase_;  // c
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
           x[time_index] * ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               x[time_index] * ts_ * 0.5 *
               std::tan(x[control_index] / wheelbase_) *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)));  // g
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ *
                  (x[state_index + 3] +
                   x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
                  std::cos(x[state_index + 2] +
                           x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                               std::tan(x[control_index] / wheelbase_)) *
                  x[time_index] * ts_ * 0.5 * x[state_index + 3] /
                  (std::cos(x[control_index] / wheelbase_) *
                   std::cos(x[control_index] / wheelbase_)) /
                  wheelbase_);  // h
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
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index] / wheelbase_)) +
           x[time_index] * ts_ * ts_ * 0.5 * x[control_index + 1] *
               std::sin(x[state_index + 2] +
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

      values[nz_index] = -1.0 * x[time_index] * ts_ *
                         std::tan(x[control_index] / wheelbase_);  // k.
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ *
                  (x[state_index + 3] +
                   x[time_index] * ts_ * 0.5 * x[control_index + 1]) /
                  (std::cos(x[control_index] / wheelbase_) *
                   std::cos(x[control_index] / wheelbase_)) /
                  wheelbase_);  // l.
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ * x[time_index] * ts_ * 0.5 *
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

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] = -1.0 * ts_ * x[time_index];  // o.
      ++nz_index;

      values[nz_index] = -1.0 * ts_ * x[control_index + 1];  // p.
      ++nz_index;

      state_index += 4;
      control_index += 2;
      time_index += 1;
    }

    ADEBUG << "After fulfilled dynamics constraints derivative, nz_index : "
           << nz_index << " nele_jac : " << nele_jac;

    // 2. control rate constraints 1 * [0, horizons-1]
    control_index = control_start_index_;
    state_index = state_start_index_;
    time_index = time_start_index_;

    // First horizon

    // with respect to u(0, 0)
    values[nz_index] = 1.0 / x[time_index] / ts_;
    ++nz_index;

    // with respect to time
    values[nz_index] = -2 * (x[control_index] - last_time_u_(0, 0)) /
                       x[time_index] / x[time_index] / ts_;
    ++nz_index;
    time_index += 1;
    control_index += 2;

    for (std::size_t i = 1; i < horizon_; ++i) {
      // with respect to u(0, i-1)

      values[nz_index] = -1.0 / x[time_index] / ts_;
      ++nz_index;

      // with respect to u(0, i)
      values[nz_index] = 1.0 / x[time_index] / ts_;
      ++nz_index;

      // with respect to time
      values[nz_index] = -1.0 * (x[control_index] - x[control_index - 2]) /
                         x[time_index] / x[time_index] / ts_;
      ++nz_index;

      control_index += 2;
      time_index += 1;
    }

    ADEBUG << "After fulfilled control rate constraints derivative, nz_index : "
           << nz_index << " nele_jac : " << nele_jac;

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

    ADEBUG << "After fulfilled time constraints derivative, nz_index : "
           << nz_index << " nele_jac : " << nele_jac;

    // 4. Three obstacles related equal constraints, one equality constraints,
    // [0, horizon_] * [0, obstacles_num_-1] * 4

    state_index = state_start_index_;
    std::size_t l_index = l_start_index_;
    std::size_t n_index = n_start_index_;

    for (std::size_t i = 0; i < horizon_ + 1; ++i) {
      std::size_t edges_counter = 0;
      for (std::size_t j = 0; j < obstacles_num_; ++j) {
        std::size_t current_edges_num = obstacles_edges_num_(j, 0);
        Eigen::MatrixXd Aj =
            obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
        Eigen::MatrixXd bj =
            obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

        // TODO(QiL) : Remove redudant calculation
        double tmp1 = 0;
        double tmp2 = 0;
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          // TODO(QiL) : replace this one directly with x
          tmp1 += Aj(k, 0) * x[l_index + k];
          tmp2 += Aj(k, 1) * x[l_index + k];
        }

        // 1. norm(A* lambda == 1)
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          // with respect to l
          values[nz_index] =
              2 * tmp1 * Aj(k, 0) + 2 * tmp2 * Aj(k, 1);  // t0~tk
          ++nz_index;
        }

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

        // 3. G' * mu + R' * lambda == 0, part 2
        // With respect to x
        values[nz_index] = -std::cos(x[state_index + 2]) * tmp1 -
                           std::sin(x[state_index + 2]) * tmp2;  // x
        ++nz_index;

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          values[nz_index] = -std::sin(x[state_index + 2]) * Aj(k, 0) -
                             std::sin(x[state_index + 2]) * Aj(k, 1);  // y0~yn
          ++nz_index;
        }

        // With respect to n
        values[nz_index] = 1.0;  // z1
        ++nz_index;

        values[nz_index] = -1.0;  // z3
        ++nz_index;

        //  3. -g'*mu + (A*t - b)*lambda > 0
        double tmp3 = 0.0;
        double tmp4 = 0.0;
        for (std::size_t k = 0; k < 4; ++k) {
          tmp3 += -g_[k] * x[n_index + k];
        }

        for (std::size_t k = 0; k < current_edges_num; ++k) {
          tmp4 += bj(k, 0) * x[l_index + k];
        }

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

        // Update index
        edges_counter += current_edges_num;
        l_index += current_edges_num;
        n_index += 4;
      }
      state_index += 4;
    }

    ADEBUG << "eval_jac_g, fulfilled obstacle constraint values";
    CHECK_EQ(nz_index, static_cast<std::size_t>(nele_jac));
  }

  ADEBUG << "eval_jac_g done";
  return true;
}

bool DistanceApproachIPOPTInterface::eval_h(int n, const double* x, bool new_x,
                                            double obj_factor, int m,
                                            const double* lambda,
                                            bool new_lambda, int nele_hess,
                                            int* iRow, int* jCol,
                                            double* values) {
  ADEBUG << "eval_h";
  return false;
}

bool DistanceApproachIPOPTInterface::eval_f(int n, const double* x, bool new_x,
                                            double& obj_value) {
  ADEBUG << "eval_f";
  // Objective is :
  // min control inputs
  // min input rate
  // min time (if the time step is not fixed)
  // regularization wrt warm start trajectory
  DCHECK(ts_ != 0) << "ts in distance_approach_ is 0";
  std::size_t control_index = control_start_index_;
  std::size_t time_index = time_start_index_;
  std::size_t state_index = state_start_index_;

  // TODO(QiL): Initial implementation towards earlier understanding and debug
  // purpose, later code refine towards improving efficiency

  obj_value = 0.0;
  // 1. objective to minimize state diff to warm up
  state_index = state_start_index_;
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    double x1_diff = x[state_index] - xWS_(0, i);
    double x2_diff = x[state_index + 1] - xWS_(1, i);
    double x3_diff = x[state_index + 2] - xWS_(2, i);
    obj_value += weight_state_x_ * x1_diff * x1_diff +
                 weight_state_y_ * x2_diff * x2_diff +
                 weight_state_phi_ * x3_diff * x3_diff;
    state_index += 4;
  }

  // 2. objective to minimize u square
  control_index = control_start_index_;
  for (std::size_t i = 0; i < horizon_; ++i) {
    obj_value += weight_input_steer_ * x[control_index] * x[control_index] +
                 weight_input_a_ * x[control_index + 1] * x[control_index + 1];
    control_index += 2;
  }

  // 3. objective to minimize input change rate for first horizon
  control_index = control_start_index_;
  time_index = time_start_index_;
  double last_time_steer_rate = (x[control_start_index_] - last_time_u_(0, 0)) /
                                x[time_start_index_] / ts_;
  double last_time_a_rate = (x[control_start_index_ + 1] - last_time_u_(1, 0)) /
                            x[time_start_index_] / ts_;
  obj_value +=
      weight_stitching_steer_ * last_time_steer_rate * last_time_steer_rate +
      weight_stitching_a_ * last_time_a_rate * last_time_a_rate;

  // 4. objective to minimize input change rates, [0- horizon_ -2]
  for (std::size_t i = 0; i < horizon_ - 1; ++i) {
    double steering_rate =
        (x[control_index + 2] - x[control_index]) / x[time_index] / ts_;
    double a_rate =
        (x[control_index + 3] - x[control_index + 1]) / x[time_index] / ts_;
    obj_value += weight_rate_steer_ * steering_rate * steering_rate +
                 weight_rate_a_ * a_rate * a_rate;
    control_index += 2;
    time_index += 1;
  }

  // 5. objective to minimize total time [0, horizon_]
  time_index = time_start_index_;
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    double first_order_penalty = weight_first_order_time_ * x[time_index];
    double second_order_penalty =
        weight_second_order_time_ * x[time_index] * x[time_index];
    obj_value += first_order_penalty + second_order_penalty;
    time_index += 1;
  }

  ADEBUG << "objective value after this iteration : " << obj_value;
  return true;
}

bool DistanceApproachIPOPTInterface::eval_grad_f(int n, const double* x,
                                                 bool new_x, double* grad_f) {
  ADEBUG << "eval_grad_f";
  std::fill(grad_f, grad_f + n, 0.0);
  std::size_t control_index = control_start_index_;
  std::size_t time_index = time_start_index_;
  std::size_t state_index = state_start_index_;
  std::size_t l_index = l_start_index_;
  std::size_t n_index = n_start_index_;

  // 1. Gradients on states
  // a. From minimizing difference from warm start, [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; i++) {
    grad_f[state_index] = weight_state_x_ * 2 * (x[state_index] - xWS_(0, i));
    grad_f[state_index + 1] =
        weight_state_y_ * 2.0 * (x[state_index + 1] - xWS_(1, i));
    grad_f[state_index + 2] =
        weight_state_phi_ * 2.0 * (x[state_index + 2] - xWS_(2, i));
    grad_f[state_index + 3] = 0.0;
    state_index += 4;
  }

  ADEBUG << "grad_f last index after over states " << state_index;
  // 2. Gradients on control.
  // a. from minimizing abosulte value square
  control_index = control_start_index_;
  time_index = time_start_index_;
  state_index = state_start_index_;
  for (std::size_t i = 0; i < horizon_; i++) {
    grad_f[control_index] += weight_input_steer_ * 2 * x[control_index];
    grad_f[control_index + 1] += weight_input_a_ * 2 * x[control_index + 1];
    control_index += 2;
  }

  ADEBUG << "grad_f last index after over controls abs squares "
         << control_index;

  // b. from change rate first horizon.
  control_index = control_start_index_;
  time_index = time_start_index_;
  grad_f[control_start_index_] +=
      weight_stitching_steer_ *
          (2 * x[control_start_index_] - 2 * last_time_u_(0, 0)) /
          (ts_ * ts_ * x[time_start_index_] * x[time_start_index_]) +
      weight_rate_steer_ *
          (-2 * x[control_start_index_ + 2] + 2 * x[control_start_index_]) /
          (ts_ * ts_ * x[time_start_index_] * x[time_start_index_]);

  grad_f[control_start_index_ + 1] +=
      weight_stitching_a_ *
          (2 * x[control_start_index_ + 1] - 2 * last_time_u_(1, 0)) /
          (ts_ * ts_ * x[time_start_index_] * x[time_start_index_]) +
      weight_rate_a_ *
          (-2 * x[control_start_index_ + 3] + 2 * x[control_start_index_ + 1]) /
          (ts_ * ts_ * x[time_start_index_] * x[time_start_index_]);

  control_index += 2;
  time_index += 1;

  // c. from change rate horizon [1, horizon-2]
  for (std::size_t i = 0; i < horizon_ - 2; i++) {
    grad_f[control_index] +=
        weight_rate_steer_ *
        ((-2 * x[control_index + 2] + 2 * x[control_index]) /
             (ts_ * ts_ * x[time_index] * x[time_index]) +
         (-2 * x[control_index - 2] + 2 * x[control_index]) /
             (ts_ * ts_ * x[time_index - 1] * x[time_index - 1]));

    grad_f[control_index + 1] +=
        weight_rate_a_ *
        ((-2 * x[control_index + 3] + 2 * x[control_index + 1]) /
             (ts_ * ts_ * x[time_index] * x[time_index]) +
         (-2 * x[control_index - 1] + 2 * x[control_index + 1]) /
             (ts_ * ts_ * x[time_index - 1] * x[time_index - 1]));
    control_index += 2;
    time_index += 1;
  }

  // d. from change rate last horizon.
  grad_f[control_index] += weight_rate_steer_ * 2 *
                           (-x[control_index - 2] + x[control_index]) /
                           (ts_ * ts_ * x[time_index - 1] * x[time_index - 1]);

  grad_f[control_index + 1] +=
      weight_rate_a_ * (-2 * x[control_index - 1] + 2 * x[control_index + 1]) /
      (ts_ * ts_ * x[time_index - 1] * x[time_index - 1]);

  ADEBUG << "grad_f last index after over controls change rate"
         << control_index;

  // 3. Grdients over time scale
  time_index = time_start_index_;
  state_index = state_start_index_;
  // a. from  control rate change, first horizon
  grad_f[time_start_index_] +=
      -2 * weight_stitching_steer_ *
          ((last_time_u_(0, 0) * last_time_u_(0, 0) +
            x[control_start_index_] * x[control_start_index_] -
            2 * last_time_u_(0, 0) * x[control_start_index_]) /
           (ts_ * ts_ * x[time_start_index_] * x[time_start_index_] *
            x[time_start_index_])) -
      2 * weight_stitching_a_ *
          ((last_time_u_(1, 0) * last_time_u_(1, 0) +
            x[control_start_index_ + 1] * x[control_start_index_ + 1] -
            2 * last_time_u_(1, 0) * x[control_start_index_ + 1]) /
           (ts_ * ts_ * x[time_start_index_] * x[time_start_index_] *
            x[time_start_index_]));

  // from gradients of control rate, horizon [0, horizon-2]
  time_index = time_start_index_;
  state_index = state_start_index_;
  for (std::size_t i = 0; i < horizon_ - 1; i++) {
    grad_f[time_index] +=
        -2 * weight_rate_steer_ *
            ((x[control_index] * x[control_index] +
              x[control_index + 2] * x[control_index + 2] -
              2 * x[control_index] * x[control_index + 2]) /
             (ts_ * ts_ * x[time_index] * x[time_index] * x[time_index])) -
        2 * weight_rate_a_ *
            ((x[control_index + 1] * x[control_index + 1] +
              x[control_index + 3] * x[control_index + 3] -
              2 * x[control_index + 1] * x[control_index + 3]) /
             (ts_ * ts_ * x[time_index] * x[time_index] * x[time_index]));
    control_index += 2;
    time_index += 1;
  }

  // from time scale minimization
  for (std::size_t i = 0; i < horizon_ + 1; i++) {
    grad_f[time_start_index_ + i] =
        weight_first_order_time_ +
        2 * weight_second_order_time_ * x[time_start_index_ + i];
  }

  // 4. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]

  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_edges_sum_; ++j) {
      grad_f[l_index] = 0.0;
      ++l_index;
    }
  }

  // 5. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < 4 * obstacles_num_; ++j) {
      grad_f[n_index] = 0.0;
      ++n_index;
    }
  }

  CHECK_EQ(n, n_index) << "No. of variables wrong in eval_grad_f. n : " << n;
  ADEBUG << "eval_grad_f done";
  return true;
}

void DistanceApproachIPOPTInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  ADEBUG << "finalize_solution";
  std::size_t state_index = state_start_index_;
  std::size_t control_index = control_start_index_;
  std::size_t time_index = time_start_index_;
  // 1. state variables, 4 * [0, horizon]
  // 2. control variables, 2 * [0, horizon_-1]

  // 3. sampling time variables, 1 * [0, horizon_]
  for (std::size_t i = 0; i < horizon_; ++i) {
    state_result_(0, i) = x[state_index];
    state_result_(1, i) = x[state_index + 1];
    state_result_(2, i) = x[state_index + 2];
    state_result_(3, i) = x[state_index + 3];

    control_result_(0, i) = x[control_index];
    control_result_(1, i) = x[control_index];

    time_result_(0, i) = x[time_index];
    state_index += 4;
    control_index += 2;
    time_index += 1;
  }
  ADEBUG << "finalize_solution horizon done!";
  // push back last horizon for state and time variables
  state_result_(0, horizon_) = x[state_index];
  state_result_(1, horizon_) = x[state_index + 1];
  state_result_(2, horizon_) = x[state_index + 2];
  state_result_(3, horizon_) = x[state_index + 3];

  time_result_(0, horizon_) = x[time_index];
  ADEBUG << "finalize_solution done!";
}

void DistanceApproachIPOPTInterface::get_optimization_results(
    Eigen::MatrixXd* state_result, Eigen::MatrixXd* control_result,
    Eigen::MatrixXd* time_result) const {
  ADEBUG << "get_optimization_results";
  *state_result = state_result_;
  *control_result = control_result_;
  *time_result = time_result_;
}

}  // namespace planning
}  // namespace apollo
