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
#include "modules/planning/open_space/trajectory_smoother/distance_approach_ipopt_interface.h"

namespace apollo {
namespace planning {

DistanceApproachIPOPTInterface::DistanceApproachIPOPTInterface(
    const size_t horizon, const double ts, const Eigen::MatrixXd& ego,
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
    const Eigen::MatrixXd& x0, const Eigen::MatrixXd& xf,
    const Eigen::MatrixXd& last_time_u, const std::vector<double>& XYbounds,
    const Eigen::MatrixXi& obstacles_edges_num, const size_t obstacles_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const PlannerOpenSpaceConfig& planner_open_space_config)
    : ts_(ts),
      ego_(ego),
      xWS_(xWS),
      uWS_(uWS),
      l_warm_up_(l_warm_up),
      n_warm_up_(n_warm_up),
      x0_(x0),
      xf_(xf),
      last_time_u_(last_time_u),
      XYbounds_(XYbounds),
      obstacles_edges_num_(obstacles_edges_num),
      obstacles_A_(obstacles_A),
      obstacles_b_(obstacles_b) {
  ACHECK(horizon < std::numeric_limits<int>::max())
      << "Invalid cast on horizon in open space planner";
  horizon_ = static_cast<int>(horizon);
  ACHECK(obstacles_num < std::numeric_limits<int>::max())
      << "Invalid cast on obstacles_num in open space planner";

  obstacles_num_ = static_cast<int>(obstacles_num);
  w_ev_ = ego_(1, 0) + ego_(3, 0);
  l_ev_ = ego_(0, 0) + ego_(2, 0);
  g_ = {l_ev_ / 2, w_ev_ / 2, l_ev_ / 2, w_ev_ / 2};
  offset_ = (ego_(0, 0) + ego_(2, 0)) / 2 - ego_(2, 0);
  obstacles_edges_sum_ = obstacles_edges_num_.sum();
  state_result_ = Eigen::MatrixXd::Zero(4, horizon_ + 1);
  dual_l_result_ = Eigen::MatrixXd::Zero(obstacles_edges_sum_, horizon_ + 1);
  dual_n_result_ = Eigen::MatrixXd::Zero(4 * obstacles_num_, horizon_ + 1);
  control_result_ = Eigen::MatrixXd::Zero(2, horizon_);
  time_result_ = Eigen::MatrixXd::Zero(1, horizon_);
  state_start_index_ = 0;
  control_start_index_ = 4 * (horizon_ + 1);
  time_start_index_ = control_start_index_ + 2 * horizon_;
  l_start_index_ = time_start_index_ + (horizon_ + 1);
  n_start_index_ = l_start_index_ + obstacles_edges_sum_ * (horizon_ + 1);

  distance_approach_config_ =
      planner_open_space_config.distance_approach_config();
  weight_state_x_ = distance_approach_config_.weight_x();
  weight_state_y_ = distance_approach_config_.weight_y();
  weight_state_phi_ = distance_approach_config_.weight_phi();
  weight_state_v_ = distance_approach_config_.weight_v();
  weight_input_steer_ = distance_approach_config_.weight_steer();
  weight_input_a_ = distance_approach_config_.weight_a();
  weight_rate_steer_ = distance_approach_config_.weight_steer_rate();
  weight_rate_a_ = distance_approach_config_.weight_a_rate();
  weight_stitching_steer_ = distance_approach_config_.weight_steer_stitching();
  weight_stitching_a_ = distance_approach_config_.weight_a_stitching();
  weight_first_order_time_ =
      distance_approach_config_.weight_first_order_time();
  weight_second_order_time_ =
      distance_approach_config_.weight_second_order_time();
  min_safety_distance_ = distance_approach_config_.min_safety_distance();
  max_steer_angle_ =
      vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio();
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
  max_steer_rate_ =
      vehicle_param_.max_steer_angle_rate() / vehicle_param_.steer_ratio();
  use_fix_time_ = distance_approach_config_.use_fix_time();
  wheelbase_ = vehicle_param_.wheel_base();
  enable_constraint_check_ =
      distance_approach_config_.enable_constraint_check();
  enable_jacobian_ad_ = distance_approach_config_.enable_jacobian_ad();
}

bool DistanceApproachIPOPTInterface::get_nlp_info(int& n, int& m,
                                                  int& nnz_jac_g,
                                                  int& nnz_h_lag,
                                                  IndexStyleEnum& index_style) {
  ADEBUG << "get_nlp_info";
  // n1 : states variables, 4 * (N+1)
  int n1 = 4 * (horizon_ + 1);
  ADEBUG << "n1: " << n1;
  // n2 : control inputs variables
  int n2 = 2 * horizon_;
  ADEBUG << "n2: " << n2;
  // n3 : sampling time variables
  int n3 = horizon_ + 1;
  ADEBUG << "n3: " << n3;
  // n4 : dual multiplier associated with obstacle shape
  lambda_horizon_ = obstacles_edges_num_.sum() * (horizon_ + 1);
  ADEBUG << "lambda_horizon_: " << lambda_horizon_;
  // n5 : dual multipier associated with car shape, obstacles_num*4 * (N+1)
  miu_horizon_ = obstacles_num_ * 4 * (horizon_ + 1);
  ADEBUG << "miu_horizon_: " << miu_horizon_;

  // m1 : dynamics constatins
  int m1 = 4 * horizon_;
  // m2 : control rate constraints (only steering)
  int m2 = horizon_;
  // m3 : sampling time equality constraints
  int m3 = horizon_;
  // m4 : obstacle constraints
  int m4 = 4 * obstacles_num_ * (horizon_ + 1);

  num_of_variables_ = n1 + n2 + n3 + lambda_horizon_ + miu_horizon_;
  num_of_constraints_ =
      m1 + m2 + m3 + m4 + (num_of_variables_ - (horizon_ + 1) + 2);

  // number of variables
  n = num_of_variables_;
  ADEBUG << "num_of_variables_ " << num_of_variables_;
  // number of constraints
  m = num_of_constraints_;
  ADEBUG << "num_of_constraints_ " << num_of_constraints_;

  generate_tapes(n, m, &nnz_jac_g, &nnz_h_lag);
  // number of nonzero in Jacobian.
  if (!enable_jacobian_ad_) {
    int tmp = 0;
    for (int i = 0; i < horizon_ + 1; ++i) {
      for (int j = 0; j < obstacles_num_; ++j) {
        int current_edges_num = obstacles_edges_num_(j, 0);
        tmp += current_edges_num * 4 + 9 + 4;
      }
    }
    nnz_jac_g = 24 * horizon_ + 3 * horizon_ + 2 * horizon_ + tmp - 1 +
                (num_of_variables_ - (horizon_ + 1) + 2);
  }

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool DistanceApproachIPOPTInterface::get_bounds_info(int n, double* x_l,
                                                     double* x_u, int m,
                                                     double* g_l, double* g_u) {
  ADEBUG << "get_bounds_info";
  ACHECK(XYbounds_.size() == 4)
      << "XYbounds_ size is not 4, but" << XYbounds_.size();

  // Variables: includes state, u, sample time and lagrange multipliers
  // 1. state variables, 4 * [0, horizon]
  // start point pose
  int variable_index = 0;
  for (int i = 0; i < 4; ++i) {
    x_l[i] = -2e19;
    x_u[i] = 2e19;
  }
  variable_index += 4;

  // During horizons, 2 ~ N-1
  for (int i = 1; i < horizon_; ++i) {
    // x
    x_l[variable_index] = -2e19;
    x_u[variable_index] = 2e19;

    // y
    x_l[variable_index + 1] = -2e19;
    x_u[variable_index + 1] = 2e19;

    // phi
    x_l[variable_index + 2] = -2e19;
    x_u[variable_index + 2] = 2e19;

    // v
    x_l[variable_index + 3] = -2e19;
    x_u[variable_index + 3] = 2e19;

    variable_index += 4;
  }

  // end point pose
  for (int i = 0; i < 4; ++i) {
    x_l[variable_index + i] = -2e19;
    x_u[variable_index + i] = 2e19;
  }
  variable_index += 4;
  ADEBUG << "variable_index after adding state variables : " << variable_index;

  // 2. control variables, 2 * [0, horizon_-1]
  for (int i = 0; i < horizon_; ++i) {
    // steer
    x_l[variable_index] = -2e19;
    x_u[variable_index] = 2e19;

    // a
    x_l[variable_index + 1] = -2e19;
    x_u[variable_index + 1] = 2e19;

    variable_index += 2;
  }
  ADEBUG << "variable_index after adding control variables : "
         << variable_index;

  // 3. sampling time variables, 1 * [0, horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    x_l[variable_index] = -2e19;
    x_u[variable_index] = 2e19;
    ++variable_index;
  }
  ADEBUG << "variable_index after adding sample time : " << variable_index;
  ADEBUG << "sample time fix time status is : " << use_fix_time_;

  // 4. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      x_l[variable_index] = 0.0;
      x_u[variable_index] = 2e19;  // nlp_upper_bound_limit
      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange l : " << variable_index;

  // 5. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      x_l[variable_index] = 0.0;
      x_u[variable_index] = 2e19;  // nlp_upper_bound_limit

      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange n : " << variable_index;

  // Constraints: includes four state Euler forward constraints, three
  // Obstacle related constraints

  // 1. dynamics constraints 4 * [0, horizons-1]
  int constraint_index = 0;
  for (int i = 0; i < 4 * horizon_; ++i) {
    g_l[i] = 0.0;
    g_u[i] = 0.0;
  }
  constraint_index += 4 * horizon_;

  ADEBUG << "constraint_index after adding Euler forward dynamics constraints: "
         << constraint_index;

  // 2. Control rate limit constraints, 1 * [0, horizons-1], only apply
  // steering rate as of now
  for (int i = 0; i < horizon_; ++i) {
    g_l[constraint_index] = -max_steer_rate_;
    g_u[constraint_index] = max_steer_rate_;
    ++constraint_index;
  }

  ADEBUG << "constraint_index after adding steering rate constraints: "
         << constraint_index;

  // 3. Time constraints 1 * [0, horizons-1]
  for (int i = 0; i < horizon_; ++i) {
    g_l[constraint_index] = 0.0;
    g_u[constraint_index] = 0.0;
    ++constraint_index;
  }

  ADEBUG << "constraint_index after adding time constraints: "
         << constraint_index;

  // 4. Three obstacles related equal constraints, one equality constraints,
  // [0, horizon_] * [0, obstacles_num_-1] * 4
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_num_; ++j) {
      // a. norm(A'*lambda) <= 1
      g_l[constraint_index] = -2e19;
      g_u[constraint_index] = 1.0;

      // b. G'*mu + R'*A*lambda = 0
      g_l[constraint_index + 1] = 0.0;
      g_u[constraint_index + 1] = 0.0;
      g_l[constraint_index + 2] = 0.0;
      g_u[constraint_index + 2] = 0.0;

      // c. -g'*mu + (A*t - b)*lambda > min_safety_distance_
      g_l[constraint_index + 3] = min_safety_distance_;
      g_u[constraint_index + 3] = 2e19;  // nlp_upper_bound_limit
      constraint_index += 4;
    }
  }

  // 5. load variable bounds as constraints
  // start configuration
  g_l[constraint_index] = x0_(0, 0);
  g_u[constraint_index] = x0_(0, 0);
  g_l[constraint_index + 1] = x0_(1, 0);
  g_u[constraint_index + 1] = x0_(1, 0);
  g_l[constraint_index + 2] = x0_(2, 0);
  g_u[constraint_index + 2] = x0_(2, 0);
  g_l[constraint_index + 3] = x0_(3, 0);
  g_u[constraint_index + 3] = x0_(3, 0);
  constraint_index += 4;

  for (int i = 1; i < horizon_; ++i) {
    g_l[constraint_index] = XYbounds_[0];
    g_u[constraint_index] = XYbounds_[1];
    g_l[constraint_index + 1] = XYbounds_[2];
    g_u[constraint_index + 1] = XYbounds_[3];
    g_l[constraint_index + 2] = -max_speed_reverse_;
    g_u[constraint_index + 2] = max_speed_forward_;
    constraint_index += 3;
  }

  // end configuration
  g_l[constraint_index] = xf_(0, 0);
  g_u[constraint_index] = xf_(0, 0);
  g_l[constraint_index + 1] = xf_(1, 0);
  g_u[constraint_index + 1] = xf_(1, 0);
  g_l[constraint_index + 2] = xf_(2, 0);
  g_u[constraint_index + 2] = xf_(2, 0);
  g_l[constraint_index + 3] = xf_(3, 0);
  g_u[constraint_index + 3] = xf_(3, 0);
  constraint_index += 4;

  for (int i = 0; i < horizon_; ++i) {
    g_l[constraint_index] = -max_steer_angle_;
    g_u[constraint_index] = max_steer_angle_;
    g_l[constraint_index + 1] = -max_acceleration_reverse_;
    g_u[constraint_index + 1] = max_acceleration_forward_;
    constraint_index += 2;
  }

  for (int i = 0; i < horizon_ + 1; ++i) {
    if (!use_fix_time_) {
      g_l[constraint_index] = min_time_sample_scaling_;
      g_u[constraint_index] = max_time_sample_scaling_;
    } else {
      g_l[constraint_index] = 1.0;
      g_u[constraint_index] = 1.0;
    }
    constraint_index++;
  }

  for (int i = 0; i < lambda_horizon_; ++i) {
    g_l[constraint_index] = 0.0;
    g_u[constraint_index] = 2e19;
    constraint_index++;
  }

  for (int i = 0; i < miu_horizon_; ++i) {
    g_l[constraint_index] = 0.0;
    g_u[constraint_index] = 2e19;
    constraint_index++;
  }

  ADEBUG << "constraint_index after adding obstacles constraints: "
         << constraint_index;
  ADEBUG << "get_bounds_info_ out";
  return true;
}

bool DistanceApproachIPOPTInterface::get_starting_point(
    int n, bool init_x, double* x, bool init_z, double* z_L, double* z_U, int m,
    bool init_lambda, double* lambda) {
  ADEBUG << "get_starting_point";
  ACHECK(init_x) << "Warm start init_x setting failed";

  CHECK_EQ(horizon_, uWS_.cols());
  CHECK_EQ(horizon_ + 1, xWS_.cols());

  // 1. state variables 4 * (horizon_ + 1)
  for (int i = 0; i < horizon_ + 1; ++i) {
    int index = i * 4;
    for (int j = 0; j < 4; ++j) {
      x[index + j] = xWS_(j, i);
    }
  }

  // 2. control variable initialization, 2 * horizon_
  for (int i = 0; i < horizon_; ++i) {
    int index = i * 2;
    x[control_start_index_ + index] = uWS_(0, i);
    x[control_start_index_ + index + 1] = uWS_(1, i);
  }

  // 2. time scale variable initialization, horizon_ + 1
  for (int i = 0; i < horizon_ + 1; ++i) {
    x[time_start_index_ + i] =
        0.5 * (min_time_sample_scaling_ + max_time_sample_scaling_);
  }

  // 3. lagrange constraint l, obstacles_edges_sum_ * (horizon_+1)
  for (int i = 0; i < horizon_ + 1; ++i) {
    int index = i * obstacles_edges_sum_;
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      x[l_start_index_ + index + j] = l_warm_up_(j, i);
    }
  }

  // 4. lagrange constraint m, 4*obstacles_num * (horizon_+1)
  for (int i = 0; i < horizon_ + 1; ++i) {
    int index = i * 4 * obstacles_num_;
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      x[n_start_index_ + index + j] = n_warm_up_(j, i);
    }
  }
  ADEBUG << "get_starting_point out";
  return true;
}

bool DistanceApproachIPOPTInterface::eval_f(int n, const double* x, bool new_x,
                                            double& obj_value) {
  eval_obj(n, x, &obj_value);
  return true;
}

bool DistanceApproachIPOPTInterface::eval_grad_f(int n, const double* x,
                                                 bool new_x, double* grad_f) {
  gradient(tag_f, n, x, grad_f);
  return true;
}

bool DistanceApproachIPOPTInterface::eval_g(int n, const double* x, bool new_x,
                                            int m, double* g) {
  eval_constraints(n, x, m, g);
  // if (enable_constraint_check_) check_g(n, x, m, g);
  return true;
}

bool DistanceApproachIPOPTInterface::eval_jac_g(int n, const double* x,
                                                bool new_x, int m, int nele_jac,
                                                int* iRow, int* jCol,
                                                double* values) {
  if (enable_jacobian_ad_) {
    if (values == nullptr) {
      // return the structure of the jacobian
      for (int idx = 0; idx < nnz_jac; idx++) {
        iRow[idx] = rind_g[idx];
        jCol[idx] = cind_g[idx];
      }
    } else {
      // return the values of the jacobian of the constraints
      sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval,
                 options_g);
      for (int idx = 0; idx < nnz_jac; idx++) {
        values[idx] = jacval[idx];
      }
    }
    return true;
  } else {
    return eval_jac_g_ser(n, x, new_x, m, nele_jac, iRow, jCol, values);
  }
}

bool DistanceApproachIPOPTInterface::eval_jac_g_ser(int n, const double* x,
                                                    bool new_x, int m,
                                                    int nele_jac, int* iRow,
                                                    int* jCol, double* values) {
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
    for (int i = 0; i < horizon_; ++i) {
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
      iRow[nz_index] = state_index + 3;
      jCol[nz_index] = control_index + 1;
      ++nz_index;

      // g(3)' with respect to time
      iRow[nz_index] = state_index + 3;
      jCol[nz_index] = time_index;
      ++nz_index;

      state_index += 4;
      control_index += 2;
      time_index++;
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
    time_index++;
    constraint_index++;

    for (int i = 1; i < horizon_; ++i) {
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
      constraint_index++;
      time_index++;
    }

    // 3. Time constraints [0, horizon_ -1]
    time_index = time_start_index_;

    for (int i = 0; i < horizon_; ++i) {
      // with respect to timescale(0, i-1)
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = time_index;
      ++nz_index;

      // with respect to timescale(0, i)
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = time_index + 1;
      ++nz_index;

      time_index++;
      constraint_index++;
    }

    // 4. Three obstacles related equal constraints, one equality constraints,
    // [0, horizon_] * [0, obstacles_num_-1] * 4
    state_index = state_start_index_;
    int l_index = l_start_index_;
    int n_index = n_start_index_;
    for (int i = 0; i < horizon_ + 1; ++i) {
      for (int j = 0; j < obstacles_num_; ++j) {
        int current_edges_num = obstacles_edges_num_(j, 0);

        // 1. norm(A* lambda == 1)
        for (int k = 0; k < current_edges_num; ++k) {
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
        for (int k = 0; k < current_edges_num; ++k) {
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
        ++nz_index;

        // with respect to l
        for (int k = 0; k < current_edges_num; ++k) {
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

        //  -g'*mu + (A*t - b)*lambda > 0
        // With respect to x
        iRow[nz_index] = constraint_index + 3;
        jCol[nz_index] = state_index;
        ++nz_index;

        iRow[nz_index] = constraint_index + 3;
        jCol[nz_index] = state_index + 1;
        ++nz_index;

        iRow[nz_index] = constraint_index + 3;
        jCol[nz_index] = state_index + 2;
        ++nz_index;

        // with respect to l
        for (int k = 0; k < current_edges_num; ++k) {
          iRow[nz_index] = constraint_index + 3;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }

        // with respect to n
        for (int k = 0; k < 4; ++k) {
          iRow[nz_index] = constraint_index + 3;
          jCol[nz_index] = n_index + k;
          ++nz_index;
        }

        // Update index
        l_index += current_edges_num;
        n_index += 4;
        constraint_index += 4;
      }
      state_index += 4;
    }

    // 5. load variable bounds as constraints
    state_index = state_start_index_;
    control_index = control_start_index_;
    time_index = time_start_index_;
    l_index = l_start_index_;
    n_index = n_start_index_;

    // start configuration
    iRow[nz_index] = constraint_index;
    jCol[nz_index] = state_index;
    nz_index++;
    iRow[nz_index] = constraint_index + 1;
    jCol[nz_index] = state_index + 1;
    nz_index++;
    iRow[nz_index] = constraint_index + 2;
    jCol[nz_index] = state_index + 2;
    nz_index++;
    iRow[nz_index] = constraint_index + 3;
    jCol[nz_index] = state_index + 3;
    nz_index++;
    constraint_index += 4;
    state_index += 4;

    for (int i = 1; i < horizon_; ++i) {
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = state_index;
      nz_index++;
      iRow[nz_index] = constraint_index + 1;
      jCol[nz_index] = state_index + 1;
      nz_index++;
      iRow[nz_index] = constraint_index + 2;
      jCol[nz_index] = state_index + 3;
      nz_index++;
      constraint_index += 3;
      state_index += 4;
    }

    // end configuration
    iRow[nz_index] = constraint_index;
    jCol[nz_index] = state_index;
    nz_index++;
    iRow[nz_index] = constraint_index + 1;
    jCol[nz_index] = state_index + 1;
    nz_index++;
    iRow[nz_index] = constraint_index + 2;
    jCol[nz_index] = state_index + 2;
    nz_index++;
    iRow[nz_index] = constraint_index + 3;
    jCol[nz_index] = state_index + 3;
    nz_index++;
    constraint_index += 4;
    state_index += 4;

    for (int i = 0; i < horizon_; ++i) {
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = control_index;
      nz_index++;
      iRow[nz_index] = constraint_index + 1;
      jCol[nz_index] = control_index + 1;
      nz_index++;
      constraint_index += 2;
      control_index += 2;
    }

    for (int i = 0; i < horizon_ + 1; ++i) {
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = time_index;
      nz_index++;
      constraint_index++;
      time_index++;
    }

    for (int i = 0; i < lambda_horizon_; ++i) {
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = l_index;
      nz_index++;
      constraint_index++;
      l_index++;
    }

    for (int i = 0; i < miu_horizon_; ++i) {
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = n_index;
      nz_index++;
      constraint_index++;
      n_index++;
    }

    CHECK_EQ(nz_index, static_cast<int>(nele_jac));
    CHECK_EQ(constraint_index, static_cast<int>(m));
  } else {
    std::fill(values, values + nele_jac, 0.0);
    int nz_index = 0;

    int time_index = time_start_index_;
    int state_index = state_start_index_;
    int control_index = control_start_index_;

    // TODO(QiL) : initially implemented to be debug friendly, later iterate
    // towards better efficiency
    // 1. state constraints 4 * [0, horizons-1]
    for (int i = 0; i < horizon_; ++i) {
      values[nz_index] = -1.0;
      ++nz_index;

      values[nz_index] =
          x[time_index] * ts_ *
          (x[state_index + 3] +
           x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
          std::sin(x[state_index + 2] +
                   x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                       std::tan(x[control_index]) / wheelbase_);  // a.
      ++nz_index;

      values[nz_index] =
          -1.0 *
          (x[time_index] * ts_ *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_) +
           x[time_index] * ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               (-1) * x[time_index] * ts_ * 0.5 * std::tan(x[control_index]) /
               wheelbase_ *
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_));  // b
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          x[time_index] * ts_ *
          (x[state_index + 3] +
           x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
          std::sin(x[state_index + 2] +
                   x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                       std::tan(x[control_index]) / wheelbase_) *
          x[time_index] * ts_ * 0.5 * x[state_index + 3] /
          (std::cos(x[control_index]) * std::cos(x[control_index])) /
          wheelbase_;  // c
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ * x[time_index] * ts_ * 0.5 *
                  std::cos(x[state_index + 2] +
                           x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                               std::tan(x[control_index]) / wheelbase_));  // d
      ++nz_index;

      values[nz_index] =
          -1.0 *
          (ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_) +
           x[time_index] * ts_ * ts_ * 0.5 * x[control_index + 1] *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_) -
           x[time_index] * ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               x[state_index + 3] * ts_ * 0.5 * std::tan(x[control_index]) /
               wheelbase_ *
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_));  // e
      ++nz_index;

      values[nz_index] = -1.0;
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ *
                  (x[state_index + 3] +
                   x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
                  std::cos(x[state_index + 2] +
                           x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                               std::tan(x[control_index]) / wheelbase_));  // f.
      ++nz_index;

      values[nz_index] =
          -1.0 *
          (x[time_index] * ts_ *
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_) +
           x[time_index] * ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               x[time_index] * ts_ * 0.5 * std::tan(x[control_index]) /
               wheelbase_ *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_));  // g
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ *
                  (x[state_index + 3] +
                   x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
                  std::cos(x[state_index + 2] +
                           x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                               std::tan(x[control_index]) / wheelbase_) *
                  x[time_index] * ts_ * 0.5 * x[state_index + 3] /
                  (std::cos(x[control_index]) * std::cos(x[control_index])) /
                  wheelbase_);  // h
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ * x[time_index] * ts_ * 0.5 *
                  std::sin(x[state_index + 2] +
                           x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                               std::tan(x[control_index]) / wheelbase_));  // i
      ++nz_index;

      values[nz_index] =
          -1.0 *
          (ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_) +
           x[time_index] * ts_ * ts_ * 0.5 * x[control_index + 1] *
               std::sin(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_) +
           x[time_index] * ts_ *
               (x[state_index + 3] +
                x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
               x[state_index + 3] * ts_ * 0.5 * std::tan(x[control_index]) /
               wheelbase_ *
               std::cos(x[state_index + 2] +
                        x[time_index] * ts_ * 0.5 * x[state_index + 3] *
                            std::tan(x[control_index]) / wheelbase_));  // j
      ++nz_index;

      values[nz_index] = -1.0;
      ++nz_index;

      values[nz_index] = -1.0 * x[time_index] * ts_ *
                         std::tan(x[control_index]) / wheelbase_;  // k.
      ++nz_index;

      values[nz_index] = 1.0;
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ *
                  (x[state_index + 3] +
                   x[time_index] * ts_ * 0.5 * x[control_index + 1]) /
                  (std::cos(x[control_index]) * std::cos(x[control_index])) /
                  wheelbase_);  // l.
      ++nz_index;

      values[nz_index] =
          -1.0 * (x[time_index] * ts_ * x[time_index] * ts_ * 0.5 *
                  std::tan(x[control_index]) / wheelbase_);  // m.
      ++nz_index;

      values[nz_index] =
          -1.0 * (ts_ *
                      (x[state_index + 3] +
                       x[time_index] * ts_ * 0.5 * x[control_index + 1]) *
                      std::tan(x[control_index]) / wheelbase_ +
                  x[time_index] * ts_ * ts_ * 0.5 * x[control_index + 1] *
                      std::tan(x[control_index]) / wheelbase_);  // n.
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
      time_index++;
    }

    // 2. control rate constraints 1 * [0, horizons-1]
    control_index = control_start_index_;
    state_index = state_start_index_;
    time_index = time_start_index_;

    // First horizon

    // with respect to u(0, 0)
    values[nz_index] = 1.0 / x[time_index] / ts_;  // q
    ++nz_index;

    // with respect to time
    values[nz_index] = -1.0 * (x[control_index] - last_time_u_(0, 0)) /
                       x[time_index] / x[time_index] / ts_;
    ++nz_index;
    time_index++;
    control_index += 2;

    for (int i = 1; i < horizon_; ++i) {
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
      time_index++;
    }

    ADEBUG << "After fulfilled control rate constraints derivative, nz_index : "
           << nz_index << " nele_jac : " << nele_jac;

    // 3. Time constraints [0, horizon_ -1]
    time_index = time_start_index_;
    for (int i = 0; i < horizon_; ++i) {
      // with respect to timescale(0, i-1)
      values[nz_index] = -1.0;
      ++nz_index;

      // with respect to timescale(0, i)
      values[nz_index] = 1.0;
      ++nz_index;

      time_index++;
    }

    ADEBUG << "After fulfilled time constraints derivative, nz_index : "
           << nz_index << " nele_jac : " << nele_jac;

    // 4. Three obstacles related equal constraints, one equality constraints,
    // [0, horizon_] * [0, obstacles_num_-1] * 4

    state_index = state_start_index_;
    int l_index = l_start_index_;
    int n_index = n_start_index_;

    for (int i = 0; i < horizon_ + 1; ++i) {
      int edges_counter = 0;
      for (int j = 0; j < obstacles_num_; ++j) {
        int current_edges_num = obstacles_edges_num_(j, 0);
        Eigen::MatrixXd Aj =
            obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
        Eigen::MatrixXd bj =
            obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

        // TODO(QiL) : Remove redundant calculation
        double tmp1 = 0;
        double tmp2 = 0;
        for (int k = 0; k < current_edges_num; ++k) {
          // TODO(QiL) : replace this one directly with x
          tmp1 += Aj(k, 0) * x[l_index + k];
          tmp2 += Aj(k, 1) * x[l_index + k];
        }

        // 1. norm(A* lambda == 1)
        for (int k = 0; k < current_edges_num; ++k) {
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
        for (int k = 0; k < current_edges_num; ++k) {
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
        for (int k = 0; k < current_edges_num; ++k) {
          values[nz_index] = -std::sin(x[state_index + 2]) * Aj(k, 0) +
                             std::cos(x[state_index + 2]) * Aj(k, 1);  // y0~yn
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
        for (int k = 0; k < 4; ++k) {
          tmp3 += -g_[k] * x[n_index + k];
        }

        for (int k = 0; k < current_edges_num; ++k) {
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
        for (int k = 0; k < current_edges_num; ++k) {
          values[nz_index] =
              (x[state_index] + std::cos(x[state_index + 2]) * offset_) *
                  Aj(k, 0) +
              (x[state_index + 1] + std::sin(x[state_index + 2]) * offset_) *
                  Aj(k, 1) -
              bj(k, 0);  // ddk
          ++nz_index;
        }

        // with respect to n
        for (int k = 0; k < 4; ++k) {
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

    // 5. load variable bounds as constraints
    state_index = state_start_index_;
    control_index = control_start_index_;
    time_index = time_start_index_;
    l_index = l_start_index_;
    n_index = n_start_index_;

    // start configuration
    values[nz_index] = 1.0;
    nz_index++;
    values[nz_index] = 1.0;
    nz_index++;
    values[nz_index] = 1.0;
    nz_index++;
    values[nz_index] = 1.0;
    nz_index++;

    for (int i = 1; i < horizon_; ++i) {
      values[nz_index] = 1.0;
      nz_index++;
      values[nz_index] = 1.0;
      nz_index++;
      values[nz_index] = 1.0;
      nz_index++;
    }

    // end configuration
    values[nz_index] = 1.0;
    nz_index++;
    values[nz_index] = 1.0;
    nz_index++;
    values[nz_index] = 1.0;
    nz_index++;
    values[nz_index] = 1.0;
    nz_index++;

    for (int i = 0; i < horizon_; ++i) {
      values[nz_index] = 1.0;
      nz_index++;
      values[nz_index] = 1.0;
      nz_index++;
    }

    for (int i = 0; i < horizon_ + 1; ++i) {
      values[nz_index] = 1.0;
      nz_index++;
    }

    for (int i = 0; i < lambda_horizon_; ++i) {
      values[nz_index] = 1.0;
      nz_index++;
    }

    for (int i = 0; i < miu_horizon_; ++i) {
      values[nz_index] = 1.0;
      nz_index++;
    }

    ADEBUG << "eval_jac_g, fulfilled obstacle constraint values";
    CHECK_EQ(nz_index, static_cast<int>(nele_jac));
  }

  ADEBUG << "eval_jac_g done";
  return true;
}  // NOLINT

bool DistanceApproachIPOPTInterface::eval_h(int n, const double* x, bool new_x,
                                            double obj_factor, int m,
                                            const double* lambda,
                                            bool new_lambda, int nele_hess,
                                            int* iRow, int* jCol,
                                            double* values) {
  if (values == nullptr) {
    // return the structure. This is a symmetric matrix, fill the lower left
    // triangle only.

    for (int idx = 0; idx < nnz_L; idx++) {
      iRow[idx] = rind_L[idx];
      jCol[idx] = cind_L[idx];
    }
  } else {
    // return the values. This is a symmetric matrix, fill the lower left
    // triangle only

    obj_lam[0] = obj_factor;

    for (int idx = 0; idx < m; idx++) {
      obj_lam[1 + idx] = lambda[idx];
    }

    set_param_vec(tag_L, m + 1, obj_lam);
    sparse_hess(tag_L, n, 1, const_cast<double*>(x), &nnz_L, &rind_L, &cind_L,
                &hessval, options_L);

    for (int idx = 0; idx < nnz_L; idx++) {
      values[idx] = hessval[idx];
    }
  }

  return true;
}

void DistanceApproachIPOPTInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  int state_index = state_start_index_;
  int control_index = control_start_index_;
  int time_index = time_start_index_;
  int dual_l_index = l_start_index_;
  int dual_n_index = n_start_index_;

  // enable_constraint_check_: for debug only
  if (enable_constraint_check_) {
    ADEBUG << "final resolution constraint checking";
    check_g(n, x, m, g);
  }
  // 1. state variables, 4 * [0, horizon]
  // 2. control variables, 2 * [0, horizon_-1]
  // 3. sampling time variables, 1 * [0, horizon_]
  // 4. dual_l obstacles_edges_sum_ * [0, horizon]
  // 5. dual_n obstacles_num * [0, horizon]
  for (int i = 0; i < horizon_; ++i) {
    state_result_(0, i) = x[state_index];
    state_result_(1, i) = x[state_index + 1];
    state_result_(2, i) = x[state_index + 2];
    state_result_(3, i) = x[state_index + 3];
    control_result_(0, i) = x[control_index];
    control_result_(1, i) = x[control_index + 1];
    time_result_(0, i) = ts_ * x[time_index];
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      dual_l_result_(j, i) = x[dual_l_index + j];
    }
    for (int k = 0; k < 4 * obstacles_num_; k++) {
      dual_n_result_(k, i) = x[dual_n_index + k];
    }
    state_index += 4;
    control_index += 2;
    time_index++;
    dual_l_index += obstacles_edges_sum_;
    dual_n_index += 4 * obstacles_num_;
  }
  state_result_(0, 0) = x0_(0, 0);
  state_result_(1, 0) = x0_(1, 0);
  state_result_(2, 0) = x0_(2, 0);
  state_result_(3, 0) = x0_(3, 0);
  // push back last horizon for states
  state_result_(0, horizon_) = xf_(0, 0);
  state_result_(1, horizon_) = xf_(1, 0);
  state_result_(2, horizon_) = xf_(2, 0);
  state_result_(3, horizon_) = xf_(3, 0);

  for (int j = 0; j < obstacles_edges_sum_; ++j) {
    dual_l_result_(j, horizon_) = x[dual_l_index + j];
  }
  for (int k = 0; k < 4 * obstacles_num_; k++) {
    dual_n_result_(k, horizon_) = x[dual_n_index + k];
  }
  // memory deallocation of ADOL-C variables
  delete[] obj_lam;
  if (enable_jacobian_ad_) {
    free(rind_g);
    free(cind_g);
    free(jacval);
  }
  free(rind_L);
  free(cind_L);
  free(hessval);
}

void DistanceApproachIPOPTInterface::get_optimization_results(
    Eigen::MatrixXd* state_result, Eigen::MatrixXd* control_result,
    Eigen::MatrixXd* time_result, Eigen::MatrixXd* dual_l_result,
    Eigen::MatrixXd* dual_n_result) const {
  ADEBUG << "get_optimization_results";
  *state_result = state_result_;
  *control_result = control_result_;
  *time_result = time_result_;
  *dual_l_result = dual_l_result_;
  *dual_n_result = dual_n_result_;

  if (!distance_approach_config_.enable_initial_final_check()) {
    return;
  }
  CHECK_EQ(state_result_.cols(), xWS_.cols());
  CHECK_EQ(state_result_.rows(), xWS_.rows());
  double state_diff_max = 0.0;
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4; ++j) {
      state_diff_max =
          std::max(std::abs(xWS_(j, i) - state_result_(j, i)), state_diff_max);
    }
  }

  // 2. control variable initialization, 2 * horizon_
  CHECK_EQ(control_result_.cols(), uWS_.cols());
  CHECK_EQ(control_result_.rows(), uWS_.rows());
  double control_diff_max = 0.0;
  for (int i = 0; i < horizon_; ++i) {
    control_diff_max = std::max(std::abs(uWS_(0, i) - control_result_(0, i)),
                                control_diff_max);
    control_diff_max = std::max(std::abs(uWS_(1, i) - control_result_(1, i)),
                                control_diff_max);
  }

  // 2. time scale variable initialization, horizon_ + 1

  // 3. lagrange constraint l, obstacles_edges_sum_ * (horizon_+1)
  CHECK_EQ(dual_l_result_.cols(), l_warm_up_.cols());
  CHECK_EQ(dual_l_result_.rows(), l_warm_up_.rows());
  double l_diff_max = 0.0;
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      l_diff_max = std::max(std::abs(l_warm_up_(j, i) - dual_l_result_(j, i)),
                            l_diff_max);
    }
  }

  // 4. lagrange constraint m, 4*obstacles_num * (horizon_+1)
  CHECK_EQ(n_warm_up_.cols(), dual_n_result_.cols());
  CHECK_EQ(n_warm_up_.rows(), dual_n_result_.rows());
  double n_diff_max = 0.0;
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      n_diff_max = std::max(std::abs(n_warm_up_(j, i) - dual_n_result_(j, i)),
                            n_diff_max);
    }
  }

  ADEBUG << "state_diff_max: " << state_diff_max;
  ADEBUG << "control_diff_max: " << control_diff_max;
  ADEBUG << "dual_l_diff_max: " << l_diff_max;
  ADEBUG << "dual_n_diff_max: " << n_diff_max;
}

//***************    start ADOL-C part ***********************************
template <class T>
void DistanceApproachIPOPTInterface::eval_obj(int n, const T* x, T* obj_value) {
  // Objective is :
  // min control inputs
  // min input rate
  // min time (if the time step is not fixed)
  // regularization wrt warm start trajectory
  DCHECK(ts_ != 0) << "ts in distance_approach_ is 0";
  int control_index = control_start_index_;
  int time_index = time_start_index_;
  int state_index = state_start_index_;

  // TODO(QiL): Initial implementation towards earlier understanding and debug
  // purpose, later code refine towards improving efficiency

  *obj_value = 0.0;
  // 1. objective to minimize state diff to warm up
  for (int i = 0; i < horizon_ + 1; ++i) {
    T x1_diff = x[state_index] - xWS_(0, i);
    T x2_diff = x[state_index + 1] - xWS_(1, i);
    T x3_diff = x[state_index + 2] - xWS_(2, i);
    T x4_abs = x[state_index + 3];
    *obj_value += weight_state_x_ * x1_diff * x1_diff +
                  weight_state_y_ * x2_diff * x2_diff +
                  weight_state_phi_ * x3_diff * x3_diff +
                  weight_state_v_ * x4_abs * x4_abs;
    state_index += 4;
  }

  // 2. objective to minimize u square
  for (int i = 0; i < horizon_; ++i) {
    *obj_value += weight_input_steer_ * x[control_index] * x[control_index] +
                  weight_input_a_ * x[control_index + 1] * x[control_index + 1];
    control_index += 2;
  }

  // 3. objective to minimize input change rate for first horizon
  control_index = control_start_index_;
  T last_time_steer_rate =
      (x[control_index] - last_time_u_(0, 0)) / x[time_index] / ts_;
  T last_time_a_rate =
      (x[control_index + 1] - last_time_u_(1, 0)) / x[time_index] / ts_;
  *obj_value +=
      weight_stitching_steer_ * last_time_steer_rate * last_time_steer_rate +
      weight_stitching_a_ * last_time_a_rate * last_time_a_rate;

  // 4. objective to minimize input change rates, [0- horizon_ -2]
  time_index++;
  for (int i = 0; i < horizon_ - 1; ++i) {
    T steering_rate =
        (x[control_index + 2] - x[control_index]) / x[time_index] / ts_;
    T a_rate =
        (x[control_index + 3] - x[control_index + 1]) / x[time_index] / ts_;
    *obj_value += weight_rate_steer_ * steering_rate * steering_rate +
                  weight_rate_a_ * a_rate * a_rate;
    control_index += 2;
    time_index++;
  }

  // 5. objective to minimize total time [0, horizon_]
  time_index = time_start_index_;
  for (int i = 0; i < horizon_ + 1; ++i) {
    T first_order_penalty = weight_first_order_time_ * x[time_index];
    T second_order_penalty =
        weight_second_order_time_ * x[time_index] * x[time_index];
    *obj_value += first_order_penalty + second_order_penalty;
    time_index++;
  }
}

template <class T>
void DistanceApproachIPOPTInterface::eval_constraints(int n, const T* x, int m,
                                                      T* g) {
  // state start index
  int state_index = state_start_index_;

  // control start index.
  int control_index = control_start_index_;

  // time start index
  int time_index = time_start_index_;

  int constraint_index = 0;

  // // 1. state constraints 4 * [0, horizons-1]
  for (int i = 0; i < horizon_; ++i) {
    // x1
    g[constraint_index] =
        x[state_index + 4] -
        (x[state_index] +
         ts_ * x[time_index] *
             (x[state_index + 3] +
              ts_ * x[time_index] * 0.5 * x[control_index + 1]) *
             cos(x[state_index + 2] + ts_ * x[time_index] * 0.5 *
                                          x[state_index + 3] *
                                          tan(x[control_index]) / wheelbase_));
    // TODO(Jinyun): evaluate performance of different models
    // g[constraint_index] =
    //     x[state_index + 4] -
    //     (x[state_index] +
    //      ts_ * x[time_index] * x[state_index + 3] * cos(x[state_index + 2]));
    // g[constraint_index] =
    //     x[state_index + 4] -
    //     ((xWS_(0, i) + ts_ * xWS_(3, i) * cos(xWS_(2, i))) +
    //      (x[state_index] - xWS_(0, i)) +
    //      (xWS_(3, i) * cos(xWS_(2, i))) * (ts_ * x[time_index] - ts_) +
    //      (ts_ * cos(xWS_(2, i))) * (x[state_index + 3] - xWS_(3, i)) +
    //      (-ts_ * xWS_(3, i) * sin(xWS_(2, i))) *
    //          (x[state_index + 2] - xWS_(2, i)));

    // x2
    g[constraint_index + 1] =
        x[state_index + 5] -
        (x[state_index + 1] +
         ts_ * x[time_index] *
             (x[state_index + 3] +
              ts_ * x[time_index] * 0.5 * x[control_index + 1]) *
             sin(x[state_index + 2] + ts_ * x[time_index] * 0.5 *
                                          x[state_index + 3] *
                                          tan(x[control_index]) / wheelbase_));
    // g[constraint_index + 1] =
    //     x[state_index + 5] -
    //     (x[state_index + 1] +
    //      ts_ * x[time_index] * x[state_index + 3] * sin(x[state_index + 2]));
    // g[constraint_index + 1] =
    //     x[state_index + 5] -
    //     ((xWS_(1, i) + ts_ * xWS_(3, i) * sin(xWS_(2, i))) +
    //      (x[state_index + 1] - xWS_(1, i)) +
    //      (xWS_(3, i) * sin(xWS_(2, i))) * (ts_ * x[time_index] - ts_) +
    //      (ts_ * sin(xWS_(2, i))) * (x[state_index + 3] - xWS_(3, i)) +
    //      (ts_ * xWS_(3, i) * cos(xWS_(2, i))) *
    //          (x[state_index + 2] - xWS_(2, i)));

    // x3
    g[constraint_index + 2] =
        x[state_index + 6] -
        (x[state_index + 2] +
         ts_ * x[time_index] *
             (x[state_index + 3] +
              ts_ * x[time_index] * 0.5 * x[control_index + 1]) *
             tan(x[control_index]) / wheelbase_);
    // g[constraint_index + 2] =
    //     x[state_index + 6] -
    //     (x[state_index + 2] + ts_ * x[time_index] * x[state_index + 3] *
    //                               tan(x[control_index]) / wheelbase_);
    // g[constraint_index + 2] =
    //     x[state_index + 6] -
    //     ((xWS_(2, i) + ts_ * xWS_(3, i) * tan(uWS_(0, i)) / wheelbase_) +
    //      (x[state_index + 2] - xWS_(2, i)) +
    //      (xWS_(3, i) * tan(uWS_(0, i)) / wheelbase_) *
    //          (ts_ * x[time_index] - ts_) +
    //      (ts_ * tan(uWS_(0, i)) / wheelbase_) *
    //          (x[state_index + 3] - xWS_(3, i)) +
    //      (ts_ * xWS_(3, i) / cos(uWS_(0, i)) / cos(uWS_(0, i)) / wheelbase_)
    //      *
    //          (x[control_index] - uWS_(0, i)));

    // x4
    g[constraint_index + 3] =
        x[state_index + 7] -
        (x[state_index + 3] + ts_ * x[time_index] * x[control_index + 1]);
    // g[constraint_index + 3] =
    //     x[state_index + 7] -
    //     ((xWS_(3, i) + ts_ * uWS_(1, i)) + (x[state_index + 3] - xWS_(3, i))
    //     +
    //      uWS_(1, i) * (ts_ * x[time_index] - ts_) +
    //      ts_ * (x[control_index + 1] - uWS_(1, i)));

    control_index += 2;
    constraint_index += 4;
    time_index++;
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
  constraint_index++;
  time_index++;

  for (int i = 1; i < horizon_; ++i) {
    g[constraint_index] =
        (x[control_index] - x[control_index - 2]) / x[time_index] / ts_;
    constraint_index++;
    control_index += 2;
    time_index++;
  }

  // 3. Time constraints 1 * [0, horizons-1]
  time_index = time_start_index_;
  for (int i = 0; i < horizon_; ++i) {
    g[constraint_index] = x[time_index + 1] - x[time_index];
    constraint_index++;
    time_index++;
  }

  ADEBUG << "constraint_index after adding time constraints "
            "updated: "
         << constraint_index;

  // 4. Three obstacles related equal constraints, one equality constraints,
  // [0, horizon_] * [0, obstacles_num_-1] * 4
  state_index = state_start_index_;
  int l_index = l_start_index_;
  int n_index = n_start_index_;

  for (int i = 0; i < horizon_ + 1; ++i) {
    int edges_counter = 0;
    for (int j = 0; j < obstacles_num_; ++j) {
      int current_edges_num = obstacles_edges_num_(j, 0);
      Eigen::MatrixXd Aj =
          obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
      Eigen::MatrixXd bj =
          obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

      // norm(A* lambda) <= 1
      T tmp1 = 0.0;
      T tmp2 = 0.0;
      for (int k = 0; k < current_edges_num; ++k) {
        // TODO(QiL) : replace this one directly with x
        tmp1 += Aj(k, 0) * x[l_index + k];
        tmp2 += Aj(k, 1) * x[l_index + k];
      }
      g[constraint_index] = tmp1 * tmp1 + tmp2 * tmp2;

      // G' * mu + R' * lambda == 0
      g[constraint_index + 1] = x[n_index] - x[n_index + 2] +
                                cos(x[state_index + 2]) * tmp1 +
                                sin(x[state_index + 2]) * tmp2;

      g[constraint_index + 2] = x[n_index + 1] - x[n_index + 3] -
                                sin(x[state_index + 2]) * tmp1 +
                                cos(x[state_index + 2]) * tmp2;

      //  -g'*mu + (A*t - b)*lambda > 0
      T tmp3 = 0.0;
      for (int k = 0; k < 4; ++k) {
        tmp3 += -g_[k] * x[n_index + k];
      }

      T tmp4 = 0.0;
      for (int k = 0; k < current_edges_num; ++k) {
        tmp4 += bj(k, 0) * x[l_index + k];
      }

      g[constraint_index + 3] =
          tmp3 + (x[state_index] + cos(x[state_index + 2]) * offset_) * tmp1 +
          (x[state_index + 1] + sin(x[state_index + 2]) * offset_) * tmp2 -
          tmp4;

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

  // 5. load variable bounds as constraints
  state_index = state_start_index_;
  control_index = control_start_index_;
  time_index = time_start_index_;
  l_index = l_start_index_;
  n_index = n_start_index_;

  // start configuration
  g[constraint_index] = x[state_index];
  g[constraint_index + 1] = x[state_index + 1];
  g[constraint_index + 2] = x[state_index + 2];
  g[constraint_index + 3] = x[state_index + 3];
  constraint_index += 4;
  state_index += 4;

  // constraints on x,y,v
  for (int i = 1; i < horizon_; ++i) {
    g[constraint_index] = x[state_index];
    g[constraint_index + 1] = x[state_index + 1];
    g[constraint_index + 2] = x[state_index + 3];
    constraint_index += 3;
    state_index += 4;
  }

  // end configuration
  g[constraint_index] = x[state_index];
  g[constraint_index + 1] = x[state_index + 1];
  g[constraint_index + 2] = x[state_index + 2];
  g[constraint_index + 3] = x[state_index + 3];
  constraint_index += 4;
  state_index += 4;

  for (int i = 0; i < horizon_; ++i) {
    g[constraint_index] = x[control_index];
    g[constraint_index + 1] = x[control_index + 1];
    constraint_index += 2;
    control_index += 2;
  }

  for (int i = 0; i < horizon_ + 1; ++i) {
    g[constraint_index] = x[time_index];
    constraint_index++;
    time_index++;
  }

  for (int i = 0; i < lambda_horizon_; ++i) {
    g[constraint_index] = x[l_index];
    constraint_index++;
    l_index++;
  }

  for (int i = 0; i < miu_horizon_; ++i) {
    g[constraint_index] = x[n_index];
    constraint_index++;
    n_index++;
  }
}

bool DistanceApproachIPOPTInterface::check_g(int n, const double* x, int m,
                                             const double* g) {
  int kN = n;
  int kM = m;
  double x_u_tmp[kN];
  double x_l_tmp[kN];
  double g_u_tmp[kM];
  double g_l_tmp[kM];

  get_bounds_info(n, x_l_tmp, x_u_tmp, m, g_l_tmp, g_u_tmp);

  const double delta_v = 1e-4;
  for (int idx = 0; idx < n; ++idx) {
    x_u_tmp[idx] = x_u_tmp[idx] + delta_v;
    x_l_tmp[idx] = x_l_tmp[idx] - delta_v;
    if (x[idx] > x_u_tmp[idx] || x[idx] < x_l_tmp[idx]) {
      AINFO << "x idx unfeasible: " << idx << ", x: " << x[idx]
            << ", lower: " << x_l_tmp[idx] << ", upper: " << x_u_tmp[idx];
    }
  }

  // m1 : dynamics constatins
  int m1 = 4 * horizon_;

  // m2 : control rate constraints (only steering)
  int m2 = m1 + horizon_;

  // m3 : sampling time equality constraints
  int m3 = m2 + horizon_;

  // m4 : obstacle constraints
  int m4 = m3 + 4 * obstacles_num_ * (horizon_ + 1);

  // 5. load variable bounds as constraints
  // start configuration
  int m5 = m4 + 3 + 1;

  // constraints on x,y,v
  int m6 = m5 + 3 * (horizon_ - 1);

  // end configuration
  int m7 = m6 + 3 + 1;

  // control variable bnd
  int m8 = m7 + 2 * horizon_;

  // time interval variable bnd
  int m9 = m8 + (horizon_ + 1);

  // lambda_horizon_
  int m10 = m9 + lambda_horizon_;

  // miu_horizon_
  int m11 = m10 + miu_horizon_;

  CHECK_EQ(m11, num_of_constraints_);

  AINFO << "dynamics constatins to: " << m1;
  AINFO << "control rate constraints (only steering) to: " << m2;
  AINFO << "sampling time equality constraints to: " << m3;
  AINFO << "obstacle constraints to: " << m4;
  AINFO << "start conf constraints to: " << m5;
  AINFO << "constraints on x,y,v to: " << m6;
  AINFO << "end constraints to: " << m7;
  AINFO << "control bnd to: " << m8;
  AINFO << "time interval constraints to: " << m9;
  AINFO << "lambda constraints to: " << m10;
  AINFO << "miu constraints to: " << m11;
  AINFO << "total constraints: " << num_of_constraints_;

  for (int idx = 0; idx < m; ++idx) {
    if (g[idx] > g_u_tmp[idx] + delta_v || g[idx] < g_l_tmp[idx] - delta_v) {
      AINFO << "constrains idx unfeasible: " << idx << ", g: " << g[idx]
            << ", lower: " << g_l_tmp[idx] << ", upper: " << g_u_tmp[idx];
    }
  }
  return true;
}

void DistanceApproachIPOPTInterface::generate_tapes(int n, int m,
                                                    int* nnz_jac_g,
                                                    int* nnz_h_lag) {
  std::vector<double> xp(n);
  std::vector<double> lamp(m);
  std::vector<double> zl(m);
  std::vector<double> zu(m);
  std::vector<adouble> xa(n);
  std::vector<adouble> g(m);
  std::vector<double> lam(m);

  double sig;
  adouble obj_value;
  double dummy = 0.0;
  obj_lam = new double[m + 1];
  get_starting_point(n, 1, &xp[0], 0, &zl[0], &zu[0], m, 0, &lamp[0]);

  trace_on(tag_f);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  eval_obj(n, &xa[0], &obj_value);
  obj_value >>= dummy;
  trace_off();

  trace_on(tag_g);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  eval_constraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    g[idx] >>= dummy;
  }
  trace_off();

  trace_on(tag_L);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  for (int idx = 0; idx < m; idx++) {
    lam[idx] = 1.0;
  }
  sig = 1.0;
  eval_obj(n, &xa[0], &obj_value);
  obj_value *= mkparam(sig);
  eval_constraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    obj_value += g[idx] * mkparam(lam[idx]);
  }
  obj_value >>= dummy;

  trace_off();

  if (enable_jacobian_ad_) {
    rind_g = nullptr;
    cind_g = nullptr;
    jacval = nullptr;
    options_g[0] = 0; /* sparsity pattern by index domains (default) */
    options_g[1] = 0; /*                         safe mode (default) */
    options_g[2] = 0;
    options_g[3] = 0; /*                column compression (default) */

    sparse_jac(tag_g, m, n, 0, &xp[0], &nnz_jac, &rind_g, &cind_g, &jacval,
               options_g);
    *nnz_jac_g = nnz_jac;
  }

  rind_L = nullptr;
  cind_L = nullptr;
  hessval = nullptr;
  options_L[0] = 0;
  options_L[1] = 1;

  sparse_hess(tag_L, n, 0, &xp[0], &nnz_L, &rind_L, &cind_L, &hessval,
              options_L);
  *nnz_h_lag = nnz_L;
}
//***************    end   ADOL-C part ***********************************

}  // namespace planning
}  // namespace apollo
