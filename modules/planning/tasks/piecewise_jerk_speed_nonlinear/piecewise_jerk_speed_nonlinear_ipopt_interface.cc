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

/**
 * @file piecewise_jerk_speed_nonlinear_ipopt_interface.cc
 **/

#include "modules/planning/tasks/piecewise_jerk_speed_nonlinear/piecewise_jerk_speed_nonlinear_ipopt_interface.h"

#include <algorithm>
#include <limits>

namespace apollo {
namespace planning {

PiecewiseJerkSpeedNonlinearIpoptInterface::
    PiecewiseJerkSpeedNonlinearIpoptInterface(
        const double s_init, const double s_dot_init, const double s_ddot_init,
        const double delta_t, const int num_of_points, const double s_max,
        const double s_dot_max, const double s_ddot_min,
        const double s_ddot_max, const double s_dddot_min,
        const double s_dddot_max)
    : curvature_curve_(0.0, 0.0, 0.0),
      v_bound_func_(0.0, 0.0, 0.0),
      s_init_(s_init),
      s_dot_init_(s_dot_init),
      s_ddot_init_(s_ddot_init),
      delta_t_(delta_t),
      num_of_points_(num_of_points),
      s_max_(s_max),
      s_dot_max_(s_dot_max),
      s_ddot_min_(-std::abs(s_ddot_min)),
      s_ddot_max_(s_ddot_max),
      s_dddot_min_(-std::abs(s_dddot_min)),
      s_dddot_max_(s_dddot_max),
      v_offset_(num_of_points),
      a_offset_(num_of_points * 2) {}

bool PiecewiseJerkSpeedNonlinearIpoptInterface::get_nlp_info(
    int &n, int &m, int &nnz_jac_g, int &nnz_h_lag,
    IndexStyleEnum &index_style) {
  num_of_variables_ = num_of_points_ * 3;

  if (use_soft_safety_bound_) {
    // complementary slack variable for soft upper and lower s bound
    num_of_variables_ += num_of_points_ * 2;

    lower_s_slack_offset_ = num_of_points_ * 3;

    upper_s_slack_offset_ = num_of_points_ * 4;
  }

  n = num_of_variables_;

  // s monotone constraints s_i+1 - s_i >= 0.0
  num_of_constraints_ = num_of_points_ - 1;

  // jerk bound constraint
  // |s_ddot_i+1 - s_ddot_i| / delta_t <= s_dddot_max
  num_of_constraints_ += num_of_points_ - 1;

  // position equality constraints
  // s_i+1 - s_i - s_dot_i * delta_t - 1/3 * s_ddot_i * delta_t^2 - 1/6 *
  // s_ddot_i+1 * delta_t^2
  num_of_constraints_ += num_of_points_ - 1;

  // velocity equality constraints
  // s_dot_i+1 - s_dot_i - 0.5 * delta_t * s_ddot_i - 0.5 * delta_t * s_ddot_i+1
  num_of_constraints_ += num_of_points_ - 1;

  if (use_v_bound_) {
    // speed limit constraints
    // s_dot_i - v_bound_func_(s_i) <= 0.0
    num_of_constraints_ += num_of_points_;
  }

  if (use_soft_safety_bound_) {
    // soft safety boundary constraints
    // s_i - soft_lower_s_i + lower_slack_i >= 0.0
    num_of_constraints_ += num_of_points_;

    // s_i - soft_upper_s_i - upper_slack_i <= 0.0
    num_of_constraints_ += num_of_points_;
  }

  m = num_of_constraints_;

  nnz_jac_g = 0;
  // s_i+1 - s_i
  nnz_jac_g += (num_of_points_ - 1) * 2;

  // (s_ddot_i+1 - s_ddot_i) / delta_t
  nnz_jac_g += (num_of_points_ - 1) * 2;

  // s_i+1 - s_i - s_dot_i * delta_t - 1/3 * s_ddot_i * delta_t^2 - 1/6 *
  // s_ddot_i+1 * delta_t^2
  nnz_jac_g += (num_of_points_ - 1) * 5;

  // s_dot_i+1 - s_dot_i - 0.5 * s_ddot_i * delta_t - 0.5 * s_ddot_i+1 * delta_t
  nnz_jac_g += (num_of_points_ - 1) * 4;

  if (use_v_bound_) {
    // speed limit constraints
    // s_dot_i - v_bound_func_(s_i) <= 0.0
    nnz_jac_g += num_of_points_ * 2;
  }

  if (use_soft_safety_bound_) {
    // soft safety boundary constraints
    // s_i - soft_lower_s_i + lower_slack_i >= 0.0
    nnz_jac_g += num_of_points_ * 2;

    // s_i - soft_upper_s_i - upper_slack_i <= 0.0
    nnz_jac_g += num_of_points_ * 2;
  }

  nnz_h_lag = num_of_points_ * 5 - 1;

  index_style = IndexStyleEnum::C_STYLE;

  return true;
}

bool PiecewiseJerkSpeedNonlinearIpoptInterface::get_bounds_info(
    int n, double *x_l, double *x_u, int m, double *g_l, double *g_u) {
  // default nlp_lower_bound_inf value in Ipopt
  double INF = 1.0e19;
  double LARGE_VELOCITY_VALUE = s_dot_max_;

  // bounds for variables
  // s
  x_l[0] = s_init_;
  x_u[0] = s_init_;
  for (int i = 1; i < num_of_points_; ++i) {
    x_l[i] = safety_bounds_[i].first;
    x_u[i] = safety_bounds_[i].second;
  }

  // s_dot
  x_l[v_offset_] = s_dot_init_;
  x_u[v_offset_] = s_dot_init_;
  for (int i = 1; i < num_of_points_; ++i) {
    x_l[v_offset_ + i] = 0.0;
    x_u[v_offset_ + i] = LARGE_VELOCITY_VALUE;
  }

  // s_ddot
  x_l[a_offset_] = s_ddot_init_;
  x_u[a_offset_] = s_ddot_init_;
  for (int i = 1; i < num_of_points_; ++i) {
    x_l[a_offset_ + i] = s_ddot_min_;
    x_u[a_offset_ + i] = s_ddot_max_;
  }

  if (use_soft_safety_bound_) {
    // lower_s_slack
    for (int i = 0; i < num_of_points_; ++i) {
      x_l[lower_s_slack_offset_ + i] = 0.0;
      x_u[lower_s_slack_offset_ + i] = INF;
    }

    // upper_s_slack
    for (int i = 0; i < num_of_points_; ++i) {
      x_l[upper_s_slack_offset_ + i] = 0.0;
      x_u[upper_s_slack_offset_ + i] = INF;
    }
  }

  // bounds for constraints
  // s monotone constraints s_i+1 - s_i
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    g_l[i] = 0.0;
    g_u[i] = LARGE_VELOCITY_VALUE * delta_t_;
  }

  // jerk bound constraint
  // |s_ddot_i+1 - s_ddot_i| / delta_t <= s_dddot_max
  int offset = num_of_points_ - 1;
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    g_l[offset + i] = s_dddot_min_;
    g_u[offset + i] = s_dddot_max_;
  }

  // position equality constraints,
  // s_i+1 - s_i - s_dot_i * delta_t - 1/3 * s_ddot_i * delta_t^2 - 1/6 *
  // s_ddot_i+1 * delta_t^2
  offset += num_of_points_ - 1;
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    g_l[offset + i] = 0.0;
    g_u[offset + i] = 0.0;
  }

  // velocity equality constraints,
  // s_dot_i+1 - s_dot_i - 0.5 * delta_t * s_ddot_i - 0.5 * delta_t *
  // s_ddot_i+1
  offset += num_of_points_ - 1;
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    g_l[offset + i] = 0.0;
    g_u[offset + i] = 0.0;
  }

  if (use_v_bound_) {
    // speed limit constraints
    // s_dot_i - v_bound_func_(s_i) <= 0.0
    offset += num_of_points_ - 1;
    for (int i = 0; i < num_of_points_; ++i) {
      g_l[offset + i] = -INF;
      g_u[offset + i] = 0.0;
    }
  }

  if (use_soft_safety_bound_) {
    // soft_s_bound constraints
    // s_i - soft_lower_s_i + lower_slack_i >= 0.0
    offset += num_of_points_;
    for (int i = 0; i < num_of_points_; ++i) {
      g_l[offset + i] = 0.0;
      g_u[offset + i] = INF;
    }

    // s_i - soft_upper_s_i - upper_slack_i <= 0.0
    offset += num_of_points_;
    for (int i = 0; i < num_of_points_; ++i) {
      g_l[offset + i] = -INF;
      g_u[offset + i] = 0.0;
    }
  }

  return true;
}

bool PiecewiseJerkSpeedNonlinearIpoptInterface::get_starting_point(
    int n, bool init_x, double *x, bool init_z, double *z_L, double *z_U, int m,
    bool init_lambda, double *lambda) {
  if (!x_warm_start_.empty()) {
    for (int i = 0; i < num_of_points_; ++i) {
      x[i] = x_warm_start_[i][0];
      x[v_offset_ + i] = x_warm_start_[i][1];
      x[a_offset_ + i] = x_warm_start_[i][2];
    }
  }

  if (use_soft_safety_bound_) {
    for (int i = 0; i < num_of_points_; ++i) {
      x[lower_s_slack_offset_ + i] = 0.0;
      x[upper_s_slack_offset_ + i] = 0.0;
    }
  }

  return true;

  // TODO(Jinyun): Implement better default warm start based on safety_bounds
  for (int i = 0; i < num_of_points_; ++i) {
    x[i] = std::min(5.0 * delta_t_ * i + s_init_, s_max_);
  }
  x[0] = s_init_;

  for (int i = 0; i < num_of_points_; ++i) {
    x[v_offset_ + i] = 5.0;
  }
  x[v_offset_] = s_dot_init_;

  for (int i = 0; i < num_of_points_; ++i) {
    x[a_offset_ + i] = 0.0;
  }
  x[a_offset_] = s_ddot_init_;

  if (use_soft_safety_bound_) {
    for (int i = 0; i < num_of_points_; ++i) {
      x[lower_s_slack_offset_ + i] = 0.0;
      x[upper_s_slack_offset_ + i] = 0.0;
    }
  }

  return true;
}

bool PiecewiseJerkSpeedNonlinearIpoptInterface::eval_f(int n, const double *x,
                                                       bool new_x,
                                                       double &obj_value) {
  obj_value = 0.0;
  // difference between ref spatial distace
  for (int i = 0; i < num_of_points_; ++i) {
    double s_diff = x[i] - s_ref_[i];
    obj_value += s_diff * s_diff * w_ref_s_;
  }

  // difference between ref speed
  for (int i = 0; i < num_of_points_; ++i) {
    double v_diff = x[v_offset_ + i] - v_ref_;
    obj_value += v_diff * v_diff * w_ref_v_;
  }

  // acceleration obj.
  for (int i = 0; i < num_of_points_; ++i) {
    double a = x[a_offset_ + i];
    obj_value += a * a * w_overall_a_;
  }

  // jerk obj.
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    double j = (x[a_offset_ + i + 1] - x[a_offset_ + i]) / delta_t_;
    obj_value += j * j * w_overall_j_;
  }

  // centripetal acceleration obj.
  for (int i = 0; i < num_of_points_; ++i) {
    double v = x[v_offset_ + i];
    double s = x[i];
    double kappa = curvature_curve_.Evaluate(0, s);
    double a_lat = v * v * kappa;
    obj_value += a_lat * a_lat * w_overall_centripetal_acc_;
  }

  if (has_end_state_target_) {
    double s_diff = x[num_of_points_ - 1] - s_target_;
    obj_value += s_diff * s_diff * w_target_s_;

    double v_diff = x[v_offset_ + num_of_points_ - 1] - v_target_;
    obj_value += v_diff * v_diff * w_target_v_;

    double a_diff = x[a_offset_ + num_of_points_ - 1] - a_target_;
    obj_value += a_diff * a_diff * w_target_a_;
  }

  if (use_soft_safety_bound_) {
    for (int i = 0; i < num_of_points_; ++i) {
      obj_value += x[lower_s_slack_offset_ + i] * w_soft_s_bound_;
      obj_value += x[upper_s_slack_offset_ + i] * w_soft_s_bound_;
    }
  }

  return true;
}

bool PiecewiseJerkSpeedNonlinearIpoptInterface::eval_grad_f(int n,
                                                            const double *x,
                                                            bool new_x,
                                                            double *grad_f) {
  std::fill(grad_f, grad_f + n, 0.0);

  // ref. spatial distance objective
  for (int i = 0; i < num_of_points_; ++i) {
    auto s_diff = x[i] - s_ref_[i];
    grad_f[i] += 2.0 * s_diff * w_ref_s_;
  }

  // ref. speed objective
  for (int i = 0; i < num_of_points_; ++i) {
    auto v_diff = x[v_offset_ + i] - v_ref_;
    grad_f[v_offset_ + i] += 2.0 * v_diff * w_ref_v_;
  }

  // jerk objective
  double c = 2.0 / (delta_t_ * delta_t_) * w_overall_j_;
  grad_f[a_offset_] += -c * (x[a_offset_ + 1] - x[a_offset_]);
  for (int i = 1; i + 1 < num_of_points_; ++i) {
    grad_f[a_offset_ + i] += c * (2.0 * x[a_offset_ + i] -
                                  x[a_offset_ + i + 1] - x[a_offset_ + i - 1]);
  }
  grad_f[a_offset_ + num_of_points_ - 1] +=
      c *
      (x[a_offset_ + num_of_points_ - 1] - x[a_offset_ + num_of_points_ - 2]);

  // acceleration objective
  for (int i = 0; i < num_of_points_; ++i) {
    grad_f[a_offset_ + i] += 2.0 * x[a_offset_ + i] * w_overall_a_;
  }

  // centripetal acceleration objective
  for (int i = 0; i < num_of_points_; ++i) {
    double v = x[v_offset_ + i];
    double v2 = v * v;
    double v3 = v2 * v;
    double v4 = v3 * v;

    double s = x[i];
    double kappa = curvature_curve_.Evaluate(0, s);
    double kappa_dot = curvature_curve_.Evaluate(1, s);

    grad_f[i] += 2.0 * w_overall_centripetal_acc_ * v4 * kappa * kappa_dot;
    grad_f[v_offset_ + i] +=
        4.0 * w_overall_centripetal_acc_ * v3 * kappa * kappa;
  }

  if (has_end_state_target_) {
    double s_diff = x[num_of_points_ - 1] - s_target_;
    grad_f[num_of_points_ - 1] += 2.0 * s_diff * w_target_s_;

    double v_diff = x[v_offset_ + num_of_points_ - 1] - v_target_;
    grad_f[v_offset_ + num_of_points_ - 1] += 2.0 * v_diff * w_target_v_;

    double a_diff = x[a_offset_ + num_of_points_ - 1] - a_target_;
    grad_f[a_offset_ + num_of_points_ - 1] += 2.0 * a_diff * w_target_a_;
  }

  if (use_soft_safety_bound_) {
    for (int i = 0; i < num_of_points_; ++i) {
      grad_f[lower_s_slack_offset_ + i] += w_soft_s_bound_;
      grad_f[upper_s_slack_offset_ + i] += w_soft_s_bound_;
    }
  }

  return true;
}

bool PiecewiseJerkSpeedNonlinearIpoptInterface::eval_g(int n, const double *x,
                                                       bool new_x, int m,
                                                       double *g) {
  int offset = 0;

  // s monotone constraints s_i+1 - s_i
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    g[i] = x[offset + i + 1] - x[i];
  }

  offset += num_of_points_ - 1;

  // jerk bound constraint, |s_ddot_i+1 - s_ddot_i| <= s_dddot_max
  // position equality constraints,
  // s_i+1 - s_i - s_dot_i * delta_t - 1/3 * s_ddot_i * delta_t^2 - 1/6 *
  // s_ddot_i+1 * delta_t^2
  // velocity equality constraints
  // s_dot_i+1 - s_dot_i - 0.5 * delta_t * (s_ddot_i + s_ddot_i+1)
  int coffset_jerk = offset;
  int coffset_position = coffset_jerk + num_of_points_ - 1;
  int coffset_velocity = coffset_position + num_of_points_ - 1;

  double t = delta_t_;
  double t2 = t * t;
  double t3 = t2 * t;

  for (int i = 0; i + 1 < num_of_points_; ++i) {
    double s0 = x[i];
    double s1 = x[i + 1];

    double v0 = x[v_offset_ + i];
    double v1 = x[v_offset_ + i + 1];

    double a0 = x[a_offset_ + i];
    double a1 = x[a_offset_ + i + 1];

    double j = (a1 - a0) / t;

    g[coffset_jerk + i] = j;
    g[coffset_position + i] =
        s1 - (s0 + v0 * t + 0.5 * a0 * t2 + 1.0 / 6.0 * j * t3);
    g[coffset_velocity + i] = v1 - (v0 + a0 * t + 0.5 * j * t2);
  }

  offset += 3 * (num_of_points_ - 1);

  if (use_v_bound_) {
    // speed limit constraints
    int coffset_speed_limit = offset;
    // s_dot_i - v_bound_func_(s_i) <= 0.0
    for (int i = 0; i < num_of_points_; ++i) {
      double s = x[i];
      double s_dot = x[v_offset_ + i];
      g[coffset_speed_limit + i] = s_dot - v_bound_func_.Evaluate(0, s);
    }

    offset += num_of_points_;
  }

  if (use_soft_safety_bound_) {
    // soft safety boundary constraints
    int coffset_soft_lower_s = offset;
    // s_i - soft_lower_s_i + lower_slack_i >= 0.0
    for (int i = 0; i < num_of_points_; ++i) {
      double s = x[i];
      double lower_s_slack = x[lower_s_slack_offset_ + i];
      g[coffset_soft_lower_s + i] =
          s - soft_safety_bounds_[i].first + lower_s_slack;
    }
    offset += num_of_points_;

    int coffset_soft_upper_s = offset;
    // s_i - soft_upper_s_i - upper_slack_i <= 0.0
    for (int i = 0; i < num_of_points_; ++i) {
      double s = x[i];
      double upper_s_slack = x[upper_s_slack_offset_ + i];
      g[coffset_soft_upper_s + i] =
          s - soft_safety_bounds_[i].second - upper_s_slack;
    }

    offset += num_of_points_;
  }

  return true;
}

bool PiecewiseJerkSpeedNonlinearIpoptInterface::eval_jac_g(
    int n, const double *x, bool new_x, int m, int nele_jac, int *iRow,
    int *jCol, double *values) {
  if (values == nullptr) {
    int non_zero_index = 0;
    int constraint_index = 0;

    // s monotone constraints s_i+1 - s_i
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // s_i
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = i;
      ++non_zero_index;

      // s_i+1
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = i + 1;
      ++non_zero_index;

      ++constraint_index;
    }

    // jerk bound constraint, |s_ddot_i+1 - s_ddot_i| / delta_t <= s_dddot_max
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // a_i
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = a_offset_ + i;
      ++non_zero_index;

      // a_i+1
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = a_offset_ + i + 1;
      ++non_zero_index;

      ++constraint_index;
    }

    // position equality constraints
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // s_i
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = i;
      ++non_zero_index;

      // v_i
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = v_offset_ + i;
      ++non_zero_index;

      // a_i
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = a_offset_ + i;
      ++non_zero_index;

      // s_i+1
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = i + 1;
      ++non_zero_index;

      // a_i+1
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = a_offset_ + i + 1;
      ++non_zero_index;

      ++constraint_index;
    }

    // velocity equality constraints
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // v_i
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = v_offset_ + i;
      ++non_zero_index;

      // a_i
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = a_offset_ + i;
      ++non_zero_index;

      // v_i+1
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = v_offset_ + i + 1;
      ++non_zero_index;

      // a_i+1
      iRow[non_zero_index] = constraint_index;
      jCol[non_zero_index] = a_offset_ + i + 1;
      ++non_zero_index;

      ++constraint_index;
    }

    if (use_v_bound_) {
      // speed limit constraints
      // s_dot_i - v_bound_func_(s_i) <= 0.0
      for (int i = 0; i < num_of_points_; ++i) {
        // s_i
        iRow[non_zero_index] = constraint_index;
        jCol[non_zero_index] = i;
        ++non_zero_index;

        // v_i
        iRow[non_zero_index] = constraint_index;
        jCol[non_zero_index] = v_offset_ + i;
        ++non_zero_index;

        ++constraint_index;
      }
    }

    if (use_soft_safety_bound_) {
      // soft_s_bound constraints
      // s_i - soft_lower_s_i + lower_slack_i >= 0.0
      for (int i = 0; i < num_of_points_; ++i) {
        // s_i
        iRow[non_zero_index] = constraint_index;
        jCol[non_zero_index] = i;
        ++non_zero_index;

        // lower_slack_i
        iRow[non_zero_index] = constraint_index;
        jCol[non_zero_index] = lower_s_slack_offset_ + i;
        ++non_zero_index;

        ++constraint_index;
      }
      // s_i - soft_upper_s_i - upper_slack_i <= 0.0

      for (int i = 0; i < num_of_points_; ++i) {
        // s_i
        iRow[non_zero_index] = constraint_index;
        jCol[non_zero_index] = i;
        ++non_zero_index;

        // upper_slack_i
        iRow[non_zero_index] = constraint_index;
        jCol[non_zero_index] = upper_s_slack_offset_ + i;
        ++non_zero_index;

        ++constraint_index;
      }
    }

  } else {
    int non_zero_index = 0;
    // s monotone constraints s_i+1 - s_i
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // s_i
      values[non_zero_index] = -1.0;
      ++non_zero_index;

      // s_i+1
      values[non_zero_index] = 1.0;
      ++non_zero_index;
    }

    // jerk bound constraint, |s_ddot_i+1 - s_ddot_i| / delta_t <= s_dddot_max
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // a_i
      values[non_zero_index] = -1.0 / delta_t_;
      ++non_zero_index;

      // a_i+1
      values[non_zero_index] = 1.0 / delta_t_;
      ++non_zero_index;
    }

    // position equality constraints
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // s_i
      values[non_zero_index] = -1.0;
      ++non_zero_index;

      // v_i
      values[non_zero_index] = -delta_t_;
      ++non_zero_index;

      // a_i
      values[non_zero_index] = -1.0 / 3.0 * delta_t_ * delta_t_;
      ++non_zero_index;

      // s_i+1
      values[non_zero_index] = 1.0;
      ++non_zero_index;

      // a_i+1
      values[non_zero_index] = -1.0 / 6.0 * delta_t_ * delta_t_;
      ++non_zero_index;
    }

    // velocity equality constraints
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      // v_i
      values[non_zero_index] = -1.0;
      ++non_zero_index;

      // a_i
      values[non_zero_index] = -0.5 * delta_t_;
      ++non_zero_index;

      // v_i+1
      values[non_zero_index] = 1.0;
      ++non_zero_index;

      // a_i+1
      values[non_zero_index] = -0.5 * delta_t_;
      ++non_zero_index;
    }

    if (use_v_bound_) {
      // speed limit constraints
      // s_dot_i - v_bound_func_(s_i) <= 0.0
      for (int i = 0; i < num_of_points_; ++i) {
        // s_i
        double s = x[i];
        values[non_zero_index] = -1.0 * v_bound_func_.Evaluate(1, s);
        ++non_zero_index;

        // v_i
        values[non_zero_index] = 1.0;
        ++non_zero_index;
      }
    }

    if (use_soft_safety_bound_) {
      // soft_s_bound constraints
      // s_i - soft_lower_s_i + lower_slack_i >= 0.0
      for (int i = 0; i < num_of_points_; ++i) {
        // s_i
        values[non_zero_index] = 1.0;
        ++non_zero_index;

        // lower_slack_i
        values[non_zero_index] = 1.0;
        ++non_zero_index;
      }

      // s_i - soft_upper_s_i - upper_slack_i <= 0.0
      for (int i = 0; i < num_of_points_; ++i) {
        // s_i
        values[non_zero_index] = 1.0;
        ++non_zero_index;

        // upper_slack_i
        values[non_zero_index] = -1.0;
        ++non_zero_index;
      }
    }
  }
  return true;
}

bool PiecewiseJerkSpeedNonlinearIpoptInterface::eval_h(
    int n, const double *x, bool new_x, double obj_factor, int m,
    const double *lambda, bool new_lambda, int nele_hess, int *iRow, int *jCol,
    double *values) {
  if (values == nullptr) {
    int nz_index = 0;
    for (int i = 0; i < num_of_points_; ++i) {
      iRow[nz_index] = i;
      jCol[nz_index] = i;
      ++nz_index;
    }

    for (int i = 0; i < num_of_points_; ++i) {
      iRow[nz_index] = i;
      jCol[nz_index] = v_offset_ + i;
      ++nz_index;
    }

    for (int i = 0; i < num_of_points_; ++i) {
      iRow[nz_index] = v_offset_ + i;
      jCol[nz_index] = v_offset_ + i;
      ++nz_index;
    }

    for (int i = 0; i < num_of_points_; ++i) {
      iRow[nz_index] = a_offset_ + i;
      jCol[nz_index] = a_offset_ + i;
      ++nz_index;
    }

    for (int i = 0; i + 1 < num_of_points_; ++i) {
      iRow[nz_index] = a_offset_ + i;
      jCol[nz_index] = a_offset_ + i + 1;
      ++nz_index;
    }

    for (int i = 0; i < nz_index; ++i) {
      int r = iRow[i];
      int c = jCol[i];
      hessian_mapper_[to_hash_key(r, c)] = i;
    }

  } else {
    std::fill(values, values + nele_hess, 0.0);

    // speed by curvature objective
    for (int i = 0; i < num_of_points_; ++i) {
      auto s_index = i;
      auto v_index = v_offset_ + i;

      auto s = x[s_index];
      auto v = x[v_index];

      auto kappa = curvature_curve_.Evaluate(0, s);
      auto kappa_dot = curvature_curve_.Evaluate(1, s);
      auto kappa_ddot = curvature_curve_.Evaluate(2, s);

      auto v2 = v * v;
      auto v3 = v2 * v;
      auto v4 = v3 * v;

      auto h_s_s_obj =
          2.0 * kappa_dot * kappa_dot * v4 + 2.0 * kappa * kappa_ddot * v4;
      auto h_s_s_index = hessian_mapper_[to_hash_key(s_index, s_index)];
      values[h_s_s_index] +=
          h_s_s_obj * w_overall_centripetal_acc_ * obj_factor;

      auto h_s_v_obj = 8.0 * kappa * kappa_dot * v3;
      auto h_s_v_index = hessian_mapper_[to_hash_key(s_index, v_index)];
      values[h_s_v_index] +=
          h_s_v_obj * w_overall_centripetal_acc_ * obj_factor;

      auto h_v_v_obj = 12.0 * kappa * kappa * v2;
      auto h_v_v_index = hessian_mapper_[to_hash_key(v_index, v_index)];
      values[h_v_v_index] +=
          h_v_v_obj * w_overall_centripetal_acc_ * obj_factor;
    }

    // spatial distance reference objective
    for (int i = 0; i < num_of_points_; ++i) {
      auto h_s_s_index = hessian_mapper_[to_hash_key(i, i)];
      values[h_s_s_index] += 2.0 * w_ref_s_ * obj_factor;
    }

    // speed limit constraint
    if (use_v_bound_) {
      int lambda_offset = 4 * (num_of_points_ - 1);
      for (int i = 0; i < num_of_points_; ++i) {
        auto s_index = i;

        auto s = x[s_index];

        auto v_bound_ddot = v_bound_func_.Evaluate(2, s);

        auto h_s_s_constr = -1.0 * v_bound_ddot;
        auto h_s_s_index = hessian_mapper_[to_hash_key(s_index, s_index)];
        values[h_s_s_index] += h_s_s_constr * lambda[lambda_offset + i];
      }
    }

    for (int i = 0; i < num_of_points_; ++i) {
      auto a_index = a_offset_ + i;
      auto h_index = hessian_mapper_[to_hash_key(a_index, a_index)];
      values[h_index] += 2.0 * w_overall_a_ * obj_factor;
    }

    auto c = 2.0 / delta_t_ / delta_t_ * w_overall_j_ * obj_factor;
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      auto a0_index = a_offset_ + i;
      auto a1_index = a_offset_ + i + 1;

      auto h_a0_a0_index = hessian_mapper_[to_hash_key(a0_index, a0_index)];
      values[h_a0_a0_index] += c;

      auto h_a0_a1_index = hessian_mapper_[to_hash_key(a0_index, a1_index)];
      values[h_a0_a1_index] += -c;

      auto h_a1_a1_index = hessian_mapper_[to_hash_key(a1_index, a1_index)];
      values[h_a1_a1_index] += c;
    }

    for (int i = 0; i < num_of_points_; ++i) {
      auto v_index = i + v_offset_;
      auto key = to_hash_key(v_index, v_index);
      auto index = hessian_mapper_[key];
      values[index] += 2.0 * w_ref_v_ * obj_factor;
    }

    if (has_end_state_target_) {
      // target s
      auto s_end_index = num_of_points_ - 1;
      auto s_key = to_hash_key(s_end_index, s_end_index);

      auto s_index = hessian_mapper_[s_key];

      values[s_index] += 2.0 * w_target_s_ * obj_factor;

      // target v
      auto v_end_index = 2 * num_of_points_ - 1;
      auto v_key = to_hash_key(v_end_index, v_end_index);

      auto v_index = hessian_mapper_[v_key];

      values[v_index] += 2.0 * w_target_v_ * obj_factor;

      // target a
      auto a_end_index = 3 * num_of_points_ - 1;
      auto a_key = to_hash_key(a_end_index, a_end_index);

      auto a_index = hessian_mapper_[a_key];

      values[a_index] += 2.0 * w_target_a_ * obj_factor;
    }
  }
  return true;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::get_optimization_results(
    std::vector<double> *ptr_opt_s, std::vector<double> *ptr_opt_v,
    std::vector<double> *ptr_opt_a) {
  *ptr_opt_s = opt_s_;
  *ptr_opt_v = opt_v_;
  *ptr_opt_a = opt_a_;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double *x, const double *z_L,
    const double *z_U, int m, const double *g, const double *lambda,
    double obj_value, const Ipopt::IpoptData *ip_data,
    Ipopt::IpoptCalculatedQuantities *ip_cq) {
  opt_s_.clear();
  opt_v_.clear();
  opt_a_.clear();

  for (int i = 0; i < num_of_points_; ++i) {
    double s = x[i];
    double v = x[v_offset_ + i];
    double a = x[a_offset_ + i];

    opt_s_.push_back(s);
    opt_v_.push_back(v);
    opt_a_.push_back(a);
  }

  if (use_soft_safety_bound_) {
    // statistic analysis on soft bound intrusion by inspecting slack variable
    double lower_s_mean_intrusion = 0.0;
    double lower_s_highest_intrusion = -std::numeric_limits<double>::infinity();
    int lower_s_highest_intrustion_index = 0.0;
    double upper_s_mean_intrusion = 0.0;
    double upper_s_highest_intrusion = -std::numeric_limits<double>::infinity();
    int upper_s_highest_intrustion_index = 0.0;

    for (int i = 0; i < num_of_points_; ++i) {
      double lower_s_slack = x[lower_s_slack_offset_ + i];
      double upper_s_slack = x[upper_s_slack_offset_ + i];

      lower_s_mean_intrusion += lower_s_slack;
      upper_s_mean_intrusion += upper_s_slack;

      if (lower_s_highest_intrusion < lower_s_slack) {
        lower_s_highest_intrusion = lower_s_slack;
        lower_s_highest_intrustion_index = i;
      }

      if (upper_s_highest_intrusion < upper_s_slack) {
        upper_s_highest_intrusion = upper_s_slack;
        upper_s_highest_intrustion_index = i;
      }
    }

    lower_s_mean_intrusion /= static_cast<double>(num_of_points_);
    upper_s_mean_intrusion /= static_cast<double>(num_of_points_);

    ADEBUG << "lower soft s boundary average intrustion is ["
           << lower_s_mean_intrusion << "] with highest value of ["
           << lower_s_highest_intrusion << "] at time ["
           << delta_t_ * static_cast<double>(lower_s_highest_intrustion_index)
           << "].";
    ADEBUG << "upper soft s boundary average intrustion is ["
           << upper_s_mean_intrusion << "] with highest value of ["
           << upper_s_highest_intrusion << "] at time ["
           << delta_t_ * static_cast<double>(upper_s_highest_intrustion_index)
           << "].";
  }
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_curvature_curve(
    const PiecewiseJerkTrajectory1d &curvature_curve) {
  curvature_curve_ = curvature_curve;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_speed_limit_curve(
    const PiecewiseJerkTrajectory1d &v_bound_f) {
  v_bound_func_ = v_bound_f;
  use_v_bound_ = true;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_reference_speed(
    const double v_ref) {
  v_ref_ = v_ref;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_safety_bounds(
    const std::vector<std::pair<double, double>> &safety_bounds) {
  safety_bounds_ = safety_bounds;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_soft_safety_bounds(
    const std::vector<std::pair<double, double>> &soft_safety_bounds) {
  soft_safety_bounds_ = soft_safety_bounds;
  use_soft_safety_bound_ = true;
}

int PiecewiseJerkSpeedNonlinearIpoptInterface::to_hash_key(const int i,
                                                           const int j) const {
  return i * num_of_variables_ + j;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_end_state_target(
    const double s_target, const double v_target, const double a_target) {
  s_target_ = s_target;
  v_target_ = v_target;
  a_target_ = a_target;
  has_end_state_target_ = true;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_w_target_state(
    const double w_target_s, const double w_target_v, const double w_target_a) {
  w_target_s_ = w_target_s;
  w_target_v_ = w_target_v;
  w_target_a_ = w_target_a;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_w_overall_a(
    const double w_overall_a) {
  w_overall_a_ = w_overall_a;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_w_overall_j(
    const double w_overall_j) {
  w_overall_j_ = w_overall_j;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_w_overall_centripetal_acc(
    const double w_overall_centripetal_acc) {
  w_overall_centripetal_acc_ = w_overall_centripetal_acc;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_w_reference_speed(
    const double w_reference_speed) {
  w_ref_v_ = w_reference_speed;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::
    set_w_reference_spatial_distance(const double w_ref_s) {
  w_ref_s_ = w_ref_s;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_w_soft_s_bound(
    const double w_soft_s_bound) {
  w_soft_s_bound_ = w_soft_s_bound;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_warm_start(
    const std::vector<std::vector<double>> &speed_profile) {
  x_warm_start_ = speed_profile;
}

void PiecewiseJerkSpeedNonlinearIpoptInterface::set_reference_spatial_distance(
    const std::vector<double> &s_ref) {
  s_ref_ = s_ref;
}
}  // namespace planning
}  // namespace apollo
