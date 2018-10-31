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

#include "modules/planning/open_space/dual_variable_warm_start_ipopt_interface.h"

#include <math.h>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

constexpr std::size_t N = 80;

DualVariableWarmStartIPOPTInterface::DualVariableWarmStartIPOPTInterface(
    int num_of_variables, int num_of_constraints, std::size_t horizon, float ts,
    const Eigen::MatrixXd& ego, const Eigen::MatrixXd& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const double rx, const double ry, const double r_yaw)
    : num_of_variables_(num_of_variables),
      num_of_constraints_(num_of_constraints),
      horizon_(horizon),
      ts_(ts),
      ego_(ego),
      obstacles_edges_num_(obstacles_edges_num),
      obstacles_A_(obstacles_A),
      obstacles_b_(obstacles_b),
      rx_(rx),
      ry_(ry),
      r_yaw_(r_yaw) {
  w_ev_ = ego_(1, 0) + ego_(3, 0);
  l_ev_ = ego_(0, 0) + ego_(2, 0);

  g_ = {l_ev_ / 2, w_ev_ / 2, l_ev_ / 2, w_ev_ / 2};
  offset_ = (ego_(0, 0) + ego_(2, 0)) / 2 - ego_(2, 0);
  obstacles_edges_sum_ = std::size_t(obstacles_edges_num_.sum());
  l_start_index_ = 0;
  n_start_index_ = l_start_index_ + obstacles_edges_sum_ * (horizon_ + 1);
  d_start_index_ = n_start_index_ + 4 * obstacles_num_ * (horizon_ + 1);
  l_warm_up_ = Eigen::MatrixXd::Zero(obstacles_edges_sum_, horizon_ + 1);
  n_warm_up_ = Eigen::MatrixXd::Zero(4 * obstacles_num_, horizon_ + 1);
}

bool DualVariableWarmStartIPOPTInterface::get_nlp_info(
    int& n, int& m, int& nnz_jac_g, int& nnz_h_lag,
    IndexStyleEnum& index_style) {
  // number of variables
  n = num_of_variables_;

  // number of constraints

  m = num_of_constraints_;

  // number of nonzero hessian and lagrangian.

  // TODO(QiL) : Update nnz_jac_g;
  nnz_jac_g = 0;

  // TOdo(QiL) : Update nnz_h_lag;
  nnz_h_lag = 0;

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool DualVariableWarmStartIPOPTInterface::get_starting_point(
    int n, bool init_x, double* x, bool init_z, double* z_L, double* z_U, int m,
    bool init_lambda, double* lambda) {
  return true;
}

bool DualVariableWarmStartIPOPTInterface::get_bounds_info(int n, double* x_l,
                                                          double* x_u, int m,
                                                          double* g_l,
                                                          double* g_u) {
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  CHECK(n == num_of_variables_) << "num_of_variables_ mismatch, n: " << n
                                << ", num_of_variables_: " << num_of_variables_;
  CHECK(m == num_of_constraints_)
      << "num_of_constraints_ mismatch, n: " << n
      << ", num_of_constraints_: " << num_of_constraints_;

  std::size_t variable_index = 0;
  // 1. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_edges_sum_; ++j) {
      x_l[variable_index] = 0.0;
      x_u[variable_index] = 0.0;
      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange l : " << variable_index;

  // 2. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < 4 * obstacles_num_; ++j) {
      x_l[variable_index] = 0.0;
      x_u[variable_index] = 0.0;

      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange n : " << variable_index;

  // 3. dual variable n, [0, obstacles_num-1] * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_num_; ++j) {
      // TODO(QiL): Load this from configuration
      x_l[variable_index] = 0.0;
      x_u[variable_index] = 0.0;

      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding dual variables n : " << variable_index;

  std::size_t constraint_index = 0;
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

      // c. -g'*mu + (A*t - b)*lambda > min_safety_distance_
      g_l[constraint_index + 3] = 0.0;
      g_u[constraint_index + 3] = 0.0;
      constraint_index += 4;
    }
  }
  ADEBUG << "constraint_index after adding obstacles constraints: "
         << constraint_index;

  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_f(int n, const double* x,
                                                 bool new_x,
                                                 double& obj_value) {
  obj_value = 0.0;
  std::size_t d_index = d_start_index_;
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_num_; ++j) {
      // TODO(QiL): Change weight to configuration
      obj_value += 1.0 * x[d_index];
      ++d_index;
    }
  }

  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_grad_f(int n, const double* x,
                                                      bool new_x,
                                                      double* grad_f) {
  // 1. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]
  std::size_t l_index = l_start_index_;
  std::size_t n_index = n_start_index_;
  std::size_t d_index = d_start_index_;

  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_edges_sum_; ++j) {
      grad_f[l_index] = 0.0;
      ++l_index;
    }
  }

  // 2. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < 4 * obstacles_num_; ++j) {
      grad_f[n_index] = 0.0;
      ++n_index;
    }
  }

  // 3. dual variable n, [0, obstacles_num-1] * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_num_; ++j) {
      // TODO(QiL): Change to weight configration
      grad_f[d_index] = 1.0;
      ++n_index;
    }
  }
  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_h(int n, const double* x,
                                                 bool new_x, double obj_factor,
                                                 int m, const double* lambda,
                                                 bool new_lambda, int nele_hess,
                                                 int* iRow, int* jCol,
                                                 double* values) {
  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_g(int n, const double* x,
                                                 bool new_x, int m, double* g) {
  ADEBUG << "eval_g";
  // state start index

  // 1. Three obstacles related equal constraints, one equality constraints,
  // [0, horizon_] * [0, obstacles_num_-1] * 4

  std::size_t l_index = l_start_index_;
  std::size_t n_index = n_start_index_;
  std::size_t d_index = d_start_index_;
  std::size_t constraint_index = 0;

  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    int edges_counter = 0;
    for (std::size_t j = 0; j < obstacles_num_; ++j) {
      std::size_t current_edges_num = obstacles_edges_num_(j, 0);
      Eigen::MatrixXd Aj =
          obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
      Eigen::MatrixXd bj =
          obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

      // norm(A* lambda) = 1
      double tmp1 = 0.0;
      double tmp2 = 0.0;
      for (std::size_t k = 0; k < current_edges_num; ++k) {
        // TODO(QiL) : replace this one directly with x
        tmp1 += Aj(k, 0) * x[l_index + k];
        tmp2 += Aj(k, 1) * x[l_index + k];
      }
      g[constraint_index] = tmp1 * tmp1 + tmp2 * tmp2;

      // G' * mu + R' * lambda == 0
      g[constraint_index + 1] = x[n_index] - x[n_index + 2] +
                                std::cos(r_yaw_) * tmp1 +
                                std::sin(r_yaw_) * tmp2;

      g[constraint_index + 2] = x[n_index + 1] - x[n_index + 3] -
                                std::sin(r_yaw_) * tmp1 +
                                std::cos(r_yaw_) * tmp2;

      //  -g'*mu + (A*t - b)*lambda > 0
      // TODO(QiL): Need to revise according to dual modeling
      double tmp3 = 0.0;
      for (std::size_t k = 0; k < 4; ++k) {
        tmp3 += g_[k] * x[n_index + k];
      }

      double tmp4 = 0.0;
      for (std::size_t k = 0; k < current_edges_num; ++k) {
        tmp4 += bj(k, 0) * x[l_index + k];
      }

      g[constraint_index + 3] =
          x[d_index] + tmp3 - (rx_ + std::cos(r_yaw_) * offset_) * tmp1 -
          (ry_ + std::sin(r_yaw_) * offset_) * tmp2 + tmp4;

      // Update index
      edges_counter += current_edges_num;
      l_index += current_edges_num;
      n_index += 4;
      d_index += 1;
      constraint_index += 4;
    }
  }
  ADEBUG << "constraint_index after obstacles avoidance constraints "
            "updated: "
         << constraint_index;
  return true;
}

bool DualVariableWarmStartIPOPTInterface::eval_jac_g(int n, const double* x,
                                                     bool new_x, int m,
                                                     int nele_jac, int* iRow,
                                                     int* jCol,
                                                     double* values) {
  ADEBUG << "eval_jac_g";
  CHECK_EQ(n, num_of_variables_)
      << "No. of variables wrong in eval_jac_g. n : " << n;
  CHECK_EQ(m, num_of_constraints_)
      << "No. of constraints wrong in eval_jac_g. n : " << m;

  if (values == nullptr) {
    std::size_t nz_index = 0;
    std::size_t constraint_index = 0;

    // 1. Three obstacles related equal constraints, one equality constraints,
    // [0, horizon_] * [0, obstacles_num_-1] * 4
    std::size_t l_index = l_start_index_;
    std::size_t n_index = n_start_index_;
    std::size_t d_index = d_start_index_;
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

        //  -g'*mu + (A*t - b)*lambda > 0
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

        // with resepct to d
        iRow[nz_index] = constraint_index + 3;
        jCol[nz_index] = d_index;

        // Update index
        l_index += current_edges_num;
        n_index += 4;
        d_index += 1;
        constraint_index += 4;
      }
    }

    CHECK_EQ(nz_index, static_cast<std::size_t>(nele_jac));
    CHECK_EQ(constraint_index, static_cast<std::size_t>(m));
  } else {
    std::fill(values, values + nele_jac, 0.0);
    std::size_t nz_index = 0;

    // 1. Three obstacles related equal constraints, one equality constraints,
    // [0, horizon_] * [0, obstacles_num_-1] * 4
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

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          values[nz_index] = std::cos(r_yaw_) * Aj(k, 0) +
                             std::sin(r_yaw_) * Aj(k, 1);  // v0~vn
          ++nz_index;
        }

        // With respect to n
        values[nz_index] = 1.0;  // w0
        ++nz_index;

        values[nz_index] = -1.0;  // w2
        ++nz_index;

        // 3. G' * mu + R' * lambda == 0, part 2

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          values[nz_index] = -std::sin(r_yaw_) * Aj(k, 0) +
                             std::cos(r_yaw_) * Aj(k, 1);  // y0~yn
          ++nz_index;
        }

        // With respect to n
        values[nz_index] = 1.0;  // z1
        ++nz_index;

        values[nz_index] = -1.0;  // z3
        ++nz_index;

        //  3. -g'*mu + (A*t - b)*lambda > 0
        // TODO(QiL) Revise dual vairables modeling here.
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

        values[nz_index] = -std::sin(r_yaw_) * offset_ * tmp1 +
                           std::cos(r_yaw_) * offset_ * tmp2;  // cc1
        ++nz_index;

        // with respect to l
        for (std::size_t k = 0; k < current_edges_num; ++k) {
          values[nz_index] = (rx_ + std::cos(r_yaw_) * offset_) * Aj(k, 0) +
                             (ry_ + std::sin(r_yaw_) * offset_) * Aj(k, 1) -
                             bj(k, 0);  // ddk
          ++nz_index;
        }

        // with respect to n
        for (std::size_t k = 0; k < 4; ++k) {
          values[nz_index] = -g_[k];  // eek
          ++nz_index;
        }

        // with respect to d
        values[nz_index] = 1;  // ffk
        ++nz_index;

        // Update index
        edges_counter += current_edges_num;
        l_index += current_edges_num;
        n_index += 4;
      }
    }

    ADEBUG << "eval_jac_g, fulfilled obstacle constraint values";
    CHECK_EQ(nz_index, static_cast<std::size_t>(nele_jac));
  }

  ADEBUG << "eval_jac_g done";
  return true;
}  // namespace planning

void DualVariableWarmStartIPOPTInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  std::size_t variable_index = 0;
  // 1. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < obstacles_edges_sum_; ++j) {
      l_warm_up_(0, i) = x[variable_index];
      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange l : " << variable_index;

  // 2. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (std::size_t i = 0; i < horizon_ + 1; ++i) {
    for (std::size_t j = 0; j < 4 * obstacles_num_; ++j) {
      n_warm_up_(0, i) = x[variable_index];
      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange n : " << variable_index;
}

void DualVariableWarmStartIPOPTInterface::get_optimization_results(
    Eigen::MatrixXd* l_warm_up, Eigen::MatrixXd* n_warm_up) const {
  *l_warm_up = l_warm_up_;
  *n_warm_up = n_warm_up_;
}
}  // namespace planning
}  // namespace apollo
