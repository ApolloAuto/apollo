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

#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_ipopt_qp_interface.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

DualVariableWarmStartIPOPTQPInterface::DualVariableWarmStartIPOPTQPInterface(
    size_t horizon, double ts, const Eigen::MatrixXd& ego,
    const Eigen::MatrixXi& obstacles_edges_num, const size_t obstacles_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const Eigen::MatrixXd& xWS,
    const PlannerOpenSpaceConfig& planner_open_space_config)
    : ts_(ts),
      ego_(ego),
      obstacles_edges_num_(obstacles_edges_num),
      obstacles_A_(obstacles_A),
      obstacles_b_(obstacles_b),
      xWS_(xWS) {
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
  l_start_index_ = 0;
  n_start_index_ = l_start_index_ + obstacles_edges_sum_ * (horizon_ + 1);
  l_warm_up_ = Eigen::MatrixXd::Zero(obstacles_edges_sum_, horizon_ + 1);
  n_warm_up_ = Eigen::MatrixXd::Zero(4 * obstacles_num_, horizon_ + 1);
  min_safety_distance_ =
      planner_open_space_config.dual_variable_warm_start_config()
          .min_safety_distance();
}

bool DualVariableWarmStartIPOPTQPInterface::get_nlp_info(
    int& n, int& m, int& nnz_jac_g, int& nnz_h_lag,
    IndexStyleEnum& index_style) {
  lambda_horizon_ = obstacles_edges_sum_ * (horizon_ + 1);

  miu_horizon_ = obstacles_num_ * 4 * (horizon_ + 1);

  dual_formulation_horizon_ = obstacles_num_ * (horizon_ + 1);

  num_of_variables_ = lambda_horizon_ + miu_horizon_;

  num_of_constraints_ = 3 * obstacles_num_ * (horizon_ + 1) + num_of_variables_;

  // number of variables
  n = num_of_variables_;

  // number of constraints
  m = num_of_constraints_;

  // number of nonzero Jacobian and Lagrangian.
  generate_tapes(n, m, &nnz_h_lag);

  int tmp = 0;
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_num_; ++j) {
      int current_edges_num = obstacles_edges_num_(j, 0);
      tmp += current_edges_num * 3 + 2 + 2 + 4;
    }
  }
  nnz_jac_g = tmp + num_of_variables_;

  ADEBUG << "nnz_jac_g : " << nnz_jac_g;
  index_style = IndexStyleEnum::C_STYLE;
  return true;
}

bool DualVariableWarmStartIPOPTQPInterface::get_starting_point(
    int n, bool init_x, double* x, bool init_z, double* z_L, double* z_U, int m,
    bool init_lambda, double* lambda) {
  ADEBUG << "get_starting_point";
  ACHECK(init_x) << "Warm start init_x setting failed";
  ACHECK(!init_z) << "Warm start init_z setting failed";
  ACHECK(!init_lambda) << "Warm start init_lambda setting failed";

  int l_index = l_start_index_;
  int n_index = n_start_index_;
  ADEBUG << "l_start_index_ : " << l_start_index_;
  ADEBUG << "n_start_index_ : " << n_start_index_;

  // 1. lagrange constraint l, obstacles_edges_sum_ * (horizon_+1)
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      x[l_index] = 0.5;
      ++l_index;
    }
  }

  // 2. lagrange constraint n, 4*obstacles_num * (horizon_+1)
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      x[n_index] = 1.0;
      ++n_index;
    }
  }

  ADEBUG << "get_starting_point out";
  return true;
}

bool DualVariableWarmStartIPOPTQPInterface::get_bounds_info(int n, double* x_l,
                                                            double* x_u, int m,
                                                            double* g_l,
                                                            double* g_u) {
  int variable_index = 0;
  // 1. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      x_l[variable_index] = -2e19;
      x_u[variable_index] = 2e19;
      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange l : " << variable_index;

  // 2. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      x_l[variable_index] = -2e19;
      x_u[variable_index] = 2e19;  // nlp_upper_bound_limit
      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange n : " << variable_index;

  int constraint_index = 0;
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_num_; ++j) {
      // a. G'*mu + R'*A*lambda = 0
      g_l[constraint_index] = 0.0;
      g_u[constraint_index] = 0.0;
      g_l[constraint_index + 1] = 0.0;
      g_u[constraint_index + 1] = 0.0;

      // b. (-g'*mu + (A*t - b)*lambda) >= 0
      g_l[constraint_index + 2] = min_safety_distance_;
      g_u[constraint_index + 2] = 2e19;
      constraint_index += 3;
    }
  }

  int l_index = l_start_index_;
  int n_index = n_start_index_;

  for (int i = 0; i < lambda_horizon_; ++i) {
    g_l[constraint_index] = 0.0;
    g_u[constraint_index] = 2e19;
    constraint_index++;
    l_index++;
  }
  for (int i = 0; i < miu_horizon_; ++i) {
    g_l[constraint_index] = 0.0;
    g_u[constraint_index] = 2e19;
    constraint_index++;
    n_index++;
  }

  ADEBUG << "constraint_index after adding obstacles constraints: "
         << constraint_index;

  return true;
}

bool DualVariableWarmStartIPOPTQPInterface::eval_f(int n, const double* x,
                                                   bool new_x,
                                                   double& obj_value) {
  eval_obj(n, x, &obj_value);
  return true;
}

bool DualVariableWarmStartIPOPTQPInterface::eval_grad_f(int n, const double* x,
                                                        bool new_x,
                                                        double* grad_f) {
  std::fill(grad_f, grad_f + n, 0.0);
  int l_index = l_start_index_;
  for (int i = 0; i < horizon_ + 1; ++i) {
    int edges_counter = 0;
    // assume: stationary obstacles
    for (int j = 0; j < obstacles_num_; ++j) {
      int current_edges_num = obstacles_edges_num_(j, 0);
      Eigen::MatrixXd Aj =
          obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
      Eigen::MatrixXd bj =
          obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

      // norm(A* lambda)
      double tmp1 = 0.0;
      double tmp2 = 0.0;
      for (int k = 0; k < current_edges_num; ++k) {
        tmp1 += Aj(k, 0) * x[l_index + k];
        tmp2 += Aj(k, 1) * x[l_index + k];
      }
      for (int k = 0; k < current_edges_num; ++k) {
        grad_f[l_index + k] += 2.0 * tmp1 * Aj(k, 0) + 2.0 * tmp2 * Aj(k, 1);
      }

      // Update index
      edges_counter += current_edges_num;
      l_index += current_edges_num;
    }
  }

  return true;
}

bool DualVariableWarmStartIPOPTQPInterface::eval_g(int n, const double* x,
                                                   bool new_x, int m,
                                                   double* g) {
  eval_constraints(n, x, m, g);
  return true;
}

bool DualVariableWarmStartIPOPTQPInterface::eval_jac_g(int n, const double* x,
                                                       bool new_x, int m,
                                                       int nele_jac, int* iRow,
                                                       int* jCol,
                                                       double* values) {
  ADEBUG << "eval_jac_g";

  if (values == nullptr) {
    int nz_index = 0;
    int constraint_index = 0;

    // 1. Three obstacles related equal constraints, one equality
    // constraints,
    // [0, horizon_] * [0, obstacles_num_-1] * 3
    int l_index = l_start_index_;
    int n_index = n_start_index_;
    for (int i = 0; i < horizon_ + 1; ++i) {
      for (int j = 0; j < obstacles_num_; ++j) {
        int current_edges_num = obstacles_edges_num_(j, 0);

        // 1. G' * mu + R' * lambda == 0, part 1
        // with respect to l
        for (int k = 0; k < current_edges_num; ++k) {
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

        // 2. G' * mu + R' * lambda == 0, part 2
        // with respect to l
        for (int k = 0; k < current_edges_num; ++k) {
          iRow[nz_index] = constraint_index + 1;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }

        // With respect to n
        iRow[nz_index] = constraint_index + 1;
        jCol[nz_index] = n_index + 1;
        ++nz_index;

        iRow[nz_index] = constraint_index + 1;
        jCol[nz_index] = n_index + 3;
        ++nz_index;

        // 3. -g'*mu + (A*t - b)*lambda > 0
        // with respect to l
        for (int k = 0; k < current_edges_num; ++k) {
          iRow[nz_index] = constraint_index + 2;
          jCol[nz_index] = l_index + k;
          ++nz_index;
        }

        // with respect to n
        for (int k = 0; k < 4; ++k) {
          iRow[nz_index] = constraint_index + 2;
          jCol[nz_index] = n_index + k;
          ++nz_index;
        }

        // Update index
        l_index += current_edges_num;
        n_index += 4;
        constraint_index += 3;
      }
    }

    l_index = l_start_index_;
    n_index = n_start_index_;
    for (int i = 0; i < lambda_horizon_; ++i) {
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = l_index;
      ++nz_index;
      ++constraint_index;
      ++l_index;
    }
    for (int i = 0; i < miu_horizon_; ++i) {
      iRow[nz_index] = constraint_index;
      jCol[nz_index] = n_index;
      ++nz_index;
      ++constraint_index;
      ++n_index;
    }

    CHECK_EQ(constraint_index, m) << "No. of constraints wrong in eval_jac_g.";

    ADEBUG << "nz_index here : " << nz_index << " nele_jac is : " << nele_jac;
  } else {
    std::fill(values, values + nele_jac, 0.0);
    int nz_index = 0;

    // 1. Three obstacles related equal constraints, one equality
    // constraints,
    // [0, horizon_] * [0, obstacles_num_-1] * 3
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

        double tmp1 = 0;
        double tmp2 = 0;
        for (int k = 0; k < current_edges_num; ++k) {
          // TODO(QiL) : replace this one directly with x
          tmp1 += Aj(k, 0) * x[l_index + k];
          tmp2 += Aj(k, 1) * x[l_index + k];
        }

        // 1. G' * mu + R' * lambda == 0, part 1
        // with respect to l
        for (int k = 0; k < current_edges_num; ++k) {
          values[nz_index] = std::cos(xWS_(2, i)) * Aj(k, 0) +
                             std::sin(xWS_(2, i)) * Aj(k, 1);  // v0~vn
          ++nz_index;
        }

        // With respect to n
        values[nz_index] = 1.0;  // w0
        ++nz_index;

        values[nz_index] = -1.0;  // w2
        ++nz_index;

        ADEBUG << "eval_jac_g, after adding part 2";
        // 1. G' * mu + R' * lambda == 0, part 2

        // with respect to l
        for (int k = 0; k < current_edges_num; ++k) {
          values[nz_index] = -std::sin(xWS_(2, i)) * Aj(k, 0) +
                             std::cos(xWS_(2, i)) * Aj(k, 1);  // y0~yn
          ++nz_index;
        }

        // With respect to n
        values[nz_index] = 1.0;  // z1
        ++nz_index;

        values[nz_index] = -1.0;  // z3
        ++nz_index;

        //  2. -g'*mu + (A*t - b)*lambda > 0
        // TODO(QiL): Revise dual variables modeling here.
        double tmp3 = 0.0;
        double tmp4 = 0.0;
        for (int k = 0; k < 4; ++k) {
          tmp3 += -g_[k] * x[n_index + k];
        }

        for (int k = 0; k < current_edges_num; ++k) {
          tmp4 += bj(k, 0) * x[l_index + k];
        }

        // with respect to l
        for (int k = 0; k < current_edges_num; ++k) {
          values[nz_index] =
              (xWS_(0, i) + std::cos(xWS_(2, i)) * offset_) * Aj(k, 0) +
              (xWS_(1, i) + std::sin(xWS_(2, i)) * offset_) * Aj(k, 1) -
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
    }

    for (int i = 0; i < lambda_horizon_; ++i) {
      values[nz_index] = 1.0;
      ++nz_index;
    }
    for (int i = 0; i < miu_horizon_; ++i) {
      values[nz_index] = 1.0;
      ++nz_index;
    }

    ADEBUG << "eval_jac_g, fulfilled obstacle constraint values";
    CHECK_EQ(nz_index, nele_jac);
  }

  ADEBUG << "eval_jac_g done";
  return true;
}

bool DualVariableWarmStartIPOPTQPInterface::eval_h(
    int n, const double* x, bool new_x, double obj_factor, int m,
    const double* lambda, bool new_lambda, int nele_hess, int* iRow, int* jCol,
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

void DualVariableWarmStartIPOPTQPInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  int variable_index = 0;
  // 1. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      l_warm_up_(j, i) = x[variable_index];
      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange l : " << variable_index;

  // 2. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      n_warm_up_(j, i) = x[variable_index];
      ++variable_index;
    }
  }
  ADEBUG << "variable_index after adding lagrange n : " << variable_index;

  // memory deallocation of ADOL-C variables
  delete[] obj_lam;
  free(rind_L);
  free(cind_L);
  free(hessval);
}

void DualVariableWarmStartIPOPTQPInterface::get_optimization_results(
    Eigen::MatrixXd* l_warm_up, Eigen::MatrixXd* n_warm_up) const {
  *l_warm_up = l_warm_up_;
  *n_warm_up = n_warm_up_;
}

void DualVariableWarmStartIPOPTQPInterface::check_solution(
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up) {
  // wrap input solution
  int kNumVariables = num_of_variables_;
  double x[kNumVariables];
  int variable_index = 0;
  // 1. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      x[variable_index] = l_warm_up(j, i);
      ++variable_index;
    }
  }

  // 2. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      x[variable_index] = n_warm_up(j, i);
      ++variable_index;
    }
  }

  // evaluate constraints
  int kNumConstraint = num_of_constraints_;
  double g[kNumConstraint];
  eval_g(num_of_variables_, x, true, num_of_constraints_, g);

  // get the boundaries
  double x_l[kNumVariables];
  double x_u[kNumVariables];
  double g_l[kNumConstraint];
  double g_u[kNumConstraint];
  get_bounds_info(num_of_variables_, x_l, x_u, num_of_constraints_, g_l, g_u);

  // compare g with g_l & g_u
  int constraint_index = 0;

  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_num_; ++j) {
      // G' * mu + R' * A * lambda == 0
      if (g_l[constraint_index + 0] > g[constraint_index + 0] ||
          g_u[constraint_index + 0] < g[constraint_index + 0]) {
        AERROR << "G' * mu + R' * A * lambda == 0 constraint fails, "
               << "constraint_index: " << constraint_index + 0
               << ", g: " << g[constraint_index + 0];
      }

      if (g_l[constraint_index + 1] > g[constraint_index + 1] ||
          g_u[constraint_index + 1] < g[constraint_index + 1]) {
        AERROR << "G' * mu + R' * A * lambda == 0 constraint fails, "
               << "constraint_index: " << constraint_index + 1
               << ", g: " << g[constraint_index + 1];
      }

      // -g' * mu + (A * t - b) * lambda) >= d_min
      if (g_l[constraint_index + 2] > g[constraint_index + 2] ||
          g_u[constraint_index + 2] < g[constraint_index + 2]) {
        AERROR << "-g' * mu + (A * t - b) * lambda) >= d_min constraint fails, "
               << "constraint_index: " << constraint_index + 2
               << ", g: " << g[constraint_index + 2];
      }

      // Update index
      constraint_index += 3;
    }
  }

  for (int i = 0; i < lambda_horizon_; ++i) {
    if (g_l[constraint_index] > g[constraint_index] ||
        g_u[constraint_index] < g[constraint_index]) {
      AERROR << "lambda box constraint fails, "
             << "constraint_index: " << constraint_index
             << ", g: " << g[constraint_index];
    }

    constraint_index++;
  }

  for (int i = 0; i < miu_horizon_; ++i) {
    if (g_l[constraint_index] > g[constraint_index] ||
        g_u[constraint_index] < g[constraint_index]) {
      AERROR << "miu box constraint fails, "
             << "constraint_index: " << constraint_index
             << ", g: " << g[constraint_index];
    }
    constraint_index++;
  }
}

//***************    start ADOL-C part ***********************************
/** Template to return the objective value */
template <class T>
bool DualVariableWarmStartIPOPTQPInterface::eval_obj(int n, const T* x,
                                                     T* obj_value) {
  ADEBUG << "eval_obj";
  *obj_value = 0.0;
  int l_index = l_start_index_;
  for (int i = 0; i < horizon_ + 1; ++i) {
    int edges_counter = 0;
    // assume: stationary obstacles
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
        tmp1 += Aj(k, 0) * x[l_index + k];
        tmp2 += Aj(k, 1) * x[l_index + k];
      }
      *obj_value += tmp1 * tmp1 + tmp2 * tmp2;

      // Update index
      edges_counter += current_edges_num;
      l_index += current_edges_num;
    }
  }

  return true;
}

/** Template to compute constraints */
template <class T>
bool DualVariableWarmStartIPOPTQPInterface::eval_constraints(int n, const T* x,
                                                             int m, T* g) {
  ADEBUG << "eval_constraints";
  // state start index

  // 1. Three obstacles related equal constraints, one equality constraints,
  // [0, horizon_] * [0, obstacles_num_-1] * 4

  int l_index = l_start_index_;
  int n_index = n_start_index_;
  int constraint_index = 0;

  for (int i = 0; i < horizon_ + 1; ++i) {
    int edges_counter = 0;
    // assume: stationary obstacles
    for (int j = 0; j < obstacles_num_; ++j) {
      int current_edges_num = obstacles_edges_num_(j, 0);
      Eigen::MatrixXd Aj =
          obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
      Eigen::MatrixXd bj =
          obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

      // A * lambda
      T tmp1 = 0.0;
      T tmp2 = 0.0;
      for (int k = 0; k < current_edges_num; ++k) {
        tmp1 += Aj(k, 0) * x[l_index + k];
        tmp2 += Aj(k, 1) * x[l_index + k];
      }

      // G' * mu + R' * A * lambda == 0
      g[constraint_index + 0] = x[n_index] - x[n_index + 2] +
                                cos(xWS_(2, i)) * tmp1 + sin(xWS_(2, i)) * tmp2;

      g[constraint_index + 1] = x[n_index + 1] - x[n_index + 3] -
                                sin(xWS_(2, i)) * tmp1 + cos(xWS_(2, i)) * tmp2;

      // (-g' * mu + (A * t - b) * lambda) >= d_min
      T tmp3 = 0.0;
      for (int k = 0; k < 4; ++k) {
        tmp3 += g_[k] * x[n_index + k];
      }

      T tmp4 = 0.0;
      for (int k = 0; k < current_edges_num; ++k) {
        tmp4 += bj(k, 0) * x[l_index + k];
      }

      g[constraint_index + 2] =
          -tmp3 + (xWS_(0, i) + cos(xWS_(2, i)) * offset_) * tmp1 +
          (xWS_(1, i) + sin(xWS_(2, i)) * offset_) * tmp2 - tmp4;

      // Update index
      edges_counter += current_edges_num;
      l_index += current_edges_num;
      n_index += 4;
      constraint_index += 3;
    }
  }
  l_index = l_start_index_;
  n_index = n_start_index_;
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

  CHECK_EQ(constraint_index, m)
      << "No. of constraints wrong in eval_g. n : " << n;
  return true;
}

/** Method to generate the required tapes */
void DualVariableWarmStartIPOPTQPInterface::generate_tapes(int n, int m,
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

  // trace_on(tag_f);

  // for (int idx = 0; idx < n; idx++) xa[idx] <<= xp[idx];

  // eval_obj(n, xa, &obj_value);

  // obj_value >>= dummy;

  // trace_off();

  // trace_on(tag_g);

  // for (int idx = 0; idx < n; idx++) xa[idx] <<= xp[idx];

  // eval_constraints(n, xa, m, g);

  // for (int idx = 0; idx < m; idx++) g[idx] >>= dummy;

  // trace_off();

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
